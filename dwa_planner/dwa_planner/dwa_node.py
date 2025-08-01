#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Point
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from visualization_msgs.msg import Marker, MarkerArray
import numpy as np
import math


class DWAPlanner(Node):
    def __init__(self):
        super().__init__('dwa_planner')

        # ROS interfaces
        self.cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom_sub = self.create_subscription(Odometry, '/odom', self.odom_callback, 10)
        self.scan_sub = self.create_subscription(LaserScan, '/scan', self.scan_callback, 10)
        self.marker_pub = self.create_publisher(MarkerArray, '/trajectory_markers', 10)

        # Robot state
        self.pose = [0.0, 0.0, 0.0]  # x, y, theta (radians)
        self.velocity = [0.0, 0.0]   # linear velocity, angular velocity
        self.scan = None
        self.obstacles = []

        # Timer and timing parameters
        self.dt = 0.1
        self.timer = self.create_timer(self.dt, self.control_loop)

        # Robot constraints and DWA parameters
        self.max_lin_vel = 0.2
        self.min_lin_vel = 0.0
        self.max_ang_vel = 0.9
        self.max_lin_acc = 0.8
        self.max_ang_acc = 2.0
        self.predict_time = 2.5
        self.dt_traj = 0.1
        self.v_res = 0.02
        self.w_res = 0.1

        # Cost function parameters
        self.to_goal_cost_gain = 0.1
        self.obstacle_cost_gain = 0.7
        self.speed_cost_gain = 0.05

        # Robot physical parameters
        self.robot_radius = 0.15
        self.safe_distance = 0.2
        self.collision_distance = 0.13

        # Specify your goal sequence
        self.goals = [
            [3.0, 0.0],
            [4.0, 5.0],
            [0.0, 5.0],
            [4.0, 5.0],
            [3.0, 0.0],
            [0.0, 0.0]
        ]
        self.current_goal_idx = 0
        self.goal_tolerance = 0.15

        self.get_logger().info(f"Starting DWA Planner. First goal: {self.goals[0]}")

    def odom_callback(self, msg):
        self.velocity[0] = msg.twist.twist.linear.x
        self.velocity[1] = msg.twist.twist.angular.z

        self.pose[0] = msg.pose.pose.position.x
        self.pose[1] = msg.pose.pose.position.y

        orientation = msg.pose.pose.orientation
        siny_cosp = 2.0 * (orientation.w * orientation.z + orientation.x * orientation.y)
        cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z)
        self.pose[2] = math.atan2(siny_cosp, cosy_cosp)

    def scan_callback(self, msg):
        self.scan = msg
        self.process_scan(msg)

    def process_scan(self, scan_msg):
        if not scan_msg.ranges:
            return
        self.obstacles = []
        angle = scan_msg.angle_min
        for dist in scan_msg.ranges:
            if 0.1 <= dist <= 3.0 and not math.isinf(dist) and not math.isnan(dist):
                x = dist * math.cos(angle)
                y = dist * math.sin(angle)
                if dist <= 2.0:
                    self.obstacles.append([x, y])
            angle += scan_msg.angle_increment

    def normalize_angle(self, angle):
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

    def motion_model(self, state, control, dt):
        x, y, theta = state
        v, w = control
        x += v * math.cos(theta) * dt
        y += v * math.sin(theta) * dt
        theta += w * dt
        theta = self.normalize_angle(theta)
        return [x, y, theta]

    def calc_dynamic_window(self):
        vs = [self.min_lin_vel, self.max_lin_vel, -self.max_ang_vel, self.max_ang_vel]
        vd = [
            self.velocity[0] - self.max_lin_acc * self.dt,
            self.velocity[0] + self.max_lin_acc * self.dt,
            self.velocity[1] - self.max_ang_acc * self.dt,
            self.velocity[1] + self.max_ang_acc * self.dt,
        ]
        dw = [
            max(vs[0], vd[0]),
            min(vs[1], vd[1]),
            max(vs[2], vd[2]),
            min(vs[3], vd[3]),
        ]
        return dw

    def predict_trajectory(self, v, w):
        traj = []
        state = self.pose.copy()
        time = 0.0
        while time <= self.predict_time:
            traj.append(state.copy())
            state = self.motion_model(state, [v, w], self.dt_traj)
            time += self.dt_traj
        return traj

    def calc_to_goal_cost(self, traj, goal):
        if not traj:
            return float('inf')
        last = traj[-1]
        dx = goal[0] - last[0]
        dy = goal[1] - last[1]
        return math.hypot(dx, dy)

    def calc_obstacle_cost(self, traj):
        if not self.obstacles or not traj:
            return 0.0
        min_dist = float('inf')
        for point in traj:
            dx = point[0] - self.pose[0]
            dy = point[1] - self.pose[1]
            cos_theta = math.cos(-self.pose[2])
            sin_theta = math.sin(-self.pose[2])
            local_x = dx * cos_theta - dy * sin_theta
            local_y = dx * sin_theta + dy * cos_theta
            for ox, oy in self.obstacles:
                dist = math.hypot(local_x - ox, local_y - oy)
                if dist < self.collision_distance:
                    return 100.0  # Collision
                min_dist = min(min_dist, dist)
        if min_dist < self.safe_distance:
            return 10.0 * (self.safe_distance - min_dist)
        return 0.0

    def calc_velocity_cost(self, v):
        return (self.max_lin_vel - v) / self.max_lin_vel

    def dwa_planning(self, goal):
        dw = self.calc_dynamic_window()
        v_samples = np.arange(dw[0], dw[1] + self.v_res, self.v_res)
        w_samples = np.arange(dw[2], dw[3] + self.w_res, self.w_res)

        best_u = [0.0, 0.0]
        min_cost = float('inf')
        best_traj = []

        all_trajs = []

        for v in v_samples:
            for w in w_samples:
                traj = self.predict_trajectory(v, w)
                to_goal = self.calc_to_goal_cost(traj, goal)
                obstacle = self.calc_obstacle_cost(traj)
                velo = self.calc_velocity_cost(v)

                if obstacle >= 50.0:
                    continue

                total_cost = (self.to_goal_cost_gain * to_goal +
                              self.obstacle_cost_gain * obstacle +
                              self.speed_cost_gain * velo)

                all_trajs.append(traj)

                if total_cost < min_cost:
                    min_cost = total_cost
                    best_u = [v, w]
                    best_traj = traj

        self.publish_trajectories(all_trajs, best_traj)

        if min_cost == float('inf'):
            return [0., 0.]
        return best_u

    def publish_trajectories(self, all_trajs, best_traj):
        marker_array = MarkerArray()
        clear_marker = Marker()
        clear_marker.action = Marker.DELETEALL
        marker_array.markers.append(clear_marker)

        for i, traj in enumerate(all_trajs[:150]):
            marker = Marker()
            marker.header.frame_id = "odom"
            marker.header.stamp = self.get_clock().now().to_msg()
            marker.id = i + 100
            marker.ns = "candidate_trajs"
            marker.type = Marker.LINE_STRIP
            marker.action = Marker.ADD
            marker.scale.x = 0.01
            marker.color.r = 1.0
            marker.color.g = 1.0
            marker.color.b = 0.2
            marker.color.a = 0.1
            marker.points = [Point(x=pt[0], y=pt[1], z=0.) for pt in traj]
            marker_array.markers.append(marker)

        if best_traj:
            best_marker = Marker()
            best_marker.header.frame_id = "odom"
            best_marker.header.stamp = self.get_clock().now().to_msg()
            best_marker.id = 0
            best_marker.ns = "best_traj"
            best_marker.type = Marker.LINE_STRIP
            best_marker.action = Marker.ADD
            best_marker.scale.x = 0.05
            best_marker.color.r = 0.0
            best_marker.color.g = 1.0
            best_marker.color.b = 0.0
            best_marker.color.a = 1.0
            best_marker.points = [Point(x=pt[0], y=pt[1], z=0.) for pt in best_traj]
            marker_array.markers.append(best_marker)

        if self.current_goal_idx < len(self.goals):
            goal_marker = Marker()
            goal_marker.header.frame_id = "odom"
            goal_marker.header.stamp = self.get_clock().now().to_msg()
            goal_marker.id = 1000
            goal_marker.ns = "goal"
            goal_marker.type = Marker.SPHERE
            goal_marker.action = Marker.ADD
            goal_marker.pose.position.x = float(self.goals[self.current_goal_idx][0])
            goal_marker.pose.position.y = float(self.goals[self.current_goal_idx][1])
            goal_marker.pose.position.z = 0.1
            goal_marker.scale.x = 0.3
            goal_marker.scale.y = 0.3
            goal_marker.scale.z = 0.3
            goal_marker.color.r = 1.0
            goal_marker.color.g = 0.0
            goal_marker.color.b = 0.0
            goal_marker.color.a = 0.8
            marker_array.markers.append(goal_marker)

        self.marker_pub.publish(marker_array)

    def control_loop(self):
        if self.scan is None:
            self.get_logger().warn("Waiting for laser scan...")
            return
        if self.current_goal_idx >= len(self.goals):
            self.get_logger().info("All goals completed. Stopping.")
            self.cmd_pub.publish(Twist())
            return
        goal = self.goals[self.current_goal_idx]
        dist_to_goal = math.hypot(goal[0] - self.pose[0], goal[1] - self.pose[1])
        if dist_to_goal < self.goal_tolerance:
            self.get_logger().info(f"Goal {self.current_goal_idx+1} reached.")
            self.current_goal_idx +=1
            if self.current_goal_idx < len(self.goals):
                self.get_logger().info(f"Next Goal: {self.goals[self.current_goal_idx]}")
            self.cmd_pub.publish(Twist())  # Stop before next goal
            return

        best_v, best_w = self.dwa_planning(goal)
        cmd = Twist()
        cmd.linear.x = np.clip(best_v, self.min_lin_vel, self.max_lin_vel)
        cmd.angular.z = np.clip(best_w, -self.max_ang_vel, self.max_ang_vel)
        self.cmd_pub.publish(cmd)

        closest_obstacle = min([math.hypot(x, y) for x,y in self.obstacles], default=float('inf'))
        self.get_logger().info(f"Goal {self.current_goal_idx+1}: Dist={dist_to_goal:.2f}, Obstacle={closest_obstacle:.2f}, Cmd_v={cmd.linear.x:.2f}, Cmd_w={cmd.angular.z:.2f}")


def main(args=None):
    rclpy.init(args=args)
    node = DWAPlanner()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
