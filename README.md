#  Custom DWA Local Planner for TurtleBot3 in ROS 2 Humble

This project implements a Dynamic Window Approach (DWA) based local planner for TurtleBot3 in a Gazebo simulation using ROS 2 Humble. The planner is built from scratch, without using the 'nav2_dwb_controller'. It allows the robot to navigate towards a goal while avoiding obstacles by sampling velocity commands, predicting trajectories, and selecting the optimal one using a cost function.

---

##  Features

-  Implements core DWA algorithm
-  Integrates with ROS 2 topics: `/odom`, `/scan`, `/cmd_vel`
-  Visualizes trajectories in RViz using `MarkerArray`
- Fully compatible with **TurtleBot3 Burger** in **Gazebo**

---

## Project Structure

```
tenx/
├── src/
│   └── dwa_planner/
│       ├── dwa_planner/
│       │   ├── __init__.py
│       │   └── dwa_node.py         # Main implementation
│       ├── package.xml
│       ├── setup.py
│       └── setup.cfg
```

---

##  Dependencies

- ROS 2 Humble
- Gazebo
- TurtleBot3 Simulation Packages

---

##  Installation & Build

### 1. Clone this repository inside your ROS 2 workspace:
```bash
cd ~/Music/tenx/src
git clone <your-repo-url> dwa_planner
```

> _Skip this if you already have the package created manually._

---

### 2. Install dependencies
Make sure you have TurtleBot3 and Gazebo installed:

```bash
sudo apt update
sudo apt install ros-humble-turtlebot3* ros-humble-gazebo-ros-pkgs
```

---

### 3. Build the workspace

```bash
cd ~/Music/tenx
source /opt/ros/humble/setup.bash
colcon build --packages-select dwa_planner
source install/setup.bash
```

---

## Run Instructions

### 1. Launch the TurtleBot3 Gazebo World
```bash
export TURTLEBOT3_MODEL=burger
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py
```

---

### 2. Run the Custom DWA Planner Node
```bash
source ~/Music/tenx/install/setup.bash
ros2 run dwa_planner dwa_node
```

---

### 3. Visualize in RViz (Optional)

```bash
ros2 launch turtlebot3_bringup rviz2.launch.py
```

- Click on “2D Nav Goal” in RViz to set a goal position.
- Watch the robot avoid obstacles and move towards the goal.

---

##  How It Works

- Velocity Sampling: Samples linear and angular velocities within robot’s dynamic window.
- Trajectory Prediction: Predicts future paths for each sampled velocity using motion equations.
- Cost Evaluation: Each trajectory is scored based on:
  - Distance to goal
  - Obstacle proximity
  - Path smoothness
- Best Command: The safest and most efficient trajectory is selected and published to `/cmd_vel`.

---

## 🔍 Topics Used

| Topic              | Type                          | Description                  |
|-------------------|-------------------------------|------------------------------|
| `/odom`           | `nav_msgs/msg/Odometry`       | Robot position & velocity    |
| `/scan`           | `sensor_msgs/msg/LaserScan`   | Obstacle data (LIDAR)        |
| `/cmd_vel`        | `geometry_msgs/msg/Twist`     | Velocity commands             |
| `/trajectory_markers` | `visualization_msgs/msg/MarkerArray` | Predicted paths for RViz   |

---

##  Debugging & Logs

- All internal states (e.g., sampled velocities, cost values, chosen velocity) are logged with helpful messages.
- Check logs with:
```bash
ros2 run dwa_planner dwa_node
```

---

##  Author

- Bharathwaj M
- https://www.linkedin.com/in/bharathwaj-darkangel0011/

---
##  Video Link

- https://drive.google.com/file/d/1JAwI80g4isZ-iq6a8H8xnnjL5rVo3taU/view?usp=sharing
