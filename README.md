# CDE2310 Simulation Workspace

TurtleBot3 Burger simulation with Gazebo, a web dashboard for camera/LIDAR/teleop,
and an ArUco visual servoing module for autonomous docking.

> **For AI assistants / LLMs**: see [llms.txt](llms.txt) for a complete technical
> reference of every package, topic, node, launch file, and command in this workspace.

---

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble (`ros-humble-desktop`)
- Python 3.10

```bash
# ROS dependencies
sudo apt install \
  ros-humble-turtlebot3-msgs \
  ros-humble-cv-bridge \
  ros-humble-ros-gz-bridge \
  ros-humble-ros-gz-sim \
  ros-humble-ros-gz-image

# Python dependencies
pip install flask opencv-python numpy
```

---

## Setup

```bash
git clone https://github.com/Arnav-Jhajharia/cde2310_sim_ws.git ~/sim_ws
cd ~/sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

---

## Running

### 1. Start the simulation (Terminal 1)

```bash
./start_sim.sh [WORLD]
```

| WORLD | Description |
|---|---|
| *(none)* | Default hexagonal arena |
| `house` | House environment |
| `maze` | Maze arena |
| `empty` | Empty world |
| `dock` | Dock/charging station arena |
| `template` | Template maze |

### 2. Start the web dashboard (Terminal 2)

```bash
cd src/sim_dashboard
./run_dashboard.sh            # http://localhost:5000
./run_dashboard.sh --ngrok    # public URL via ngrok
```

Dashboard features:
- Live camera feeds (front, left, right)
- LiDAR polar visualization
- Keyboard / joystick / D-pad teleop (WASD + QE + Space)
- Odometry readout
- Navigation presets and custom distance/angle commands

### 3. Start visual servoing (Terminal 3, optional)

```bash
./start_vs.sh
```

The robot will search for ArUco marker ID 42 and autonomously dock in front of it.

### Clean restart

```bash
./reset_sim.sh [WORLD]    # kills all processes, restarts fresh
```

---

## Packages

| Package | Type | Description |
|---|---|---|
| `turtlebot3_gazebo` | ament_cmake (C++) | Gazebo worlds, models, launch files, LIDAR drive node |
| `turtlebot3_fake_node` | ament_cmake (C++) | Fake odometry for offline testing |
| `tb3_cv` | ament_python | ArUco marker detection + visual servoing controller |
| `sim_dashboard` | plain Python | Flask web dashboard (not a ROS package) |

---

## Key Topics

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | Twist | Send velocity commands to the robot |
| `/scan` | LaserScan | 360° LIDAR |
| `/odom` | Odometry | Position and velocity |
| `/camera_front/image_raw` | Image | Front camera |
| `/camera_left/image_raw` | Image | Left camera |
| `/camera_right/image_raw` | Image | Right camera |
| `/aruco_debug/image_raw` | Image | ArUco detection overlay |

---

## Quick Commands

```bash
# Source environment
source /opt/ros/humble/setup.bash && source ~/sim_ws/install/setup.bash

# List running nodes
ros2 node list

# Drive forward manually
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.1}, angular: {z: 0.0}}"

# Stop the robot
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  "{linear: {x: 0.0}, angular: {z: 0.0}}"

# Build a single package
colcon build --symlink-install --packages-select tb3_cv
```

---

See [llms.txt](llms.txt) for the full technical reference.
