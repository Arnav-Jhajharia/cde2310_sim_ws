# CDE2310 Simulation Workspace

TurtleBot3 Burger simulation environment with a web dashboard for camera, LIDAR, and teleop control.

## Prerequisites

- Ubuntu 22.04
- ROS 2 Humble
- Gazebo (comes with `ros-humble-desktop`)
- Python 3.10 with `flask`, `opencv-python`, `cv_bridge`, `numpy`

Install ROS 2 dependencies:

```bash
sudo apt install ros-humble-turtlebot3-msgs ros-humble-cv-bridge
pip install flask opencv-python
```

## Setup

```bash
git clone https://github.com/Arnav-Jhajharia/cde2310_sim_ws.git ~/sim_ws
cd ~/sim_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install
source install/setup.bash
```

## Running the Simulation

Start a Gazebo world (default is `world`):

```bash
./start_sim.sh          # default world
./start_sim.sh house    # house world
./start_sim.sh maze     # maze world
./start_sim.sh empty    # empty world
./start_sim.sh dock     # dock world
```

To kill all sim processes and restart clean:

```bash
./reset_sim.sh          # restarts with default world
./reset_sim.sh maze     # restarts with maze world
```

## Web Dashboard

In a separate terminal (with ROS 2 sourced):

```bash
cd src/sim_dashboard
./run_dashboard.sh
```

Access the dashboard in your browser to view camera feeds, LIDAR data, and teleop controls.

## Packages

- **turtlebot3_simulations** — TurtleBot3 Gazebo launch files and worlds
- **sim_dashboard** — Flask-based web dashboard for robot monitoring and control
