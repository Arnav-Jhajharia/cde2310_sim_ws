#!/bin/bash
# Start the ArUco visual servoing algorithm (PBVS)
# Usage: ./start_vs.sh

set -e

# Remove conda from PATH so ROS uses system Python 3.10
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')

# Source ROS 2
source /opt/ros/humble/setup.bash

# Rebuild
echo "Building workspace..."
cd ~/sim_ws
colcon build --packages-select tb3_cv --symlink-install
source install/setup.bash

# Set model
export TURTLEBOT3_MODEL=burger

echo "Starting visual servoing node (ArUco PBVS)..."
ros2 run tb3_cv aruco_pose
