#!/bin/bash
# Start TurtleBot3 Burger Gazebo simulation
# Usage: ./start_sim.sh [world]
#   world: "world" (default), "house", "empty", "dock", "maze", "template", or "dashboard"

set -e

# Remove conda from PATH so ROS uses system Python 3.10
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')

# Source ROS 2
source /opt/ros/humble/setup.bash

# Rebuild
echo "Building workspace..."
cd ~/sim_ws
colcon build --packages-select turtlebot3_gazebo --symlink-install
source install/setup.bash

# Set model
export TURTLEBOT3_MODEL=burger

# Pick world
WORLD="${1:-world}"
case "$WORLD" in
    world) LAUNCH="turtlebot3_world.launch.py" ;;
    house) LAUNCH="turtlebot3_house.launch.py" ;;
    empty) LAUNCH="empty_world.launch.py" ;;
    dock)  LAUNCH="arena_dock.launch.py" ;;
    maze)     LAUNCH="arena_maze.launch.py" ;;
    template) LAUNCH="arena_template_maze.launch.py" ;;
    dashboard)
        exec ~/sim_ws/src/sim_dashboard/run_dashboard.sh "${@:2}"
        ;;
    *)
        echo "Unknown world: $WORLD (use: world, house, empty, dock, maze, template, dashboard)"
        exit 1
        ;;
esac

echo "Launching $LAUNCH ..."
ros2 launch turtlebot3_gazebo "$LAUNCH"
