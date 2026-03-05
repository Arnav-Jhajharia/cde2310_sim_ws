#!/bin/bash
# Kill all running sim processes and restart clean
# Usage: ./reset_sim.sh [world]

set -e

echo "Killing existing sim processes..."
pkill -f "gz sim" 2>/dev/null || true
pkill -f "ros2" 2>/dev/null || true
pkill -f "gzserver" 2>/dev/null || true
pkill -f "gzclient" 2>/dev/null || true
pkill -f "parameter_bridge" 2>/dev/null || true
pkill -f "image_bridge" 2>/dev/null || true
pkill -f "robot_state_publisher" 2>/dev/null || true

# Wait for processes to die
sleep 2

echo "Starting fresh sim..."
exec ~/sim_ws/start_sim.sh "${1:-world}"
