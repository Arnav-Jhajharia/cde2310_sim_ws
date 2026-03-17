#!/bin/bash
# Run the web dashboard
# Usage: ./run_dashboard.sh [--ngrok]

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"

# Remove conda from PATH
export PATH=$(echo "$PATH" | tr ':' '\n' | grep -v conda | tr '\n' ':' | sed 's/:$//')

# Source ROS 2
source /opt/ros/humble/setup.bash
source ~/sim_ws/install/setup.bash 2>/dev/null || true

# Install Flask if needed
pip3 install flask 2>/dev/null | grep -v "already satisfied" || true

echo ""
echo "=================================="
echo "  TurtleBot3 Web Dashboard"
echo "=================================="
echo ""

# Launch ngrok in background if requested
if [ "$1" = "--ngrok" ]; then
    echo "Starting ngrok tunnel..."
    ngrok http 5000 --log=stdout > /dev/null &
    NGROK_PID=$!
    sleep 2
    # Get the public URL
    NGROK_URL=$(curl -s http://localhost:4040/api/tunnels | python3 -c "import sys,json; print(json.load(sys.stdin)['tunnels'][0]['public_url'])" 2>/dev/null)
    if [ -n "$NGROK_URL" ]; then
        echo "  ngrok URL: $NGROK_URL"
    else
        echo "  ngrok started but couldn't get URL. Check http://localhost:4040"
    fi
    echo ""
    trap "kill $NGROK_PID 2>/dev/null" EXIT
fi

echo "  Local:  http://localhost:5000"
echo ""

python3 "$SCRIPT_DIR/dashboard.py"
