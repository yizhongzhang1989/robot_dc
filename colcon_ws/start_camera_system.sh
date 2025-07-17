#!/bin/bash

# Camera System Startup Script
# Convenient script to start both cam_node and web_node together

echo "=== Starting Camera System ==="

# Source ROS2 environment
source install/setup.bash

echo "Starting cam_node..."
ros2 launch cam_node cam_launch.py &
CAM_PID=$!

echo "Starting web interface..."
ros2 launch robot_web web_launch.py &
WEB_PID=$!

echo "=== Camera System Started ==="
echo "cam_node PID: $CAM_PID"
echo "web_node PID: $WEB_PID"
echo ""
echo "Both services are now running with auto-reconnection support"
echo "To stop the system, run: kill $CAM_PID $WEB_PID"
echo "Or press Ctrl+C to stop all processes"
echo ""

# Wait for user interrupt
trap "echo 'Stopping camera system...'; kill $CAM_PID $WEB_PID 2>/dev/null; exit" INT
wait
