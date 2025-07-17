#!/bin/bash

# Web-based URDF Visualization Script (Fixed version)
# This script launches all necessary components for web-based robot visualization

# Set up ROS2 environment
cd /home/a/Documents/robot_dc/colcon_ws
source install/setup.bash

echo "Starting web-based URDF visualization..."

# Kill any existing ROS2 processes
echo "Cleaning up existing processes..."
pkill -f "robot_state_publisher" || true
pkill -f "joint_state_publisher" || true  
pkill -f "rosbridge_websocket" || true

# Kill processes by port if they exist
for port in 9090 9091 9092; do
    pid=$(lsof -ti :$port 2>/dev/null || true)
    if [ ! -z "$pid" ]; then
        echo "Killing process on port $port (PID: $pid)"
        kill -9 $pid 2>/dev/null || true
    fi
done

# Wait a moment for processes to clean up
sleep 3

# Create a temporary parameter file for robot_description
URDF_FILE="install/duco_gcr5_910_urdf/share/duco_gcr5_910_urdf/urdf/duco_gcr5_910_urdf.urdf"
PARAM_FILE="/tmp/robot_description_params.yaml"

echo "robot_state_publisher:" > $PARAM_FILE
echo "  ros__parameters:" >> $PARAM_FILE
echo "    robot_description: |" >> $PARAM_FILE
sed 's/^/      /' $URDF_FILE >> $PARAM_FILE

# Start robot_state_publisher using parameter file
echo "Starting robot_state_publisher..."
ros2 run robot_state_publisher robot_state_publisher --ros-args --params-file $PARAM_FILE &

# Wait for robot_state_publisher to start
sleep 3

# Start joint_state_publisher
echo "Starting joint_state_publisher..."
ros2 run joint_state_publisher joint_state_publisher &

# Wait for joint_state_publisher to start
sleep 2

# Start rosbridge_websocket
echo "Starting rosbridge_websocket on port 9092..."
ros2 run rosbridge_server rosbridge_websocket --ros-args -p port:=9092 -p address:="0.0.0.0" &

# Wait for rosbridge to start
sleep 3

echo "All services started!"
echo "ROS Bridge WebSocket server running on port 9092"
echo ""
echo "To visualize in browser:"
echo "1. Open https://studio.foxglove.dev/"
echo "2. Choose 'Open connection'"
echo "3. Select 'Rosbridge (ROS 1 & 2)'"
echo "4. Enter WebSocket URL: ws://$(hostname -I | awk '{print $1}'):9092"
echo "5. Click 'Open'"
echo ""
echo "Available topics:"
ros2 topic list

echo ""
echo "Press Ctrl+C to stop all services"

# Wait for user interrupt
wait
