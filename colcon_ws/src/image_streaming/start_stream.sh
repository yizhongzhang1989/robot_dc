#!/bin/bash
# Quick start script for image streaming

# Source ROS 2 workspace
cd /home/a3r/liza_2_4_testing/robot_dc/colcon_ws
source install/setup.bash

# Default values
IMAGE_TOPIC="${1:-/camera1/camera1/color/image_raw}"
PORT="${2:-8080}"
QUALITY="${3:-85}"

echo "=========================================="
echo "Starting Image Streaming Server"
echo "=========================================="
echo "Image Topic: $IMAGE_TOPIC"
echo "Port: $PORT"
echo "Quality: $QUALITY"
echo ""
echo "View stream at: http://$(hostname -I | awk '{print $1}'):$PORT/"
echo ""
echo "Press Ctrl+C to stop"
echo "=========================================="

ros2 run image_streaming image_streamer --ros-args \
  -p image_topic:=$IMAGE_TOPIC \
  -p port:=$PORT \
  -p quality:=$QUALITY
