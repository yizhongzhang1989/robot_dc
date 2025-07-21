# Cam Node Package

## Overview
This package provides ROS2 camera snapshot service, supporting image snapshots from two RTSP camera streams (high and low resolution).

## Features
- Connect to two RTSP camera streams (192.168.1.100 and 192.168.1.101)
- Provide ROS2 snapshot service (`/snapshot`)
- Return base64-encoded JPEG images
- Basic error handling and reconnection mechanism

## Current Protection Mechanisms

### 1. **Basic Timeout Detection**
- ffprobe stream resolution acquisition: 10-second timeout
- Default resolution fallback: 640x480

### 2. **Process Management**
- Use subprocess.Popen to start ffmpeg
- Daemon threads handle frame reading
- Graceful process cleanup and resource release

### 3. **Thread Safety**
- Use threading.Lock() to protect frame data access
- Daemon threads prevent blocking main process exit

### 4. **Basic Error Handling**
- Incomplete frame detection and skipping
- Exception catching and logging
- Frame counting and periodic status output

### 5. **Resource Management**
- Automatic process termination (stop method)
- Thread synchronization and waiting
- Memory management (frame data copying)

## Launch Commands

### Launch cam_node independently
```bash
cd /home/jetson/Desktop/robot_dc/colcon_ws
source install/setup.bash
ros2 launch cam_node cam_launch.py
```

## Service Interface

### /snapshot
- **Type**: std_srvs/srv/Trigger
- **Function**: Get snapshots from both cameras
- **Return**: JSON format response containing base64-encoded images from both cameras

#### Test Command
```bash
ros2 service call /snapshot std_srvs/srv/Trigger
```

## Configuration
- High resolution camera: rtsp://admin:123456@192.168.1.100/stream0
- Low resolution camera: rtsp://admin:123456@192.168.1.101/stream0

## Dependencies
- rclpy
- opencv-python
- numpy
- ffmpeg/ffprobe

## Notes
1. Ensure both cameras have normal network connections
2. Source install/setup.bash before launching
3. ffmpeg and ffprobe tools must be installed
4. Current version is a basic implementation, main protection mechanisms focus on basic error handling and resource management

## Relationship to Original Implementation
- Based on design concepts from `scripts/cam.py`
- Maintains compatibility with original RTSPStream class
- Adapted to ROS2 service interface specifications
