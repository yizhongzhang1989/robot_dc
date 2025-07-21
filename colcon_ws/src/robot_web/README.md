# Robot Web Package

## Overview
This package provides a web server interface for interacting with ROS2 robot systems through HTTP API, including camera snapshots, motor control, and other functions.

## Features
- FastAPI Web server
- Camera snapshot HTTP interface
- Motor status monitoring and control
- ROS2 service bridging

## Launch Commands

### Launch robot_web independently
```bash
cd /home/jetson/Desktop/robot_dc/colcon_ws
source install/setup.bash
ros2 launch robot_web web_server_launch.py
```

### Launch camera and web services together (recommended)
```bash
cd /home/jetson/Desktop/robot_dc/colcon_ws
source install/setup.bash
ros2 launch cam_node camera_web_launch.py
```

### Launch web server directly
```bash
cd /home/jetson/Desktop/robot_dc/colcon_ws
source install/setup.bash
uvicorn robot_web.web_server:app --host 0.0.0.0 --port 8000
```

## API Interfaces

### Camera Snapshot
- **URL**: `POST /snapshot`
- **Function**: Get snapshots from all cameras (camera_100 and camera_101)
- **Return**: JSON format containing base64-encoded images from both cameras

- **URL**: `POST /snapshot/{camera_name}`
- **Function**: Get snapshot from specific camera
- **Parameters**: camera_name (camera_100 or camera_101)
- **Return**: JSON format containing base64-encoded image from specified camera

#### Test Commands
```bash
# Get snapshots from all cameras
curl -X POST http://localhost:8000/snapshot

# Get snapshot from specific camera
curl -X POST http://localhost:8000/snapshot/camera_100
curl -X POST http://localhost:8000/snapshot/camera_101
```

### Camera Restart
- **URL**: `POST /restart_camera`
- **Function**: Restart all camera nodes
- **Return**: JSON format with restart status for all cameras

- **URL**: `POST /restart_camera/{camera_name}`
- **Function**: Restart specific camera node
- **Parameters**: camera_name (camera_100 or camera_101)
- **Return**: JSON format with restart status for specified camera

### Motor Control
- **URL**: `POST /api/{motor_id}/cmd`
- **Function**: Send motor command
- **Parameters**: motor_id (motor ID), command (command), value (value)

### Motor Status
- **URL**: `GET /api/{motor_id}/status`
- **Function**: Get motor status

### All Status
- **URL**: `GET /api/all_status`
- **Function**: Get all motor status

## Web Interface
- Access `http://localhost:8000` to view the web interface
- Supports motor control and status monitoring
- Camera snapshot functionality

## Configuration
- Server address: 0.0.0.0:8000
- Supported motors: motor1, motor2, motor17, motor18, platform
- Supported cameras: camera_100 (192.168.1.100), camera_101 (192.168.1.101)
- Camera services: 
  - camera_100_snapshot, restart_camera_100_node
  - camera_101_snapshot, restart_camera_101_node

## Dependencies
- fastapi
- uvicorn
- rclpy
- std_srvs

## Notes
1. Ensure ROS2 environment is properly configured
2. Ensure camera nodes are running (if using snapshot functionality):
   - cam_node_100 for camera_100 (192.168.1.100)
   - cam_node_101 for camera_101 (192.168.1.101)
3. Port 8000 must be available
4. Source install/setup.bash before launching
5. Camera nodes now use parameterized architecture - each camera runs as separate node
