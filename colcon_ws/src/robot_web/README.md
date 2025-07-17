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
- **Function**: Get camera snapshot
- **Return**: JSON format containing base64-encoded images from both cameras

#### Test Command
```bash
curl -X POST http://localhost:8000/snapshot
```

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

## Dependencies
- fastapi
- uvicorn
- rclpy
- std_srvs

## Notes
1. Ensure ROS2 environment is properly configured
2. Ensure camera node is running (if using snapshot functionality)
3. Port 8000 must be available
4. Source install/setup.bash before launching
