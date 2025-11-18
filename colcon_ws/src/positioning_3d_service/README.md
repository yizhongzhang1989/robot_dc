# 3D Positioning Service - ROS2 Package

ROS2 wrapper for the 3D positioning triangulation service that uses FlowFormer++ for multi-view keypoint tracking.

## Overview

This package provides a ROS2 node that launches and manages the positioning_3d web service. The service enables multi-robot triangulation by:

1. Tracking keypoints across multiple camera views using FlowFormer++
2. Managing detection sessions from multiple robots
3. Performing 3D triangulation from 2D keypoint correspondences
4. Providing a web dashboard for monitoring and visualization

## Prerequisites

1. **Robot Vision Submodule**: The web service is located in the `robot_vision` submodule
   ```bash
   cd ~/Documents/robot_dc/scripts/ThirdParty
   git submodule update --init --recursive
   ```

2. **FlowFormer++ Server**: Must be running and accessible
   - Default: `http://msraig-ubuntu-4:8001`
   - Or specify custom URL when launching

## Installation

```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select positioning_3d_service
source install/setup.bash
```

## Usage

### Basic Launch (with defaults)

```bash
ros2 launch positioning_3d_service positioning_3d_launch.py
```

Default parameters:
- FFPP URL: `http://msraig-ubuntu-4:8001`
- Dataset Path: `./dataset`
- Port: `8004`
- Host: `0.0.0.0`

### Launch with Custom Parameters

```bash
# Custom FFPP server URL
ros2 launch positioning_3d_service positioning_3d_launch.py \
    ffpp_url:=http://192.168.1.100:8001

# Custom dataset path
ros2 launch positioning_3d_service positioning_3d_launch.py \
    dataset_path:=/home/robot/Documents/robot_dc/dataset

# Custom port
ros2 launch positioning_3d_service positioning_3d_launch.py \
    port:=9000

# All custom parameters
ros2 launch positioning_3d_service positioning_3d_launch.py \
    ffpp_url:=http://192.168.1.100:8001 \
    dataset_path:=/path/to/dataset \
    port:=9000 \
    host:=0.0.0.0
```

## Web Interface

Once launched, access the web dashboard at:
```
http://localhost:8004
```

Or from another machine (replace with actual IP):
```
http://<robot-ip>:8004
```

## API Endpoints

The service provides REST API endpoints:

- `POST /detect` - Submit image for keypoint detection
- `GET /triangulate/{session_id}` - Get triangulation results
- `GET /sessions` - List active sessions
- `GET /status` - Service status

See the web interface documentation for detailed API usage.

## Architecture

```
positioning_3d_service (ROS2 Node)
    │
    └──> Launches positioning_3d Flask App
            │
            ├──> FFPP Client (keypoint tracking)
            ├──> Session Manager (multi-robot)
            ├──> Task Queue (serialized processing)
            ├──> Triangulation Engine
            └──> Web Dashboard
```

## Troubleshooting

### Service won't start
- Check that the robot_vision submodule is initialized
- Verify FFPP server is accessible: `curl http://msraig-ubuntu-4:8001/health`
- Check port 8004 is not already in use: `lsof -i :8004`

### Can't access web interface
- Verify firewall allows port 8004: `sudo ufw allow 8004`
- Check service is running: `ros2 node list`
- View logs: Check terminal output

### FFPP connection issues
- Ping the FFPP server: `ping msraig-ubuntu-4`
- Try with IP address instead of hostname
- Check FFPP service logs on the server

## Development

Package structure:
```
positioning_3d_service/
├── launch/
│   └── positioning_3d_launch.py          # Launch file
├── positioning_3d_service/
│   ├── __init__.py
│   └── positioning_3d_node.py            # ROS2 wrapper node
├── package.xml
├── setup.py
├── setup.cfg
└── README.md
```

The actual web service code is in:
```
scripts/ThirdParty/robot_vision/web/positioning_3d/
```

## Related Packages

- `ur15_web` - UR15 robot web interface
- `robot_arm_web` - DUCO robot arm web interface
- `camera_node` - Camera streaming node

## License

MIT

## Author

Yizhong Zhang (yizhongzhang1989@gmail.com)
