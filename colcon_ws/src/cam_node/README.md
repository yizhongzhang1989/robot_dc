# Cam Node - ROS2 Camera Service Package

[![ROS2 Humble](https://img.shields.io/badge/ROS2-Humble-blue.svg)](https://docs.ros.org/en/humble/)
[![License: MIT](https://img.shields.io/badge/License-MIT-yellow.svg)](https://opensource.org/licenses/MIT)
[![Python 3.8+](https://img.shields.io/badge/python-3.8+-blue.svg)](https://www.python.org/downloads/)

## Overview

The `cam_node` package provides ROS2 camera services for RTSP streams with web-based visualization. It offers snapshot services and real-time video streaming with automatic error handling and reconnection.

## Features

- **RTSP Stream Support**: Connect to multiple camera streams
- **ROS2 Services**: Snapshot and camera control services
- **Web Interface**: Real-time streaming via embedded Flask server
- **Error Recovery**: Automatic reconnection and graceful degradation
- **Performance Tuning**: Configurable quality and streaming parameters

## Quick Start

### Installation

```bash
# Install dependencies
sudo apt install ffmpeg python3-opencv python3-flask

# Build package
cd /path/to/your/colcon_ws
colcon build --packages-select cam_node
source install/setup.bash
```

### Launch

#### Single Camera with Web Interface
```bash
ros2 launch cam_node cam_stream_web.py
```
Access at: `http://localhost:8080`

#### Dual Camera System
```bash
ros2 launch cam_node cam_launch.py
```
- Camera 100: `http://localhost:8010`
- Camera 101: `http://localhost:8011`

## Configuration

### Key Parameters

| Parameter | Default | Description |
|-----------|---------|-------------|
| `camera_name` | `cam100` | Camera identifier |
| `rtsp_url` | `rtsp://admin:123456@192.168.1.100/stream0` | RTSP stream URL |
| `camera_ip` | `192.168.1.100` | IP for connectivity monitoring |
| `stream_port` | `8010` | Web server port |
| `stream_fps` | `25` | Streaming frame rate |
| `jpeg_quality` | `75` | Image quality (1-100) |
| `max_width` | `480` | Maximum stream width |

### Example Configuration

```python
parameters=[{
    'camera_name': 'stream1',
    'rtsp_url': 'rtsp://admin:123456@192.168.1.100/stream0',
    'camera_ip': '192.168.1.100',
    'stream_port': 8080
}]
```

## Usage

### ROS2 Services

#### Take Snapshot
```bash
ros2 service call /stream1_snapshot std_srvs/srv/Trigger
```

#### Restart Camera
```bash
ros2 service call /restart_stream1_node std_srvs/srv/Trigger
```

### Web Interface

- **Live Stream**: `GET /`
- **Video Feed**: `GET /video_feed`
- **Status**: `GET /status`

## Package Structure

```
cam_node/
├── cam_node/
│   ├── __init__.py
│   └── cam_node.py          # Main ROS2 node
├── launch/
│   ├── cam_launch.py        # Dual camera setup
│   └── cam_stream_web.py    # Single camera with web
├── package.xml
├── setup.py
└── README.md
```

## Dependencies

- ROS2 Humble
- Python 3.8+
- ffmpeg
- opencv-python
- flask

## Troubleshooting

### Common Issues

**Stream Connection Failed**
```bash
ping 192.168.1.100
ffplay rtsp://admin:123456@192.168.1.100/stream0
```

**Service Not Found**
```bash
ros2 service list | grep snapshot
```

**Web Interface Not Loading**
```bash
netstat -tlnp | grep 8080
```

## Maintainer

- **Author**: yizhongzhang1989
- **Email**: yizhongzhang1989@gmail.com
- **Version**: 0.1.0
- **License**: MIT
- Graceful process cleanup and resource release
- Proper availability checking before operations

### 5. **Thread Safety**
- Use threading.Lock() to protect frame data access
- Daemon threads prevent blocking main process exit
- Viewer count tracking for streaming optimization

### 6. **Enhanced Resource Management**
- Automatic process termination with availability checks
- Thread synchronization and waiting
- Memory management (frame data copying)
- Clean shutdown handling

## Launch Commands

### Launch dual camera system (original)
```bash
cd /home/jetson/Documents/robot_dc/colcon_ws
source install/setup.bash
ros2 launch cam_node cam_launch.py
```

### Launch single camera with web visualization (new)
```bash
cd /home/jetson/Documents/robot_dc/colcon_ws
source install/setup.bash
ros2 launch cam_node cam_web_launch.py
```

### Launch cam_node independently
```bash
cd /home/jetson/Documents/robot_dc/colcon_ws
source install/setup.bash
ros2 launch cam_node cam_launch.py
```

## Web Interfaces

### Camera Stream Interface (cam_node)
- **URL**: http://localhost:8010 (for cam100) or http://localhost:8011 (for cam101)
- **Function**: Simple camera stream display from cam_node
- **Video Feed**: /video_feed endpoint
- **Status**: /status endpoint

### Camera Display Interface (cam_node_display)
- **URL**: http://localhost:8080 (default)
- **Function**: Full-featured web interface with controls
- **Features**: Live stream, snapshot, restart, status monitoring

## Service Interface

### /{camera_name}_snapshot
- **Type**: std_srvs/srv/Trigger
- **Function**: Get snapshot from the specified camera
- **Return**: JSON format response containing base64-encoded image and metadata

### /restart_{camera_name}_node
- **Type**: std_srvs/srv/Trigger  
- **Function**: Restart the camera node (useful for reconnection)
- **Return**: Success/failure status

#### Test Commands
```bash
# Take snapshot from cam100
ros2 service call /cam100_snapshot std_srvs/srv/Trigger

# Restart cam100 node
ros2 service call /restart_cam100_node std_srvs/srv/Trigger
```

## Configuration
- Default camera: rtsp://admin:123456@192.168.1.100/stream0
- Configurable via ROS2 parameters:
  - `camera_name`: Camera identifier (e.g., 'cam100', 'cam101')
  - `rtsp_url`: Full RTSP stream URL
  - `camera_ip`: IP address for connectivity monitoring
  - `stream_port`: Port for embedded Flask streaming server
  - `display_port`: Port for web visualization interface
  - `stream_fps`, `jpeg_quality`, `max_width`: Performance tuning parameters

## Dependencies
- rclpy
- opencv-python
- numpy
- ffmpeg/ffprobe
- flask
- requests

## Notes
1. Ensure cameras have normal network connections
2. Source install/setup.bash before launching
3. ffmpeg and ffprobe tools must be installed
4. Each camera node runs on a separate port for streaming
5. Stream availability is automatically detected and handled gracefully
6. Web interface provides real-time status and control capabilities

## Error Handling
- **Stream Unavailable**: Shows error image with status information
- **Network Issues**: Automatic ping-based connectivity monitoring
- **Process Failures**: Retry logic with graceful degradation
- **Service Failures**: Proper error reporting and recovery options

## Relationship to Original Implementation
- Based on design concepts from `scripts/cam.py`
- Maintains compatibility with original RTSPStream class
- Adapted to ROS2 service interface specifications
