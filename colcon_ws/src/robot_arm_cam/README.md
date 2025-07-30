# Robot Arm Camera Node

A ROS2 node for robot arm camera RTSP streaming and control, providing both web interface and ROS2 integration.

## Features

- **RTSP Stream Support**: Connect to IP cameras via RTSP protocol
- **Multiple Resolutions**: Support for both 1080p and 360p streams
- **Web Interface**: Beautiful web UI for camera control and viewing
- **ROS2 Integration**: Publishes camera frames as ROS2 Image messages
- **Snapshot Service**: ROS2 service for taking and saving snapshots
- **Real-time Streaming**: Low-latency video streaming with adjustable quality
- **Error Handling**: Robust connection handling with automatic reconnection

## Dependencies

- ROS2 (tested with Humble)
- OpenCV (python3-opencv)
- NumPy (python3-numpy)
- Flask (python3-flask)
- cv_bridge
- FFmpeg (for RTSP stream processing)

## Installation

1. Clone this package into your ROS2 workspace:
```bash
cd ~/your_ros2_workspace/src
# This package should already be in your workspace
```

2. Install dependencies:
```bash
sudo apt update
sudo apt install python3-opencv python3-numpy python3-flask ffmpeg
```

3. Build the package:
```bash
cd ~/your_ros2_workspace
colcon build --packages-select robot_arm_cam
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch robot_arm_cam robot_arm_cam_launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch robot_arm_cam robot_arm_cam_launch.py \
    camera_name:=MyCamera \
    rtsp_url_1080p:=rtsp://admin:123456@192.168.1.100/stream0 \
    rtsp_url_360p:=rtsp://admin:123456@192.168.1.100/stream1 \
    server_port:=8012 \
    max_width:=640
```

### Direct Node Execution

```bash
ros2 run robot_arm_cam robot_arm_cam
```

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `camera_name` | string | "RobotArmCamera" | Name identifier for the camera |
| `rtsp_url_1080p` | string | "rtsp://admin:123456@192.168.1.102/stream0" | RTSP URL for high quality stream |
| `rtsp_url_360p` | string | "rtsp://admin:123456@192.168.1.102/stream1" | RTSP URL for low latency stream |
| `camera_ip` | string | "192.168.1.102" | IP address of the camera |
| `server_port` | int | 8011 | Port for Flask web server |
| `stream_fps` | int | 25 | Target FPS for video streaming |
| `jpeg_quality` | int | 75 | JPEG compression quality (1-100) |
| `max_width` | int | 800 | Maximum width for video streaming |
| `ros_publish_rate` | double | 10.0 | ROS2 image publishing rate (Hz) |

## Topics

### Published Topics

- `/robot_arm_camera/image_raw` (sensor_msgs/Image): Raw camera images

### Services

- `/robotarmcamera/take_snapshot` (std_srvs/Trigger): Take and save a snapshot

## Web Interface

The node provides a web interface accessible at `http://localhost:{server_port}` (default: http://localhost:8011).

### Features:
- **Live Video Stream**: Real-time camera feed
- **Camera Controls**: Start/stop camera streaming
- **Resolution Switching**: Toggle between 1080p and 360p
- **Snapshot Capture**: Take and download snapshots
- **Status Monitoring**: Real-time connection status
- **System Information**: Node and topic details

## API Endpoints

- `GET /` - Web interface
- `GET /video_feed` - MJPEG video stream
- `POST /start` - Start camera
- `POST /stop` - Stop camera
- `GET /status` - Get camera status
- `POST /change_resolution` - Change stream resolution
- `POST /snapshot` - Take snapshot
- `GET /download_snapshot` - Download latest snapshot

## File Structure

```
robot_arm_cam/
├── robot_arm_cam/
│   ├── __init__.py
│   └── robot_arm_cam.py       # Main node implementation
├── launch/
│   └── robot_arm_cam_launch.py # Launch file
├── resource/
│   └── robot_arm_cam          # Package marker
├── package.xml                # Package manifest
├── setup.py                   # Python package setup
├── setup.cfg                  # Setup configuration
└── README.md                  # This file
```

## Examples

### Taking a Snapshot via ROS2 Service

```bash
ros2 service call /robotarmcamera/take_snapshot std_srvs/srv/Trigger
```

### Monitoring ROS2 Image Topic

```bash
ros2 topic echo /robot_arm_camera/image_raw
```

### Checking Node Status

```bash
ros2 node info /robot_arm_cam
```

## Troubleshooting

### Camera Connection Issues

1. **Check RTSP URL**: Ensure the camera is accessible
2. **Network Connectivity**: Verify camera IP is reachable
3. **FFmpeg Installation**: Ensure FFmpeg is properly installed
4. **Firewall Settings**: Check firewall rules for RTSP ports

### Web Interface Issues

1. **Port Conflicts**: Try a different server port
2. **Browser Cache**: Clear browser cache and refresh
3. **Network Access**: Ensure the port is not blocked

### Performance Issues

1. **Reduce Resolution**: Use 360p for better performance
2. **Lower Quality**: Reduce JPEG quality setting
3. **Adjust FPS**: Lower target FPS for smoother streaming
4. **Network Bandwidth**: Check available network bandwidth

## License

MIT License

## Author

Robot Arm Camera Node - Integrated RTSP streaming and ROS2 camera functionality
