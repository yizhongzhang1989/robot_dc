# Camera Node

A generic ROS2 node for IP camera RTSP streaming and control.

## Features

- RTSP stream processing with FFmpeg
- Web-based camera control interface
- ROS2 Image topic publishing
- Camera snapshot service
- Multi-resolution support (main/sub streams)
- Real-time performance tuning
- Event-driven image publishing

## Dependencies

- ROS2 (tested with Humble)
- Python packages:
  - rclpy
  - sensor_msgs
  - std_srvs
  - cv_bridge
  - opencv-python
  - numpy
  - flask
- System packages:
  - ffmpeg
  - ffprobe

## Installation

1. Clone this package to your ROS2 workspace:
```bash
cd ~/your_workspace/src
# Package should already be here
```

2. Install dependencies:
```bash
sudo apt update
sudo apt install ffmpeg
pip3 install flask opencv-python numpy
```

3. Build the package:
```bash
cd ~/your_workspace
colcon build --packages-select camera_node
```

4. Source the workspace:
```bash
source install/setup.bash
```

## Usage

### Basic Launch

```bash
ros2 launch camera_node camera_launch.py
```

### Launch with Custom Parameters

```bash
ros2 launch camera_node camera_launch.py \
    camera_name:=MyCam \
    rtsp_url_main:=rtsp://admin:password@192.168.1.100/stream0 \
    rtsp_url_sub:=rtsp://admin:password@192.168.1.100/stream1 \
    camera_ip:=192.168.1.100 \
    server_port:=8010 \
    ros_topic_name:=/my_camera/image_raw
```

### Available Parameters

- `camera_name`: Camera identifier (default: 'GenericCamera')
- `rtsp_url_main`: Main stream URL (high quality)
- `rtsp_url_sub`: Sub stream URL (low quality)  
- `camera_ip`: Camera IP address
- `server_port`: Web interface port (default: 8010)
- `stream_fps`: Target streaming FPS (default: 25)
- `jpeg_quality`: JPEG compression quality 1-100 (default: 75)
- `max_width`: Maximum streaming width in pixels (default: 800)
- `publish_ros_image`: Enable ROS2 image publishing (default: true)
- `ros_topic_name`: ROS2 image topic name (default: '/camera/image_raw')

## Web Interface

Access the web interface at: `http://localhost:8010` (or your configured port)

Features:
- Live camera preview
- Start/stop camera controls
- Stream quality selection
- Snapshot capture
- ROS2 publishing toggle

## ROS2 Topics and Services

### Published Topics

- `/{camera_name}/image_raw` (sensor_msgs/Image): Camera image stream

### Services

- `/{camera_name}/take_snapshot` (std_srvs/Trigger): Take a snapshot
- `/restart_{camera_name}_node` (std_srvs/Trigger): Restart the camera node

## Configuration Examples

### Multiple Camera Setup

You can run multiple camera nodes with different configurations:

```bash
# Camera 1
ros2 launch camera_node camera_launch.py \
    camera_name:=Camera1 \
    rtsp_url_main:=rtsp://admin:123456@192.168.1.100/stream0 \
    server_port:=8010 \
    ros_topic_name:=/camera1/image_raw

# Camera 2  
ros2 launch camera_node camera_launch.py \
    camera_name:=Camera2 \
    rtsp_url_main:=rtsp://admin:123456@192.168.1.101/stream0 \
    server_port:=8011 \
    ros_topic_name:=/camera2/image_raw
```

### Performance Tuning

For high-performance scenarios:

```bash
ros2 launch camera_node camera_launch.py \
    stream_fps:=30 \
    jpeg_quality:=85 \
    max_width:=1280 \
    publish_ros_image:=false  # Disable ROS2 publishing for web-only use
```

For low-bandwidth scenarios:

```bash
ros2 launch camera_node camera_launch.py \
    stream_fps:=15 \
    jpeg_quality:=50 \
    max_width:=640
```

## Troubleshooting

### Camera Connection Issues

1. Verify RTSP URL is accessible:
```bash
ffplay rtsp://admin:password@192.168.1.100/stream0
```

2. Check network connectivity:
```bash
ping 192.168.1.100
```

3. Verify camera credentials and stream paths

### Performance Issues

1. Reduce `stream_fps` and `jpeg_quality`
2. Decrease `max_width`
3. Use sub stream instead of main stream
4. Disable ROS2 publishing if not needed

### Build Issues

1. Ensure all dependencies are installed
2. Check ROS2 environment is sourced
3. Verify package.xml and setup.py are correct

## License

MIT License
