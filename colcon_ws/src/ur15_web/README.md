# UR15 Web Package

ROS2 package for UR15 robot web interface with camera integration and control capabilities.

## Overview

This package provides a web-based interface for controlling and monitoring the UR15 robot, including:
- Robot control via web interface
- Real-time camera streaming
- Dataset collection and management
- Camera calibration tools
- Robot state monitoring

## Prerequisites

- ROS2 Humble
- UR robot driver (`ur_robot_driver` package)
- Camera node package
- Python 3.10+

## Complete System Bringup (Beijing Lab Configuration)

**`ur15_beijing_bringup.py`** - Launches the complete UR15 system with Beijing Lab's specific hardware setup

```bash
ros2 launch ur15_web ur15_beijing_bringup.py
```

This sequentially starts:
1. UR15 robot driver (ur_control) - immediately
2. UR15 camera node - after 5 seconds
3. UR15 web interface - after 8 seconds

**Note:** This is optimized for Beijing Lab's hardware configuration. Other sites should use individual launch files to match their specific setup.

**Parameters:**
```bash
ros2 launch ur15_web ur15_beijing_bringup.py \
  ur15_ip:=192.168.1.15 \
  camera_topic:=/ur15_camera/image_raw \
  rtsp_url:=rtsp://admin:123456@192.168.1.101/stream0 \
  launch_rviz:=false
```

**Available Arguments:**
- `ur15_ip` - UR15 robot IP address (default: `192.168.1.15`)
- `camera_topic` - Camera topic name (default: `/ur15_camera/image_raw`)
- `rtsp_url` - RTSP camera stream URL (default: `rtsp://admin:123456@192.168.1.101/stream0`)
- `dataset_dir` - Dataset storage directory (default: `../dataset`)
- `calib_data_dir` - Calibration data directory (default: `../temp/ur15_cam_calibration_data`)
- `chessboard_config` - Chessboard config file path (default: `../temp/ur15_cam_calibration_data/chessboard_config.json`)
- `launch_rviz` - Launch RViz visualization (default: `false`)

## Individual Component Launches (For Different Hardware Setups)

Different labs have different hardware configurations - different IPs, camera setups, or external control systems. Use individual launch files to adapt to your specific setup without code changes.

### **1. Robot Control Only**

**`ur15_control_launch.py`** - Launches only the UR15 robot driver

```bash
ros2 launch ur15_web ur15_control_launch.py
```

```bash
ros2 launch ur15_web ur15_control_launch.py \
  ur15_ip:=192.168.1.15 \
  ur_type:=ur15 \
  launch_rviz:=false
```

Available Arguments:
- `ur15_ip` - UR15 robot IP address (default: `192.168.1.15`)
- `ur_type` - UR robot type (default: `ur15`)
- `launch_rviz` - Launch RViz visualization (default: `false`)

### **2. Camera Only**

**`ur15_cam_launch.py`** - Launches only the UR15 camera node (from camera_node package). This camera is used in Beijing Lab only.

```bash
ros2 launch camera_node ur15_cam_launch.py
```

```bash
ros2 launch camera_node ur15_cam_launch.py \
  rtsp_url_main:=rtsp://admin:123456@192.168.1.101/stream0 \
  ros_topic_name:=/ur15_camera/image_raw \
  server_port:=8019
```

Available Arguments:
- `ros_topic_name` - ROS topic name (default: `/ur15_camera/image_raw`)
- `rtsp_url_main` - RTSP stream URL (default: `rtsp://admin:123456@192.168.1.101/stream0`)
- `camera_ip` - Camera IP address (default: `192.168.1.101`)
- `camera_name` - Camera name (default: `RobotArmCamera`)
- `server_port` - HTTP server port (default: `8012`)
- `stream_fps` - Stream FPS (default: `25`)
- `jpeg_quality` - JPEG quality 1-100 (default: `75`)
- `max_width` - Maximum image width (default: `800`)
- `publish_ros_image` - Publish to ROS topic (default: `true`)

### **3. Web Interface Only**

**`ur15_web_launch.py`** - Launches only the web interface node. (**Note:** This assumes ur_control and camera are already running.)

```bash
ros2 launch ur15_web ur15_web_launch.py
```

```bash
ros2 launch ur15_web ur15_web_launch.py \
  ur15_ip:=192.168.1.15 \
  camera_topic:=/ur15_camera/image_raw \
  web_port:=8030
```

Available Arguments:
- `ur15_ip` - UR15 robot IP address (default: `192.168.1.15`)
- `camera_topic` - Camera topic name (default: `/ur15_camera/image_raw`)
- `web_port` - Web server port (default: `8030`)
- `ur15_port` - UR15 robot port (default: `30002`)
- `dataset_dir` - Dataset storage directory
- `calib_data_dir` - Calibration data directory
- `chessboard_config` - Chessboard config file path


## Usage Examples

### Quick Start - Complete System

```bash
# Launch everything with defaults
ros2 launch ur15_web ur15_beijing_bringup.py

# Launch with custom robot IP
ros2 launch ur15_web ur15_beijing_bringup.py ur15_ip:=192.168.1.20

# Launch with RViz visualization
ros2 launch ur15_web ur15_beijing_bringup.py launch_rviz:=true
```

### Separate Component Launch

```bash
# Terminal 1: Launch robot control
ros2 launch ur15_web ur15_control_launch.py ur15_ip:=192.168.1.15

# Terminal 2: Launch camera (wait for robot to initialize)
ros2 launch camera_node ur15_cam_launch.py \
  camera_name:=UR15Camera \
  server_port:=8019

# Terminal 3: Launch web interface
ros2 launch ur15_web ur15_web_launch.py
```

## Web Interface

After launching, access the web interface at:

```
http://localhost:8030
```

Or from another machine:

```
http://<robot_ip>:8030
```

Replace `<robot_ip>` with the IP address of the machine running the web node.

## Camera Stream

The camera HTTP stream is available at:

```
http://localhost:8019
```

Or:

```
http://<robot_ip>:8019
```

## ROS Topics

### Published Topics
- `/ur15_camera/image_raw` - Camera image stream (sensor_msgs/Image)

### Subscribed Topics
- Depends on robot driver configuration

## Directory Structure

```
ur15_web/
├── launch/
│   ├── ur15_beijing_bringup.py           # Complete system bringup
│   ├── ur15_control_launch.py            # Robot control only
│   └── ur15_web_launch.py                # Web interface only
├── ur15_web/
│   └── ur15_web_node.py                  # Main web node
└── README.md

camera_node/
├── launch/
│   └── ur15_cam_launch.py                # Camera launch (used by ur15_web)
└── ...
```

## Troubleshooting

### Camera not connecting

Check the RTSP URL and camera IP:

```bash
# Test RTSP stream
ffplay rtsp://admin:123456@192.168.1.101/stream0
```

### Robot not responding

Verify robot IP and network connection:

```bash
ping 192.168.1.15
```

Check that the UR15 is powered on and in remote control mode.

### Web interface not accessible

Check if the web server is running:

```bash
ros2 node list | grep ur15_web_node
```

Verify port 8030 is not in use:

```bash
lsof -i:8030
```

### Camera or web node fails to start

Make sure to wait for previous components to initialize:
- Robot driver needs ~5 seconds to connect
- Camera needs ~3 seconds to establish RTSP stream

Use the complete bringup launch file for proper timing:

```bash
ros2 launch ur15_web ur15_beijing_bringup.py
```

## Configuration

### Dataset Directory

By default, datasets are stored in `../dataset` relative to the workspace.

To change:

```bash
ros2 launch ur15_web ur15_beijing_bringup.py \
  dataset_dir:=/path/to/your/dataset
```

### Calibration Data

Calibration data is stored in `../temp/ur15_cam_calibration_data` by default.

To change:

```bash
ros2 launch ur15_web ur15_beijing_bringup.py \
  calib_data_dir:=/path/to/calibration/data \
  chessboard_config:=/path/to/chessboard_config.json
```

## Network Configuration

Default IP addresses and ports:

| Component | IP/Host | Port |
|-----------|---------|------|
| UR15 Robot | 192.168.1.15 | 30002 |
| Camera | 192.168.1.101 | RTSP |
| Camera Stream | localhost | 8019 |
| Web Interface | localhost | 8030 |

Modify these in the launch file parameters as needed.

## Dependencies

```bash
# Install UR robot driver
sudo apt install ros-humble-ur-robot-driver

# Camera node (if not already installed)
# Build from source in your workspace
```

## Support

For issues or questions about this package, check:
1. Robot network connectivity
2. Camera RTSP stream accessibility
3. ROS2 node status: `ros2 node list`
4. ROS2 topic list: `ros2 topic list`
5. Node logs in `~/.ros/log/`

## License

See main repository license.
