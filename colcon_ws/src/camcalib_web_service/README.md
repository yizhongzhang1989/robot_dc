# Camera Calibration Web Service

ROS2 wrapper for the camera calibration toolkit web service.

## Overview

This package provides a ROS2 node that launches and manages the camera calibration toolkit web application.

## Usage

Launch the service:
```bash
ros2 launch camcalib_web_service camcalib_web_launch.py
```

Launch with custom port (e.g., port 8006):
```bash
ros2 launch camcalib_web_service camcalib_web_launch.py port:=8006
```

## Configuration

The service reads its configuration from `config/robot_config.yaml`:

```yaml
services:
  camcalib_web:
    host: "0.0.0.0"
    port: 8006
```

Launch arguments can override the configuration:
```bash
ros2 launch camcalib_web_service camcalib_web_launch.py port:=9999 host:=127.0.0.1
```

Default values (if not specified in config):
- Port: 8006
- Host: 0.0.0.0

## Dependencies

- rclpy
- common (workspace utility package)
- camera_calibration_toolkit at `scripts/ThirdParty/camera_calibration_toolkit/`
