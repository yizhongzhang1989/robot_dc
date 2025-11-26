# Camera Calibration Web Service

ROS2 launch configuration for the camera calibration toolkit web service.

## Overview

This package provides a ROS2 launch file that starts the camera calibration toolkit web application using `ExecuteProcess` for direct Flask execution without a node wrapper.

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

## Architecture

The launch file uses `ExecuteProcess` to directly run Flask:
```bash
flask --app scripts/ThirdParty/camera_calibration_toolkit/web/app.py run --host <host> --port <port>
```

Benefits:
- **Clean shutdown**: Proper signal handling (SIGTERM → SIGKILL)
- **No ROS overhead**: Direct Flask execution
- **Simple**: Launch file only, no node wrapper needed

## Dependencies

- common (workspace utility package for path resolution)
- Flask (for running the web application)
- camera_calibration_toolkit at `scripts/ThirdParty/camera_calibration_toolkit/`

## Troubleshooting

Check if service is running:
```bash
ps aux | grep flask | grep camera_calibration
sudo lsof -i :8006
```

Stop manually if needed:
```bash
pkill -f "flask.*camera_calibration"
```
