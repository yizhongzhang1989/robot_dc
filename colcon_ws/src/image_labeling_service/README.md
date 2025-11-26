# Image Labeling Service

ROS2 launch configuration for the Image Labeling Web service.

## Overview

This package provides a ROS2 launch file that starts the Image Labeling web application. The web service allows users to upload and label images with keypoints, featuring sub-pixel precision, zoom/pan functionality, and JSON export.

**Note:** This package uses `ExecuteProcess` to directly launch the web service without a ROS node wrapper, ensuring clean startup/shutdown.

## Features

- **Web-based image labeling interface**
- **Sub-pixel precision keypoints**
- **Zoom and pan functionality**
- **Export labels as JSON**
- **Automatic startup and shutdown management**
- **Configurable host and port**

## Installation

### Prerequisites

Ensure the robot_vision submodule is initialized:

```bash
cd ~/Documents/robot_dc/scripts/ThirdParty
git submodule update --init --recursive
```

### Build

```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select image_labeling_service
source install/setup.bash
```

## Usage

### Launch with default settings (port 8002)

```bash
ros2 launch image_labeling_service image_labeling_launch.py
```

### Launch with custom port

```bash
ros2 launch image_labeling_service image_labeling_launch.py port:=8003
```

### Launch with custom host (allow external connections)

```bash
ros2 launch image_labeling_service image_labeling_launch.py host:=0.0.0.0
```

### Access the web interface

Once launched, open your browser and navigate to:

```
http://localhost:8002
```

Or from another machine on the network:

```
http://<robot-ip>:8002
```

## Parameters

- **port** (int, default: 8002): Port for the web service
- **host** (string, default: '0.0.0.0'): Host address for the web service

## Web Service Features

### Labeling Interface

- Upload images for labeling
- Add keypoints with sub-pixel precision
- Zoom in/out for detailed labeling
- Pan around the image
- Delete unwanted keypoints
- Export labels as JSON

### Keyboard Shortcuts

(Refer to the web interface for available shortcuts)

## Architecture

The launch file uses `ExecuteProcess` to directly run the `launch_server.py` script:
```
scripts/ThirdParty/robot_vision/ThirdParty/ImageLabelingWeb/launch_server.py
```

Process management:
1. Launch file locates the script using workspace utilities
2. Starts it with `--no-browser` flag (non-interactive)
3. Passes configured host and port parameters
4. Handles graceful shutdown with SIGTERM/SIGKILL on Ctrl+C

## Benefits of Direct Launch

- **Clean shutdown**: Proper signal handling ensures processes terminate cleanly
- **No ROS overhead**: Direct process execution without unnecessary node wrapper
- **Simple architecture**: Launch file only, no Python node code needed
- **Standard tooling**: Uses standard Flask server, easy to debug

## Troubleshooting

### Port already in use

If you see an error about port already in use:

```bash
ros2 launch image_labeling_service image_labeling_launch.py port:=8003
```

### Submodule not initialized

If you see "launch_server.py not found":

```bash
cd ~/Documents/robot_dc/scripts/ThirdParty
git submodule update --init --recursive
```

### Check if service is running

```bash
# Check if process is running
ps aux | grep launch_server.py

# Check if port is in use
sudo lsof -i :8002
```

### Stop service manually if needed

```bash
pkill -f "launch_server.py"
```

## License

MIT
