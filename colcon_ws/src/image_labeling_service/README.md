# Image Labeling Service

ROS2 wrapper package for the Image Labeling Web service.

## Overview

This package provides a ROS2 node that manages the lifecycle of the Image Labeling web application. The web service allows users to upload and label images with keypoints, featuring sub-pixel precision, zoom/pan functionality, and JSON export.

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

The node wraps the `launch_server.py` script located at:
```
scripts/ThirdParty/robot_vision/ThirdParty/ImageLabelingWeb/launch_server.py
```

The node:
1. Locates the launch_server.py script
2. Starts it as a subprocess with configured parameters
3. Monitors output for errors and warnings
4. Handles graceful shutdown on node termination

## Logging

The node filters output to show only critical messages:
- **Errors**: Displayed with ERROR level
- **Warnings**: Displayed with WARN level
- **HTTP requests**: Filtered out (not displayed)
- **Startup messages**: Displayed with INFO level

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
ros2 node list | grep image_labeling
```

### View logs

```bash
ros2 node info /image_labeling_service_node
```

## License

MIT
