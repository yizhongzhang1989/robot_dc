# Image Streaming Package

A ROS 2 package for streaming image topics to a web interface with MJPEG streaming.

## Features

- Subscribe to any ROS 2 image topic
- Serve video stream via HTTP/MJPEG
- Built-in web interface for viewing the stream
- Configurable JPEG quality and port
- Real-time streaming with minimal latency

## Installation

Build the package in your ROS 2 workspace:

```bash
cd ~/colcon_ws
colcon build --packages-select image_streaming
source install/setup.bash
```

## Usage

### Basic Usage

Stream an image topic with default settings (port 8080):

```bash
ros2 run image_streaming image_streamer --ros-args -p image_topic:=/camera/image_raw
```

### Using Launch File

```bash
ros2 launch image_streaming image_streamer.launch.py image_topic:=/camera1/camera1/color/image_raw port:=8080 quality:=85
```

### Parameters

- `image_topic` (string, default: `/camera/image_raw`): The ROS 2 image topic to subscribe to
- `port` (int, default: `8080`): HTTP server port
- `quality` (int, default: `85`): JPEG compression quality (0-100)

## Viewing the Stream

Once the node is running, open a web browser and navigate to:

```
http://<robot-ip>:<port>/
```

For example: `http://192.168.12.157:8080/`

### Available Endpoints

- `/` - Main web interface with embedded video player
- `/stream` - MJPEG stream endpoint
- `/image.jpg` - Single frame snapshot

## Examples

### Stream Camera 1 on Port 8080

```bash
ros2 run image_streaming image_streamer --ros-args \
  -p image_topic:=/camera1/camera1/color/image_raw \
  -p port:=8080 \
  -p quality:=90
```

### Stream Camera 2 on Port 8081

```bash
ros2 run image_streaming image_streamer --ros-args \
  -p image_topic:=/camera2/camera2/color/image_raw \
  -p port:=8081 \
  -p quality:=85
```

### Stream Multiple Cameras

You can run multiple instances with different ports:

```bash
# Terminal 1 - Camera 1
ros2 run image_streaming image_streamer --ros-args \
  -p image_topic:=/camera1/camera1/color/image_raw \
  -p port:=8080

# Terminal 2 - Camera 2  
ros2 run image_streaming image_streamer --ros-args \
  -p image_topic:=/camera2/camera2/color/image_raw \
  -p port:=8081
```

## Dependencies

- `rclpy` - ROS 2 Python client library
- `sensor_msgs` - ROS 2 sensor messages
- `cv_bridge` - ROS-OpenCV bridge
- `opencv-python` - OpenCV for image processing

## License

Apache-2.0
