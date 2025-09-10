# Image Process Node

This ROS2 node performs image undistortion using camera calibration parameters.

## Features

- Subscribes to raw camera images
- Applies undistortion using camera calibration parameters
- Publishes undistorted images
- Configurable input/output topics
- Efficient undistortion using pre-computed remap tables

## Topics

### Subscribed Topics
- `/camera/image_raw` (sensor_msgs/Image): Raw camera images

### Published Topics  
- `/camera/image_undistorted` (sensor_msgs/Image): Undistorted camera images

## Parameters

- `input_topic` (string, default: `/camera/image_raw`): Input image topic
- `output_topic` (string, default: `/camera/image_undistorted`): Output image topic  
- `calibration_file` (string, default: `/home/a/Documents/robot_dc2/temp/calibration_result.json`): Path to calibration file

## Usage

### Build the package
```bash
cd /home/a/Documents/robot_dc2/colcon_ws
colcon build --packages-select image_process
source install/setup.bash
```

### Run the node
```bash
# Using launch file
ros2 launch image_process image_process_launch.py

# Or run directly
ros2 run image_process image_process_node

# With custom parameters
ros2 launch image_process image_process_launch.py input_topic:=/my_camera/image_raw output_topic:=/my_camera/image_corrected
```

### Check topics
```bash
# List available topics
ros2 topic list

# View undistorted images
ros2 run rqt_image_view rqt_image_view /camera/image_undistorted
```

## Calibration File Format

The calibration file should be in JSON format with the following structure:
```json
{
  "success": true,
  "rms_error": 0.23457141766886752,
  "camera_matrix": [
    [1790.389259551211, 0.0, 953.5898505730135],
    [0.0, 1789.554648753286, 549.3115267217438],
    [0.0, 0.0, 1.0]
  ],
  "distortion_coefficients": [-0.40617142193712, 0.2741874374871549, 0.00022715645816590962, 0.0004245559597442624, -0.19016465031541452]
}
```

## Dependencies

- rclpy
- sensor_msgs
- cv_bridge  
- opencv-python
- numpy
