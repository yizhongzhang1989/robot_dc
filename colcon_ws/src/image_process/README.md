# Image Process Node

This ROS2 node performs camera image processing with multiple output formats and conditional processing.

## Features

- **Conditional Processing**: Only processes outputs that are actually needed
- **Multiple Output Formats**: Uncompressed, compressed (JPEG), and resized variants
- **Smart Resizing**: Automatic aspect ratio preservation or custom dimensions
- **Camera Undistortion**: Applies lens distortion correction using calibration parameters
- **Performance Optimized**: Uses BEST_EFFORT QoS and pre-computed undistortion maps
- **Priority Processing**: Resized original images processed first for lowest latency

## Topics

### Subscribed Topics
- `input_topic` (sensor_msgs/Image): Raw camera images

### Published Topics (All Optional)
- `output_resized_topic` (sensor_msgs/Image): **Resized original images** (no undistortion)
- `output_topic` (sensor_msgs/Image): Full-size undistorted images
- `output_compressed_topic` (sensor_msgs/CompressedImage): Full-size undistorted JPEG images
- `output_resized_compressed_topic` (sensor_msgs/CompressedImage): Resized undistorted JPEG images
- `camera_info_*` (sensor_msgs/CameraInfo): Original camera calibration info (when `output_topic` enabled)

## Parameters

### Topics
- `input_topic` (string, default: `/camera/image_raw`): Input image topic
- `output_resized_topic` (string, default: `""` - disabled): Resized original image topic
- `output_topic` (string, default: `""` - disabled): Full-size undistorted image topic  
- `output_compressed_topic` (string, default: `""` - disabled): Compressed undistorted image topic
- `output_resized_compressed_topic` (string, default: `""` - disabled): Resized compressed undistorted image topic

### Processing Options
- `resize_width` (int, default: `640`): Target width for resized images
- `resize_height` (int, default: `0`): Target height (0 = keep aspect ratio)
- `jpeg_quality` (int, default: `85`): JPEG compression quality (1-100)
- `calibration_file` (string): Path to camera calibration JSON file

## Usage Examples

### 1. Only Resized Original Images (Fastest)
```bash
ros2 run image_process image_process_node --ros-args \
  -p input_topic:=/my_camera/image_raw \
  -p output_resized_topic:=/my_camera/image_640_raw \
  -p resize_width:=640
# resize_height=0 by default (keeps aspect ratio)
# No calibration needed, no undistortion processing
```

### 2. Multiple Output Formats
```bash
ros2 run image_process image_process_node --ros-args \
  -p input_topic:=/camera/image_raw \
  -p output_resized_topic:=/camera/image_small \
  -p output_topic:=/camera/image_undistorted \
  -p output_compressed_topic:=/camera/image_compressed \
  -p resize_width:=320 \
  -p resize_height:=240
```

### 3. Custom Aspect Ratio vs Fixed Dimensions
```bash
# Keep aspect ratio (recommended)
ros2 run image_process image_process_node --ros-args \
  -p output_resized_topic:=/camera/image_640 \
  -p resize_width:=640 \
  -p resize_height:=0

# Force specific dimensions (may distort image)
ros2 run image_process image_process_node --ros-args \
  -p output_resized_topic:=/camera/image_640x480 \
  -p resize_width:=640 \
  -p resize_height:=480
```

### 4. Build and Run
```bash
cd /path/to/colcon_ws
colcon build --packages-select image_process
source install/setup.bash

# Run with your specific configuration
ros2 run image_process image_process_node --ros-args -p [your_parameters]
```

## Processing Flow

1. **Convert** ROS Image → OpenCV image
2. **Process resized original** images FIRST (highest priority, lowest latency)
3. **Apply undistortion** (only if undistorted outputs are enabled)
4. **Publish outputs** in order of complexity

## Performance Benefits

- ✅ **Conditional Processing**: Only enabled outputs are computed
- ✅ **Priority Order**: Fastest operations (resize original) processed first  
- ✅ **No Unnecessary Undistortion**: Skipped when only original images needed
- ✅ **Efficient Memory**: BEST_EFFORT QoS prevents buffer buildup
- ✅ **Pre-computed Maps**: Undistortion maps calculated once per image size

## Output Types Comparison

| Output Type | Source | Undistorted | Compressed | Resized | Use Case |
|-------------|--------|-------------|------------|---------|----------|
| `output_resized_topic` | Original | ❌ | ❌ | ✅ | Fast computer vision, low bandwidth |
| `output_topic` | Original | ✅ | ❌ | ❌ | Web viewers, precise applications |
| `output_compressed_topic` | Original | ✅ | ✅ | ❌ | Network transmission, storage |
| `output_resized_compressed_topic` | Original | ✅ | ✅ | ✅ | Mobile apps, low bandwidth |

## Calibration File Format

The calibration file should be in JSON format:
```json
{
  "success": true,
  "rms_error": 0.23457141766886752,
  "camera_matrix": [
    [1790.389259551211, 0.0, 953.5898505730135],
    [0.0, 1789.554648753286, 549.3115267217438],
    [0.0, 0.0, 1.0]
  ],
  "distortion_coefficients": [-0.40617142193712, 0.2741874374871549, 0.00022715645816590962, 0.0004245559597442624, -0.19016465031541452],
  "image_count": 15
}
```

**Note**: Calibration file is only required when undistorted outputs are enabled.

## Dependencies

- rclpy
- sensor_msgs  
- cv_bridge
- opencv-python
- numpy

## QoS Configuration

- **Video QoS**: BEST_EFFORT, VOLATILE, KEEP_LAST (depth=1)
- **Camera Info QoS**: Default RELIABLE (depth=10)

This ensures real-time performance without blocking when subscribers are slow.
