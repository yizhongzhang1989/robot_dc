# DUCO GCR5-910 URDF Package

This package contains the URDF (Unified Robot Description Format) model for the DUCO GCR5-910 robot arm. It includes 3D meshes, joint configurations, and visualization tools for the 6-DOF robotic arm.

## Overview

The DUCO GCR5-910 is a 6-axis industrial robot arm with the following specifications:
- **6 Degrees of Freedom**: Six revolute joints for full spatial manipulation
- **Payload**: Industrial-grade payload capacity
- **Reach**: Extended workspace for various applications
- **Control**: Precise joint control with position feedback

## Package Contents

```
duco_gcr5_910_urdf/
├── urdf/
│   ├── duco_gcr5_910_urdf.urdf      # Main URDF file
│   └── duco_gcr5_910_urdf.csv       # Joint parameters
├── meshes/
│   ├── base_link.STL                # Base link mesh
│   ├── link1.STL                    # Joint 1 link mesh
│   ├── link2.STL                    # Joint 2 link mesh
│   ├── link3.STL                    # Joint 3 link mesh
│   ├── link4.STL                    # Joint 4 link mesh
│   ├── link5.STL                    # Joint 5 link mesh
│   └── link6.STL                    # Joint 6 link mesh
├── textures/                        # Texture files (if any)
├── config/                          # Configuration files
├── launch/                          # Launch files
├── package.xml                      # Package metadata
└── README.md                        # This file
```

## Robot Structure

The robot consists of the following links and joints:

### Links
- **base_link**: Fixed base of the robot
- **link1**: First rotational link (base rotation)
- **link2**: Second link (shoulder)
- **link3**: Third link (elbow)
- **link4**: Fourth link (wrist rotation)
- **link5**: Fifth link (wrist pitch)
- **link6**: Sixth link (end effector mount)

### Joints
- **joint1**: Base rotation (Z-axis)
- **joint2**: Shoulder pitch (Y-axis)
- **joint3**: Elbow pitch (Y-axis)
- **joint4**: Wrist roll (X-axis)
- **joint5**: Wrist pitch (Y-axis)
- **joint6**: End effector rotation (X-axis)

## Visualization

### Method 1: Web-based Visualization (Recommended)

Use the `urdf_web_viewer` package for interactive 3D visualization:

```bash
# Launch the web viewer
ros2 launch urdf_web_viewer urdf_web_viewer.launch.py

# Or with explicit URDF file path
ros2 launch urdf_web_viewer urdf_web_viewer.launch.py \
  urdf_file:=/path/to/your/workspace/src/duco_gcr5_910_urdf/urdf/duco_gcr5_910_urdf.urdf
```

Then open your web browser and navigate to:
```
http://localhost:8080
```

**Features:**
- Interactive 3D model with mouse controls
- Real-time joint state visualization
- STL mesh rendering with proper materials
- Responsive web interface
- Real-time updates from ROS2 joint state topics

### Method 2: RViz2 Visualization

For traditional ROS2 visualization:

```bash
# Launch RViz2 with URDF display
ros2 run rviz2 rviz2

# In RViz2:
# 1. Add -> Robot Model
# 2. Set Robot Description Topic to /robot_description
# 3. Publish the robot description:
ros2 run robot_state_publisher robot_state_publisher \
  --ros-args -p robot_description:="$(xacro /path/to/duco_gcr5_910_urdf.urdf)"
```

### Method 3: Joint State Publisher GUI

For manual joint control:

```bash
# Launch joint state publisher with GUI
ros2 run joint_state_publisher_gui joint_state_publisher_gui \
  --ros-args -p robot_description:="$(xacro /path/to/duco_gcr5_910_urdf.urdf)"

# In another terminal, launch RViz2
ros2 run rviz2 rviz2
```

## Usage Examples

### Basic Visualization

```bash
# Quick web-based visualization
ros2 launch urdf_web_viewer urdf_web_viewer.launch.py
```

### With Live Joint States

If you have a running robot system publishing joint states:

```bash
# Start the web viewer
ros2 launch urdf_web_viewer urdf_web_viewer.launch.py

# The viewer will automatically display live joint positions
# from the /joint_states topic
```

### Custom Port

```bash
# Run web viewer on a different port
ros2 launch urdf_web_viewer urdf_web_viewer.launch.py port:=8090
```

## Integration with Robot Control

This URDF package integrates with the robot control system:

### Joint State Subscription

The web viewer automatically subscribes to:
- `/joint_states` (sensor_msgs/JointState)

### Joint Names

The following joint names are used:
- `joint1` - Base rotation
- `joint2` - Shoulder pitch  
- `joint3` - Elbow pitch
- `joint4` - Wrist roll
- `joint5` - Wrist pitch
- `joint6` - End effector rotation

### Coordinate System

The robot uses a right-handed coordinate system:
- **X-axis**: Forward direction (red in visualization)
- **Y-axis**: Left direction (green in visualization)  
- **Z-axis**: Upward direction (blue in visualization)

## Technical Specifications

### Joint Limits

Each joint has defined position limits specified in the URDF file:
- Continuous joints: No limits (full 360° rotation)
- Limited joints: Specific angle ranges for safe operation

### Mesh Files

- **Format**: STL (STereoLithography)
- **Units**: Meters
- **Coordinate System**: Right-handed, Z-up
- **Optimization**: Meshes are optimized for web rendering

### URDF Features

- **Collision Geometry**: Simplified collision meshes
- **Visual Geometry**: Detailed visual meshes
- **Inertial Properties**: Mass and inertia specifications
- **Joint Dynamics**: Damping and friction parameters

## Development

### Modifying the Model

1. **Edit URDF**: Modify `urdf/duco_gcr5_910_urdf.urdf`
2. **Update Meshes**: Replace STL files in `meshes/` directory
3. **Test Changes**: Use web viewer to verify modifications

### Adding New Links

1. Create new STL mesh file
2. Add link definition to URDF
3. Define joint connecting new link
4. Update joint limits and properties

### Mesh Optimization

For better web performance:
- Keep triangle count reasonable (< 10k per mesh)
- Use appropriate mesh resolution
- Consider mesh compression

## Troubleshooting

### Common Issues

1. **Mesh Not Loading**
   - Check file paths in URDF are correct
   - Verify STL files exist in meshes/ directory
   - Ensure proper file permissions

2. **Joint Not Moving**
   - Verify joint names match those in joint_states messages
   - Check joint limits are not exceeded
   - Confirm joint types are correct

3. **Web Viewer Not Loading**
   - Check port is not in use
   - Verify URDF file path is correct
   - Ensure all mesh files are accessible

### Debug Commands

```bash
# Check URDF syntax
check_urdf /path/to/duco_gcr5_910_urdf.urdf

# Visualize URDF tree
urdf_to_graphiz /path/to/duco_gcr5_910_urdf.urdf
```

## Dependencies

### Required Packages

- `urdf_web_viewer` (for web-based visualization)
- `robot_state_publisher` (for ROS2 integration)
- `joint_state_publisher` (for manual control)

### Optional Packages

- `rviz2` (for traditional ROS2 visualization)
- `joint_state_publisher_gui` (for GUI-based joint control)

## License

This package is licensed under the terms specified in package.xml.

## Contributing

When contributing to this package:

1. **Test Visualization**: Ensure web viewer works correctly
2. **Verify Joint Names**: Match control system expectations
3. **Optimize Meshes**: Keep file sizes reasonable
4. **Update Documentation**: Reflect any changes made

## Support

For issues with the URDF model:

1. **Check Syntax**: Use `check_urdf` command
2. **Test Visualization**: Use web viewer for quick testing
3. **Verify Meshes**: Ensure all STL files load correctly
4. **Check Joint States**: Confirm joint names and limits

## Integration Examples

### With Robot Control System

```bash
# Launch robot control system
ros2 launch robot_bringup robot_bringup.launch.py

# Launch web viewer for visualization
ros2 launch urdf_web_viewer urdf_web_viewer.launch.py

# Now you can see live robot motion in the web browser
```

### With Simulation

```bash
# Launch gazebo simulation (if available)
ros2 launch robot_gazebo robot_gazebo.launch.py

# Launch web viewer
ros2 launch urdf_web_viewer urdf_web_viewer.launch.py

# Control robot in simulation, view in web browser
```

## Version History

### Version 1.0.0
- Initial URDF model
- Complete STL mesh set
- Web viewer compatibility
- ROS2 integration support
