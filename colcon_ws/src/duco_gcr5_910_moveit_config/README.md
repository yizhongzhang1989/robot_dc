# DUCO GCR5-910 ROS2 Control & MoveIt2 Configuration

This package provides the official ROS2 Control implementation for the DUCO GCR5-910 6-DOF robotic arm with two control modes:
- **MoveIt2 Motion Planning**: Advanced trajectory planning with obstacle avoidance
- **Direct Cartesian Control**: Real-time Cartesian space control via command line

## Overview

This implementation follows the **official ROS2 Control architecture** with support for both joint space (MoveIt2) and Cartesian space control methods.

## Architecture (Simplified)

### ROS2 Control Stack
```
┌─────────────────────────────────────────────────────────────┐
│                     Control Methods                        │
│  ┌─────────────────┐              ┌─────────────────────┐   │
│  │ MoveIt2         │              │ Cartesian Control   │   │
│  │ - Planning      │              │ - Direct Control    │   │
│  │ - RViz2         │              │ - /target_frame     │   │
│  └─────────────────┘              └─────────────────────┘   │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                   ROS2 Control                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │
│  │ Controller      │  │ Joint/Cartesian │  │ Joint State │ │
│  │ Manager         │  │ Controllers     │  │ Broadcaster │ │
│  └─────────────────┘  └─────────────────┘  └─────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                Hardware Interface                          │
│  ┌─────────────────────────────────────────────────────────┐ │
│  │            DucoHardwareInterface                        │ │
│  │  ┌─────────────┐  ┌─────────────┐  ┌─────────────────┐  │ │
│  │  │ State       │  │ Command     │  │ DUCO RPC        │  │ │
│  │  │ Interfaces  │  │ Interfaces  │  │ Communication   │  │ │
│  │  └─────────────┘  └─────────────┘  └─────────────────┘  │ │
│  └─────────────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                   DUCO Robot                               │
│         (192.168.1.10:7003 - TCP/Thrift)                  │
└─────────────────────────────────────────────────────────────┘
```

## Package Structure

```
duco_gcr5_910_moveit_config/           # MoveIt Configuration Package
├── config/                            # Configuration files
│   ├── ros2_controllers.yaml          # Standard joint control config
│   ├── cartesian_controller_manager.yaml  # Cartesian control config
│   ├── joint_limits.yaml              # Joint limits and scaling
│   ├── gcr5_910.ros2_control.xacro     # Hardware interface config
│   └── gcr5_910.urdf.xacro            # Robot description
├── launch/                            # Launch files
│   ├── demo.launch.py                 # MoveIt2 demo with planning
│   └── cartesian_controller.launch.py # Direct Cartesian control
└── README.md                          # This documentation
```

### Related Packages
```
├── duco_hardware/                      # Hardware Interface Package
├── duco_ros_driver/                   # DUCO RPC Library
├── duco_gcr5_910_urdf/               # Robot URDF files
└── cartesian_controllers/             # Cartesian control packages
```

## Technical Details

### Controllers Available

#### 1. Joint Trajectory Controller (MoveIt2)
- **Type**: `joint_trajectory_controller/JointTrajectoryController`
- **Purpose**: Executes planned trajectories from MoveIt2
- **Update Rate**: 100Hz (configurable to 250Hz for 4ms control)
- **Interface**: Position control with trajectory interpolation

#### 2. Cartesian Motion Controller
- **Type**: `cartesian_motion_controller/CartesianMotionController`
- **Purpose**: Direct Cartesian space control
- **Input Topic**: `/target_frame` (geometry_msgs/PoseStamped)
- **Control**: Real-time end-effector positioning

#### 3. Joint State Broadcaster
- **Type**: `joint_state_broadcaster/JointStateBroadcaster`
- **Purpose**: Publishes joint states for visualization
- **Output Topic**: `/joint_states` (sensor_msgs/JointState)

### Hardware Interface
- **Plugin**: `duco_hardware/DucoHardwareInterface`
- **Communication**: TCP/Thrift protocol
- **Default Network**: 192.168.1.10:7003
- **Update Rate**: 100Hz (standard) / 250Hz (high-frequency mode)

### Network Configuration
- **Protocol**: TCP/Thrift
- **Default IP**: 192.168.1.10
- **Default Port**: 7003
- **Connection**: Ethernet

## Control Methods

### 1. MoveIt2 Motion Planning (Joint Space Control)

Launch the complete MoveIt2 system with planning capabilities:

```bash
# Navigate to workspace
cd /home/robot/robot_dc/colcon_ws
source install/setup.bash

# Launch MoveIt2 demo (default: 192.168.1.10:7003)
ros2 launch duco_gcr5_910_moveit_config demo.launch.py

# Launch with custom robot IP
ros2 launch duco_gcr5_910_moveit_config demo.launch.py robot_ip:=192.168.20.128
```

**Features:**
- Advanced trajectory planning with obstacle avoidance
- Multiple planning algorithms (OMPL, Pilz, CHOMP)
- RViz2 visualization and interactive planning
- Collision detection and safety monitoring

### 2. Direct Cartesian Control (Real-time Control)

Launch the Cartesian controller for direct end-effector control:

```bash
# Launch Cartesian controller
ros2 launch duco_gcr5_910_moveit_config cartesian_controller.launch.py robot_ip:=192.168.20.128
```

**Send target positions using command line:**

```bash
# Basic target position (x=0.3m, y=0.0m, z=0.4m, no rotation)
ros2 topic pub --once /target_frame geometry_msgs/msg/PoseStamped '{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: "base_link"
  },
  pose: {
    position: {x: 0.3, y: 0.0, z: 0.4},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'

# Move to different position (x=0.2m, y=0.1m, z=0.5m)
ros2 topic pub --once /target_frame geometry_msgs/msg/PoseStamped '{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: "base_link"
  },
  pose: {
    position: {x: 0.2, y: 0.1, z: 0.5},
    orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}
  }
}'

# Move with rotation (45 degrees around Z-axis)
ros2 topic pub --once /target_frame geometry_msgs/msg/PoseStamped '{
  header: {
    stamp: {sec: 0, nanosec: 0},
    frame_id: "base_link"
  },
  pose: {
    position: {x: 0.25, y: 0.0, z: 0.4},
    orientation: {x: 0.0, y: 0.0, z: 0.383, w: 0.924}
  }
}'
```

**Cartesian Control Features:**
- Real-time end-effector position control
- Direct command line interface
- No trajectory planning (immediate response)
- Suitable for manual positioning and testing

**Safety Notes:**
- Ensure robot workspace is clear before sending commands
- Start with small position changes to test system response
- Monitor robot movement and be ready to stop if needed
- Position coordinates are relative to robot base frame (`base_link`)

## Usage

### Available Launch Parameters
- `robot_ip`: Robot IP address (default: `192.168.1.10`)
- `robot_port`: Robot communication port (default: `7003`)
- `use_rviz`: Launch RViz2 visualization (default: `true`)
- `debug`: Enable debug mode (default: `false`)

## System Verification

### Check Active Controllers
```bash
# List all controllers
ros2 control list_controllers

# Monitor joint states
ros2 topic echo /joint_states

# Check Cartesian control topic
ros2 topic echo /target_frame
```

### Monitor Robot Status
```bash
# Check hardware interface status
ros2 control list_hardware_interfaces

# View controller manager status
ros2 service call /controller_manager/list_controllers controller_manager_msgs/srv/ListControllers
```

## Troubleshooting

### Connection Issues
```bash
# Check network connectivity
ping 192.168.20.128  # or your robot IP

# Verify robot response
ros2 topic echo /joint_states --once

# Check active topics
ros2 topic list | grep -E "(joint_states|target_frame)"
```

### Controller Issues
```bash
# List active controllers
ros2 control list_controllers

# Restart specific controller
ros2 control switch_controllers --stop cartesian_motion_controller
ros2 control switch_controllers --start cartesian_motion_controller

# Check controller status
ros2 control list_controllers | grep cartesian
```

### Cartesian Control Issues
```bash
# Check if target_frame topic exists
ros2 topic info /target_frame

# Monitor commands being sent
ros2 topic echo /target_frame

# Test with a simple position
ros2 topic pub --once /target_frame geometry_msgs/msg/PoseStamped '{
  header: {frame_id: "base_link"},
  pose: {
    position: {x: 0.3, y: 0.0, z: 0.4},
    orientation: {w: 1.0}
  }
}'
```

## Quick Start Guide

### 1. For MoveIt2 Planning
```bash
cd /home/robot/robot_dc/colcon_ws
source install/setup.bash
ros2 launch duco_gcr5_910_moveit_config demo.launch.py robot_ip:=192.168.20.128
# Use RViz2 interface for interactive planning
```

### 2. For Direct Cartesian Control  
```bash
cd /home/robot/robot_dc/colcon_ws
source install/setup.bash
ros2 launch duco_gcr5_910_moveit_config cartesian_controller.launch.py robot_ip:=192.168.20.128
# Use command line to send target positions
```
