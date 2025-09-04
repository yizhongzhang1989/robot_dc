# DUCO GCR5-910 ROS2 Control & MoveIt2 Configuration

This package provides the official ROS2 Control implementation for the DUCO GCR5-910 6-DOF robotic arm, fully integrated with MoveIt2 for motion planning and control.

## Overview

This implementation follows the **official ROS2 Control architecture**, replacing custom DUCO nodes with standardized hardware interfaces and controllers. The system provides seamless integration between DUCO robots and the ROS2 ecosystem.

## Architecture

### ROS2 Control Stack
```
┌─────────────────────────────────────────────────────────────┐
│                        MoveIt2                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │
│  │ Motion Planning │  │ Trajectory      │  │ RViz2       │ │
│  │ Framework       │  │ Execution       │  │ Interface   │ │
│  └─────────────────┘  └─────────────────┘  └─────────────┘ │
└─────────────────────────────────────────────────────────────┘
                              │
┌─────────────────────────────────────────────────────────────┐
│                   ROS2 Control                             │
│  ┌─────────────────┐  ┌─────────────────┐  ┌─────────────┐ │
│  │ Controller      │  │ Joint Trajectory│  │ Joint State │ │
│  │ Manager         │  │ Controller      │  │ Broadcaster │ │
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

### 1. Configuration Files

```
duco_gcr5_910_moveit_config/
├── config/
│   ├── gcr5_910.ros2_control.xacro      # ROS2 Control hardware definition
│   ├── ros2_controllers.yaml            # Controller configuration
│   ├── joint_limits.yaml               # Joint limits and safety parameters
│   ├── kinematics.yaml                 # Kinematics solver configuration
│   ├── ompl_planning.yaml              # OMPL planner configuration
│   ├── pilz_cartesian_limits.yaml      # Cartesian velocity/acceleration limits
│   ├── sensors_3d.yaml                 # 3D sensor configuration
│   └── moveit.rviz                     # RViz2 configuration
├── launch/
│   ├── demo.launch.py                  # Main launch file
│   ├── move_group.launch.py            # MoveGroup launch
│   ├── moveit_rviz.launch.py          # RViz2 launch
│   ├── rsp.launch.py                  # Robot State Publisher
│   ├── spawn_controllers.launch.py    # Controller spawning
│   ├── static_virtual_joint_tfs.launch.py  # Virtual joint TFs
│   └── warehouse_db.launch.py         # Database launch
├── srdf/
│   └── gcr5_910.srdf.xacro            # Semantic Robot Description
└── README.md
```

### 2. Related Packages

```
robot_dc/colcon_ws/src/
├── duco_hardware/                      # Hardware Interface Package
│   ├── include/duco_hardware/
│   │   └── duco_hardware_interface.hpp # Hardware interface header
│   ├── src/
│   │   └── duco_hardware_interface.cpp # Hardware interface implementation
│   ├── duco_hardware.xml              # Plugin description
│   ├── CMakeLists.txt                 # Build configuration
│   └── package.xml                    # Package manifest
├── duco_gcr5_910_urdf/                # Robot Description
│   └── urdf/
│       └── gcr5_910.urdf.xacro       # Robot URDF model
└── duco_ros_driver/                   # DUCO RPC Library
    ├── include/duco_ros_driver/
    │   └── DucoCobot.h               # DUCO RPC interface
    └── lib/
        └── libDucoCobotAPI.a         # Static library
```

## Core Components

### 1. Hardware Interface (`duco_hardware`)

**Purpose**: Official ROS2 Control hardware interface for DUCO robots

**Key Features**:
- Implements `hardware_interface::SystemInterface`
- Standard lifecycle management (`configure` → `activate` → `read/write` → `deactivate`)
- Real-time communication with DUCO robot via RPC
- 100Hz update rate for smooth control

**Interfaces**:
```cpp
State Interfaces (12 total):
- arm_1_joint_1/position, arm_1_joint_1/velocity
- arm_1_joint_2/position, arm_1_joint_2/velocity
- arm_1_joint_3/position, arm_1_joint_3/velocity
- arm_1_joint_4/position, arm_1_joint_4/velocity
- arm_1_joint_5/position, arm_1_joint_5/velocity
- arm_1_joint_6/position, arm_1_joint_6/velocity

Command Interfaces (6 total):
- arm_1_joint_1/position
- arm_1_joint_2/position
- arm_1_joint_3/position
- arm_1_joint_4/position
- arm_1_joint_5/position
- arm_1_joint_6/position
```

### 2. ROS2 Control Configuration

#### Hardware Configuration (`gcr5_910.ros2_control.xacro`)
```xml
<ros2_control name="FakeSystem" type="system">
  <hardware>
    <plugin>duco_hardware/DucoHardwareInterface</plugin>
    <param name="robot_ip">192.168.1.10</param>
    <param name="robot_port">7003</param>
  </hardware>
  <!-- Joint definitions with position/velocity interfaces -->
</ros2_control>
```

#### Controller Configuration (`ros2_controllers.yaml`)
```yaml
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz
    
    # Active Controllers
    arm_1_controller:
      type: joint_trajectory_controller/JointTrajectoryController
    joint_state_broadcaster:
      type: joint_state_broadcaster/JointStateBroadcaster
```

### 3. Controllers

#### Joint State Broadcaster
- **Package**: `joint_state_broadcaster`
- **Purpose**: Publishes robot joint states to `/joint_states` topic
- **Update Rate**: 100Hz
- **Topics Published**: `/joint_states` (sensor_msgs/JointState)

#### Joint Trajectory Controller
- **Package**: `joint_trajectory_controller`
- **Purpose**: Executes joint trajectory commands from MoveIt2
- **Action Server**: `/arm_1_controller/follow_joint_trajectory`
- **Command Interface**: Position control
- **Features**: 
  - Trajectory interpolation
  - Goal tolerance checking
  - Real-time trajectory execution

### 4. MoveIt2 Integration

#### Move Group Node
- **Package**: `moveit_ros_move_group`
- **Purpose**: Main MoveIt2 planning and execution node
- **Services**: Motion planning, trajectory execution, scene management
- **Action Servers**: `/move_action`, `/execute_trajectory`

#### Planning Pipelines
1. **OMPL** (Open Motion Planning Library)
   - Sampling-based motion planning
   - Multiple planning algorithms (RRT, PRM, etc.)

2. **Pilz Industrial Motion Planner**
   - Deterministic planning for industrial applications
   - LIN, PTP, CIRC motion types

3. **CHOMP** (Covariant Hamiltonian Optimization)
   - Trajectory optimization
   - Obstacle avoidance

## Usage

### 1. Launch the Complete System
```bash
# Navigate to workspace
cd /home/robot/robot_dc/colcon_ws

# Source the workspace
source install/setup.bash

# Launch the complete system
ros2 launch duco_gcr5_910_moveit_config demo.launch.py
```

### 2. Verify System Status
```bash
# Check active controllers
ros2 control list_controllers

# Monitor joint states
ros2 topic echo /joint_states

# Check hardware interface status
ros2 control list_hardware_interfaces
```

### 3. Programming Interface
```python
#!/usr/bin/env python3
import rclpy
from moveit2_python_interface import MoveGroupInterface

# Initialize MoveIt2 interface
move_group = MoveGroupInterface("arm_1")

# Plan and execute motion
move_group.go_to_joint_state([0.0, -1.57, 0.0, -1.57, 0.0, 0.0])
```

## Communication Flow

### 1. Data Flow (Read Cycle)
```
DUCO Robot → RPC/Thrift → DucoHardwareInterface::read() → 
Controller Manager → JointStateBroadcaster → /joint_states → 
MoveIt2/RViz2
```

### 2. Command Flow (Write Cycle)
```
MoveIt2 → FollowJointTrajectory Action → JointTrajectoryController → 
Controller Manager → DucoHardwareInterface::write() → 
RPC/Thrift → DUCO Robot
```

## Network Configuration

- **Robot IP**: 192.168.1.10
- **Robot Port**: 7003
- **Protocol**: TCP/Thrift
- **Update Rate**: 100Hz
- **Connection**: Ethernet

## Dependencies

### ROS2 Packages
```xml
<!-- Core ROS2 Control -->
<depend>controller_manager</depend>
<depend>joint_state_broadcaster</depend>
<depend>joint_trajectory_controller</depend>
<depend>hardware_interface</depend>

<!-- MoveIt2 -->
<depend>moveit_ros_move_group</depend>
<depend>moveit_ros_planning_interface</depend>
<depend>moveit_planners_ompl</depend>
<depend>pilz_industrial_motion_planner</depend>

<!-- Visualization -->
<depend>rviz2</depend>
<depend>moveit_ros_visualization</depend>
```

### System Requirements
- **ROS2**: Humble or later
- **Operating System**: Ubuntu 22.04
- **Network**: Ethernet connection to DUCO robot
- **Real-time Kernel**: Recommended for optimal performance

## Troubleshooting

### 1. Connection Issues
```bash
# Check network connectivity
ping 192.168.1.10

# Verify robot status
ros2 topic echo /joint_states --once
```

### 2. Controller Issues
```bash
# Restart controllers
ros2 control switch_controllers --stop arm_1_controller
ros2 control switch_controllers --start arm_1_controller
```

### 3. Hardware Interface Issues
```bash
# Check hardware interface logs
ros2 run rqt_console rqt_console

# Manual hardware interface reset
ros2 service call /controller_manager/reload_controller_libraries \
  controller_manager_msgs/srv/ReloadControllerLibraries
```

## Development

### Adding New Controllers
1. Define controller in `ros2_controllers.yaml`
2. Add controller spawning in launch file
3. Configure controller parameters

### Modifying Hardware Interface
1. Edit `duco_hardware_interface.cpp`
2. Rebuild package: `colcon build --packages-select duco_hardware`
3. Restart system

### Custom Planning Pipelines
1. Add configuration in `ompl_planning.yaml`
2. Define new planning groups in SRDF
3. Update MoveGroup configuration

## References

- [ROS2 Control Documentation](https://control.ros.org/)
- [MoveIt2 Documentation](https://moveit.ros.org/)
- [DUCO Robot Manual](https://www.duco.com/)

## License

This package is released under the BSD License.

## Maintainers

- DUCO Development Team
- ROS2 Control Contributors
