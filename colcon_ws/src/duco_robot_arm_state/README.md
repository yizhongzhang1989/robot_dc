# Duco Robot Arm State Package

This ROS2 package provides real-time monitoring of Duco robot arm state via TCP connection on port 2001.

## Features

- **Real-time monitoring**: Continuously reads robot state data at ~10Hz
- **Comprehensive data**: Monitors joint positions, velocities, accelerations, torques, temperatures, and more
- **TCP state data**: Provides actual and expected TCP (Tool Center Point) information
- **Driver status**: Monitors driver error states and status
- **ROS2 integration**: Publishes data to standard ROS2 topics
- **Shared interface**: Uses same parameter interface as `duco_robot_arm` control package

## Topics Published

All topics are namespaced with `/arm{device_id}/`:

- `/arm{device_id}/joint_state` - Joint actual and expected state data (JSON)
- `/arm{device_id}/tcp_state` - TCP actual and expected state data (JSON) 
- `/arm{device_id}/robot_status` - Driver status and base torque data (JSON)
- `/arm{device_id}/raw_data` - Raw monitoring data for debugging (JSON)

## Parameters

- `device_id` (int, default: 1): Device ID for the robot arm
- `ip` (string, default: '192.168.1.10'): IP address of the robot
- `port` (int, default: 2001): TCP port for state monitoring
- `frame_size` (int, default: 1468): Size of each data frame in bytes

## Usage

### Launch state monitoring only:
```bash
ros2 launch duco_robot_arm_state duco_robot_arm_state.launch.py
```

### Launch both control and state monitoring:
```bash
ros2 launch duco_robot_arm_state duco_robot_arm_full.launch.py
```

### With custom parameters:
```bash
ros2 launch duco_robot_arm_state duco_robot_arm_state.launch.py device_id:=2 ip:=192.168.1.20
```

## Data Structure

The robot sends data frames containing:
- Joint actual data: position, velocity, acceleration, torque, temperature, current
- Joint expected data: position, velocity, acceleration, torque
- TCP actual data: position, velocity, acceleration, torque
- TCP expected data: position, velocity, acceleration, torque
- Driver status: error IDs and states
- Base torque data: actual and expected

## Integration with duco_robot_arm

This package is designed to work alongside the `duco_robot_arm` control package:
- Uses the same parameter interface (`device_id`, `ip`)
- Publishes to compatible topic namespaces
- Can be launched together with the full launch file

## Building

```bash
cd /path/to/colcon_ws
colcon build --packages-select duco_robot_arm_state
source install/setup.bash
```

## Testing

```bash
colcon test --packages-select duco_robot_arm_state
```
