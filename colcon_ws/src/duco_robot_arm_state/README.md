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

- `/arm{device_id}/robot_state` - Comprehensive robot state data including all joint, TCP, and driver information (JSON)

## Data Structure

The published JSON message contains all robot data with original field names for extensibility:

```json
{
  "jointActualPosition": [7 floats],
  "jointActualVelocity": [7 floats],
  "jointActualAccelera": [7 floats],
  "jointActualTorque": [7 floats],
  "jointExpectPosition": [7 floats],
  "jointExpectVelocity": [7 floats],
  "jointExpectAccelera": [7 floats],
  "jointExpectTorque": [7 floats],
  "jointActualTemperature": [7 floats],
  "jointActualCurrent": [7 floats],
  "driverErrorID": [7 ints],
  "driverState": [7 ints],
  "TCPActualPosition": [6 floats],
  "TCPActualVelocity": [6 floats],
  "TCPActualAccelera": [6 floats],
  "TCPActualTorque": [6 floats],
  "TCPExpectPosition": [6 floats],
  "TCPExpectVelocity": [6 floats],
  "TCPExpectAccelera": [6 floats],
  "TCPExpectTorque": [6 floats],
  "baseActualTorque": [6 floats],
  "baseExpectTorque": [6 floats],
  "timestamp": "ROS timestamp"
}
```

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

All data is published in a single JSON message with original field names preserved for future extensibility.

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
