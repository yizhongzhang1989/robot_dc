# duco\_robot\_arm

`duco_robot_arm` is a ROS 2 package that provides a comprehensive interface for controlling DUCO collaborative robot arms via Thrift RPC protocol. The package includes a complete Python API wrapper and a ROS 2 node for topic-based robot control.

This package wraps the DUCO robot's Thrift-based communication protocol in a reusable Python class and provides a ROS 2 node to issue commands over topics.

---

## 📦 Package Structure

```
duco_robot_arm/
├── duco_robot_arm/
│   ├── duco_robot_arm_node.py      # ROS 2 node that processes string commands
│   ├── DucoCobot.py                # DucoCobot class with complete robot API
│   ├── gen_py/                     # Generated Thrift Python code
│   │   └── robot/                  # Robot-specific Thrift definitions
│   │       ├── RPCRobot.py         # RPC client interface
│   │       ├── ttypes.py           # Thrift data types
│   │       └── constants.py        # Constants
│   └── lib/                        # Thrift library
│       └── thrift/                 # Thrift communication library
├── test/                           # Unit tests
└── scripts/                        # Standalone test scripts
```

---

## ⚙️ Features

### 🤖 Robot Control
* ✅ **Power Management**: Power on/off, enable/disable robot
* ✅ **Motion Control**: Joint, linear, circular, and TCP movements
* ✅ **Real-time Control**: Servo control with position/velocity/force feedback
* ✅ **Safety**: Emergency stop, collision detection, speed limits

### 🎯 Movement Types
* ✅ **Joint Motion**: `movej`, `movej2`, `movej_pose`
* ✅ **Linear Motion**: `movel`, `tcp_move`, `tcp_move_2p`
* ✅ **Circular Motion**: `movec`, `move_circle`
* ✅ **Servo Control**: `servoj`, `servoj_pose`, `servo_tcp`
* ✅ **Speed Control**: `speedj`, `speedl`, `speed_stop`

### 🔧 I/O & Configuration
* ✅ **Digital I/O**: Standard and tool digital inputs/outputs
* ✅ **Analog I/O**: Voltage and current inputs/outputs
* ✅ **Modbus Communication**: Read/write Modbus registers
* ✅ **Tool Configuration**: Tool data, TCP offset, work object setup

### 📊 Monitoring & Feedback
* ✅ **Position Feedback**: Joint and TCP positions
* ✅ **Velocity/Acceleration**: Real-time motion data
* ✅ **Force Feedback**: TCP force/torque readings
* ✅ **Robot State**: Comprehensive status monitoring

---

## 🛠️ Dependencies

* **ROS 2** (Humble tested)
* **Python 3.8+**
* **Thrift** (included in package)

No additional external dependencies required - all necessary libraries are bundled with the package.

---

## 🚀 Usage

### 1. Build the Workspace

```bash
cd ~/colcon_ws
colcon build --packages-select duco_robot_arm
source install/setup.bash
```

### 2. Launch Robot Node

Run the robot node with IP and port configuration:

```bash
ros2 run duco_robot_arm duco_robot_arm_node \
  --ros-args \
  -p ip:=192.168.1.10 \
  -p port:=7003 \
  -p device_id:=1
```

### 3. Alternative: Use as Python Library

The `DucoCobot` class can be used directly in Python scripts:

```python
from duco_robot_arm.DucoCobot import DucoCobot

# Connect to robot
robot = DucoCobot("192.168.1.10", 7003)
robot.open()

# Basic operations
robot.power_on(True)
robot.enable(True)

# Get current position
tcp_pose = robot.get_tcp_pose()
print(f"Current TCP pose: {tcp_pose}")

# Move robot
robot.tcp_move([0.0, 0.0, -0.02, 0.0, 0.0, 0.0], 1.0, 1.0, 0.0, "", True)

# Cleanup
robot.disable(True)
robot.power_off(True)
robot.close()
```

---

## 🎮 Command Interface

The robot node subscribes to `/arm{device_id}/cmd` (type: `std_msgs/String`) for command input. Commands are simple space-separated strings.

### ✅ Supported Commands

| Command      | Description                        |
| ------------ | ---------------------------------- |
| `power_on`   | Power on the robot                 |
| `power_off`  | Power off the robot                |
| `enable`     | Enable robot motors                |
| `disable`    | Disable robot motors               |

### 🧪 Command Examples

```bash
# Power on the robot
ros2 topic pub /arm1/cmd std_msgs/String "data: 'power_on'"

# Enable robot
ros2 topic pub /arm1/cmd std_msgs/String "data: 'enable'"

# Disable robot
ros2 topic pub /arm1/cmd std_msgs/String "data: 'disable'"

# Power off robot
ros2 topic pub /arm1/cmd std_msgs/String "data: 'power_off'"
```

---

## 🔌 Network Configuration

### Default Parameters

| Parameter   | Default Value  | Description                    |
| ----------- | -------------- | ------------------------------ |
| `ip`        | `192.168.1.10` | Robot IP address               |
| `port`      | `7003`         | Robot Thrift service port     |
| `device_id` | `1`            | Robot device ID for topics    |

### Connection Setup

1. **Physical Connection**: Connect to robot via Ethernet
2. **Network Setup**: Configure robot IP address (typically `192.168.1.10`)
3. **Firewall**: Ensure port 7003 is accessible
4. **Test Connection**: Use `ping` and `telnet` to verify connectivity

---

## 📋 API Reference

### Core Robot Operations

```python
# Connection management
robot.open()                    # Open connection
robot.close()                   # Close connection

# Power and enable
robot.power_on(block=True)      # Power on robot
robot.power_off(block=True)     # Power off robot
robot.enable(block=True)        # Enable motors
robot.disable(block=True)       # Disable motors

# Motion control
robot.movej(joints, v, a, r, block, op)           # Joint motion
robot.movel(pose, v, a, r, q_near, tool, wobj, block, op)  # Linear motion
robot.tcp_move(offset, v, a, r, tool, block, op)  # TCP relative motion

# Status monitoring
robot.get_robot_state()         # Get robot state
robot.get_tcp_pose()           # Get TCP position
robot.get_actual_joints_position()  # Get joint positions
```

### I/O Operations

```python
# Digital I/O
robot.set_standard_digital_out(num, value, block)
robot.get_standard_digital_in(num)

# Analog I/O
robot.set_standard_analog_voltage_out(num, value, block)
robot.get_standard_analog_voltage_in(num)
```

---

## 🧪 Testing

### Integration Test

Test the complete system with a real robot:

```bash
# Test connection and basic operations
python3 scripts/duco_z_move_test.py
```

### ROS Node Test

Test the ROS interface:

```bash
# Start robot node
ros2 run duco_robot_arm duco_robot_arm_node --ros-args -p ip:=192.168.1.10

# In another terminal, send commands
ros2 topic pub /arm1/cmd std_msgs/String "data: 'power_on'"
ros2 topic pub /arm1/cmd std_msgs/String "data: 'enable'"
```

---

## 🔧 Configuration

### Robot Parameters

The node accepts the following ROS parameters:

```yaml
# Example parameter configuration
ip: "192.168.1.10"      # Robot IP address
port: 7003               # Robot port
device_id: 1             # Device ID for topic naming
```

### Launch File Example

```python
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='duco_robot_arm',
            executable='duco_robot_arm_node',
            name='duco_robot_arm_node',
            parameters=[{
                'ip': '192.168.1.10',
                'port': 7003,
                'device_id': 1
            }]
        )
    ])
```

---

## 🛡️ Safety Considerations

⚠️ **Important Safety Notes**:

1. **Emergency Stop**: Always have an emergency stop accessible
2. **Workspace**: Ensure clear workspace before robot operation
3. **Speed Limits**: Use appropriate velocity and acceleration limits
4. **Collision Detection**: Enable collision detection when available
5. **Power Management**: Always disable and power off when not in use

---

## 🐛 Troubleshooting

### Common Issues

**Connection Failed**:
```bash
# Check network connectivity
ping 192.168.1.10
telnet 192.168.1.10 7003
```

**Import Errors**:
```bash
# Rebuild package
colcon build --packages-select duco_robot_arm --symlink-install
source install/setup.bash
```

**Robot Not Responding**:
1. Check robot power and network connection
2. Verify IP address and port settings
3. Ensure robot is in remote control mode
4. Check for robot errors on teach pendant

---

## 📄 License

MIT License

---

## 👤 Maintainer

**Yi-Zhong Zhang** - [yizhongzhang1989@gmail.com](mailto:yizhongzhang1989@gmail.com)

---

## 🔗 Related Packages

* `robot_bringup` - Complete robot system launch
* `robot_teleop` - Teleoperation interface
* `modbus_driver` - Modbus device communication

---

## 📚 Additional Resources

* [DUCO Robot Documentation](https://www.duco-robot.com)
* [ROS 2 Documentation](https://docs.ros.org/en/humble/)
* [Thrift Documentation](https://thrift.apache.org/)
