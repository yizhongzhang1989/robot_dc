# Robotiq 2F-140 Gripper ROS2 Driver

ROS2 driver for controlling the Robotiq 2F-140 gripper via RS485/Modbus interface.

## Packages

- **robotiq_gripper_msgs**: Custom message definitions for gripper control
- **robotiq_2f140_gripper**: ROS2 node for gripper control

## Installation

```bash
cd ~/Documents/robot_dc/colcon_ws
colcon build --packages-select robotiq_gripper_msgs robotiq_2f140_gripper
source install/setup.bash
```

## Usage

### 1. Launch the Gripper Node

```bash
# Default configuration (device_id=9, host=192.168.1.15, port=54321)
ros2 launch robotiq_2f140_gripper robotiq_gripper.launch.py

# Custom configuration
ros2 launch robotiq_2f140_gripper robotiq_gripper.launch.py \
    device_id:=9 \
    rs485_host:=192.168.1.15 \
    rs485_port:=54321 \
    status_publish_rate:=10.0
```

### 2. Activate the Gripper

Before using the gripper, it must be activated:

```bash
ros2 service call /gripper/activate std_srvs/srv/Trigger
```

### 3. Control the Gripper

Send commands via topic:

```bash
# Close the gripper (position=255, speed=255, force=255)
ros2 topic pub --once /gripper/command robotiq_gripper_msgs/msg/GripperCommand \
    "{position: 255, speed: 255, force: 255, emergency_stop: false}"

# Open the gripper (position=0)
ros2 topic pub --once /gripper/command robotiq_gripper_msgs/msg/GripperCommand \
    "{position: 0, speed: 255, force: 0, emergency_stop: false}"

# Move to 50% position
ros2 topic pub --once /gripper/command robotiq_gripper_msgs/msg/GripperCommand \
    "{position: 128, speed: 200, force: 150, emergency_stop: false}"

# Emergency stop
ros2 topic pub --once /gripper/command robotiq_gripper_msgs/msg/GripperCommand \
    "{position: 0, speed: 0, force: 0, emergency_stop: true}"
```

### 4. Monitor Gripper Status

```bash
# Echo status messages
ros2 topic echo /gripper/status

# View status at 1 Hz
ros2 topic hz /gripper/status
```

## Topics

### Subscribed Topics

- `/gripper/command` (`robotiq_gripper_msgs/GripperCommand`)
  - **position**: 0 (fully open) to 255 (fully closed)
  - **speed**: 0 (slowest) to 255 (fastest)
  - **force**: 0 (weakest) to 255 (strongest)
  - **emergency_stop**: Stop gripper immediately

### Published Topics

- `/gripper/status` (`robotiq_gripper_msgs/GripperStatus`)
  - **is_activated**: Gripper activation status
  - **is_moving**: True if gripper is currently moving
  - **object_detected**: True if object is detected/gripped
  - **fault**: Fault status
  - **position**: Current position (0-255)
  - **force**: Current force
  - **raw_registers**: Raw Modbus register values

## Services

- `/gripper/activate` (`std_srvs/Trigger`)
  - Activates the gripper (must be called once before use)

## Parameters

- **device_id** (int, default: 9): Modbus device ID
- **rs485_host** (string, default: "192.168.1.15"): RS485 gateway IP address
- **rs485_port** (int, default: 54321): RS485 gateway port
- **status_publish_rate** (double, default: 10.0): Status publishing rate in Hz

## Python API Example

```python
import rclpy
from rclpy.node import Node
from robotiq_gripper_msgs.msg import GripperCommand

class GripperController(Node):
    def __init__(self):
        super().__init__('gripper_controller')
        self.pub = self.create_publisher(GripperCommand, '/gripper/command', 10)
    
    def close_gripper(self):
        msg = GripperCommand()
        msg.position = 255
        msg.speed = 255
        msg.force = 255
        msg.emergency_stop = False
        self.pub.publish(msg)
    
    def open_gripper(self):
        msg = GripperCommand()
        msg.position = 0
        msg.speed = 255
        msg.force = 0
        msg.emergency_stop = False
        self.pub.publish(msg)

def main():
    rclpy.init()
    controller = GripperController()
    
    # Activate gripper first (call service)
    # ...
    
    # Close gripper
    controller.close_gripper()
    
    rclpy.shutdown()
```

## Troubleshooting

### Connection Issues

1. Check RS485 gateway connectivity:
   ```bash
   ping 192.168.1.15
   ```

2. Verify port is accessible:
   ```bash
   nc -zv 192.168.1.15 54321
   ```

3. Check node logs:
   ```bash
   ros2 run robotiq_2f140_gripper robotiq_gripper_node --ros-args --log-level debug
   ```

### Gripper Not Responding

1. Make sure gripper is activated:
   ```bash
   ros2 service call /gripper/activate std_srvs/srv/Trigger
   ```

2. Check device ID matches your hardware configuration

3. Verify Modbus communication by monitoring status:
   ```bash
   ros2 topic echo /gripper/status
   ```

## Hardware Setup

- **RS485 Gateway**: Connect UR robot's RS485 port to TCP/IP gateway
- **Gripper**: Connect Robotiq 2F-140 to RS485 bus
- **Device ID**: Configure gripper Modbus device ID (default: 9)

## License

MIT
