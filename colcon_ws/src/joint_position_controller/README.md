# Joint Position Controller

Simple ROS2 controller for direct joint position control with smooth interpolation.

## Features

- ✅ Direct joint angle control via topic
- ✅ Smooth interpolation between positions
- ✅ Configurable motion time
- ✅ Easy integration with ROS2 Control framework

## Usage

### 1. Add to controller configuration

Edit your `ros2_controllers.yaml`:

```yaml
controller_manager:
  ros__parameters:
    update_rate: 100
    joint_position_controller:
      type: joint_position_controller/JointPositionController

joint_position_controller:
  ros__parameters:
    joints:
      - arm_1_joint_1
      - arm_1_joint_2
      - arm_1_joint_3
      - arm_1_joint_4
      - arm_1_joint_5
      - arm_1_joint_6
    interpolation_time: 5.0  # seconds to reach target
    max_velocity: 1.0
    max_acceleration: 1.0
```

### 2. Launch the controller

```bash
# Load and activate the controller
ros2 control load_controller joint_position_controller
ros2 control set_controller_state joint_position_controller active
```

### 3. Send joint commands

```bash
# Command format: [j1, j2, j3, j4, j5, j6] in radians
ros2 topic pub /joint_position_controller/joint_command std_msgs/msg/Float64MultiArray \
  "data: [0.0, -0.5, 0.5, 0.0, 0.5, 0.0]" --once
```

### Python Example

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray

class JointCommandPublisher(Node):
    def __init__(self):
        super().__init__('joint_command_publisher')
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/joint_command',
            10
        )
    
    def send_command(self, joint_angles):
        msg = Float64MultiArray()
        msg.data = joint_angles
        self.publisher.publish(msg)
        self.get_logger().info(f'Sent joint command: {joint_angles}')

def main():
    rclpy.init()
    node = JointCommandPublisher()
    
    # Example: Move to home position
    home_position = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    node.send_command(home_position)
    
    rclpy.spin_once(node, timeout_sec=1.0)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

## Topics

### Subscribed
- `~/joint_command` (`std_msgs/msg/Float64MultiArray`): Target joint positions

### Published
- `~/state` (`sensor_msgs/msg/JointState`): Current joint state feedback

## Parameters

| Parameter | Type | Default | Description |
|-----------|------|---------|-------------|
| `joints` | string[] | - | List of joint names |
| `interpolation_time` | double | 5.0 | Time to reach target (seconds) |
| `max_velocity` | double | 1.0 | Maximum joint velocity (rad/s) |
| `max_acceleration` | double | 1.0 | Maximum joint acceleration (rad/s²) |

## How It Works

1. Controller subscribes to `~/joint_command` topic
2. When new command arrives:
   - Stores current position as start position
   - Stores command as target position
   - Resets interpolation timer
3. In each control cycle:
   - Calculates smooth interpolation (cubic ease-in-out)
   - Writes interpolated position to hardware interface
   - Continues until target is reached

## Integration with DucoHardwareInterface

This controller works seamlessly with the existing `duco_hardware` package:

```
JointPositionController::update()
    ↓ writes to CommandInterface
    ↓ (shared memory: hw_commands_)
    ↓
DucoHardwareInterface::write()
    ↓ calls servoj()
    ↓
DUCO Robot Hardware
```
