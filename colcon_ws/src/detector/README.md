# detector

A ROS2 node for periodically reading positions from two Leadshine motors and positions/PWM from two Feetech servos, publishing them at 5Hz.

## Features
- Reads position from motor1 and motor2
- Reads position and PWM (torque) from servo17 and servo18
- Publishes to topics:
  - /motor1/position (std_msgs/Int32)
  - /motor2/position (std_msgs/Int32)
  - /servo17/position (std_msgs/Int32)
  - /servo18/position (std_msgs/Int32)
  - /servo17/pwm (std_msgs/Float32)
  - /servo18/pwm (std_msgs/Float32)
- Frequency: 5Hz

## Dependencies
- ROS2 (Humble recommended)
- leadshine_motor package
- feetech_servo package

## Usage

### 1. Build the workspace
```bash
cd colcon_ws
colcon build
source install/setup.bash
```

### 2. Launch modbus_driver (required)
```bash
ros2 launch modbus_driver modbus_manager_launch.py
```

### 3. Run detector node
```bash
ros2 run detector detector_node
```

### 4. Echo topics in a new terminal
```bash
ros2 topic echo /motor1/position
ros2 topic echo /servo17/pwm
```

## Maintainer
[yizhongzhang1989@gmail.com](mailto:yizhongzhang1989@gmail.com) 