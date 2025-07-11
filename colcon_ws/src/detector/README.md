# detector

A ROS2 node for reading positions from two Leadshine motors and positions/PWM from two Feetech servos. **Now, the node only publishes data on demand, and supports per-topic 5Hz reading controlled by command.**

## Features
- Reads position from motor1 and motor2 on demand
- Reads position and PWM (torque) from servo17 and servo18 on demand
- Publishes to topics:
  - /motor1/position (std_msgs/Int32)
  - /motor2/position (std_msgs/Int32)
  - /servo17/position (std_msgs/Int32)
  - /servo18/position (std_msgs/Int32)
  - /servo17/pwm (std_msgs/Float32)
  - /servo18/pwm (std_msgs/Float32)
- **No periodic publishing by default.**
- **Supports per-topic 5Hz reading: you can start/stop 5Hz reading for each motor/servo topic independently via command.**

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

### 3. Launch detector node (recommended)
```bash
ros2 launch detector detector_launch.py
```

Or run the node directly:
```bash
ros2 run detector detector_node
```

### 4. Manually control 5Hz reading for each topic
You can use the following commands to control 5Hz reading for each motor/servo topic individually:

#### Start/Stop 5Hz reading for each topic

- Start 5Hz reading for /motor1/position:
  ```bash
  ros2 topic pub --once /motor1/read_ctrl std_msgs/String "data: 'start'"
  ```
- Stop 5Hz reading for /motor1/position:
  ```bash
  ros2 topic pub --once /motor1/read_ctrl std_msgs/String "data: 'stop'"
  ```
- Start 5Hz reading for /motor2/position:
  ```bash
  ros2 topic pub --once /motor2/read_ctrl std_msgs/String "data: 'start'"
  ```
- Stop 5Hz reading for /motor2/position:
  ```bash
  ros2 topic pub --once /motor2/read_ctrl std_msgs/String "data: 'stop'"
  ```
- Start 5Hz reading for /servo17/position and /servo17/pwm:
  ```bash
  ros2 topic pub --once /servo17/read_ctrl std_msgs/String "data: 'start'"
  ```
- Stop 5Hz reading for /servo17/position and /servo17/pwm:
  ```bash
  ros2 topic pub --once /servo17/read_ctrl std_msgs/String "data: 'stop'"
  ```
- Start 5Hz reading for /servo18/position and /servo18/pwm:
  ```bash
  ros2 topic pub --once /servo18/read_ctrl std_msgs/String "data: 'start'"
  ```
- Stop 5Hz reading for /servo18/position and /servo18/pwm:
  ```bash
  ros2 topic pub --once /servo18/read_ctrl std_msgs/String "data: 'stop'"
  ```

### 5. Echo topics in a new terminal
You can view the published data for each topic with:
```bash
ros2 topic echo /motor1/position
ros2 topic echo /motor2/position
ros2 topic echo /servo17/position
ros2 topic echo /servo18/position
ros2 topic echo /servo17/pwm
ros2 topic echo /servo18/pwm
```

### 6. Logging behavior
- **In the terminal running ros2 launch detector detector_launch.py**: Only error logs (such as read failures) will be shown, no info/warn logs.
- **In the terminal running ros2 topic pub --once ...**: Only topic output will be shown, no extra logs.

## Maintainer
[yizhongzhang1989@gmail.com](mailto:yizhongzhang1989@gmail.com) 