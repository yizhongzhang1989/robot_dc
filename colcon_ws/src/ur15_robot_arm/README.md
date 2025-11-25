# UR15 Robot Arm Package

This package provides control interface for UR15 robot arm.

## Usage

Launch the UR15 robot driver:
```bash
ros2 launch ur_robot_driver ur_control.launch.py \
  ur_type:=ur15 \
  robot_ip:=192.168.1.15
```

Then run the UR15 robot arm node:
```bash
ros2 run ur15_robot_arm ur15_robot_arm_node
```

Or use the provided launch file:
```bash
ros2 launch ur15_robot_arm ur15_robot_arm.launch.py
```
