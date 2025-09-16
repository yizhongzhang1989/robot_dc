# Lift Robot Platform Controller

Lift platform controller using standard Modbus relay pulses (flash) to trigger motion.

## Build & Launch

```bash
# Build
colcon build --packages-select lift_robot_platform

# Launch
source install/setup.bash
ros2 launch robot_bringup lift_robot_bringup.py
```

## Control Commands

### Stop
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"stop\"}"'
```

### Up
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"up\"}"'
```

### Down
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"down\"}"'
```

## Relay Mapping

- Relay 0: stop
- Relay 1: up
- Relay 2: down

Each command sends a relay pulse: ON -> 100ms delay -> OFF.