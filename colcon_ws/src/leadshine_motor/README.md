# leadshine\_motor

`leadshine_motor` is a ROS 2 package that provides a modular interface for controlling Leadshine motors via Modbus RTU. It delegates all Modbus communication to the centralized `modbus_driver` service, enabling multiple devices to share a single RS-485 serial line without bus contention.

This package wraps motor logic in a reusable Python class and provides a ROS 2 node to issue commands over topics.

---

## 📦 Package Structure

```
leadshine_motor/
├── leadshine_motor/
│   ├── motor_node.py            # ROS 2 node that processes string commands
│   ├── motor_controller.py      # LeadshineMotor class with protocol logic
├── launch/
│   └── motor_control_launch.py  # Launch file to start motor nodes
├── test/
│   └── test_motor_controller.py # Unit tests (optional)
```

---

## ⚙️ Features

* ✅ Initialize and configure Leadshine motor registers
* ✅ Support absolute, relative, and velocity movement modes
* ✅ Accept signed target values (e.g., negative velocities or positions)
* ✅ Read/write motor parameters:

  * Position (signed 32-bit)
  * Velocity, acceleration, deceleration (signed 16-bit)
* ✅ ROS 2 topic-based command interface (`std_msgs/String`)
* ✅ Isolated motor logic for reuse in scripts or other nodes
* ✅ Inline parameter configuration (no YAML required)

---

## 🛠️ Dependencies

* ROS 2 (**Humble** tested)
* `modbus_driver` (ROS 2 package)
* `modbus_driver_interfaces/srv/ModbusRequest`

Install Python dependencies:

```bash
pip install pymodbus
```

---

## 🚀 Usage

### 1. Build the Workspace

```bash
cd colcon_ws
colcon build
source install/setup.bash
```

### 2. Launch `modbus_driver` (required)

```bash
ros2 launch modbus_driver modbus_manager_launch.py
```

### 3. Launch Motor Nodes

Use the launch file to start multiple motors, each with its own `motor_id`:

```bash
ros2 launch leadshine_motor motor_control_launch.py
```

Or run a single instance manually:

```bash
ros2 run leadshine_motor motor_node --ros-args -p device_id:=1
```

---

## 🎮 Command Interface

Each motor node subscribes to its own topic, e.g., `/motor1/cmd` (type: `std_msgs/String`). Commands are simple space-separated strings.

### ✅ Supported Commands

| Command                                      | Description                                                                |
| -------------------------------------------- | -------------------------------------------------------------------------- |
| `jog_left`                                  | Jog motor left                                                             |
| `jog_right`                                 | Jog motor right                                                            |
| `stop`                                      | Abrupt stop                                                                |
| `get_pos`                                   | Log current position                                                       |
| `set_zero`                                  | Set current position to zero                                               |
| `set_pos X`                                 | Set target position (int32, unit: pulse)                                   |
| `set_vel X`                                 | Set target velocity (int16, unit: rpm)                                     |
| `set_acc X`                                 | Set acceleration (int16, unit: ms/1000rpm)                                 |
| `set_dec X`                                 | Set deceleration (int16, unit: ms/1000rpm)                                 |
| `move_abs`                                  | Move to previously set position                                            |
| `move_abs X`                                | Set and move to absolute position (unit: pulse)                            |
| `move_rel`                                  | Move by previously set offset                                              |
| `move_rel X`                                | Set and move by relative offset (unit: pulse)                              |
| `move_vel`                                  | Move using previously set velocity                                         |
| `move_vel X`                                | Set and move at velocity (unit: rpm)                                       |
| `home_pos`                                  | Torque home in positive direction (use default parameters)                 |
| `home_neg`                                  | Torque home in negative direction (use default parameters)                 |
| `set_home sta cur hig low acc dec`           | Set home parameters only, params: sta=StallTime(ms), cur=CurrentPercent(%) (torque homing current), hig=HighSpeed(rpm), low=LowSpeed(rpm), acc=Acceleration(ms/1000rpm), dec=Deceleration(ms/1000rpm) |
| `set_limit P N`                             | Set software limits, P=positive limit, N=negative limit (int32, unit: pulse, e.g. 100000 -100000) |
| `reset_limit`                                 | Reset (disable) software limits, disables both positive and negative software limits |

Parameter description:  
sta=StallTime(ms), cur=CurrentPercent(%) (torque homing current, set as current percent), hig=HighSpeed(rpm), low=LowSpeed(rpm), acc=Acceleration(ms/1000rpm), dec=Deceleration(ms/1000rpm)

Note: Software limit is disabled by default on startup.

### 🧪 Examples

```bash
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_limit 300000 -300000'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_zero'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_vel -500'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'move_rel -200000'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'get_pos'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'jog_left'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'jog_right'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'home_pos'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'home_neg'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_home 1200 60 500 300 200 200'"  # sta cur hig low acc dec
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_limit 100000 -100000'"

ros2 topic pub --once /motor2/cmd std_msgs/String "data: 'set_zero'"
ros2 topic pub --once /motor2/cmd std_msgs/String "data: 'set_vel 800'"
ros2 topic pub --once /motor2/cmd std_msgs/String "data: 'move_rel 50000'"
```

---

## 🧪 Testing

### Manual Testing

```bash
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_zero'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_vel 500'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'move_rel 10000'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'home_pos'"
ros2 topic pub --once /motor2/cmd std_msgs/String "data: 'set_zero'"
```

### Unit Tests

If implemented:

```bash
colcon test --packages-select leadshine_motor
```

---

## 🧠 Internals

The `LeadshineMotor` class abstracts raw Modbus logic. It handles:

* Signed/unsigned value conversions
* ROS command mapping
* Motion mode management

All values are converted for Modbus in `send()`, and decoded in `recv()` to ensure compatibility.

---

## 📄 License

MIT License

---

## 👤 Maintainer

[jetson@todo.todo](mailto:jetson@todo.todo)

---
