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

| Command      | Description                        |
| ------------ | ---------------------------------- |
| `jog_left`   | Jog motor left                     |
| `jog_right`  | Jog motor right                    |
| `stop`       | Abrupt stop                        |
| `get_pos`    | Log current position               |
| `set_zero`   | Set current position to zero       |
| `set_pos X`  | Set target position (int32)        |
| `set_vel X`  | Set target velocity (int16)        |
| `set_acc X`  | Set acceleration (int16)           |
| `set_dec X`  | Set deceleration (int16)           |
| `move_abs`   | Move to previously set position    |
| `move_abs X` | Set and move to absolute position  |
| `move_rel`   | Move by previously set offset      |
| `move_rel X` | Set and move by relative offset    |
| `move_vel`   | Move using previously set velocity |
| `move_vel X` | Set and move at velocity           |
| `+`          | 正向力矩回零（使用默认参数）        |
| `-`          | 反向力矩回零（使用默认参数）        |
| `+ S O H L A D`| 正向力矩回零，参数依次为：堵转时间 出力值 高速 低速 加速度 减速度 |
| `- S O H L A D`| 反向力矩回零，参数依次为：堵转时间 出力值 高速 低速 加速度 减速度 |

参数说明：  
S=堵转时间(ms)，O=出力值(%)，H=回零高速(rpm)，L=回零低速(rpm)，A=回零加速度(ms/1000rpm)，D=回零减速度(ms/1000rpm)

### 🧪 Examples

```bash
ros2 topic pub /motor1/cmd std_msgs/String "data: 'set_vel -1000'"
ros2 topic pub /motor1/cmd std_msgs/String "data: 'move_vel'"
ros2 topic pub /motor1/cmd std_msgs/String "data: 'get_pos'"
ros2 topic pub /motor1/cmd std_msgs/String "data: 'stop'"
```

---

## 🧪 Testing

### Manual Testing

```bash
ros2 topic pub /motor1/cmd std_msgs/String "data: 'set_pos 10000'"
ros2 topic pub /motor1/cmd std_msgs/String "data: 'move_abs'"
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
