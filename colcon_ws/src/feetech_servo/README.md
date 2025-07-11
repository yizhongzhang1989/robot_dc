# feetech_servo

'feetech_servo' is a ROS 2 package providing a modular interface for controlling Feetech servos via Modbus RTU. All Modbus communication is handled by a centralized 'modbus_driver' service, allowing multiple servos to share a single RS-485 bus without conflicts.

---

## 📦 Package Structure

```
feetech_servo/
├── feetech_servo/
│   ├── servo_node.py            # ROS 2 node that processes string commands
│   ├── servo_controller.py      # FeetechServo class with protocol logic
├── launch/
│   └── servo_control_launch.py  # Launch file to start multiple servo nodes
├── test/
│   └── ...                      # Code style/compliance tests
```

---

## ⚙️ Features

* ✅ Initialize and configure Feetech servo parameters
* ✅ Support for position, velocity, acceleration, and torque settings
* ✅ ROS 2 topic-based string command interface ('std_msgs/String')
* ✅ Reusable servo control class for use in scripts or other nodes
* ✅ Support for concurrent multi-servo control

---

## 🛠️ Dependencies

* ROS 2 (**Humble** tested)
* 'modbus_driver' (ROS 2 package)
* 'modbus_devices' (custom Modbus device base class)

---

## 🚀 Usage

### 1. Build the Workspace

```bash
cd colcon_ws
colcon build
source install/setup.bash
```

### 2. Launch 'modbus_driver' (required)

```bash
ros2 launch modbus_driver modbus_manager_launch.py
```

### 3. Launch Servo Nodes

Use the launch file to start two servo nodes:

```bash
ros2 launch feetech_servo servo_control_launch.py
```

This command will automatically start:
- 'motor17' (device_id=17), subscribing to '/motor17/cmd'
- 'motor18' (device_id=18), subscribing to '/motor18/cmd'

To run a single instance manually:

```bash
ros2 run feetech_servo servo_node --ros-args -p device_id:=17
```

---

## 🎮 Command Interface

Each servo node subscribes to its own topic, e.g., '/motor17/cmd', '/motor18/cmd', with message type 'std_msgs/String'. Commands are simple space-separated strings.

### ✅ Supported Commands

| Command                | Description                                         |
|:---------------------- |:----------------------------------------------------|
| 'stop'                 | Abrupt stop, servo stops immediately                |
| 'set_pos X'            | Set target position (int, unit: pulse)              |
| 'set_vel X'            | Set target velocity (int, unit: rpm)                |
| 'set_acc X'            | Set acceleration (int, unit: custom/see manual)     |
| 'set_torque X'         | Set torque limit (int, unit: custom)                |
| 'enable_torque 1/0'    | Enable/disable torque (1=enable, 0=disable)         |
| 'get_status'           | Get current status (position, velocity, current, etc.) |
| 'get_pos'              | Get current position                               |
| 'get_vel'              | Get current velocity                               |
| 'get_temp'             | Get current temperature                            |
| 'get_voltage'          | Get current voltage                                |
| 'get_current'          | Get current current                                |
| 'init'                 | Initialize servo parameters                        |
| 'help'                 | Print supported command list                       |

> Currently implemented: 'stop', 'set_pos', 'set_vel', 'set_acc'. Other commands can be added as needed.
> 新增命令：'get_pos' 读取舵机当前位置，'get_torque' 读取当前扭矩（基于PWM估算）。

---

### 📖 位置与扭矩读取说明

| 功能         | 寄存器地址 (hex) | 单位         | 计算方式                                      |
|--------------|------------------|--------------|-----------------------------------------------|
| 位置读取     | 0x0101 (257)     | step (0-4095)| 直接使用返回值，0~4095对应0°~360°              |
| 扭矩估算     | 0x0103 (259)     | 0.1% (PWM)   | Torque (N·m) = (PWM_RAW / 1000) × 4.413       |

- **位置读取**：
  - 调用 `get_position()` 方法，读取寄存器 0x0101，返回步数（step）。
  - 例：返回 2048 → 约180°。

- **扭矩读取**：
  - 调用 `get_torque()` 方法，读取寄存器 0x0103，返回 PWM_RAW。
  - 扭矩计算：`Torque (N·m) = (PWM_RAW / 1000) × 4.413`
  - 例：PWM_RAW=500 → Torque=2.2065 N·m。

---

### 🧪 位置与扭矩读取命令示例

```bash
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'get_pos'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'get_torque'"
```

节点会在日志中输出当前步数和估算扭矩。

---

### 📋 Parameter Descriptions

- **'set_pos X'**: X = target position (int, unit: pulse, range depends on servo model)
- **'set_vel X'**: X = target velocity (int, unit: rpm)
- **'set_acc X'**: X = acceleration (int, unit: ms/1000rpm or custom)
- **'set_torque X'**: X = torque limit (int, unit: custom)
- **'enable_torque 1/0'**: 1 to enable, 0 to disable

---

### 🧪 Topic Command Examples

```bash
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'set_pos 2048'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'set_vel 10'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'set_acc 5'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'set_torque 800'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'enable_torque 1'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'stop'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'get_status'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'get_pos'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'get_vel'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'get_temp'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'get_voltage'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'get_current'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'init'"
ros2 topic pub --once /motor17/cmd std_msgs/String "data: 'help'"

ros2 topic pub --once /motor18/cmd std_msgs/String "data: 'set_pos 1024'"
ros2 topic pub --once /motor18/cmd std_msgs/String "data: 'set_vel 5'"
ros2 topic pub --once /motor18/cmd std_msgs/String "data: 'stop'"
```

---

## 🧠 Internals

The 'FeetechServo' class abstracts the low-level Modbus logic, handling:

* Parameter and register read/write
* ROS command mapping
* Motion mode management

All values are converted to Modbus format in 'send()', and decoded in 'recv()' to ensure compatibility.

---

## 📄 License

MIT License (update if different)

---

## 👤 Maintainer

[yizhongzhang1989@gmail.com](mailto:yizhongzhang1989@gmail.com)

--- 