# leadshine_motor

`leadshine_motor` is a ROS 2 package that provides a modular interface for controlling Leadshine motors via Modbus RTU. It delegates all Modbus communication to the centralized `modbus_driver` service, enabling multiple devices to share a single RS-485 serial line without bus contention.

This package wraps motor logic in a reusable Python class and provides a ROS 2 node to issue commands over topics.

---

## 📦 Package Structure

```

leadshine\_motor/
├── leadshine\_motor/
│   ├── motor\_node.py            # ROS 2 node that processes string commands
│   ├── motor\_controller.py      # LeadshineMotor class with protocol logic
├── launch/
│   └── motor\_launch.py          # (Optional) launch file to run motor\_node
├── test/
│   └── test\_motor\_controller.py # Unit tests (optional)

````

---

## ⚙️ Features

- ✅ Initialize and configure Leadshine motor registers
- ✅ Support absolute, relative, and velocity movement modes
- ✅ Accept signed target values (e.g., negative velocities or positions)
- ✅ Read/write motor parameters:
  - Position (signed 32-bit)
  - Velocity, acceleration, deceleration (signed 16-bit)
- ✅ ROS 2 topic-based command interface (`std_msgs/String`)
- ✅ Isolated motor logic for reuse in scripts or other nodes

---

## 🛠️ Dependencies

- ROS 2 (tested with **Humble**)
- `modbus_driver` (required service dependency)
- `modbus_driver_interfaces/srv/ModbusRequest`

Install Python dependencies:

```bash
pip install pymodbus
````

---

## 🚀 Usage

### 1. Build the Workspace

```bash
cd ~/colcon_ws
colcon build
source install/setup.bash
```

### 2. Launch `modbus_driver` (required)

```bash
ros2 launch modbus_driver modbus_manager_launch.py
```

### 3. Run the Motor Node

```bash
ros2 run leadshine_motor motor_node
```

You may optionally specify `motor_id`:

```bash
ros2 run leadshine_motor motor_node --ros-args -p motor_id:=2
```

---

## 🎮 Command Interface

This node subscribes to `/motor_cmd` (type: `std_msgs/String`). Commands are space-separated strings.

### ✅ Supported Commands

| Command           | Description                        |
| ----------------- | ---------------------------------- |
| `jog_left`        | Jog the motor left                 |
| `jog_right`       | Jog the motor right                |
| `stop`            | Abrupt stop                        |
| `set_zero`        | Set current position to zero       |
| `set_position X`  | Set target position to X (int32)   |
| `set_velocity X`  | Set target velocity to X (int16)   |
| `set_acc X`       | Set acceleration to X (int16)      |
| `set_dec X`       | Set deceleration to X (int16)      |
| `move_absolute`   | Move using current position target |
| `move_absolute X` | Set position and move (absolute)   |
| `move_relative`   | Move relative to current position  |
| `move_relative X` | Set relative target and move       |
| `move_velocity`   | Move at current velocity           |
| `move_velocity X` | Set velocity and move              |

Examples:

```bash
ros2 topic pub /motor_cmd std_msgs/String "data: 'set_velocity -1000'"
ros2 topic pub /motor_cmd std_msgs/String "data: 'move_velocity'"
ros2 topic pub /motor_cmd std_msgs/String "data: 'stop'"
```

---

## 🧪 Testing

### Manual Testing

Publish commands:

```bash
ros2 topic pub /motor_cmd std_msgs/String "data: 'set_position 10000'"
ros2 topic pub /motor_cmd std_msgs/String "data: 'move_absolute'"
```

### Unit Tests

If tests are implemented in `test/`:

```bash
colcon test --packages-select leadshine_motor
```

---

## 🧠 Internals

The `LeadshineMotor` class abstracts the raw Modbus register logic. It handles:

* Conversion between signed and unsigned formats
* Mapping ROS commands to register writes
* Managing motion mode and triggering moves

All values are converted before calling the `send()` function, ensuring consistent Modbus compliance. Values read via `recv()` are converted back to signed values internally.

---

## 📄 License

MIT License

---

## 👤 Maintainer

[jetson@todo.todo](mailto:jetson@todo.todo)

