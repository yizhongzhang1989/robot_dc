# leadshine\_motor

`leadshine_motor` 是一个 ROS 2 包，提供了一个模块化接口来通过 Modbus RTU 控制 Leadshine 电机。它将所有 Modbus 通信委托给集中式 `modbus_driver` 服务，使多个设备能够共享单个 RS-485 串行线而不发生总线冲突。

这个包将电机逻辑包装在一个可重用的 Python 类中，并提供了一个 ROS 2 节点来通过主题发布命令。

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

| Command                                   | Description                                         |
|:------------------------------------------|:----------------------------------------------------|
| 'jog_left'                                | Jog motor left                                      |
| 'jog_right'                               | Jog motor right                                     |
| 'stop'                                    | Abrupt stop                                         |
| 'get_pos'                                 | Log current position                                |
| 'get_status'                              | Read motion status                                  |
| 'get_alarm'                               | Read alarm/fault status                             |
| 'reset_alarm'                             | Reset alarm/fault status                            |
| 'set_zero'                                | Set current position to zero                        |
| 'set_pos X'                               | Set target position (int32, pulse)                  |
| 'set_vel X'                               | Set target velocity (int16, rpm)                    |
| 'set_acc X'                               | Set acceleration (int16, ms/1000rpm)                |
| 'set_dec X'                               | Set deceleration (int16, ms/1000rpm)                |
| 'move_abs'                                | Move to previously set position                     |
| 'move_abs X'                              | Set and move to absolute position (pulse)           |
| 'move_rel'                                | Move by previously set offset                       |
| 'move_rel X'                              | Set and move by relative offset (pulse)             |
| 'move_vel'                                | Move using previously set velocity                  |
| 'move_vel X'                              | Set and move at velocity (rpm)                      |
| 'home_pos'                                | Torque home in positive direction                   |
| 'home_neg'                                | Torque home in negative direction                   |
| 'set_home sta cur hig low acc dec'        | Set home parameters                                 |
| 'set_limit P N'                           | Set software limits                                 |
| 'reset_limit'                             | Reset (disable) software limits                     |
| 'home_back'                               | Automatic reverse movement after homing             |
| 'save_params'                             | Save all parameters to EEPROM                       |
| 'factory_reset'                           | Restore all parameters to factory defaults          |

### 📋 Parameter Descriptions

#### Basic Motion Commands
- **'jog_left'/'jog_right'**: Manual jogging commands, no parameters required
- **'stop'**: Abrupt stop command, no parameters required
- **'set_zero'**: Set current position to zero, no parameters required

#### Position and Velocity Commands
- **'set_pos X'**: X = target position (int32, unit: pulse)
- **'set_vel X'**: X = target velocity (int16, unit: rpm)
- **'set_acc X'**: X = acceleration (int16, unit: ms/1000rpm)
- **'set_dec X'**: X = deceleration (int16, unit: ms/1000rpm)

#### Movement Commands
- **'move_abs'**: Move to previously set position, no parameters required
- **'move_abs X'**: X = absolute position (unit: pulse)
- **'move_rel'**: Move by previously set offset, no parameters required
- **'move_rel X'**: X = relative offset (unit: pulse)
- **'move_vel'**: Move using previously set velocity, no parameters required
- **'move_vel X'**: X = velocity (unit: rpm)

#### Homing Commands
- **'home_pos'**: Torque home in positive direction, no parameters required
- **'home_neg'**: Torque home in negative direction, no parameters required
- **'home_back'**: Automatic reverse movement after homing, no parameters required

#### Home Parameters ('set_home sta cur hig low acc dec')
- **sta**: Stall time (ms) - Time to wait for stall detection
- **cur**: Current percent (%) - Torque homing current percentage
- **hig**: High speed (rpm) - High speed during homing
- **low**: Low speed (rpm) - Low speed during homing
- **acc**: Acceleration (ms/1000rpm) - Acceleration time
- **dec**: Deceleration (ms/1000rpm) - Deceleration time

#### Software Limits ('set_limit P N')
- **P**: Positive limit (int32, unit: pulse) - Maximum positive position
- **N**: Negative limit (int32, unit: pulse) - Maximum negative position
- Example: `set_limit 100000 -100000`

#### Status and Alarm Commands
- **'get_pos'**: Returns current position (int32, unit: pulse)
- **'get_status'**: Returns status bits from register 0x1003:
  - **fault**: Fault status (1=fault, 0=no fault)
  - **enabled**: Enable status (1=enabled, 0=disabled)
  - **running**: Running status (1=running, 0=stopped)
  - **command_completed**: Command completion status
  - **path_completed**: Path completion status
  - **homing_completed**: Homing completion status
- **'get_alarm'**: Returns fault information from register 0x2203:
  - **fault_code**: Hexadecimal fault code
  - **fault_description**: Human-readable fault description
  - **alm_blink_count**: ALM LED blink count for hardware indication
- **'reset_alarm'**: Reset alarm/fault status, no parameters required

#### Common Fault Codes
| Fault Code (Hex) | Fault Description                 | ALM Blink Count |
|:----------------:|:----------------------------------|:---------------:|
| 0x0000           | No fault                          | 0               |
| 0x00E0           | Overcurrent                       | 1               |
| 0x00C0           | Overvoltage                       | 2               |
| 0x00A1           | Current sampling circuit fault    | 3               |
| 0x0152           | Shaft lock (phase loss) fault     | 4               |
| 0x0240           | EEPROM fault                      | 5               |
| 0x05F0           | Parameter auto-tuning fault       | 6               |
| 0x0180           | Position error alarm              | 7               |
| 0x0150           | Encoder disconnection detection   | 8               |
| 0x00F0           | Overtemperature                   | 9               |
| 0x0210           | Input IO configuration duplicate  | 10              |

#### Control Word (0x1801) Features
| Control Word Value (Hex) | Feature Description                         |
|:-----------------------:|:---------------------------------------------|
| 0x1111                  | Reset current alarm (clear current fault)    |
| 0x1122                  | Reset history alarm                          |
| 0x2211                  | Save all parameters to EEPROM                |
| 0x2222                  | Parameter initialization (keep motor params) |
| 0x2233                  | Restore all parameters to factory defaults   |
| 0x2244                  | Save all mapping parameters to EEPROM        |
| 0x4001                  | JOG left (jog/continuous move)               |
| 0x4002                  | JOG right (jog/continuous move)              |

**Note:**
- Features are executed immediately upon writing.
- Alarm reset is performed automatically before each motion command; manual reset is rarely needed.
- After a control word operation, you can read the status word to confirm execution. Once the status word is read, it will auto-reset to the initial state.
- JOG is for jog/manual debug. If the interval between trigger commands >50ms, it will only jog once. For continuous movement, send the command repeatedly within 50ms intervals (like a watchdog).


### 🧪 Examples

```bash
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_limit 300000 -300000'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_zero'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_vel -500'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'move_rel -200000'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'get_pos'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'get_status'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'get_alarm'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'reset_alarm'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'jog_left'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'jog_right'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'home_pos'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'home_neg'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_home 1200 60 250 250 200 200'"  # sta cur hig low acc dec
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'set_limit 100000 -100000'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'save_params'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'factory_reset'"
```
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
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'get_status'"
ros2 topic pub --once /motor1/cmd std_msgs/String "data: 'get_alarm'"
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
