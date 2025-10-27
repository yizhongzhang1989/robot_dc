# Lift Robot Platform Controller

Lift platform controller using standard Modbus relay pulses (flash) to trigger motion.

**Features:**
- Manual control (up/down/stop)
- Timed movements
- **NEW:** Closed-loop height control with smart Modbus throttling

## Build & Launch

```bash
# Build
colcon build --packages-select lift_robot_platform

# Launch
source install/setup.bash
ros2 launch robot_bringup lift_robot_bringup.py
```

## Control Commands

### Manual Control

#### Stop
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"stop\"}"'
```

#### Up
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"up\"}"'
```

#### Down
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"down\"}"'
```

### Closed-Loop Height Control (NEW)

The platform now supports automatic height control with sensor feedback.

#### Go to Specific Height
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"goto_height\", \"target_height\": 750.0}"'
```

#### Enable/Disable Control Loop
```bash
# Enable
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"enable_control\"}"'

# Disable
ros2 topic pub --once /lift_robot_platform/command std_msgs/String 'data: "{\"command\": \"disable_control\"}"'
```

## Control Loop Architecture

### High-Frequency Control with Smart Throttling

The controller implements a sophisticated control strategy:

| Component | Frequency | Purpose |
|-----------|-----------|---------|
| **Control Loop** | 10 Hz | Runs locally, checks error & timing |
| **Sensor Feedback** | 10 Hz | Draw-wire sensor updates |
| **Modbus Commands** | ≤2 Hz | Only when necessary (every 0.5s max) |

### Parameters

```python
CONTROL_RATE = 0.1              # Control loop: 10 Hz
COMMAND_INTERVAL = 1.0          # Modbus throttle (coarse): max 1 Hz
POSITION_TOLERANCE = 2.0        # Target reached within ±2mm
CHANGE_THRESHOLD = 0.5          # Command sent if target changes >0.5mm
MAX_STEP = 10.0                 # Movement limited to ±10mm per command
STOPPING_DISTANCE = 2.5         # Pre-stop distance for predictive stopping
```

### Behavior

Modbus commands are sent **only** when:
1. ✅ Sufficient time passed (≥0.2-1.0s since last command, depending on distance)
2. ✅ Target changed significantly (>0.5mm) **OR**
3. ✅ Position error is large (>2mm tolerance)

**Benefits:**
- 🚀 Smooth, responsive motion (10 Hz control)
- ⚡ No Modbus delay accumulation
- 📉 Reduced bus traffic (~80% fewer commands)
- 🎯 Precise positioning (±2mm tolerance)

## Relay Mapping

- Relay 0: stop
- Relay 1: up
- Relay 2: down

Each command sends a relay pulse: ON → 100ms delay → OFF.