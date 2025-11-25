# Lift Robot Action Control Commands

This document provides command-line examples for controlling the lift robot platform using ROS2 Actions.

## Prerequisites

```bash
# Source the workspace
cd /home/robot/Documents/robot_dc/colcon_ws
source install/setup.bash

# Verify Action servers are running
ros2 action list
# Should show:
# /lift_robot/goto_height
# /lift_robot/force_control
# /lift_robot/hybrid_control
# /lift_robot/manual_move
```

## 1. Height Control (GotoHeight)

Move the platform to a specific height with overshoot compensation.

### Basic Usage

```bash
# Move platform to 1000mm
ros2 action send_goal /lift_robot/goto_height lift_robot_interfaces/action/GotoHeight "{target: 'platform', target_height: 1000.0}" --feedback

```

### Features
- **Automatic overshoot compensation** using polynomial-fitted calibration data
- **Predictive early stop** to account for platform inertia
- **Range limit protection** - automatically stops at min/max limits
- **Completion reasons**: `target_reached`, `limit_reached`, `emergency_reset`, `cancelled`

---

## 2. Force Control (ForceControl)

Move the platform until a target force is reached.

### Upward Movement (Increase Force)

```bash
# Apply 240N force (moving up to compress sensors)
ros2 action send_goal /lift_robot/force_control lift_robot_interfaces/action/ForceControl "{target_force: 240.0, direction: 'up'}" --feedback

# Apply 150N force
ros2 action send_goal /lift_robot/force_control lift_robot_interfaces/action/ForceControl "{target_force: 150.0, direction: 'up'}" --feedback
```

### Downward Movement (Decrease Force)

```bash
# Reduce force to 100N (moving down to release sensors)
ros2 action send_goal /lift_robot/force_control lift_robot_interfaces/action/ForceControl "{target_force: 100.0, direction: 'down'}" --feedback

# Reduce force to 50N
ros2 action send_goal /lift_robot/force_control lift_robot_interfaces/action/ForceControl "{target_force: 50.0, direction: 'down'}" --feedback
```

### Features
- **Dual force sensor support** (left and right sensors combined)
- **Direction-aware thresholds**: up → force ≥ target, down → force ≤ target
- **Range limit protection**
- **Completion reasons**: `target_reached`, `limit_exceeded`, `emergency_reset`, `cancelled`

---

## 3. Hybrid Control (HybridControl)

Move to a target height OR target force - whichever is reached first.

### Basic Usage

```bash
# Move to 1000mm OR 240N (whichever reached first)
ros2 action send_goal /lift_robot/hybrid_control lift_robot_interfaces/action/HybridControl "{target_height: 1000.0, target_force: 240.0}" --feedback

# Move down to 900mm OR 100N
ros2 action send_goal /lift_robot/hybrid_control lift_robot_interfaces/action/HybridControl "{target_height: 900.0, target_force: 100.0}" --feedback
```

### Features
- **OR logic**: Stops when EITHER height OR force target is reached
- **Automatic direction**: Movement direction determined by `height_error = target_height - current_height`
- **Polynomial overshoot compensation** (same as GotoHeight)
- **Predictive early stop** for height dimension
- **Range limit protection**
- **Completion reasons**: `height_reached`, `force_reached`, `both_height_and_force_reached`, `limit_reached`, `emergency_reset`, `cancelled`

### Important Notes

**Direction Determination:**
- Direction is **automatically determined** from height error - no manual specification needed
- Force control direction is determined by **initial** height error and remains fixed throughout execution
- When height is already at target, force control defaults to 'up' direction

**Target Selection Guidelines:**
- **To move upward**: Both `target_height` and `target_force` should be **greater than** current values
  - Example: current = 900mm/150N → target = 1000mm/240N ✅
- **To move downward**: Both `target_height` and `target_force` should be **less than** current values
  - Example: current = 1000mm/240N → target = 900mm/100N ✅
- **Why?** Moving up compresses force sensors (force increases), moving down releases them (force decreases)
- **Mixed targets** (e.g., height up but force down) will cause the Action to stop at whichever condition is reached first, which may not be the intended behavior


---

## 4. Manual Move (ManualMove)

Continuous movement in a specified direction until cancelled or limit reached.

### Basic Usage

```bash
# Move platform up continuously
ros2 action send_goal /lift_robot/manual_move lift_robot_interfaces/action/ManualMove "{target: 'platform', direction: 'up'}" --feedback

# Move platform down continuously
ros2 action send_goal /lift_robot/manual_move lift_robot_interfaces/action/ManualMove "{target: 'platform', direction: 'down'}" --feedback
```

**Valid directions:** Only `'up'` or `'down'` - **`'stop'` is NOT supported**

### How to Stop

**Method 1: Ctrl+C (Recommended)**
```bash
# Press Ctrl+C in the terminal running ManualMove
^C
```

**Method 2: Send Opposite Direction (Two terminals needed)**
```bash
# Terminal 1: Moving platform up
ros2 action send_goal /lift_robot/manual_move lift_robot_interfaces/action/ManualMove "{target: 'platform', direction: 'up'}" --feedback

# Terminal 2: Stop by sending opposite direction
ros2 action send_goal /lift_robot/manual_move lift_robot_interfaces/action/ManualMove "{target: 'platform', direction: 'down'}" --feedback
```

**Method 3: Emergency Reset (For emergencies)**
```bash
ros2 topic pub --once /lift_robot_platform/command std_msgs/msg/String "{data: '{\"command\": \"reset\"}'}"
```

**Why no `direction: 'stop'`?**
- `ManualMove` is designed for continuous motion until cancelled
- Stopping is achieved through **cancellation mechanisms** (Ctrl+C, new goal, or emergency reset)
- This follows ROS2 Action pattern where running Actions are cancelled, not stopped via parameters

### Features
- **Continuous movement** until explicitly cancelled
- **Automatic stop at range limits** (returns success)
- **Completion reasons**: `limit_reached`, `cancelled`, `emergency_reset`

---

## Action Interruption and Cancellation

### Mutual Exclusion Rules

The platform implements strict **mutual exclusion** - only one Action can execute at a time.

**Same-type Actions can interrupt each other:**
- New `ManualMove` → cancels previous `ManualMove`
- New `GotoHeight` → cancels previous `GotoHeight`
- New `ForceControl` → cancels previous `ForceControl`
- New `HybridControl` → cancels previous `HybridControl`

**Different-type Actions are rejected:**
- `GotoHeight` running → new `ForceControl` is **REJECTED**
- `ManualMove` running → new `HybridControl` is **REJECTED**

### Cancellation Methods

| Method | Scope | Speed | Side Effects | Use Case |
|--------|-------|-------|--------------|----------|
| **Ctrl+C** | Single Action | ~20ms | None | Quick manual stop |
| **Same-type Action** | Replaces goal | ~100ms | None | Change target mid-execution |
| **Emergency Reset** | All Actions | <20ms | Blocks future Actions | Safety emergency |


---

## Action Status Monitoring

### Check Platform Status

```bash
# Real-time status updates
ros2 topic echo /lift_robot_platform/status

# Example output:
# {
#   "current_height": 1050.23,
#   "movement_state": "stopped",
#   "task_state": "completed",
#   "task_type": "goto_height",
#   "completion_reason": "target_reached"
# }
```

### List Available Actions

```bash
ros2 action list
ros2 action list -t  # Show with types
```

### View Action Interface Definitions

```bash
ros2 interface show lift_robot_interfaces/action/GotoHeight
ros2 interface show lift_robot_interfaces/action/ForceControl
ros2 interface show lift_robot_interfaces/action/HybridControl
ros2 interface show lift_robot_interfaces/action/ManualMove
```

### Monitor Action Feedback (Real-time)

```bash
# GotoHeight feedback
ros2 topic echo /lift_robot/goto_height/_action/feedback

# Force sensor readings
ros2 topic echo /force_sensor          # Right sensor
ros2 topic echo /force_sensor_2        # Left sensor

# Draw-wire sensor (height)
ros2 topic echo /draw_wire_sensor/data
```

---

## Emergency Stop

Emergency stop is implemented via **topic command**, different from Action cancellation.

### Command

```bash
# Publish topic directly (works even if web server is down)
ros2 topic pub --once /lift_robot_platform/command std_msgs/msg/String "{data: '{\"command\": \"reset\"}'}"

# Via Web API
curl -X POST http://localhost:8090/api/cmd \
  -H "Content-Type: application/json" \
  -d '{"command": "reset"}'

# Web interface: Click 🚨 RESET button
```

**Note:** Emergency reset stops **both platform and pushrod** simultaneously - no need to specify target.

### What Happens

**Execution Flow:**

1. **Web Server** cancels all Actions and publishes reset topic
2. **Platform Node** receives topic:
   - Sets `emergency_reset` flag
   - Waits for Actions to exit (40ms)
   - Stops all timers and motors
3. **Running Actions** detect flag and exit immediately

**Effect:**
- ✅ All Actions immediately aborted (`completion_reason: emergency_reset`)
- ✅ Platform stops within 20ms
- ⚠️ System enters emergency state
- ⚠️ Blocks all future Actions until restart

### Recovery

```bash
# Restart platform node (recommended)
# Press Ctrl+C in robot_bringup terminal, then:
cd /home/robot/Documents/robot_dc/colcon_ws
source install/setup.bash
ros2 launch robot_bringup lift_robot_bringup.py
```

**Note:** `clear_emergency` command may not be implemented yet, restart is the only reliable method.

---

## Range Limits

The platform has configurable range limits to prevent mechanical damage:

```bash
# View current range limits
cat /home/robot/Documents/robot_dc/colcon_ws/config/platform_range.json

# Example:
# {
#   "actual_min": 702.64,
#   "actual_max": 1221.48,
#   "safe_min": 752.64,
#   "safe_max": 1171.48
# }
```

- **Actual limits**: Detected mechanical endpoints (702.64 - 1221.48mm)
- **Safe limits**: Recommended operating range (752.64 - 1171.48mm)
- **Protection margin**: ±1mm from actual limits for emergency stop

---

## Pushrod Control (Integrated into Platform Node)


### 1. Pushrod Manual Control

```bash
# Move pushrod up continuously
ros2 action send_goal /lift_robot/manual_move lift_robot_interfaces/action/ManualMove "{target: 'pushrod', direction: 'up'}" --feedback

# Move pushrod down continuously
ros2 action send_goal /lift_robot/manual_move lift_robot_interfaces/action/ManualMove "{target: 'pushrod', direction: 'down'}" --feedback

# Stop pushrod (Ctrl+C or send opposite direction)
```

### 2. Pushrod Height Control

Pushrod supports two height control modes:

#### Absolute Mode (Default)
```bash
# Move pushrod to absolute position 950mm
ros2 action send_goal /lift_robot/goto_height lift_robot_interfaces/action/GotoHeight "{target: 'pushrod', target_height: 950.0, mode: 'absolute'}" --feedback

# Shorter form (mode defaults to 'absolute')
ros2 action send_goal /lift_robot/goto_height lift_robot_interfaces/action/GotoHeight "{target: 'pushrod', target_height: 950.0}" --feedback
```

#### Relative Mode
```bash
# Move pushrod UP by 10mm from current position
ros2 action send_goal /lift_robot/goto_height lift_robot_interfaces/action/GotoHeight "{target: 'pushrod', target_height: 10.0, mode: 'relative'}" --feedback

# Move pushrod DOWN by 5mm from current position
ros2 action send_goal /lift_robot/goto_height lift_robot_interfaces/action/GotoHeight "{target: 'pushrod', target_height: -5.0, mode: 'relative'}" --feedback
```

**Mode Comparison:**

| Mode | `target_height` Interpretation | Example |
|------|-------------------------------|---------|
| **absolute** | Absolute target position | `950.0` → move to 950mm |
| **relative** | Offset from current position | `+10.0` → move 10mm up<br>`-5.0` → move 5mm down |

**How Relative Mode Works:**
1. Read current height (e.g., 940mm)
2. Add input height: `target = current + input` (e.g., 940 + 10 = 950mm)
3. Move to calculated target (950mm)

**Note:** Pushrod height control does **NOT use overshoot compensation** (unlike platform). It stops immediately when target height is detected, as pushrod has higher precision and minimal inertia.


---


*Last Updated: 2025-11-25*
