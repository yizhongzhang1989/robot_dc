# CourierRobot Quick Test Commands

All commands should be executed in the `/home/robot/Desktop/robot_dc_test` directory

**Important Note**: All commands now automatically print execution results, no need to manually add `print`!

## 1. Status Query Commands

### View Complete Status (Platform + Sensors)
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.get_status()"
```

### View Height and Force Only
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); s=r.get_status(); print(f\"Height:{s['sensors']['height']}mm, Force:{s['sensors']['combined_force']}N\")"
```

### View Platform Movement Status
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); s=r.get_status(); print(f\"Task State:{s['data']['platform']['task_state']}, Movement State:{s['data']['platform']['movement_state']}\")"
```

## 2. Platform Manual Control Commands

### Platform Up
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_up()"
```

### Platform Down
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_down()"
```

### Platform Stop
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_stop()"
```

## 3. Platform Height Control Commands

### Move to Specific Height (800mm)
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_goto_height(800)"
```

### Move to Height and Wait for Completion (850mm)
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_goto_height(850); r.wait_for_completion('platform', timeout=30)"
```

## 4. Platform Force Control Commands

### Force Control Up to 50N
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_force_up(50)"
```

### Force Control Down to 30N
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_force_down(30)"
```

### Force Control Down to 20N and Wait for Completion
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_force_down(20); r.wait_for_completion('platform', timeout=30)"
```

## 5. Platform Hybrid Control Commands

### Hybrid Control: 800mm OR 40N (whichever is reached first)
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_hybrid_control(800, 40)"
```

### Hybrid Control and Wait for Completion
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_hybrid_control(850, 35); r.wait_for_completion('platform', timeout=30)"
```

## 6. Pushrod Control Commands

### Pushrod Up
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_up()"
```

### Pushrod Down
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_down()"
```

### Pushrod Stop
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_stop()"
```

### Pushrod Absolute Positioning to 750mm
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_goto_height(750, mode='absolute')"
```

### Pushrod Relative Move +10mm
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_goto_height(10, mode='relative')"
```

### Pushrod Relative Move -5mm
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.pushrod_goto_height(-5, mode='relative')"
```

## 7. Force Sensor Calibration Commands

### Tare Right Force Sensor (Zero Calibration)
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.tare_force_sensor('right')"
```

### Tare Left Force Sensor
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.tare_force_sensor('left')"
```

### Tare Both Force Sensors
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.tare_both_force_sensors()"
```

## 8. Emergency Reset Command

### Emergency Stop and Reset All States
```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.emergency_reset()"
```

## 9. Combined Test Commands (Simplified)

### Complete Flow Test: Move to Target Height and Verify
```bash
python3 -c "
from scripts.courier_robot import CourierRobot
r = CourierRobot()
r.platform_goto_height(820)
r.wait_for_completion('platform', timeout=30)
"
```

### Test Force Control Accuracy
```bash
python3 -c "
from scripts.courier_robot import CourierRobot
r = CourierRobot()
r.platform_force_down(25)
r.wait_for_completion('platform', timeout=30)
"
```

### Continuous Sensor Monitoring (10 times, 1 second interval)
```bash
python3 -c "
from scripts.courier_robot import CourierRobot
import time
r = CourierRobot()
for i in range(10):
    s = r.get_status()
    print(f'{i+1}. Height:{s[\"sensors\"][\"height\"]:.2f}mm, Force:{s[\"sensors\"][\"combined_force\"]:.2f}N')
    time.sleep(1)
"
```

### Test State Checking Mechanism
```bash
python3 -c "
from scripts.courier_robot import CourierRobot
r = CourierRobot()
r.platform_goto_height(800)  # First command
r.platform_goto_height(900)  # Second command (will be rejected)
r.wait_for_completion('platform', timeout=30)
r.platform_goto_height(900)  # Third command (success)
"
```

## 10. Silent Mode (No Auto-Output)

If you don't want to see automatic output, set `verbose=False`:

```bash
python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(verbose=False); r.platform_goto_height(800)"
```

## 11. Common Command Aliases

You can add the following aliases to `~/.bashrc`:

```bash
# Add to ~/.bashrc
alias robot_status='cd /home/robot/Desktop/robot_dc_test && python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.get_status()"'

alias robot_up='cd /home/robot/Desktop/robot_dc_test && python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_up()"'

alias robot_down='cd /home/robot/Desktop/robot_dc_test && python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_down()"'

alias robot_stop='cd /home/robot/Desktop/robot_dc_test && python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_stop()"'

alias robot_reset='cd /home/robot/Desktop/robot_dc_test && python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.emergency_reset()"'

alias robot_tare='cd /home/robot/Desktop/robot_dc_test && python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.tare_both_force_sensors()"'
```

Then execute:
```bash
source ~/.bashrc
```

Use aliases (can be executed from any directory):
```bash
robot_status          # View status
robot_up              # Platform up
robot_down            # Platform down
robot_stop            # Platform stop
robot_reset           # Emergency reset
robot_tare            # Tare both force sensors
```

## 12. Interactive Python Usage

If you need multiple operations, it's recommended to enter Python interactive mode:

```bash
cd /home/robot/Desktop/robot_dc_test
python3
```

```python
from scripts.courier_robot import CourierRobot
r = CourierRobot()

# All commands automatically display results
r.get_status()
r.platform_goto_height(800)
r.wait_for_completion('platform', timeout=30)

# If you don't want auto-output
r_quiet = CourierRobot(verbose=False)
result = r_quiet.platform_goto_height(850)
print(result)  # Manually print the info you need
```

## Notes

1. **Auto-output**: Default `verbose=True`, all operations will automatically print friendly status information

2. **Silent Mode**: Set `verbose=False` to disable auto-output, suitable for use in scripts

3. **State Checking**: Except for `stop` and `emergency_reset`, all motion commands check the state and will only execute when in `idle` or `completed` state

4. **Return Values**: All commands return a complete status dictionary containing a `status` field, which can be used for programmatic decisions

5. **base_url**: Default is `http://192.168.1.3:8090`, to modify:
   ```bash
   python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(base_url='http://localhost:8090'); r.platform_up()"
   ```

6. **Timeout Settings**: `wait_for_completion()` default timeout is 60 seconds, customizable:
   ```bash
   python3 -c "from scripts.courier_robot import CourierRobot; r=CourierRobot(); r.platform_goto_height(800); r.wait_for_completion('platform', timeout=45)"
   ```
