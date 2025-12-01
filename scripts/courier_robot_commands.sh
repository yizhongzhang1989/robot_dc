#!/bin/bash
# Courier Robot Command Line Quick Reference
# One-line commands to call courier_robot.py functions and get JSON output

# ==================== Status Query ====================

# Get complete status
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.get_status(), indent=2))"

# Get sensor data
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.get_sensor_data(), indent=2))"

# ==================== Platform Height Control ====================

# Move to 900mm (blocking, wait for completion)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_goto_height(900), indent=2))"

# Move to 900mm (non-blocking mode)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_goto_height(900, wait=False), indent=2))"

# Move to 900mm (custom timeout 30 seconds)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_goto_height(900, timeout=30), indent=2))"

# ==================== Platform Force Control ====================

# Force control up to 50N (blocking)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_force_up(50), indent=2))"

# Force control down to 30N (blocking)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_force_down(30), indent=2))"

# Force control up to 50N (non-blocking)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_force_up(50, wait=False), indent=2))"

# ==================== Platform Hybrid Control ====================

# Hybrid control: 900mm OR 50N (whichever reached first)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_hybrid_control(900, 50), indent=2))"

# Hybrid control (non-blocking)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_hybrid_control(900, 50, wait=False), indent=2))"

# ==================== Platform Manual Control ====================

# Manual up
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_up(), indent=2))"

# Manual down
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_down(), indent=2))"

# Stop platform
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_stop(), indent=2))"

# ==================== Pushrod Control ====================

# Pushrod move to absolute position 100mm (blocking)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_goto_height(100, mode='absolute'), indent=2))"

# Pushrod relative move +10mm (blocking)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_goto_height(10, mode='relative'), indent=2))"

# Pushrod relative move -5mm (blocking)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_goto_height(-5, mode='relative'), indent=2))"

# Pushrod move to absolute position 100mm (non-blocking)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_goto_height(100, mode='absolute', wait=False), indent=2))"

# Pushrod manual up
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_up(), indent=2))"

# Pushrod manual down
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_down(), indent=2))"

# Stop pushrod
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_stop(), indent=2))"

# ==================== Emergency Reset ====================

# Emergency reset (stop all movements)
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.emergency_reset(), indent=2))"

# ==================== Usage Examples ====================

# Save result to variable
# result=$(python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.get_status()))")
# echo $result | jq '.platform.task_state'

# Check if command succeeded
# if python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); result = r.platform_goto_height(900); exit(0 if result.get('success') else 1)"; then
#     echo "Success"
# else
#     echo "Failed"
# fi
