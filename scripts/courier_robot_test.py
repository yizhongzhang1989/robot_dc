#!/usr/bin/env python3
"""
CourierRobot Quick Test Script
Comment out tests you don't need, only run the ones you want

✅ All control functions now default to blocking mode (auto-wait for completion)
✅ Can call sequentially without manual wait_for_completion()
✅ Non-blocking mode (wait=False) auto-stops old tasks to override commands
"""

from courier_robot import CourierRobot
import time

# Initialize
robot = CourierRobot()

# ==================== Test Area ====================
# Uncomment the commands you want to test

# 1. Query status
# robot.get_status()

# 2. Query sensors
# robot.get_sensor_data()

# ========== Blocking Mode Tests (default wait=True) ==========

# 3. goto 900mm (blocking, auto-wait for completion)
# robot.platform_goto_height(900)

# 4. goto 850mm (blocking, auto-wait for completion)
# robot.platform_goto_height(850)

# 5. Force control up to 50N (blocking)
# robot.platform_force_up(50.0)

# 6. Force control down to 30N (blocking)
# robot.platform_force_down(30.0)

# 7. Hybrid control (height 900mm, force 50N) (blocking)
# robot.platform_hybrid_control(900, 50.0)

# 8. Pushrod goto 100mm (blocking)
# robot.pushrod_goto_height(100)

# ========== Non-blocking Mode Tests (wait=False, auto-override) ==========

# 9. Non-blocking goto 900, then immediately change to 850 (auto-stop old task)
# robot.platform_goto_height(900, wait=False)
# time.sleep(0.5)  # Let it start moving
# robot.platform_goto_height(850, wait=False)  # ⚠️ Will auto-stop, then goto 850

# 10. Non-blocking force control, then immediately change target force (auto-override)
# robot.platform_force_up(50.0, wait=False)
# time.sleep(0.5)
# robot.platform_force_up(30.0, wait=False)  # ⚠️ Auto-stop, change to 30N

# ========== Manual Control (always non-blocking) ==========

# 11. Manual up (non-blocking, requires manual stop)
# robot.platform_up()
# time.sleep(2)
# robot.platform_stop()

# 12. Manual down (non-blocking, requires manual stop)
# robot.platform_down()
# time.sleep(2)
# robot.platform_stop()

# 13. Pushrod manual up (non-blocking)
# robot.pushrod_up()
# time.sleep(2)
# robot.pushrod_stop()

# ========== Emergency Stop ==========

# 14. Emergency stop
# robot.emergency_reset()

print("\n✅ Script execution completed")
