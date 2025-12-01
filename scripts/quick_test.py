#!/usr/bin/env python3
"""
CourierRobot 快速测试脚本
注释掉不需要的测试，只运行需要的部分
"""

from courier_robot import CourierRobot
import time

# 初始化
robot = CourierRobot()

# ==================== 测试区域 ====================
# 取消注释需要测试的命令

# 1. 查询状态
# robot.get_status()

# 2. 查询传感器
# robot.get_sensor_data()

# 3. goto 900mm
# robot.platform_goto_height(900)
# robot.wait_for_completion()

# 4. goto 850mm
# robot.platform_goto_height(850)
# robot.wait_for_completion()

# 5. 手动上升
# robot.platform_up()

# 6. 手动下降
# robot.platform_down()

# 7. 停止
# robot.platform_stop()
# robot.wait_for_completion()

# 8. 力控上升到 50N
# robot.platform_force_up(50.0)
# robot.wait_for_completion()

# 9. 力控下降到 30N
# robot.platform_force_down(30.0)
# robot.wait_for_completion()

# 10. 混合控制（高度 900mm，力 50N）
# robot.platform_hybrid_control(900, 50.0)
# robot.wait_for_completion()

# 11. 推杆 goto 100mm
# robot.pushrod_goto_height(100)
# robot.wait_for_completion(target='pushrod')

# 12. 推杆手动上升
# robot.pushrod_up()

# 13. 推杆停止
# robot.pushrod_stop()
# robot.wait_for_completion(target='pushrod')

# 14. 紧急停止
# robot.emergency_reset()

# 15. 力传感器去皮（右侧）
# robot.tare_force_sensor('right')

# 16. 力传感器去皮（左侧）
# robot.tare_force_sensor('left')

# 17. 两侧力传感器去皮
# robot.tare_both_force_sensors()

# ==================== 连续控制示例 ====================

# 示例A：连续 goto
# robot.platform_goto_height(900)
# robot.wait_for_completion()
# robot.platform_goto_height(850)
# robot.wait_for_completion()
# robot.platform_goto_height(950)
# robot.wait_for_completion()

# 示例B：手动控制 + 停止
# robot.platform_up()
# time.sleep(2)
# robot.platform_stop()
# robot.wait_for_completion()

# 示例C：goto 中途停止
# robot.platform_goto_height(900)
# time.sleep(1)
# robot.platform_stop()
# robot.wait_for_completion()

print("\n✅ 脚本执行完成")
