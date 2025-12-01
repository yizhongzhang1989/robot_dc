#!/bin/bash
# Courier Robot Command Line Quick Reference
# 一行命令直接调用 courier_robot.py 中的函数并获取 JSON 输出

# ==================== 状态查询 ====================

# 获取完整状态
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.get_status(), indent=2))"

# 获取传感器数据
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.get_sensor_data(), indent=2))"

# ==================== 平台高度控制 ====================

# 移动到 900mm（阻塞等待完成）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_goto_height(900), indent=2))"

# 移动到 900mm（非阻塞模式）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_goto_height(900, wait=False), indent=2))"

# 移动到 900mm（自定义超时 30 秒）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_goto_height(900, timeout=30), indent=2))"

# ==================== 平台力控 ====================

# 向上力控到 50N（阻塞）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_force_up(50), indent=2))"

# 向下力控到 30N（阻塞）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_force_down(30), indent=2))"

# 向上力控到 50N（非阻塞）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_force_up(50, wait=False), indent=2))"

# ==================== 平台混合控制 ====================

# 混合控制：900mm 或 50N（先达到哪个就停止）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_hybrid_control(900, 50), indent=2))"

# 混合控制（非阻塞）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_hybrid_control(900, 50, wait=False), indent=2))"

# ==================== 平台手动控制 ====================

# 手动向上
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_up(), indent=2))"

# 手动向下
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_down(), indent=2))"

# 停止平台
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.platform_stop(), indent=2))"

# ==================== 推杆控制 ====================

# 推杆移动到绝对位置 100mm（阻塞）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_goto_height(100, mode='absolute'), indent=2))"

# 推杆相对移动 +10mm（阻塞）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_goto_height(10, mode='relative'), indent=2))"

# 推杆相对移动 -5mm（阻塞）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_goto_height(-5, mode='relative'), indent=2))"

# 推杆移动到绝对位置 100mm（非阻塞）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_goto_height(100, mode='absolute', wait=False), indent=2))"

# 推杆手动向上
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_up(), indent=2))"

# 推杆手动向下
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_down(), indent=2))"

# 停止推杆
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.pushrod_stop(), indent=2))"

# ==================== 紧急停止 ====================

# 紧急复位（停止所有运动）
python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.emergency_reset(), indent=2))"

# ==================== 使用示例 ====================

# 保存结果到变量
# result=$(python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); print(json.dumps(r.get_status()))")
# echo $result | jq '.platform.task_state'

# 判断命令是否成功
# if python3 -c "from courier_robot import CourierRobotWebAPI; import json; r = CourierRobotWebAPI(verbose=False); result = r.platform_goto_height(900); exit(0 if result.get('success') else 1)"; then
#     echo "Success"
# else
#     echo "Failed"
# fi
