import requests
import json
from math import pi

# 机器人臂Web服务器地址
BASE_URL = "http://localhost:8080"

# Move to Target Joint Position (关节角度位置移动)
def move_to_target_joint_position():
    """
    网页按钮: Move to Target Angle
    对应 confirmMoveToTarget() 函数
    url_moveToTargetJoint = "http://localhost:8080/api/robot_arm/cmd"
    params_moveToTargetJoint = {"command": "servoj [0.000000,-1.570796,1.570796,0.000000,1.570796,0.000000] 1.0 1.0 False 200 65"}
    headers = {'Content-Type': 'application/json'}
    """
    # 目标关节角度 (度)
    # target_joint_angles_deg = [0,0,0,0,0,0]
    target_joint_angles_deg = [-47.34, 6.09, -103.28, 11.85, 88.99, 40.72]
    
    # 转换为弧度
    target_joint_angles_rad = [angle * pi / 180 for angle in target_joint_angles_deg]
    
    # 生成 servoj 命令
    joint_angles_str = ','.join([f"{angle:.6f}" for angle in target_joint_angles_rad])
    servoj_command = f"servoj [{joint_angles_str}] 0.5 0.5 False 200 65"
    
    # POST 请求设置
    url_moveToTargetJoint = f"{BASE_URL}/api/robot_arm/cmd"
    params_moveToTargetJoint = {
        "command": servoj_command
    }
    headers = {'Content-Type': 'application/json'}
    
    try:
        response = requests.post(url_moveToTargetJoint, json=params_moveToTargetJoint, headers=headers)
        if response.ok:
            result = response.json()
            print(f"✅ Move to target joint position success: {result}")
            print(f"Target angles (deg): {target_joint_angles_deg}")
        else:
            print(f"❌ Failed to move to target joint position: {response.status_text}")
    except Exception as e:
        print(f"❌ Error sending move to target joint command: {e}")

# Move to Target TCP Position (TCP位置移动)
def move_to_target_tcp_position():
    """
    网页按钮: Move to Target Position
    对应 confirmMoveToTargetTcp() 函数
    注意: 这个需要先获取当前TCP位置来计算偏移量
    url_moveToTargetTcp = "http://localhost:8080/api/robot_arm/cmd"
    params_moveToTargetTcp = {"command": "servo_tcp [0.100000,-0.050000,0.050000,-3.141593,0.000000,0.000000] 1.0 1.0 \"\" False 150 35"}
    headers = {'Content-Type': 'application/json'}
    """
    # 目标TCP位置 (mm 和 度)
    target_tcp_position = {
        'x': 500.0,    # mm
        'y': 200.0,    # mm  
        'z': 400.0,    # mm
        'rx': 180.0,   # 度
        'ry': 0.0,     # 度
        'rz': 0.0      # 度
    }
    
    # 首先获取当前机器人状态来计算偏移量
    # 注意: 实际使用时需要先获取当前TCP位置
    # 这里为示例使用假设的当前位置
    current_tcp_position = [0.4, 0.15, 0.35, pi, 0.0, 0.0]  # [x(m), y(m), z(m), rx(rad), ry(rad), rz(rad)]
    
    # 计算偏移量 (使用网页中的特殊偏移计算逻辑)
    current_x_mm = current_tcp_position[0] * 1000
    current_y_mm = current_tcp_position[1] * 1000  
    current_z_mm = current_tcp_position[2] * 1000
    current_rx_deg = current_tcp_position[3] * 180 / pi
    current_ry_deg = current_tcp_position[4] * 180 / pi
    current_rz_deg = current_tcp_position[5] * 180 / pi
    
    # 特殊偏移计算逻辑: 正值用 -(target - current), 负值用 (target - current)
    if target_tcp_position['x'] >= 0:
        offset_x = -(target_tcp_position['x'] - current_x_mm) / 1000
    else:
        offset_x = (target_tcp_position['x'] - current_x_mm) / 1000
        
    if target_tcp_position['y'] >= 0:
        offset_y = -(target_tcp_position['y'] - current_y_mm) / 1000
    else:
        offset_y = (target_tcp_position['y'] - current_y_mm) / 1000
        
    if target_tcp_position['z'] >= 0:
        offset_z = -(target_tcp_position['z'] - current_z_mm) / 1000
    else:
        offset_z = (target_tcp_position['z'] - current_z_mm) / 1000
    
    # 旋转偏移计算
    if target_tcp_position['rx'] >= 0:
        offset_rx = -(target_tcp_position['rx'] - current_rx_deg) * pi / 180
    else:
        offset_rx = (target_tcp_position['rx'] - current_rx_deg) * pi / 180
        
    if target_tcp_position['ry'] >= 0:
        offset_ry = -(target_tcp_position['ry'] - current_ry_deg) * pi / 180
    else:
        offset_ry = (target_tcp_position['ry'] - current_ry_deg) * pi / 180
        
    if target_tcp_position['rz'] >= 0:
        offset_rz = -(target_tcp_position['rz'] - current_rz_deg) * pi / 180
    else:
        offset_rz = (target_tcp_position['rz'] - current_rz_deg) * pi / 180
    
    # 生成 servo_tcp 命令
    servo_tcp_command = f"servo_tcp [{offset_x:.6f},{offset_y:.6f},{offset_z:.6f},{offset_rx:.6f},{offset_ry:.6f},{offset_rz:.6f}] 0.5 0.5 \"\" False 150 35"
    
    # POST 请求设置
    url_moveToTargetTcp = f"{BASE_URL}/api/robot_arm/cmd"
    params_moveToTargetTcp = {
        "command": servo_tcp_command
    }
    headers = {'Content-Type': 'application/json'}
    
    try:
        response = requests.post(url_moveToTargetTcp, json=params_moveToTargetTcp, headers=headers)
        if response.ok:
            result = response.json()
            print(f"✅ Move to target TCP position success: {result}")
            print(f"Target TCP: X={target_tcp_position['x']}mm, Y={target_tcp_position['y']}mm, Z={target_tcp_position['z']}mm")
            print(f"Target orientation: Rx={target_tcp_position['rx']}°, Ry={target_tcp_position['ry']}°, Rz={target_tcp_position['rz']}°")
        else:
            print(f"❌ Failed to move to target TCP position: {response.status_text}")
    except Exception as e:
        print(f"❌ Error sending move to target TCP command: {e}")

# 增量移动关节 (对应网页的 adjustJoint 功能)
def adjust_joint_position():
    """
    网页按钮: Joint +/- 按钮
    对应 adjustJoint() 和 sendServoJCommand() 函数
    url_adjustJoint = "http://localhost:8080/api/robot_arm/cmd"
    params_adjustJoint = {"command": "servoj [0.087266,0.000000,0.000000,0.000000,0.000000,0.000000] 1.0 1.0 False 200 65"}
    headers = {'Content-Type': 'application/json'}
    """
    # 当前关节角度 (度) - 实际使用时应该从机器人状态获取
    current_joint_angles_deg = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    # 调整第1个关节增加5度
    joint_index = 1  # 关节1 (1-6)
    adjustment_deg = 5.0  # 增加5度
    
    current_joint_angles_deg[joint_index - 1] += adjustment_deg
    
    # 转换为弧度
    current_joint_angles_rad = [angle * pi / 180 for angle in current_joint_angles_deg]
    
    # 生成 servoj 命令
    joint_angles_str = ','.join([f"{angle:.6f}" for angle in current_joint_angles_rad])
    servoj_command = f"servoj [{joint_angles_str}] 1.0 1.0 False 200 65"
    
    # POST 请求设置
    url_adjustJoint = f"{BASE_URL}/api/robot_arm/cmd"
    params_adjustJoint = {
        "command": servoj_command
    }
    headers = {'Content-Type': 'application/json'}
    
    try:
        response = requests.post(url_adjustJoint, json=params_adjustJoint, headers=headers)
        if response.ok:
            result = response.json()
            print(f"✅ Adjust joint {joint_index} success: {result}")
            print(f"Joint {joint_index} adjusted by {adjustment_deg}° to {current_joint_angles_deg[joint_index-1]}°")
        else:
            print(f"❌ Failed to adjust joint: {response.status_text}")
    except Exception as e:
        print(f"❌ Error sending adjust joint command: {e}")

# 增量移动TCP (对应网页的 adjustTcp 功能)
def adjust_tcp_position():
    """
    网页按钮: TCP +/- 按钮  
    对应 adjustTcp() 和 sendServoTcpCommandIncremental() 函数
    url_adjustTcp = "http://localhost:8080/api/robot_arm/cmd"
    params_adjustTcp = {"command": "servo_tcp [0.005000,0.000000,0.000000,0.000000,0.000000,0.000000] 1.0 1.0 \"\" False 150 35"}
    headers = {'Content-Type': 'application/json'}
    """
    # TCP轴和方向
    axis = 'x'  # 'x', 'y', 'z', 'rx', 'ry', 'rz'
    direction = 1  # 1 为正方向, -1 为负方向
    
    # 步长设置
    position_step_size = 5.0  # mm
    rotation_step_size = 2.0  # 度
    
    # 创建偏移量数组
    pose_offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    
    if axis in ['x', 'y', 'z']:
        # 位置移动 - 转换mm到米
        axis_index = {'x': 0, 'y': 1, 'z': 2}[axis]
        pose_offset[axis_index] = direction * position_step_size / 1000  # 转换mm到米
    else:
        # 旋转移动 - 转换度到弧度  
        axis_index = {'rx': 3, 'ry': 4, 'rz': 5}[axis]
        pose_offset[axis_index] = direction * rotation_step_size * pi / 180  # 转换度到弧度
    
    # 生成 servo_tcp 命令
    pose_offset_str = ','.join([f"{offset:.6f}" for offset in pose_offset])
    servo_tcp_command = f"servo_tcp [{pose_offset_str}] 1.0 1.0 \"\" False 150 35"
    
    # POST 请求设置
    url_adjustTcp = f"{BASE_URL}/api/robot_arm/cmd"
    params_adjustTcp = {
        "command": servo_tcp_command
    }
    headers = {'Content-Type': 'application/json'}
    
    try:
        response = requests.post(url_adjustTcp, json=params_adjustTcp, headers=headers)
        if response.ok:
            result = response.json()
            print(f"✅ Adjust TCP {axis} success: {result}")
            if axis in ['x', 'y', 'z']:
                print(f"TCP {axis.upper()} adjusted by {direction * position_step_size}mm")
            else:
                print(f"TCP {axis.upper()} adjusted by {direction * rotation_step_size}°")
        else:
            print(f"❌ Failed to adjust TCP: {response.status_text}")
    except Exception as e:
        print(f"❌ Error sending adjust TCP command: {e}")

# 基本机器人控制命令
def robot_basic_commands():
    """
    网页基本控制按钮: Power On/Off, Enable/Disable
    对应 sendCommand() 函数
    url_basicCmd = "http://localhost:8080/api/robot_arm/cmd"
    params_basicCmd = {"command": "power_on"}  # 或 "power_off", "enable", "disable"
    headers = {'Content-Type': 'application/json'}
    """
    commands = ['power_on', 'power_off', 'enable', 'disable']
    
    for cmd in commands:
        # POST 请求设置
        url_basicCmd = f"{BASE_URL}/api/robot_arm/cmd"
        params_basicCmd = {
            "command": cmd
        }
        headers = {'Content-Type': 'application/json'}
        
        try:
            response = requests.post(url_basicCmd, json=params_basicCmd, headers=headers)
            if response.ok:
                result = response.json()
                print(f"✅ {cmd} command success: {result}")
            else:
                print(f"❌ Failed to send {cmd} command: {response.status_text}")
        except Exception as e:
            print(f"❌ Error sending {cmd} command: {e}")

if __name__ == "__main__":
    print("=== 机器人移动控制 POST 请求示例 ===\n")
    
    # 示例1: 移动到目标关节位置
    print("1. Move to Target Joint Position:")
    move_to_target_joint_position()
    print()
    
    # 示例2: 移动到目标TCP位置  
    print("2. Move to Target TCP Position:")
    move_to_target_tcp_position()
    print()
    
    # 示例3: 增量调整关节
    print("3. Adjust Joint Position:")
    adjust_joint_position()
    print()
    
    # 示例4: 增量调整TCP
    print("4. Adjust TCP Position:")
    adjust_tcp_position()
    print()
    
    # 示例5: 基本机器人命令
    print("5. Basic Robot Commands:")
    # robot_basic_commands()  # 注释掉避免意外执行
    print("(Commented out for safety)")
