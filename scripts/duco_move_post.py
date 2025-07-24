import requests
import json
from math import pi
import time

# 机器人臂Web服务器地址
BASE_URL = "http://localhost:8080"

# 新增函数：获取实际关节角度和TCP位置
def get_actual_joint_angles_and_tcp_position():
    """
    通过HTTP POST获取机器人实际关节角度和TCP位置
    
    对应网页API: GET /api/robot_arm/state
    返回机器人当前实际状态，包括：
    - jointActualPosition: 实际关节角度 (弧度)
    - TCPActualPosition: 实际TCP位置 [X, Y, Z, Rx, Ry, Rz] (米, 弧度)
    
    Returns:
        dict: 包含关节角度和TCP位置的字典，如果失败返回None
        {
            'joint_angles_deg': [j1, j2, j3, j4, j5, j6],  # 关节角度(度)
            'joint_angles_rad': [j1, j2, j3, j4, j5, j6],  # 关节角度(弧度)
            'tcp_position_mm': [x, y, z, rx, ry, rz],      # TCP位置(毫米, 度)
            'tcp_position_raw': [x, y, z, rx, ry, rz],     # TCP位置(米, 弧度)
            'success': True/False
        }
    """
    # GET 请求获取机器人状态
    url_getRobotState = f"{BASE_URL}/api/robot_arm/state"
    headers = {'Content-Type': 'application/json'}
    
    try:
        response = requests.get(url_getRobotState, headers=headers)
        
        if response.ok:
            robot_state = response.json()
            
            # 提取关节实际位置 (弧度)
            joint_angles_rad = robot_state.get('jointActualPosition', [])
            if not joint_angles_rad or len(joint_angles_rad) < 6:
                print("❌ 无法获取关节实际位置数据")
                return {
                    'joint_angles_deg': [],
                    'joint_angles_rad': [],
                    'tcp_position_mm': [],
                    'tcp_position_raw': [],
                    'success': False,
                    'error': '关节位置数据无效'
                }
            
            # 提取TCP实际位置 (米, 弧度)
            tcp_position_raw = robot_state.get('TCPActualPosition', [])
            if not tcp_position_raw or len(tcp_position_raw) < 6:
                print("❌ 无法获取TCP实际位置数据")
                return {
                    'joint_angles_deg': [],
                    'joint_angles_rad': [],
                    'tcp_position_mm': [],
                    'tcp_position_raw': [],
                    'success': False,
                    'error': 'TCP位置数据无效'
                }
            
            # 只取前6个关节 (第7个关节是保留的)
            joint_angles_rad_6dof = joint_angles_rad[:6]
            
            # 转换关节角度：弧度 -> 度
            joint_angles_deg = [angle * 180 / pi for angle in joint_angles_rad_6dof]
            
            # 转换TCP位置：米 -> 毫米，弧度 -> 度
            tcp_position_mm = [
                tcp_position_raw[0] * 1000,  # X: 米 -> 毫米
                tcp_position_raw[1] * 1000,  # Y: 米 -> 毫米  
                tcp_position_raw[2] * 1000,  # Z: 米 -> 毫米
                tcp_position_raw[3] * 180 / pi,  # Rx: 弧度 -> 度
                tcp_position_raw[4] * 180 / pi,  # Ry: 弧度 -> 度
                tcp_position_raw[5] * 180 / pi,  # Rz: 弧度 -> 度
            ]
            
            result = {
                'joint_angles_deg': joint_angles_deg,
                'joint_angles_rad': joint_angles_rad_6dof,
                'tcp_position_mm': tcp_position_mm,
                'tcp_position_raw': tcp_position_raw,
                'success': True
            }
            
            # # 打印结果
            # print("✅ 成功获取机器人实际状态:")
            # print(f"关节角度 (度): {[f'{angle:.2f}' for angle in joint_angles_deg]}")
            # print(f"关节角度 (弧度): {[f'{angle:.4f}' for angle in joint_angles_rad_6dof]}")
            # print(f"TCP位置 (mm, °): X={tcp_position_mm[0]:.1f}, Y={tcp_position_mm[1]:.1f}, Z={tcp_position_mm[2]:.1f}")
            # print(f"TCP姿态 (°): Rx={tcp_position_mm[3]:.2f}, Ry={tcp_position_mm[4]:.2f}, Rz={tcp_position_mm[5]:.2f}")
            # print(f"TCP位置 (m, rad): X={tcp_position_raw[0]:.4f}, Y={tcp_position_raw[1]:.4f}, Z={tcp_position_raw[2]:.4f}")
            # print(f"TCP姿态 (rad): Rx={tcp_position_raw[3]:.4f}, Ry={tcp_position_raw[4]:.4f}, Rz={tcp_position_raw[5]:.4f}")
            
            return joint_angles_deg,tcp_position_mm
            
        else:
            print(f"❌ 获取机器人状态失败: HTTP {response.status_code}")
            print(f"响应内容: {response.text}")
            return {
                'joint_angles_deg': [],
                'joint_angles_rad': [],
                'tcp_position_mm': [],
                'tcp_position_raw': [],
                'success': False,
                'error': f'HTTP {response.status_code}: {response.text}'
            }
            
    except requests.exceptions.ConnectionError:
        print("❌ 无法连接到机器人Web服务器，请确保服务器正在运行")
        return {
            'joint_angles_deg': [],
            'joint_angles_rad': [],
            'tcp_position_mm': [],
            'tcp_position_raw': [],
            'success': False,
            'error': '无法连接到Web服务器'
        }
    except Exception as e:
        print(f"❌ 获取机器人状态时发生错误: {e}")
        return {
            'joint_angles_deg': [],
            'joint_angles_rad': [],
            'tcp_position_mm': [],
            'tcp_position_raw': [],
            'success': False,
            'error': str(e)
        }

# Move to Target Joint Position (关节角度位置移动)
def move_to_target_joint_position(target_angles_deg):
    """
    网页按钮: Move to Target Angle
    对应 confirmMoveToTarget() 函数
    url_moveToTargetJoint = "http://localhost:8080/api/robot_arm/cmd"
    params_moveToTargetJoint = {"command": "servoj [0.000000,-1.570796,1.570796,0.000000,1.570796,0.000000] 0.5 0.5 False 150 35"}
    headers = {'Content-Type': 'application/json'}
    """
    # 目标关节角度 (度)
    # target_joint_angles_deg = [0,0,0,0,0,0]
    # target_joint_angles_deg = [-47.34, 6.09, -103.28, 11.85, 88.99, 40.72]
    target_joint_angles_deg = target_angles_deg  # 传入的目标角度
    
    # 转换为弧度
    target_joint_angles_rad = [angle * pi / 180 for angle in target_joint_angles_deg]
    
    # 生成 servoj 命令
    joint_angles_str = ','.join([f"{angle:.6f}" for angle in target_joint_angles_rad])
    servoj_command = f"servoj [{joint_angles_str}] 0.5 0.5 False 150 35"
    
    # POST 请求设置
    url_moveToTargetJoint = f"{BASE_URL}/api/robot_arm/cmd"
    params_moveToTargetJoint = {
        "command": servoj_command
    }
    headers = {'Content-Type': 'application/json'}
    
    try:
        response = requests.post(url_moveToTargetJoint, json=params_moveToTargetJoint, headers=headers)
        time.sleep(2)  # 等待2秒以确保命令发送成功
        if response.ok:
            result = response.json()
            print(f"✅ Move to target joint position success: {result}")
            # print(f"Target angles (deg): {target_joint_angles_deg}")
        else:
            print(f"❌ Failed to move to target joint position: {response.status_text}")
    except Exception as e:
        print(f"❌ Error sending move to target joint command: {e}")

# Move to Target TCP Position (TCP位置移动)
def move_to_target_tcp_position(target_position_mm):
    """
    网页按钮: Move to Target Position
    对应 confirmMoveToTargetTcp() 函数
    注意: 这个需要先获取当前TCP位置来计算偏移量
    url_moveToTargetTcp = "http://localhost:8080/api/robot_arm/cmd"
    params_moveToTargetTcp = {"command": "servo_tcp [0.100000,-0.050000,0.050000,-3.141593,0.000000,0.000000] 0.5 0.5 \"\" False 150 35"}
    headers = {'Content-Type': 'application/json'}
    """
    # 目标TCP位置 (mm 和 度)
    target_tcp_position = {
        'x': target_position_mm[0],    # mm
        'y': target_position_mm[1],    # mm  
        'z': target_position_mm[2],    # mm
        'rx': target_position_mm[3],   # 度
        'ry': target_position_mm[4],     # 度
        'rz': target_position_mm[5]      # 度
    }
    
    # 首先获取当前机器人状态来计算偏移量
    current_tcp_position = get_actual_joint_angles_and_tcp_position()[1]

    # 计算偏移量 (使用网页中的特殊偏移计算逻辑)
    current_x_mm = current_tcp_position[0]
    current_y_mm = current_tcp_position[1]
    current_z_mm = current_tcp_position[2]
    current_rx_deg = current_tcp_position[3]
    current_ry_deg = current_tcp_position[4]
    current_rz_deg = current_tcp_position[5]
    
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
        time.sleep(2)  # 等待2秒以确保命令发送成功
        if response.ok:
            result = response.json()
            print(f"✅ Move to target TCP position success: {result}")
            # print(f"Target TCP: X={target_tcp_position['x']}mm, Y={target_tcp_position['y']}mm, Z={target_tcp_position['z']}mm")
            # print(f"Target orientation: Rx={target_tcp_position['rx']}°, Ry={target_tcp_position['ry']}°, Rz={target_tcp_position['rz']}°")
        else:
            print(f"❌ Failed to move to target TCP position: {response.status_text}")
    except Exception as e:
        print(f"❌ Error sending move to target TCP command: {e}")


# Movej2 Position (关节角度定位移动)
def movej2(target_angles_deg, velocity=1.0, acceleration=1.0, radius=1.0, block=True):
    """
    网页按钮: movej2
    对应 confirmMovej2() 函数
    使用movej2指令进行关节角度定位移动
    url_movej2 = "http://localhost:8080/api/robot_arm/cmd"
    params_movej2 = {"command": "movej2 [0.000000,-1.570796,1.570796,0.000000,1.570796,0.000000] 1.0 1.0 1.0 True"}
    headers = {'Content-Type': 'application/json'}
    """
    # 检查输入参数
    if not isinstance(target_angles_deg, list) or len(target_angles_deg) != 6:
        print("❌ Error: target_angles_deg must be a list of 6 joint angles")
        return
    
    # 转换关节角度：度 -> 弧度
    target_joint_angles_rad = [angle * pi / 180 for angle in target_angles_deg]
    
    # 生成 movej2 命令
    joint_angles_str = ','.join([f"{angle:.6f}" for angle in target_joint_angles_rad])
    movej2_command = f"movej2 [{joint_angles_str}] {velocity} {acceleration} {radius} {block}"
    
    # POST 请求设置
    url_movej2 = f"{BASE_URL}/api/robot_arm/cmd"
    params_movej2 = {
        "command": movej2_command
    }
    headers = {'Content-Type': 'application/json'}
    
    try:
        response = requests.post(url_movej2, json=params_movej2, headers=headers)
        time.sleep(2)  # 等待2秒以确保命令发送成功
        if response.ok:
            result = response.json()
            print(f"✅ Movej2 command success: {result}")
            print(f"Target joint angles (deg): {target_angles_deg}")
            print(f"Command sent: {movej2_command}")
        else:
            print(f"❌ Failed to send movej2 command: {response.status_text}")
    except Exception as e:
        print(f"❌ Error sending movej2 command: {e}")


# TCP Move (TCP坐标系下的偏移移动)
def tcp_move(offset_mm, velocity=0.3, acceleration=0.2, radius=0.0, tool="", block=True):
    """
    网页按钮: Move TCP
    对应 confirmMoveTcp() 函数
    使用tcp_move指令进行TCP坐标系下的偏移移动
    url_tcp_move = "http://localhost:8080/api/robot_arm/cmd"
    params_tcp_move = {"command": "tcp_move [0.100000,-0.050000,0.050000,-0.174533,0.087266,0.000000] 0.3 0.2 0.0 \"\" True"}
    headers = {'Content-Type': 'application/json'}
    """
    # 检查输入参数
    if not isinstance(offset_mm, list) or len(offset_mm) != 6:
        print("❌ Error: offset_mm must be a list of 6 values [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]")
        return
    
    # 提取偏移量并转换单位
    offset_x_m = offset_mm[0] / 1000.0      # mm -> m
    offset_y_m = offset_mm[1] / 1000.0      # mm -> m
    offset_z_m = offset_mm[2] / 1000.0      # mm -> m
    offset_rx_rad = offset_mm[3] * pi / 180  # deg -> rad
    offset_ry_rad = offset_mm[4] * pi / 180  # deg -> rad
    offset_rz_rad = offset_mm[5] * pi / 180  # deg -> rad
    
    # 生成 tcp_move 命令
    pose_offset_str = ','.join([
        f"{offset_x_m:.6f}",
        f"{offset_y_m:.6f}",
        f"{offset_z_m:.6f}",
        f"{offset_rx_rad:.6f}",
        f"{offset_ry_rad:.6f}",
        f"{offset_rz_rad:.6f}"
    ])
    tcp_move_command = f"tcp_move [{pose_offset_str}] {velocity} {acceleration} {radius} \"{tool}\" {block}"
    
    # POST 请求设置
    url_tcp_move = f"{BASE_URL}/api/robot_arm/cmd"
    params_tcp_move = {
        "command": tcp_move_command
    }
    headers = {'Content-Type': 'application/json'}
    
    try:
        response = requests.post(url_tcp_move, json=params_tcp_move, headers=headers)
        time.sleep(2)  # 等待2秒以确保命令发送成功
        if response.ok:
            result = response.json()
            print(f"✅ TCP Move command success: {result}")
            print(f"TCP offset (mm, deg): X={offset_mm[0]}, Y={offset_mm[1]}, Z={offset_mm[2]}, Rx={offset_mm[3]}, Ry={offset_mm[4]}, Rz={offset_mm[5]}")
            print(f"Command sent: {tcp_move_command}")
        else:
            print(f"❌ Failed to send tcp_move command: {response.status_text}")
    except Exception as e:
        print(f"❌ Error sending tcp_move command: {e}")




if __name__ == "__main__":
    # # result1, result2 = joint_angles_deg , tcp_position_mm
    # result1,result2 = get_actual_joint_angles_and_tcp_position()
    # print(result1,'\n',result2)

    # result2 = [result2[0], result2[1]-100, result2[2], result2[3], result2[4], result2[5]]  # 确保是6个元素
    # move_to_target_tcp_position(result2)
    # time.sleep(5)  # 等待2秒以确保TCP移动完成
    # move_to_target_joint_position(result1)
    # time.sleep(5)  # 等待2秒以确保TCP移动完成

    # 测试movej2功能
    # target_angles_deg = [0,0,0,0,0,0]
    # movej2(target_angles_deg)
    
    
    tcp_offset_mm = [0.0, 0.0, 20.0, 0.0, 0.0, 0.0]  # [x_mm, y_mm, z_mm, rx_deg, ry_deg, rz_deg]
    tcp_move(tcp_offset_mm)