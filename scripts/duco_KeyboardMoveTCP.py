#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import keyboard
import time
import math

# 把duco_robot_arm目录加入搜索路径
duco_robot_arm_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'colcon_ws', 'src', 'duco_robot_arm', 'duco_robot_arm'))
sys.path.append(duco_robot_arm_path)
sys.path.append(os.path.join(duco_robot_arm_path, 'gen_py'))
sys.path.append(os.path.join(duco_robot_arm_path, 'lib'))

from DucoCobot import DucoCobot
from thrift import Thrift
from gen_py.robot.ttypes import Op

def ConvertPose2Display(pose_tcp):
    """将TCP位姿转换为显示格式（位置单位：mm，角度单位：度）"""
    result = []
    # 前三个是位置（米转毫米）
    for i in range(3):
        result.append(pose_tcp[i] * 1000)
    # 后三个是角度（弧度转度）
    for i in range(3, 6):
        result.append(math.degrees(pose_tcp[i]))
    return result

def ConvertDisplayToPose(display_pose):
    """将显示格式转换为TCP位姿（位置单位：米，角度单位：弧度）"""
    result = []
    # 前三个是位置（毫米转米）
    for i in range(3):
        result.append(display_pose[i] / 1000)
    # 后三个是角度（度转弧度）
    for i in range(3, 6):
        result.append(math.radians(display_pose[i]))
    return result

class RobotTCPKeyboardController:
    def __init__(self, ip_robot='192.168.1.10', port_robot=7003):
        self.duco_cobot = DucoCobot(ip_robot, port_robot)
        self.current_tcp_pose = [0, 0, 0, 0, 0, 0]  # 当前TCP位姿 [x, y, z, rx, ry, rz]（显示单位：mm, degree）
        self.position_step = 2.0  # 每次按键移动的距离（毫米）
        self.rotation_step = 1.0  # 每次按键旋转的角度（度）
        self.v = 1.0  # 速度
        self.a = 1.0  # 加速度
        self.kp = 150  # 比例增益
        self.kd = 35   # 微分增益
        
        # 按键映射
        self.key_mappings = {
            # 位置控制
            'q': (0, 1),   # X轴正方向
            'a': (0, -1),  # X轴负方向
            'w': (1, 1),   # Y轴正方向
            's': (1, -1),  # Y轴负方向
            'e': (2, 1),   # Z轴正方向
            'd': (2, -1),  # Z轴负方向
            # 旋转控制
            'r': (3, 1),   # 绕X轴旋转（Rx增加）
            'f': (3, -1),  # 绕X轴旋转（Rx减少）
            't': (4, 1),   # 绕Y轴旋转（Ry增加）
            'g': (4, -1),  # 绕Y轴旋转（Ry减少）
            'y': (5, 1),   # 绕Z轴旋转（Rz增加）
            'h': (5, -1),  # 绕Z轴旋转（Rz减少）
        }
        
        # TCP限制（安全范围，单位：毫米和度）
        self.tcp_limits = [
            (-1000, 1000),  # X轴位置限制 (mm)
            (-1000, 1000),  # Y轴位置限制 (mm)
            (0, 1000),      # Z轴位置限制 (mm) - 避免碰撞地面
            (-180, 180),    # Rx旋转限制 (degree)
            (-180, 180),    # Ry旋转限制 (degree)
            (-180, 180),    # Rz旋转限制 (degree)
        ]
        
    def initialize_robot(self):
        """初始化机器人连接"""
        try:
            rlt = self.duco_cobot.open()
            if rlt == 0:
                print("机器人连接成功")
                return True
            else:
                print("机器人连接失败")
                return False
        except Exception as e:
            print(f"初始化机器人失败: {e}")
            return False
            
    def close_robot(self):
        """关闭机器人连接"""
        try:
            rlt = self.duco_cobot.close()
            if rlt == 0:
                print("机器人连接已关闭")
            else:
                print("关闭机器人连接失败")
        except Exception as e:
            print(f"关闭机器人连接出错: {e}")
            
    def get_current_tcp_pose(self):
        """获取机器人当前TCP位姿"""
        try:
            tcp_pose_raw = self.duco_cobot.get_tcp_pose()  # 获取原始TCP位姿 [x, y, z, rx, ry, rz] (m, rad)
            self.current_tcp_pose = ConvertPose2Display(tcp_pose_raw)  # 转换为显示格式 (mm, degree)
            return True
        except Exception as e:
            print(f"获取当前TCP位姿失败: {e}")
            return False
            
    def clamp_tcp_value(self, value, axis_index):
        """限制TCP值在安全范围内"""
        min_val, max_val = self.tcp_limits[axis_index]
        return max(min_val, min(max_val, value))
        
    def move_tcp(self, axis_index, direction):
        """移动TCP坐标"""
        try:
            # 计算偏移量
            if axis_index < 3:  # 位置控制
                offset_value = direction * self.position_step
                step_name = f"{self.position_step}mm"
            else:  # 旋转控制
                offset_value = direction * self.rotation_step
                step_name = f"{self.rotation_step}°"
            
            # 计算新的值
            new_value = self.current_tcp_pose[axis_index] + offset_value
            
            # 限制范围
            new_value = self.clamp_tcp_value(new_value, axis_index)
            
            # 更新当前位置
            self.current_tcp_pose[axis_index] = new_value
            
            # 创建偏移量数组（相对于当前工具坐标系）
            offset = [0, 0, 0, 0, 0, 0]
            
            if axis_index < 3:  # 位置偏移（单位：米）
                offset[axis_index] = offset_value / 1000.0  # 转换为米
            else:  # 角度偏移（单位：弧度）
                offset[axis_index] = math.radians(offset_value)  # 转换为弧度
            
            # 发送servo_tcp指令
            self.duco_cobot.servo_tcp(offset, self.v, self.a, '', False, 
                                     kp=self.kp, kd=self.kd)
            
            # 轴名称
            axis_names = ['X', 'Y', 'Z', 'Rx', 'Ry', 'Rz']
            axis_units = ['mm', 'mm', 'mm', '°', '°', '°']
            
            # 简化输出
            print(f"\r{axis_names[axis_index]}: {new_value:6.1f}{axis_units[axis_index]} | "
                  f"TCP: [X:{self.current_tcp_pose[0]:6.1f} Y:{self.current_tcp_pose[1]:6.1f} Z:{self.current_tcp_pose[2]:6.1f} "
                  f"Rx:{self.current_tcp_pose[3]:5.1f} Ry:{self.current_tcp_pose[4]:5.1f} Rz:{self.current_tcp_pose[5]:5.1f}]", 
                  end='', flush=True)
            
        except Exception as e:
            print(f"移动TCP轴{axis_index + 1}时出错: {e}")
            
    def on_key_event(self, e):
        """处理按键事件"""
        if e.event_type == keyboard.KEY_DOWN:
            key = e.name.lower()
            if key in self.key_mappings:
                axis_index, direction = self.key_mappings[key]
                self.move_tcp(axis_index, direction)
                
    def print_instructions(self):
        """打印操作说明"""
        print("\n" + "="*70)
        print("机械臂TCP控制说明:")
        print("="*70)
        print("位置控制 (单位: 毫米):")
        print("  q/a - X轴增加/减少")
        print("  w/s - Y轴增加/减少")
        print("  e/d - Z轴增加/减少")
        print()
        print("旋转控制 (单位: 度):")
        print("  r/f - 绕X轴旋转增加/减少 (Rx)")
        print("  t/g - 绕Y轴旋转增加/减少 (Ry)")
        print("  y/h - 绕Z轴旋转增加/减少 (Rz)")
        print()
        print("Esc - 退出")
        print(f"每次按键移动距离: ±{self.position_step}mm")
        print(f"每次按键旋转角度: ±{self.rotation_step}°")
        print("="*70)
        
    def run(self):
        """运行主程序"""
        print("正在初始化机器人...")
        if not self.initialize_robot():
            return
            
        # 获取机器人当前TCP位姿
        print("正在获取机器人当前TCP位姿...")
        if not self.get_current_tcp_pose():
            print("无法获取机器人当前TCP位姿，程序退出")
            self.close_robot()
            return
            
        self.print_instructions()
        
        # 显示当前位姿
        print(f"\n当前TCP位姿:")
        print(f"位置: X={self.current_tcp_pose[0]:.1f}mm, Y={self.current_tcp_pose[1]:.1f}mm, Z={self.current_tcp_pose[2]:.1f}mm")
        print(f"旋转: Rx={self.current_tcp_pose[3]:.1f}°, Ry={self.current_tcp_pose[4]:.1f}°, Rz={self.current_tcp_pose[5]:.1f}°")
        
        # 设置按键监听
        keyboard.hook(self.on_key_event)
        
        try:
            print("\n程序运行中... 请按相应按键控制机械臂TCP")
            print("按 ESC 退出程序")
            
            # 主循环
            while True:
                if keyboard.is_pressed('esc'):
                    print("\n检测到ESC按键，准备退出...")
                    break
                time.sleep(0.1)
                
        except KeyboardInterrupt:
            print("\n程序被用户中断")
        except Exception as e:
            print(f"\n程序运行出错: {e}")
        finally:
            keyboard.unhook_all()
            self.close_robot()
            print("程序已退出")

def main():
    """主函数"""
    # 可以在这里修改机器人的IP和端口
    ip_robot = '192.168.1.10'
    port_robot = 7003
    
    controller = RobotTCPKeyboardController(ip_robot, port_robot)
    controller.run()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"程序启动失败: {e}")
        input("按任意键退出...")
