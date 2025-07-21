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

def ConvertPose2Rad(pose):
    """将角度转换为弧度"""
    result = []
    for val in pose:
        result.append(math.radians(val))
    return result

def ConvertRad2Pose(pose_rad):
    """将弧度转换为角度"""
    result = []
    for val in pose_rad:
        result.append(math.degrees(val))
    return result

class RobotKeyboardController:
    def __init__(self, ip_robot='192.168.1.10', port_robot=7003):
        self.duco_cobot = DucoCobot(ip_robot, port_robot)
        self.current_pose = [0, 0, 0, 0, 0, 0]  # 当前关节角度（度）
        self.angle_step = 0.1  # 每次按键增减的角度（度）- 调小步长实现实时控制
        self.v = 2.0  # 速度
        self.a = 2.0  # 加速度
        self.kp = 250  # 比例增益 - 调整以获得更好的响应
        self.kd = 25   # 微分增益 - 调整以减少抖动
        
        # 按键映射
        self.key_mappings = {
            'q': (0, 1),   # 第一关节增加
            'a': (0, -1),  # 第一关节减少
            'w': (1, 1),   # 第二关节增加
            's': (1, -1),  # 第二关节减少
            'e': (2, 1),   # 第三关节增加
            'd': (2, -1),  # 第三关节减少
            'r': (3, 1),   # 第四关节增加
            'f': (3, -1),  # 第四关节减少
            't': (4, 1),   # 第五关节增加
            'g': (4, -1),  # 第五关节减少
            'y': (5, 1),   # 第六关节增加
            'h': (5, -1),  # 第六关节减少
        }
        
        # 关节角度限制（度）
        self.joint_limits = [
            (-180, 180),  # 关节1
            (-180, 180),  # 关节2
            (-180, 180),  # 关节3
            (-180, 180),  # 关节4
            (-180, 180),  # 关节5
            (-180, 180),  # 关节6
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
            
    def get_current_position(self):
        """获取机器人当前关节位置"""
        try:
            joints_rad = self.duco_cobot.get_actual_joints_position()
            self.current_pose = ConvertRad2Pose(joints_rad)
            return True
        except Exception as e:
            print(f"获取当前位置失败: {e}")
            return False
            
    def clamp_angle(self, angle, joint_index):
        """限制关节角度在安全范围内"""
        min_angle, max_angle = self.joint_limits[joint_index]
        return max(min_angle, min(max_angle, angle))
        
    def move_joint(self, joint_index, direction):
        """移动指定关节"""
        try:
            # 计算新的角度
            angle_change = direction * self.angle_step
            new_angle = self.current_pose[joint_index] + angle_change
            
            # 限制角度范围
            new_angle = self.clamp_angle(new_angle, joint_index)
            
            # 更新当前位置
            self.current_pose[joint_index] = new_angle
            
            # 转换为弧度
            pose_rad = ConvertPose2Rad(self.current_pose)
            
            # 发送servo指令
            self.duco_cobot.servoj(pose_rad, self.v, self.a, 
                                  False, kp=self.kp, kd=self.kd)
            
            # 简化输出，减少屏幕刷新
            print(f"\rJ{joint_index + 1}: {new_angle:6.1f}° | 位置: [{', '.join([f'{x:5.1f}' for x in self.current_pose])}]", end='', flush=True)
            
        except Exception as e:
            print(f"移动关节{joint_index + 1}时出错: {e}")
            
    def on_key_event(self, e):
        """处理按键事件"""
        if e.event_type == keyboard.KEY_DOWN:
            key = e.name.lower()
            if key in self.key_mappings:
                joint_index, direction = self.key_mappings[key]
                self.move_joint(joint_index, direction)
                
    def print_instructions(self):
        """打印操作说明"""
        print("\n" + "="*60)
        print("机械臂关节控制说明:")
        print("="*60)
        print("q/a - 第一关节增加/减少")
        print("w/s - 第二关节增加/减少")
        print("e/d - 第三关节增加/减少")
        print("r/f - 第四关节增加/减少")
        print("t/g - 第五关节增加/减少")
        print("y/h - 第六关节增加/减少")
        print("Esc - 退出")
        print("每次按键移动角度: ±{:.1f}度".format(self.angle_step))
        print("="*60)
        
        
    def run(self):
        """运行主程序"""
        print("正在初始化机器人...")
        if not self.initialize_robot():
            return
            
        # 获取机器人当前位置
        print("正在获取机器人当前位置...")
        if not self.get_current_position():
            print("无法获取机器人当前位置，程序退出")
            self.close_robot()
            return
            
        self.print_instructions()
        
        # 设置按键监听
        keyboard.hook(self.on_key_event)
        
        try:
            print("\n程序运行中... 请按相应按键控制机械臂")
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
    
    controller = RobotKeyboardController(ip_robot, port_robot)
    controller.run()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"程序启动失败: {e}")
        input("按任意键退出...")
