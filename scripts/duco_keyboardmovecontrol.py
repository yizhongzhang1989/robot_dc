#!/usr/bin/env python
# -*- coding: utf-8 -*-
"""
键盘控制机械臂移动
按键映射：
- q/a: 第一关节 +/-
- w/s: 第二关节 +/-
- e/d: 第三关节 +/-
- r/f: 第四关节 +/-
- t/g: 第五关节 +/-
- y/h: 第六关节 +/-
- ESC: 退出程序
"""

import sys
import time
import threading
import math
import os

# 添加路径
sys.path.append(os.path.abspath(os.path.join(os.path.dirname(__file__), '..')))
sys.path.append('../gen_py')
sys.path.append('../lib')

from DucoCobot import DucoCobot
from thrift import Thrift
from gen_py.robot.ttypes import Op

# 尝试导入keyboard库
try:
    import keyboard
    KEYBOARD_AVAILABLE = True
except ImportError:
    KEYBOARD_AVAILABLE = False
    print("警告：keyboard库未安装，请运行 'pip install keyboard' 安装")
    print("将使用替代方案（需要按回车确认）")

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
    def __init__(self, ip='192.168.1.10', port=7003):
        self.ip = ip
        self.port = port
        self.duco_cobot = None
        self.running = False
        self.current_joints = [0, 0, 0, 0, 0, 0]  # 当前关节角度（度）
        self.target_joints = [0, 0, 0, 0, 0, 0]   # 目标关节角度（度）
        self.move_speed = 5.0  # 移动速度
        self.move_acceleration = 5.0  # 移动加速度
        
        # 按键映射
        self.key_mappings = {
            'q': (0, 1),   # 第一关节 +
            'a': (0, -1),  # 第一关节 -
            'w': (1, 1),   # 第二关节 +
            's': (1, -1),  # 第二关节 -
            'e': (2, 1),   # 第三关节 +
            'd': (2, -1),  # 第三关节 -
            'r': (3, 1),   # 第四关节 +
            'f': (3, -1),  # 第四关节 -
            't': (4, 1),   # 第五关节 +
            'g': (4, -1),  # 第五关节 -
            'y': (5, 1),   # 第六关节 +
            'h': (5, -1),  # 第六关节 -
        }
        
        # 移动步长
        self.move_step = 3.0  # 每次按键移动的角度（度）
        
        # 按键时长控制参数
        self.key_press_start_time = {}  # 记录每个按键开始按下的时间
        self.single_press_threshold = 0.3  # 单次按键阈值（秒）
        self.single_press_step = 3.0  # 单次按键步长（度）
        self.continuous_base_step = 5.0  # 连续按键基础步长（度）
        self.continuous_max_step = 20.0  # 连续按键最大步长（度）
        self.step_growth_rate = 1.5  # 步长增长率
        
        # 显示相关变量
        self.last_move_info = ""  # 上次移动信息
        self.current_press_info = ""  # 当前按键信息（实时显示）
        
        # 操作参数
        self.op = Op()
        self.op.time_or_dist_1 = 0
        self.op.trig_io_1 = 1
        self.op.trig_value_1 = False
        self.op.trig_time_1 = 0.0
        self.op.trig_dist_1 = 0.0
        self.op.trig_event_1 = ""
        self.op.time_or_dist_2 = 0
        self.op.trig_io_2 = 1
        self.op.trig_value_2 = False
        self.op.trig_time_2 = 0.0
        self.op.trig_dist_2 = 0.0
        self.op.trig_event_2 = ""

    def connect_robot(self):
        """连接机器人"""
        try:
            self.duco_cobot = DucoCobot(self.ip, self.port)
            result = self.duco_cobot.open()
            if result == 0:
                print("机器人连接成功")
                return True
            else:
                print("机器人连接失败")
                return False
        except Exception as e:
            print(f"连接机器人时发生错误: {e}")
            return False

    def disconnect_robot(self):
        """断开机器人连接"""
        if self.duco_cobot:
            try:
                self.duco_cobot.close()
                print("机器人连接已断开")
            except Exception as e:
                print(f"断开连接时发生错误: {e}")

    def update_current_joints(self):
        """更新当前关节角度"""
        if self.duco_cobot:
            try:
                joints_rad = self.duco_cobot.get_actual_joints_position()
                self.current_joints = ConvertRad2Pose(joints_rad)
                return True
            except Exception as e:
                print(f"获取关节角度时发生错误: {e}")
                return False
        return False

    def move_robot(self, target_joints):
        """移动机器人到目标关节角度"""
        if self.duco_cobot:
            try:
                # 检查角度限制（简单检查，避免过大的角度）
                for i, angle in enumerate(target_joints):
                    if abs(angle) > 360:
                        return False
                
                target_joints_rad = ConvertPose2Rad(target_joints)
                result = self.duco_cobot.movej2(
                    target_joints_rad, 
                    self.move_speed, 
                    self.move_acceleration, 
                    0.0, 
                    False,  # 非阻塞模式
                    self.op
                )
                
                if result == 0:
                    self.target_joints = target_joints.copy()
                    time.sleep(0.01)  # 短暂等待确保命令发送
                    return True
                else:
                    return False
                    
            except Exception as e:
                return False
        return False

    def print_status(self):
        """打印当前状态"""
        # 如果有当前按键信息，优先显示；否则显示上次移动信息
        display_info = self.current_press_info if self.current_press_info else self.last_move_info
        status = f"\r当前关节角度: [{self.current_joints[0]:6.2f}, {self.current_joints[1]:6.2f}, {self.current_joints[2]:6.2f}, {self.current_joints[3]:6.2f}, {self.current_joints[4]:6.2f}, {self.current_joints[5]:6.2f}] {display_info}"
        print(status, end='', flush=True)

    def keyboard_control_loop(self):
        """键盘控制循环（使用keyboard库）"""
        print("键盘控制已启动")
        print("按键说明：")
        print("q/a: 第一关节 +/-")
        print("w/s: 第二关节 +/-")
        print("e/d: 第三关节 +/-")
        print("r/f: 第四关节 +/-")
        print("t/g: 第五关节 +/-")
        print("y/h: 第六关节 +/-")
        print("ESC: 退出程序")
        print(f"智能按键：单次按键(<{self.single_press_threshold}s)={self.single_press_step}°，连续按键={self.continuous_base_step}-{self.continuous_max_step}°")
        print("-" * 50)
        
        while self.running:
            try:
                current_time = time.time()
                
                # 更新当前关节角度
                if self.update_current_joints():
                    self.print_status()
                
                # 检查退出键
                if keyboard.is_pressed('esc'):
                    print("\n检测到ESC键，退出程序")
                    break
                
                # 检查按键状态变化
                any_key_pressed = False
                for key, (joint_idx, direction) in self.key_mappings.items():
                    if keyboard.is_pressed(key):
                        any_key_pressed = True
                        # 按键正在按下
                        if key not in self.key_press_start_time:
                            # 新按下的按键，记录开始时间
                            self.key_press_start_time[key] = current_time
                        
                        # 计算当前按键持续时间和预期移动步长
                        press_duration = current_time - self.key_press_start_time[key]
                        
                        if press_duration < self.single_press_threshold:
                            # 单次按键
                            step = self.single_press_step
                            move_type = "单次"
                        else:
                            # 连续按键，根据时长计算步长
                            continuous_duration = press_duration - self.single_press_threshold
                            step = min(
                                self.continuous_base_step * (1 + continuous_duration * self.step_growth_rate),
                                self.continuous_max_step
                            )
                            move_type = "连续"
                        
                        # 实时显示当前按键信息
                        direction_str = "+" if direction > 0 else "-"
                        self.current_press_info = f"| 关节{joint_idx+1} {direction_str}{step:.1f}° ({move_type}{press_duration:.2f}s)"
                        
                    else:
                        # 按键已释放
                        if key in self.key_press_start_time:
                            # 计算按键持续时间
                            press_duration = current_time - self.key_press_start_time[key]
                            
                            # 根据按键时长决定移动步长
                            if press_duration < self.single_press_threshold:
                                # 单次按键
                                step = self.single_press_step
                                move_type = "单次"
                            else:
                                # 连续按键，根据时长计算步长
                                continuous_duration = press_duration - self.single_press_threshold
                                step = min(
                                    self.continuous_base_step * (1 + continuous_duration * self.step_growth_rate),
                                    self.continuous_max_step
                                )
                                move_type = "连续"
                            
                            # 计算目标角度并移动
                            target_joints = self.current_joints.copy()
                            target_joints[joint_idx] += direction * step
                            
                            # 记录移动信息
                            direction_str = "+" if direction > 0 else "-"
                            self.last_move_info = f"| 关节{joint_idx+1} {direction_str}{step:.1f}° ({move_type}{press_duration:.2f}s)"
                            
                            self.move_robot(target_joints)
                            
                            # 移除按键记录
                            del self.key_press_start_time[key]
                
                # 如果没有按键被按下，清除当前按键信息
                if not any_key_pressed:
                    self.current_press_info = ""
                
                # 控制循环频率
                time.sleep(0.05)
                    
            except Exception as e:
                print(f"\n控制循环中发生错误: {e}")
                break

    def simple_control_loop(self):
        """简单控制循环（不使用keyboard库）"""
        print("简单控制模式已启动")
        print("按键说明：")
        print("q/a: 第一关节 +/-")
        print("w/s: 第二关节 +/-")
        print("e/d: 第三关节 +/-")
        print("r/f: 第四关节 +/-")
        print("t/g: 第五关节 +/-")
        print("y/h: 第六关节 +/-")
        print("quit: 退出程序")
        print("-" * 50)
        
        while self.running:
            try:
                # 更新当前关节角度
                if self.update_current_joints():
                    self.print_status()
                
                # 等待用户输入
                print("\n请输入按键（然后按回车）：", end='')
                user_input = input().strip().lower()
                
                if user_input == 'quit':
                    print("退出程序")
                    break
                
                if user_input in self.key_mappings:
                    joint_idx, direction = self.key_mappings[user_input]
                    
                    # 计算目标角度
                    target_joints = self.current_joints.copy()
                    target_joints[joint_idx] += direction * self.move_step
                    
                    # 移动机器人
                    if self.move_robot(target_joints):
                        print(f"关节{joint_idx+1} {'增加' if direction > 0 else '减少'} {self.move_step}°")
                    else:
                        print("移动失败")
                else:
                    print("无效按键")
                    
            except KeyboardInterrupt:
                print("\n检测到Ctrl+C，退出程序")
                break
            except Exception as e:
                print(f"\n控制循环中发生错误: {e}")
                break

    def start_control(self):
        """启动控制"""
        if not self.connect_robot():
            return False
        
        self.running = True
        
        # 获取初始关节角度
        if not self.update_current_joints():
            print("无法获取初始关节角度")
            self.disconnect_robot()
            return False
        
        # 初始化目标关节角度
        self.target_joints = self.current_joints.copy()
        print(f"初始关节角度: {self.current_joints}")
        print(f"单次按键步长: {self.single_press_step}度")
        print(f"连续按键步长: {self.continuous_base_step}-{self.continuous_max_step}度")
        print(f"按键时长阈值: {self.single_press_threshold}秒")
        
        try:
            if KEYBOARD_AVAILABLE:
                self.keyboard_control_loop()
            else:
                self.simple_control_loop()
        finally:
            self.running = False
            self.disconnect_robot()
        
        return True

    def stop_control(self):
        """停止控制"""
        self.running = False

def main():
    """主函数"""
    print("机械臂键盘控制程序")
    print("=" * 50)
    
    # 创建控制器
    controller = RobotKeyboardController()
    
    try:
        # 启动控制
        controller.start_control()
    except KeyboardInterrupt:
        print("\n程序被用户中断")
    except Exception as e:
        print(f"程序运行时发生错误: {e}")
    finally:
        controller.stop_control()
        print("程序结束")

if __name__ == '__main__':
    main()
