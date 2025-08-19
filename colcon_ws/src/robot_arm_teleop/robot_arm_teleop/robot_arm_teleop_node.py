import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import datetime
import math
import time
import numpy as np

from duco_robot_arm.DucoCobot import DucoCobot
from duco_robot_arm.gen_py.robot.ttypes import Op  
from thrift import Thrift
from .FTCApiPost import *  

def ConvertDeg2Rad(pose):

    result = []
    for val in pose:
        result.append(math.radians(val))
    return result

def ConvertRad2Deg(pose_rad):

    result = []
    for val in pose_rad:
        result.append(math.degrees(val))
    return result

def compute_recovery_force(position_error, velocity, Kp, Kd):
    """
    计算恢复力
    position_error: 位置误差 (target_pose - current_pose)
    velocity: 当前速度
    Kp: 比例增益
    Kd: 微分增益
    
    当position_error > 0时，说明目标位置大于当前位置，需要正向力
    当position_error < 0时，说明目标位置小于当前位置，需要负向力
    """
    max_force = 100  # 单轴最大力
    
    # 标准PD控制器：力 = Kp * 位置误差 - Kd * 速度
    # 减去速度项是为了阻尼，防止震荡
    force = Kp * position_error - Kd * velocity
    
    # 限制力的大小
    for i in range(3):
        force[i] = max(-max_force, min(max_force, force[i]))
    
    # 角度分量设为0（不控制姿态）
    force[3] = 0
    force[4] = 0
    force[5] = 0
    
    return force

class RobotArmTeleop(Node):
    def __init__(self):
        super().__init__('robot_arm_teleop')
        self.seq_id = 0

        # 声明参数
        self.declare_parameter('device_id', 1)
        self.declare_parameter('deadzone', 0.1)
        self.declare_parameter('robot_ip', '192.168.1.10')  # 机械臂IP地址
        self.declare_parameter('robot_port', 7003)        # 机械臂端口
        
        # 获取参数
        self.device_id = self.get_parameter('device_id').get_parameter_value().integer_value
        self.deadzone = self.get_parameter('deadzone').get_parameter_value().double_value
        self.robot_ip = self.get_parameter('robot_ip').get_parameter_value().string_value
        self.robot_port = self.get_parameter('robot_port').get_parameter_value().integer_value

        # 发布器
        topic_name = f'/arm_teleop_{self.device_id}/cmd'
        self.arm_cmd_pub = self.create_publisher(String, topic_name, 10)

        # 订阅手柄输入
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # 指令发送频率
        self.timer = self.create_timer(0.05, self.command_timer_cb)  # 20Hz
        
        # 状态监控定时器
        self.status_timer = self.create_timer(1.0, self.status_timer_cb)  # 1Hz

        # 状态变量
        self.arm_enabled = False
        self.last_joy_msg = None
        self.ftc_program_enabled = False  # 力控程序使能状态，默认未使能
        self.joystick_enabled = False  # 摇杆使能状态，默认关闭
        
        # target_pose控制相关变量
        self.target_pose = None  # 目标位置，初始化为None
        self.pos_step = 0.005  # 每次摇杆移动的步长 (1mm)，与键盘脚本保持一致
        self.last_axes = [0.0] * 10  # 记录上一次的摇杆状态，用于检测变化
        
        # 按钮状态跟踪（用于检测按下事件）
        self.last_buttons = [0] * 20  # 假设最多20个按钮

        # 初始化机械臂连接
        self.robot = None
        self.init_robot_connection()

        # 启动FTC
        self.init_ftc()

        self.get_logger().info('🎮 Robot Arm Teleop Node Started')
        self.get_logger().info(f'🎮 Arm Teleop topic publishing to: {topic_name}')
        self.get_logger().info('🕹️ Joystick is disabled by default. Press A button to enable/disable joystick.')
        self.get_logger().info('🔧 Usage: A=Enable Joystick, X=Enable/Disable FTC Force Control, Y=Toggle Gripper')

    def init_robot_connection(self):
        """初始化机械臂连接"""
        try:
            self.get_logger().info(f'🤖 Initializing robot connection to {self.robot_ip}:{self.robot_port}')
            self.robot = DucoCobot(self.robot_ip, self.robot_port)
            op = Op()
            op.time_or_dist_1 = 0
            op.trig_io_1 = 1
            op.trig_value_1 = False
            op.trig_time_1 = 0.0
            op.trig_dist_1 = 0.0
            op.trig_event_1 = ""
            op.time_or_dist_2 = 0
            op.trig_io_2 = 1
            op.trig_value_2 = False
            op.trig_time_2 = 0.0
            op.trig_dist_2 = 0.0
            op.trig_event_2 = ""
            self.op = op

            # 尝试连接
            if self.robot.open() == 0:
                self.get_logger().info('🤖 Robot connection established successfully')
            else:
                self.get_logger().error('❌ Failed to establish robot connection')
                self.robot = None
                
        except Exception as e:
            self.get_logger().error(f'❌ Error initializing robot: {e}')
            self.robot = None

    def init_ftc(self):
        """初始化FTC"""
        try:
            FTC_stop()
            time.sleep(0.5)  # 确保FTC停止
            FTC_start()
            time.sleep(0.5)  # 确保FTC停止

            # 设置FTSET_INDEX 18的初始参数
            isProgram = False
            ftcProgram = None
            onlyMonitor = False
            graCalcIndex = 0    # 选择第1个负载参数，与键盘脚本保持一致
            ftEnabled = [True, True, True, False, False, False]  # 力的方向设置，与键盘脚本保持一致
            ftSet = [0, 0, 0, 0, 0, 0]  # 初始零力拖动状态
            dead_zone = [1, 1, 1, 0.1, 0.1, 0.1]    # 死区设置（小于死区阈值的传感数据默认不检测）
            disEndLimit = 5000  # 终止类型0“距离”对应的参数mm
            timeEndLimit = 0.5  # 终止类型3“时间”对应的参数s
            ftEndLimit = [0, 0, 0, 0, 0, 0]  # 终止类型1“力”对应的参数
            disAng6D_EndLimit = [0, 0, 0, 0, 0, 0]  # 终止类型7“距离角度六维”对应的参数
            ftcEndType = 6  # 默认终止类型6，API控制标志位决定是否运行或停止
            quickSetIndex = [0, 0, 0, 0, 0, 0]  # 专家设置参数，不用管
            B = [6000, 6000, 6000, 4500, 4500, 4500]  # 力传感器的B参数
            M = [20, 20, 20, 25, 25, 25]    # 力传感器的M参数
            vel_limit = [1500, 1500, 1500, 500, 500, 500] # 速度限制mm deg/s
            cor_pos_limit = [10, 10, 10, 5, 5, 5]   # 位置限制mm deg/count
            maxForce_1 = [0, 0, 0, 0, 0, 0]
            ifDKStopOnMaxForce_1 = False
            ifRobotStopOnMaxForce_1 = False
            maxForce_2 = [0, 0, 0, 0, 0, 0]
            ifDKStopOnMaxForce_2 = False
            ifRobotStopOnMaxForce_2 = False
            ifDKStopOnTimeDisMon = False
            ifRobotStopOnTimeDisMon = False
            ifNeedInit = True
            withGroup = False
            ftcSetGroup = [17]  # 对应名称为default的单程, idx = FTSET_INDEX-1
            ignoreSensor = False

            FTC_setFTValueAll(isProgram, ftcProgram, onlyMonitor, graCalcIndex, ftEnabled, ftSet, dead_zone, disEndLimit,
                            timeEndLimit, ftEndLimit, disAng6D_EndLimit, ftcEndType, quickSetIndex, B, M, vel_limit,
                            cor_pos_limit, maxForce_1, ifDKStopOnMaxForce_1, ifRobotStopOnMaxForce_1, maxForce_2,
                            ifDKStopOnMaxForce_2, ifRobotStopOnMaxForce_2, ifDKStopOnTimeDisMon, ifRobotStopOnTimeDisMon,
                            ifNeedInit, withGroup, ftcSetGroup, ignoreSensor)
            time.sleep(0.5)  # 确保FTC设置完成
            FTC_SetIndex(19)  # 选择执行程序FTPRO_INDEX 19
            time.sleep(0.5)  # 确保FTC设置完成

            self.get_logger().info('✅ FTC restart successfully')
            self.get_logger().info('✅ Set FTPRO_INDEX 19 successfully')
        except Exception as e:
            self.get_logger().error(f'❌ Error starting FTC: {e}')

#=======================================回调处理函数===================================
    def joy_callback(self, msg: Joy):
        """处理手柄输入回调"""
        # 缓存消息
        self.last_joy_msg = msg
        
        # 处理按钮事件（仅在按下时触发）
        self.handle_buttons(msg)
        
        # 打印摇杆方向信息
        self.print_joystick_directions(msg)

    def handle_buttons(self, msg: Joy):
        """处理按钮按下事件"""
        buttons = msg.buttons
        
        # 检测按钮按下事件（从0变为1）
        for i, button in enumerate(buttons):
            if i < len(self.last_buttons) and button == 1 and self.last_buttons[i] == 0:
                self.handle_button_press(i)
                
        # 更新按钮状态
        self.last_buttons = buttons[:]

    def handle_button_press(self, button_id):
        """处理具体的按钮按下事件"""
        if button_id == 0:  # X按钮
            self.X_function()
        elif button_id == 1:  # A按钮
            self.A_function()
        elif button_id == 2:  # B按钮
            self.B_function()
        elif button_id == 3:  # Y按钮
            self.Y_function()
        elif button_id == 4:  # LB按钮
            self.LB_function()
        elif button_id == 5:  # RB按钮
            self.RB_function()
        elif button_id == 6:  # LT按钮
            self.LT_function()
        elif button_id == 7:  # RT按钮
            self.RT_function()
        elif button_id == 8:  # Back按钮
            self.Back_function()
        elif button_id == 9:  # Start按钮
            self.Start_function()

    def print_joystick_directions(self, msg: Joy):
        """打印摇杆方向信息并控制target_pose"""
        # 如果摇杆未使能，直接返回
        if not self.joystick_enabled:
            return
            
        # 如果target_pose未初始化或机器人未连接，直接返回
        if self.target_pose is None or self.robot is None:
            return
            
        axes = msg.axes
        
        # 检查摇杆是否有移动（超过死区）并更新target_pose
        joystick_moved = False
        directions = []
        target_pose_changed = False
        
        # 检查摇杆
        if len(axes) >= 4:
            # 左摇杆控制 y(上下) 和 z(左右) 方向
            if abs(axes[1]) > self.deadzone:  # 左摇杆垂直
                joystick_moved = True
                if axes[1] > self.deadzone:  # 上
                    self.target_pose[1] -= self.pos_step  # 上对应y值减小
                    directions.append(f"y-{self.pos_step:.3f}")
                    target_pose_changed = True
                elif axes[1] < -self.deadzone:  # 下
                    self.target_pose[1] += self.pos_step  # 下对应y值增大
                    directions.append(f"y+{self.pos_step:.3f}")
                    target_pose_changed = True
                    
            if abs(axes[0]) > self.deadzone:  # 左摇杆水平
                joystick_moved = True
                if axes[0] > self.deadzone:  # 左
                    self.target_pose[2] -= self.pos_step  # 左对应z值减小
                    directions.append(f"z-{self.pos_step:.3f}")
                    target_pose_changed = True
                elif axes[0] < -self.deadzone:  # 右
                    self.target_pose[2] += self.pos_step  # 右对应z值增大
                    directions.append(f"z+{self.pos_step:.3f}")
                    target_pose_changed = True
            
            # 右摇杆控制 x 方向(上下)
            if abs(axes[3]) > self.deadzone:  # 右摇杆垂直
                joystick_moved = True
                if axes[3] > self.deadzone:  # 上
                    self.target_pose[0] += self.pos_step  # 上对应x值增大
                    directions.append(f"x+{self.pos_step:.3f}")
                    target_pose_changed = True
                elif axes[3] < -self.deadzone:  # 下
                    self.target_pose[0] -= self.pos_step  # 下对应x值减小
                    directions.append(f"x-{self.pos_step:.3f}")
                    target_pose_changed = True
        
        # 如果target_pose有变化，打印相关信息
        if target_pose_changed:
            # 构建摇杆状态描述
            joystick_desc = []
            if abs(axes[0]) > self.deadzone or abs(axes[1]) > self.deadzone:
                left_desc = "左摇杆:"
                if abs(axes[1]) > self.deadzone:
                    left_desc += " 上" if axes[1] > self.deadzone else " 下"
                if abs(axes[0]) > self.deadzone:
                    left_desc += " 左" if axes[0] > self.deadzone else " 右"
                joystick_desc.append(left_desc)
            
            if abs(axes[3]) > self.deadzone:
                right_desc = "右摇杆:"
                right_desc += " 上" if axes[3] > self.deadzone else " 下"
                joystick_desc.append(right_desc)
            
            self.get_logger().info(f"🕹️ {' | '.join(joystick_desc)} -> {' | '.join(directions)}")
            self.get_logger().info(f"🎯 Target updated: [{', '.join([f'{x:.4f}' for x in self.target_pose[:3]])}]")

    def command_timer_cb(self):
        """定时器的回调函数 - 力控制循环"""
        if not self.joystick_enabled or self.target_pose is None or self.robot is None:
            return
            
        if not self.ftc_program_enabled:
            return
            
        try:
            # 获取当前位置和速度
            current_pose = np.array(self.robot.get_tcp_pose())  # 机械臂实时TCP位置
            current_speed = np.array(self.robot.get_tcp_speed())
            target_pose_array = np.array(self.target_pose)      # 摇杆控制的目标位置
            
            # 计算位置误差 (目标位置 - 当前位置)
            # 当误差为正时，说明需要朝正方向运动
            position_error = target_pose_array - current_pose
            
            # 力控参数
            Kp = np.array([1500, 1500, 1500, 0, 0, 0])
            Kd = np.array([300, 300, 300, 0, 0, 0])
            
            # 计算控制力
            force = compute_recovery_force(position_error, current_speed, Kp, Kd)
            ftSet = force.tolist()
            
            # 设置力到 FTC
            FTC_setFTValueRT(ftSet)
            
            # 调试信息（可选，只在需要调试时开启）
            if abs(position_error[0]) > 0.001 or abs(position_error[1]) > 0.001 or abs(position_error[2]) > 0.001:
                self.get_logger().debug(f"🔧 Position error: [{', '.join([f'{x:.4f}' for x in position_error[:3]])}]")
                self.get_logger().debug(f"🔧 Force command: [{', '.join([f'{x:.1f}' for x in force[:3]])}]")
            
        except Exception as e:
            self.get_logger().error(f'❌ Error in force control loop: {e}')

    def status_timer_cb(self):
        """状态监控定时器 - 定期打印位置信息"""
        if not self.joystick_enabled or self.target_pose is None or self.robot is None:
            return
            
        try:
            current_pose = np.array(self.robot.get_tcp_pose())    # 机械臂实时位置
            target_pose_array = np.array(self.target_pose)        # 摇杆控制的目标位置
            position_error = target_pose_array - current_pose     # 位置误差 = 目标 - 当前
            
            self.get_logger().info(f"📊 Status - Target: [{', '.join([f'{x:.4f}' for x in self.target_pose[:3]])}]")
            self.get_logger().info(f"📊 Status - Current: [{', '.join([f'{x:.4f}' for x in current_pose[:3]])}]")
            self.get_logger().info(f"📊 Status - Error: [{', '.join([f'{x:.4f}' for x in position_error[:3]])}]")
            self.get_logger().info(f"📊 FTC Enabled: {self.ftc_program_enabled}\n")
            
        except Exception as e:
            self.get_logger().error(f'❌ Error getting status: {e}')
            
    def X_function(self):
        """X按钮功能 - 力控程序使能/断使能切换"""
        self.get_logger().info('🎮 X Button: X button pressed')
        FTC_SetIndex(19)  # 选择执行程序FTPRO_INDEX 19
        try:
            if self.ftc_program_enabled:
                # 清零力
                FTC_setFTValueRT([0, 0, 0, 0, 0, 0])
                # 当前已使能，执行断使能操作
                FTC_SetDKAssemFlag(0)  # 关闭程序
                self.ftc_program_enabled = False
                time.sleep(0.5)  # 等待设置生效
               
                self.get_logger().info('🔧 FTC program disabled and forces cleared')
            else:
                # 当前未使能，执行使能操作
                FTC_SetDKAssemFlag(1)  # 开启程序
                self.ftc_program_enabled = True
                time.sleep(0.5)
                self.get_logger().info('🔧 FTC program enabled - Force control active')
                
            time.sleep(1)  # 等待设置生效
            
        except Exception as e:
            self.get_logger().error(f'❌ Error toggling FTC program: {e}')


    def A_function(self):
        """A按钮功能 - 摇杆使能/断使能切换"""
        self.joystick_enabled = not self.joystick_enabled
        status = "enabled" if self.joystick_enabled else "disabled"
        
        if self.joystick_enabled and self.robot is not None:
            # 启用摇杆时，初始化target_pose为当前TCP位置
            try:
                self.target_pose = self.robot.get_tcp_pose()
                self.get_logger().info(f'🎯 Target pose initialized: {[round(x, 4) for x in self.target_pose]}')
                self.get_logger().info('🔧 Force control will start when FTC program is enabled (press X)')
            except Exception as e:
                self.get_logger().error(f'❌ Error getting current TCP pose: {e}')
                self.target_pose = None
        else:
            # 禁用摇杆时，停止力控制
            if self.target_pose is not None:
                try:
                    # 清零力
                    FTC_setFTValueRT([0, 0, 0, 0, 0, 0])
                    self.get_logger().info('🔧 Force control stopped and forces cleared')
                except Exception as e:
                    self.get_logger().error(f'❌ Error clearing forces: {e}')
                self.target_pose = None
        
        self.get_logger().info(f'🎮 A Button: A button pressed, Joystick {status}')


    def B_function(self):
        """B按钮功能"""
        self.get_logger().info('🎮 B Button: B button pressed')

    def Y_function(self):
        """Y按钮功能 - 夹爪开启/闭合切换"""
        self.get_logger().info('🎮 Y Button: Y button pressed - Toggle gripper')
        
        if self.robot is None:
            self.get_logger().error('❌ Robot not connected, cannot control gripper')
            return
            
        try:
            # 读取夹爪状态，False=0（张开）,True=1（关闭）
            State_Gripper = self.robot.get_standard_digital_out(10)
            self.get_logger().info(f'🤏 Original gripper state: {"Closed" if State_Gripper else "Open"}')
            
            # 切换夹爪状态
            if State_Gripper:
                # 当前关闭，执行张开操作
                self.robot.set_standard_digital_out(10, False, True)
            else:
                # 当前张开，执行关闭操作
                self.robot.set_standard_digital_out(10, True, True)
            time.sleep(0.5)
            
            # 读取夹爪状态
            State_Gripper = self.robot.get_standard_digital_out(10)
            self.get_logger().info(f'🤏 Current Gripper state: {"Closed" if State_Gripper else "Open"}')
            
        except Exception as e:
            self.get_logger().error(f'❌ Error controlling gripper: {e}')

    def LB_function(self):
        """LB按钮功能"""
        self.get_logger().info('🎮 LB Button: LB button pressed')

    def RB_function(self):
        """RB按钮功能"""
        self.get_logger().info('🎮 RB Button: RB button pressed')

    def LT_function(self):
        """LT按钮功能"""
        self.get_logger().info('🎮 LT Button: LT button pressed')

    def RT_function(self):
        """RT按钮功能"""
        self.get_logger().info('🎮 RT Button: RT button pressed')

    def Start_function(self):
        """Start按钮功能：移动到任务执行的起始位置"""
        self.get_logger().info('🎮 Start Button: Start button pressed')
        FTC_SetIndex(19)  # 选择执行程序FTPRO_INDEX 19
        time.sleep(0.5)
        FTC_SetDKAssemFlag(0)  # 关闭程序
        time.sleep(0.5)
        self.get_logger().info('🔧 FTC program disabled')

        pose = [-97.71, -46.95, 131.34, -84.59, -81.33, 269.72]
        pose_rad = ConvertDeg2Rad(pose)
        res = self.robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, self.op)
        time.sleep(0.5)
        self.get_logger().info('🤖 Robot arm move to task start position')

    def Back_function(self):
        """Back按钮功能：回到机械臂零位"""
        self.get_logger().info('🎮 Back Button: Back button pressed')
        FTC_SetIndex(19)  # 选择执行程序FTPRO_INDEX 19
        time.sleep(0.5)
        FTC_SetDKAssemFlag(0)  # 关闭程序
        time.sleep(0.5)
        self.get_logger().info('🔧 FTC program disabled')

        pose = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
        pose_rad = ConvertDeg2Rad(pose)
        res = self.robot.movej2(pose_rad, 1.5, 1.0, 0.0, True, self.op)
        time.sleep(0.5)
        self.get_logger().info('🤖 Robot arm back to zero position')


def main(args=None):
    rclpy.init(args=args)
    node = RobotArmTeleop()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('🛑 Robot Arm Teleop Node Stopped')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
