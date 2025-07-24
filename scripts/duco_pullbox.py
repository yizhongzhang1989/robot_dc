import sys
import time
import sys
import os
import math

# get current file path
current_file_path = os.path.dirname(os.path.abspath(__file__))
repo_root_path = os.path.abspath(os.path.join(current_file_path, '..'))
duco_script_path = os.path.join(repo_root_path, 'colcon_ws/src/duco_robot_arm/duco_robot_arm')
gen_py_path = os.path.join(duco_script_path, 'gen_py')
lib_path = os.path.join(current_file_path, 'lib')
# Add the required paths for thrift and generated code
sys.path.append(duco_script_path)
sys.path.append(gen_py_path)
sys.path.append(lib_path)

from DucoCobot import DucoCobot  
from gen_py.robot.ttypes import Op  
from thrift import Thrift  



# 配置机器人的IP地址和端口号
ip_robot = '192.168.1.10'
port_robot = 7003


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

def Arm_movetozero(robot, op):
    
    pose_zero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  
    pose_zero_rad = ConvertPose2Rad(pose_zero)
    res = robot.movej2(pose_zero_rad, 1.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    return res

def main():
    # 定义robot对象
    duco_cobot = DucoCobot(ip_robot, port_robot)

    # 初始化
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

    # =======================建立TCP通讯，上电，使能=======================
    rlt = duco_cobot.open()
    print("open:", rlt)
    # rlt = duco_cobot.power_on(True)
    # print("power_on:", rlt)
    # rlt = duco_cobot.enable(True)
    # print("enable:", rlt)
    
    # 推点的之前

    #======从初始状态开始====

    rlt = Arm_movetozero(duco_cobot, op)

    pose4 = [16.65,-16.55,44.33,-51.63,-80.88,80.26]
    pose4_rad = ConvertPose2Rad(pose4)
    res = duco_cobot.movej2(pose4_rad, 1.5, 1.0, 0.0, True, op)
    time.sleep(0.5)

    pose3 = [86.11,-38.55,81.25,-38.91,-63.96,80.26]
    pose3_rad = ConvertPose2Rad(pose3)
    res = duco_cobot.movej2(pose3_rad, 1.5, 1.0, 0.0, True, op)
    time.sleep(0.5)

    pose2 = [245.98,-38.55,81.25,-38.91,-63.96,80.26]
    pose2_rad = ConvertPose2Rad(pose2)
    res = duco_cobot.movej2(pose2_rad, 1.5, 1.0, 0.0, True, op)
    time.sleep(0.5)

    # 推点的附近
    pose1 = [245.95,-39.58,111.74,-70.76,-65.75,80.64]
    pose1_rad = ConvertPose2Rad(pose1)
    res = duco_cobot.movej2(pose1_rad, 1.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    pose0 = [223.50,-31.75,110.18,-76.58,-43.31,79.88]
    pose0_rad = ConvertPose2Rad(pose0)
    res = duco_cobot.movej2(pose0_rad, 1.0, 1.0, 0.0, True, op)
    time.sleep(0.5)
 
    offset = [0.0, 0.0, 0.40, 0.0, 0.0, 0.0]  
    res = duco_cobot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(2)

    offset = [0.0, 0.0, -0.40, 0.0, 0.0, 0.0]  
    res = duco_cobot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(2)

    rlt = Arm_movetozero(duco_cobot, op)
    time.sleep(0.5)

    # =======================结束任务，断使能，断点，断连接=======================
    # rlt = duco_cobot.disable(True)  # 断使能
    # print("\nDisable result:", rlt)
    # rlt= duco_cobot.power_off(True)  # 断电
    # print("Power off result:", rlt)
    rlt = duco_cobot.close()
    print("close:", rlt)

if __name__ == '__main__':
    try:
        main()
    except Thrift.TException as tx:
        print('%s' % tx.message)