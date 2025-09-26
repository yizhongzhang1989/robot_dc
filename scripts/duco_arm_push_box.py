import sys
import time
import os
import math
import numpy as np

# get current file path
current_file_path = os.path.dirname(os.path.abspath(__file__))
repo_root_path = os.path.abspath(os.path.join(current_file_path, '..'))
duco_script_path = os.path.join(repo_root_path, 'colcon_ws/src/duco_robot_arm/duco_robot_arm')
gen_py_path = os.path.join(duco_script_path, 'gen_py')
lib_path = os.path.join(duco_script_path, 'lib')  # Fixed: Use correct lib path
# Add the required paths for thrift and generated code
sys.path.append(duco_script_path)
sys.path.append(gen_py_path)
sys.path.append(lib_path)

from DucoCobot import DucoCobot  
from gen_py.robot.ttypes import Op  
from thrift import Thrift  
from duco_FTCApiPost import *

# 配置机器人的IP地址和端口号
ip_robot = '192.168.1.10'
port_robot = 7003

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

def Arm_movezero2taskbegin(duco_cobot, op):

    pose_begin = [65.91, -25.97, 75.00, 46.30, -92.78, -111.48]
    pose_begin_rad = ConvertDeg2Rad(pose_begin)
    res = duco_cobot.movej2(pose_begin_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    pose3 = [-97.71, -46.95, 131.34, -84.59, -81.33, 91.20]
    pose3_rad = ConvertDeg2Rad(pose3)
    res = duco_cobot.movej2(pose3_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    return res

def Arm_move2pushbegin(duco_cobot, op, flag):

    # flag = True = Push server, flag = False = back to start point
    if flag:
        # # 接近推服务器处附近
        offset = [-60/1000, -155/1000, 300/1000, np.radians(0), np.radians(0), np.radians(0)]    # push server
    else:
        offset = [60/1000, 155/1000, -300/1000, np.radians(0), np.radians(0), np.radians(0)]   # back to start point
    res = duco_cobot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    return res

def Task_pushbox(duco_cobot, op):
    
    # 接近推服务器处附近
    Arm_move2pushbegin(duco_cobot, op, True)  # True = close to push point

    offset1 = [0/1000, 0/1000, 180/1000, np.radians(0), np.radians(0), np.radians(0)]   # back to start point
    res = duco_cobot.tcp_move(offset1, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    offset2 = [0/1000, 0/1000, -180/1000, np.radians(0), np.radians(0), np.radians(0)]   # back to start point
    res = duco_cobot.tcp_move(offset2, 0.5, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    # 执行完任务后返回Task begin
    Arm_move2pushbegin(duco_cobot, op, False)  # False = back to start point

    pose3 = [-97.71, -46.95, 131.34, -84.59, -81.33, 91.20]
    pose3_rad = ConvertDeg2Rad(pose3)
    res = duco_cobot.movej2(pose3_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    return res

def Task_closeleft(duco_cobot, op):
    
    # 接近左边的把手，到关闭的开始点位(这里选择了一个极限位置，防止没推到)
    offset1 = [-40.5/1000, 80/1000, 280/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = duco_cobot.tcp_move(offset1, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    # 推动动作序列
    offset2 = [0/1000, 0/1000, 30/1000, np.radians(0), np.radians(0), np.radians(0)]   
    res = duco_cobot.tcp_move(offset2, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)
    
    offset3 = [0/1000, -100/1000, 20/1000, np.radians(0), np.radians(0), np.radians(0)]   
    res = duco_cobot.tcp_move(offset3, 0.4, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    offset4 = [0/1000, 0/1000, 80/1000, np.radians(0), np.radians(0), np.radians(0)]   
    res = duco_cobot.tcp_move(offset4, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)
    
    offset5 = [0/1000, -50/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]   
    res = duco_cobot.tcp_move(offset5, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    offset6 = [-10/1000, 0/1000, 55/1000, np.radians(0), np.radians(0), np.radians(0)]   
    res = duco_cobot.tcp_move(offset6, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    offset7 = [5/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]   
    res = duco_cobot.tcp_move(offset7, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    offset7 = [0/1000, 25/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]   
    res = duco_cobot.tcp_move(offset7, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    offset7 = [0/1000, 10/1000, 31/1000, np.radians(0), np.radians(0), np.radians(0)]   
    res = duco_cobot.tcp_move(offset7, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)
   
    # back 
    offset_backz = [0/1000, 0/1000, -216/1000, np.radians(0), np.radians(0), np.radians(0)]   
    res = duco_cobot.tcp_move(offset_backz, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    # # 参数确认一下再注释
    # offset_backyx = [5/1000, 115/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]   
    # res = duco_cobot.tcp_move(offset_backyx, 0.3, 0.2, 0.0, '', True, op)
    # time.sleep(0.5)

    # # 回到任务开始点位
    # offset1 = [40.5/1000, -80/1000, -280/1000, np.radians(0), np.radians(0), np.radians(0)]  
    # res = duco_cobot.tcp_move(offset1, 0.3, 0.2, 0.0, '', True, op)
    # time.sleep(0.5)

    # # z退出后等效了
    # pose3 = [-97.71, -46.95, 131.34, -84.59, -81.33, 91.20]
    # pose3_rad = ConvertDeg2Rad(pose3)
    # res = duco_cobot.movej2(pose3_rad, 2.0, 1.0, 0.0, True, op)
    # time.sleep(0.5)
    
    return res

def Task_closeright(duco_cobot, op):

    # 关闭左边之后,退出z之后
    pose = [-89.77, -1.26, 100.35, -99.30, -89.26, 91.25]
    pose_rad = ConvertDeg2Rad(pose)
    res = duco_cobot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    # 去往右边
    offset1 = [-16/1000, -200/1000, 140/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = duco_cobot.tcp_move(offset1, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    offset2 = [-10/1000, 0/1000, 45/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = duco_cobot.tcp_move(offset2, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    # 下压
    offset2 = [15/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = duco_cobot.tcp_move(offset2, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    offset2 = [0/1000, -60/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = duco_cobot.tcp_move(offset2, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    # # 摁进去旋钮
    offset8 = [0/1000, -10/1000, 30/1000, np.radians(0), np.radians(0), np.radians(0)]   
    res = duco_cobot.tcp_move(offset8, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    # back
    offset_z = [0/1000, 0/1000, -215/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = duco_cobot.tcp_move(offset_z, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    # offset_yx = [5/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    # res = duco_cobot.tcp_move(offset_yx, 0.3, 0.2, 0.0, '', True, op)
    # time.sleep(0.5)

    # # 回到关闭左边之后的位姿
    # offset2 = [16/1000, 200/1000, -140/1000, np.radians(0), np.radians(0), np.radians(0)]  
    # res = duco_cobot.tcp_move(offset2, 0.3, 0.2, 0.0, '', True, op)
    # time.sleep(0.5)

    # 退回z以后等效
    pose3 = [-97.71, -46.95, 131.34, -84.59, -81.33, 91.20]
    pose3_rad = ConvertDeg2Rad(pose3)
    res = duco_cobot.movej2(pose3_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)
    return res

def main():
    
    # 创建DucoCobot实例
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
    
    # =====================================================================
    # 示教器上的xyzrpy是世界坐标系的轴，tcp_move是工具坐标系的轴
    # 力控程序结束后要disabled，然后再执行运动程序
    # =====================================================================

    # Arm_movezero2taskbegin(duco_cobot, op)  # 移动到抓取完工具后的点位，再运动到执行任务的起点

    # Task_pushbox(duco_cobot, op)  # 推服务器任务

    # Task_closeleft(duco_cobot, op)  # 关闭左边把手任务

    # Task_closeright(duco_cobot, op)  # 关闭右边把手任务


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



