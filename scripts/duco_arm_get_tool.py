import sys
import time
import os
import math

try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'colcon_ws', 'src'))
    from common.workspace_utils import get_workspace_root
    repo_root_path = get_workspace_root()
except ImportError:
    current_file_path = os.path.dirname(os.path.abspath(__file__))
    repo_root_path = os.path.abspath(os.path.join(current_file_path, '..'))
duco_script_path = os.path.join(repo_root_path, 'colcon_ws/src/duco_robot_arm/duco_robot_arm')
gen_py_path = os.path.join(duco_script_path, 'gen_py')
lib_path = os.path.join(duco_script_path, 'lib')  # Fixed: Use correct lib path
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

def arm_move2zero(duco_cobot, op):
    pose_allzero = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
    pose_rad = ConvertDeg2Rad(pose_allzero)
    res = duco_cobot.movej2(pose_rad, 1.5, 1.0, 0.0, True, op)
    time.sleep(0.5)

    pose_begin = [65.91, -25.97, 75.00, 46.30, -92.78, -111.48]
    pose_begin_rad = ConvertDeg2Rad(pose_begin)
    res = duco_cobot.movej2(pose_begin_rad, 1.5, 1.0, 0.0, True, op)
    time.sleep(0.5)
    return res

def task_zero2stick(duco_cobot, op):
    program_name = "zero2stick.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res

def task_stick2zero(duco_cobot, op):
    program_name = "stick2zero.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res

def task_zero2jaw(duco_cobot, op):
    program_name = "zero2jaw.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res

def task_jaw2zero(duco_cobot, op):
    program_name = "jaw2zero.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res

def task_zero2holder(duco_cobot, op):
    program_name = "zero2holder.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res

def task_holder2zero(duco_cobot, op):
    program_name = "holder2zero.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


def main():
    # Create the DucoCobot instance and open connection
    duco_cobot = DucoCobot(ip_robot, port_robot)
    
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

    rlt = duco_cobot.open()
    print("open:", rlt)
    # rlt = duco_cobot.power_on(True)
    # print("power_on:", rlt)
    # rlt = duco_cobot.enable(True)
    # print("enable:", rlt)

    # ------------------------------            
    task_zero2stick(duco_cobot,op)    
    # ------------------------------  
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
        print("Thrift Exception:", tx.message)
    except Exception as e:
        print("General Exception:", str(e))