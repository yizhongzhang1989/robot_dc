import sys
import os
import requests
import math
import time
import numpy as np

current_file_path = os.path.dirname(os.path.abspath(__file__))
repo_root_path = os.path.abspath(os.path.join(current_file_path, '..'))
duco_script_path = os.path.join(repo_root_path, 'colcon_ws/src/duco_robot_arm/duco_robot_arm')
gen_py_path = os.path.join(duco_script_path, 'gen_py')
lib_path = os.path.join(current_file_path, 'lib')
sys.path.append(duco_script_path)
sys.path.append(gen_py_path)
sys.path.append(lib_path)

from DucoCobot import DucoCobot  
from gen_py.robot.ttypes import Op  
from thrift import Thrift  
from duco_FTCApiPost import *
from duco_FTCexe_control import *
   
# Robot connection parameters  
ip = '192.168.1.10'     # robot_arm
port = 7003  
# make sure the web is running
base_url = "http://localhost:8080"

# ============================mathmatical functions========================================
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

# ===========================execution functions for robot arm moving=======================
def Move2_task_startpoint(robot,op):

    pose = [48.07, -21.83, -99.26, -56.87, -40.17, -91.59]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    print("Robot arm move to default starting point of all tasks.")
    return res

# ===========================execution functions for FTC tasks===============================
'''general function of FTC tasks'''
def FTC_setparams(ftEnabled, ftSet, isProgram=False, ftcProgram=None, onlyMonitor=False, graCalcIndex=3, 
                  dead_zone=[1,1,1,0.1,0.1,0.1], disEndLimit=1000, timeEndLimit=60, ftEndLimit=[0,0,0,0,0,0], 
                  disAng6D_EndLimit=[0,0,0,0,0,0], ftcEndType=6, quickSetIndex=[0,0,0,0,0,0], 
                  B=[2000,2000,2000,1500,1500,1500], M=[200,200,200,150,150,150], vel_limit=[500,500,500,500,500,500], cor_pos_limit=[1,1,1,0.5,0.5,0.5], 
                  maxForce_1=[0,0,0,0,0,0], ifDKStopOnMaxForce_1=False, ifRobotStopOnMaxForce_1=False, 
                  maxForce_2=[0,0,0,0,0,0], ifDKStopOnMaxForce_2=False, ifRobotStopOnMaxForce_2=False, 
                  ifDKStopOnTimeDisMon=False, ifRobotStopOnTimeDisMon=False, ifNeedInit=True, 
                  withGroup=False, ftcSetGroup=[9], ignoreSensor=False):

    # for move alone positive direction of z-axis, ftSet_z = negtive 

    res = FTC_setFTValueAll(isProgram, ftcProgram, onlyMonitor, graCalcIndex, ftEnabled, ftSet, dead_zone, disEndLimit,
                      timeEndLimit, ftEndLimit, disAng6D_EndLimit, ftcEndType, quickSetIndex, B, M, vel_limit,
                      cor_pos_limit, maxForce_1, ifDKStopOnMaxForce_1, ifRobotStopOnMaxForce_1, maxForce_2,
                      ifDKStopOnMaxForce_2, ifRobotStopOnMaxForce_2, ifDKStopOnTimeDisMon, ifRobotStopOnTimeDisMon,
                      ifNeedInit, withGroup, ftcSetGroup, ignoreSensor)
    return res

# ==========================main function===================================================
def main():

    # =======================create the DucoCobot instance======================
    robot = DucoCobot(ip, port)
    
    # Set up an Op instance with no triggering events (default)  
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

    # =======================start robot arm system=============================
    # open connection, power on and enable the robot arm
    res = robot.open()  
    print("Open connection:", res)  
    # res = robot.power_on(True)  
    # print("Power on:", res)  
    # res = robot.enable(True)  
    # print("Enable:", res)  
    # ================ensure the ForceMaster.exe are running====================
    # check the status of ForceMaster.exe
    status = get_FTCexe_status()['status']
    time.sleep(0.5)
    if status != 'running':
        res = start_FTCexe()
        print(f"start FTCexe: {res}")
        time.sleep(10.0)
    # Wait for status to become 'running'
    while True:
        current_status = get_FTCexe_status()['status']
        if current_status == 'running':
            print("ForceMaster.exe is 'running', waiting for program initialize and then continue execution...")
            time.sleep(1)
            break
        else:
            # print(f"Current status: {current_status}, waiting...")
            time.sleep(1)  # Wait 1 second before checking again
    # ====================Functions of task execution===========================

    # # 1. check the tools status (after opening the web) 
    # status = get_tool_status()
    # print(f"current tools status: {status}")
    # # if stickP is not picked up, retrieve it, then execute FTC tasks.
    # if status[3]:
    #     res = switch_tool("stickP")
    #     print(f"switch stickP: {res}\n")

    Move2_task_startpoint(robot,op)
    
    res = FTC_start()
    if res.status_code == 200:
        print(f"FTC started! Response:{res.text}")
    time.sleep(1)

    ftEnabled = [False,False,False,False,False,True]
    ftSet = [0,0,0,0,0,0]
    ftcEndType = 7
    disAng6D_EndLimit = [0,0,0,0,90,0]
    maxForce_1 = [0,0,0,0,0,1]   # threshold of max force 1
    ifDKStopOnMaxForce_1 = True
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=ftcEndType, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1, disAng6D_EndLimit=disAng6D_EndLimit)
    if res.status_code == 200:
        print(f"Set FTC parameters successfully! Response:{res.text}")
    time.sleep(1)
    
    # set FTC program index
    FTC_program_index = 10    
    res = FTC_SetIndex(FTC_program_index)
    if res.status_code == 200:
        print(f"Set FTC Program Index to {FTC_program_index}! Response:{res.text}")
    time.sleep(1)

    # enable the program
    res = FTC_SetDKAssemFlag(1)   
    if res.status_code == 200:
        print(f"Enable the FTC Program Successfully! Response:{res.text}")
    time.sleep(1)

    # gradually increase the force
    target_force = [0,0,0,0,0,-5]
    current_force = ftSet[:]  # create a copy
    ftSetRT = [0, 0, 0, 0, 0, 0]
    while current_force[5] > target_force[5]:
        current_force[5] = current_force[5] - 1
        if current_force[5] < target_force[5]:
            current_force[5] = target_force[5]
        ftSetRT[5] = current_force[5]
        FTC_setFTValueRT(ftSetRT)
        time.sleep(0.5)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

    

    # ======================close the robot arm================================
    # disable, power off, and close connection
    # res = robot.disable(True)
    # print("\nDisable result:", res)
    # res = robot.power_off(True)  
    # print("Power off result:", res)  
    res = robot.close()  
    print("Close connection result:", res)  
     
if __name__ == '__main__':  
    try:  
        main()  
    except Thrift.TException as tx:  
        print("Thrift Exception:", tx.message)  