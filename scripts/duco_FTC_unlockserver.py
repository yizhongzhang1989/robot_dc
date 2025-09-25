import sys
import os
import requests
import math
import time
import numpy as np

# Add the duco_robot_arm directory and its lib subdirectory to the Python path
script_dir = os.path.dirname(os.path.abspath(__file__))
project_root = os.path.dirname(script_dir)
duco_robot_arm_dir = os.path.join(project_root, 'colcon_ws', 'src', 'duco_robot_arm', 'duco_robot_arm')
sys.path.insert(0, duco_robot_arm_dir)
sys.path.insert(0, os.path.join(duco_robot_arm_dir, 'lib'))

from DucoCobot import DucoCobot  
from gen_py.robot.ttypes import Op  
from thrift import Thrift  
from duco_FTCApiPost import *
from duco_FTCexe_control import *
   
# Robot connection parameters  
ip = '192.168.1.10'     # robot_arm
port = 7003  
# make sure the web is running
robot_arm_web_url = "http://localhost:8080"

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

# ===========================execution functions for exchange tools========================
def get_tool_status():

    try:
        response = requests.get(f"{robot_arm_web_url}/api/tool_control/status", timeout=5)
        if response.status_code == 200:
            data = response.json()
            tool_states = data.get('tool_states', {})
            program_completed = data.get('program_completed', False)
            res = [
                program_completed,
                tool_states.get('gripper', False),
                tool_states.get('frame', False),
                tool_states.get('stickP', False),
                tool_states.get('stickR', False)
            ]
            return res
        else:
            return None
    except Exception:
        return None

def switch_tool(tool_type):

    try:
        payload = {"tool_type": tool_type}
        response = requests.post(f"{robot_arm_web_url}/api/tool_control/execute", json=payload, timeout=300)
        return True
    except Exception:
        return False

# ===========================execution functions for robot arm moving=======================
def Move2_task_startpoint(robot,op):


    # move to safty middle points 
    pose = [65.91, -23.59, -118.22, 44.30, -92.78, -111.48]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    # move to safty middle points
    pose = [43.07, 6.51, -124.52, 39.71, -87.72, -100.47]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    # estimated target pose
    pose = [64.92, -43.72, -97.76, -39.30, -26.61, -89.56]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    offset = [-70/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    print("Robot arm move to default starting point of all tasks.")
    return res

def Move2_taskunlockleft_startpoint(robot,op):

    # estimated target pose
    pose = [64.92, -43.72, -97.76, -39.30, -26.61, -89.56]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    offset = [-70/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    print("Robot arm is moved to starting point of \"task unlockleft\".")
    return res

def Move2_taskunlockright_startpoint(robot,op):

    # estimated target pose
    pose = [64.92, -43.72, -97.76, -39.30, -26.61, -89.56]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    offset = [-70/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    offset = [5/1000, -205/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    print("Robot arm is moved to starting point of \"task unlockright\".")
    return res

# ===========================execution functions for FTC tasks===============================
'''general function of FTC tasks'''
def FTC_setparams(ftEnabled, ftSet, isProgram=False, ftcProgram=None, onlyMonitor=False, graCalcIndex=3, 
                  dead_zone=[1,1,1,0.1,0.1,0.1], disEndLimit=5000, angleEndLimit=30,timeEndLimit=60, ftEndLimit=[0,0,0,0,0,0], 
                  disAng6D_EndLimit=[0,0,0,0,0,0], ftcEndType=6, quickSetIndex=[0,0,0,0,0,0], 
                  B=[2000,2000,2000,1500,1500,1500], M=[200,200,200,150,150,150], vel_limit=[500,500,500,500,500,500], cor_pos_limit=[1,1,1,0.5,0.5,0.5], 
                  maxForce_1=[0,0,0,0,0,0], ifDKStopOnMaxForce_1=False, ifRobotStopOnMaxForce_1=False, 
                  maxForce_2=[0,0,0,0,0,0], ifDKStopOnMaxForce_2=False, ifRobotStopOnMaxForce_2=False, 
                  ifDKStopOnTimeDisMon=False, ifRobotStopOnTimeDisMon=False, ifNeedInit=True, 
                  withGroup=False, ftcSetGroup=[9], ignoreSensor=False):

    # for move alone positive direction of z-axis, ftSet_z = negtive 

    res = FTC_setFTValueAll(isProgram, ftcProgram, onlyMonitor, graCalcIndex, ftEnabled, ftSet, dead_zone, disEndLimit, angleEndLimit,
                      timeEndLimit, ftEndLimit, disAng6D_EndLimit, ftcEndType, quickSetIndex, B, M, vel_limit,
                      cor_pos_limit, maxForce_1, ifDKStopOnMaxForce_1, ifRobotStopOnMaxForce_1, maxForce_2,
                      ifDKStopOnMaxForce_2, ifRobotStopOnMaxForce_2, ifDKStopOnTimeDisMon, ifRobotStopOnTimeDisMon,
                      ifNeedInit, withGroup, ftcSetGroup, ignoreSensor)
    return res

'''FTC Task: step unlock left knob in task extract server'''
def FTC_task_unlockleftknob(robot,op):

    # ============================5.1 locate to the position of task unlock left knob=============================
    Move2_taskunlockleft_startpoint(robot,op)

    # ============================5.2 move stickR to hold the knob (FTC control)==================================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    maxForce_1 = [0,0,10,0,0,0]
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=3, timeEndLimit=20, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=True)
    if res.status_code == 200:
        print(f"Response:{res.text}, set FTC parameters successfully!")
    time.sleep(0.5)
    
    # set FTC program index
    FTC_program_index = 10    
    res = FTC_SetIndex(FTC_program_index)
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program Index is set to {FTC_program_index}! ")
    time.sleep(0.5)

    # enable the program
    res = FTC_SetDKAssemFlag(1)   
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program is enabled! Arm is going to hold the left knob.")
    time.sleep(0.5)

    # gradually increase the force
    target_force = [0,0,-40,0,0,0]
    current_force = ftSet[:]  # create a copy
    ftSetRT = [0, 0, 0, 0, 0, 0]
    while current_force[2] > target_force[2]:
        current_force[2] = current_force[2] - 10
        if current_force[2] < target_force[2]:
            current_force[2] = target_force[2]
        ftSetRT[2] = current_force[2]
        FTC_setFTValueRT(ftSetRT)
        time.sleep(0.5)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]...")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC stopped! Arm is holding the left knob.")
    time.sleep(1)

    # =============================5.3 release the force=============================================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    ifNeedInit = False  # must use False, cause at this time, FTC has experienced force.
    B = [15000,15000,18000,1500,1500,1500]
    M = [1500,1500,1800,150,150,150]
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=3, timeEndLimit=2, ifNeedInit=ifNeedInit, B=B, M=M)
    if res.status_code == 200:
        print(f"Response:{res.text}, set FTC parameters successfully! ")
    time.sleep(0.5)
    
    # set FTC program index
    FTC_program_index = 10    
    res = FTC_SetIndex(FTC_program_index)
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program Index is set to {FTC_program_index}! ")
    time.sleep(0.5)

    # enable the program
    res = FTC_SetDKAssemFlag(1)   
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program is enabled! Arm is releasing the holding force.")
    time.sleep(0.5)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]...")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC stopped!")
    time.sleep(1)

    # ============================5.4 turn the left knob to the limited max angle (FTC control)=================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started! ")
    time.sleep(0.5)

    ftEnabled = [False,False,True,False,True,True]
    ftSet = [0,0,-2,0,0,3]
    maxForce_1 = [0,0,50,0,0,1]
    ifDKStopOnMaxForce_1 = True
    B = [12000,12000,4000,1500,7500,1500]
    M = [1200,1200,400,150,750,150]
    ifNeedInit = False  # must use False, cause at this time, FTC has experienced force.
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=7, disAng6D_EndLimit=[0,0,0,0,135,0], maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1, B=B, M=M, ifNeedInit=ifNeedInit)
    if res.status_code == 200:
        print(f"Response:{res.text}, set FTC parameters successfully!")
    time.sleep(0.5)
    
    # set FTC program index
    FTC_program_index = 10    
    res = FTC_SetIndex(FTC_program_index)
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program Index is set to {FTC_program_index}!")
    time.sleep(0.5)

    # enable the program
    res = FTC_SetDKAssemFlag(1)   
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program is enabled! Arm is unlocking the left knob.")
    time.sleep(0.5)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]...")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC stopped!")
    time.sleep(1)

    # ==========================5.6 reverse rotate to release the knob=================================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [True,True,True,False,True,True]
    ftSet = [0,0,3,0,0,-2]
    maxForce_1 = [20,20,20,0,0,0.5]
    ifDKStopOnMaxForce_1 = True
    B = [15000,15000,15000,1500,7500,1500]
    M = [1000,1000,1000,150,750,150]
    ifNeedInit = False  # must use False, cause at this time, FTC has experienced force.
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=7 , disAng6D_EndLimit=[0,0,0,0,45,0], maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1, B=B, M=M, ifNeedInit=ifNeedInit)
    if res.status_code == 200:
        print(f"Response:{res.text}, set FTC parameters successfully!")
    time.sleep(0.5)
    
    # set FTC program index
    FTC_program_index = 10    
    res = FTC_SetIndex(FTC_program_index)
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program Index is set to {FTC_program_index}!")
    time.sleep(0.5)

    # enable the program
    res = FTC_SetDKAssemFlag(1)   
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program is enabled! Arm is releasing the left knob.")
    time.sleep(0.5)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC stopped!")
    time.sleep(1)

    # ===========================5.7 backward to avoid the handle=================================
    offset = [0/1000, 0/1000, -80/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0, '', True, op)
    time.sleep(1)

    Move2_taskunlockleft_startpoint(robot,op)
    
    return res

'''FTC Task: step unlock right knob in task extract server'''
def FTC_task_unlockrightknob(robot,op):

    # ========================5.1 locate at the position of unlock right knob==========================
    Move2_taskunlockright_startpoint(robot,op)

    # ========================5.2 move stickR to hold the knob (FTC control)=========================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    maxForce_1 = [0,0,10,0,0,0]
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=3, timeEndLimit=20, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=True)
    if res.status_code == 200:
        print(f"Response:{res.text}, set FTC parameters successfully!")
    time.sleep(0.5)
    
    # set FTC program index
    FTC_program_index = 10    
    res = FTC_SetIndex(FTC_program_index)
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program Index is set to {FTC_program_index}! ")
    time.sleep(0.5)

    # enable the program
    res = FTC_SetDKAssemFlag(1)   
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program is enabled! Arm is going to hold the right knob.")
    time.sleep(0.5)

    # gradually increase the force
    target_force = [0,0,-40,0,0,0]
    current_force = ftSet[:]  # create a copy
    ftSetRT = [0, 0, 0, 0, 0, 0]
    while current_force[2] > target_force[2]:
        current_force[2] = current_force[2] - 10
        if current_force[2] < target_force[2]:
            current_force[2] = target_force[2]
        ftSetRT[2] = current_force[2]
        FTC_setFTValueRT(ftSetRT)
        time.sleep(0.5)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]...")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC stopped! Arm is holding the right knob.")
    time.sleep(1)

    # ============================5.3 release the force=============================================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    ifNeedInit = False  # must use False, cause at this time, FTC has experienced force.
    B = [12000,12000,15000,1500,1500,1500]
    M = [1500,1500,1500,150,150,150]
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=3, timeEndLimit=2, ifNeedInit=ifNeedInit, B=B, M=M)
    if res.status_code == 200:
        print(f"Response:{res.text}, set FTC parameters successfully! ")
    time.sleep(0.5)
    
    # set FTC program index
    FTC_program_index = 10    
    res = FTC_SetIndex(FTC_program_index)
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program Index is set to {FTC_program_index}! ")
    time.sleep(0.5)

    # enable the program
    res = FTC_SetDKAssemFlag(1)   
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program is enabled! Arm is releasing the holding force.")
    time.sleep(0.5)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]...")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC stopped!")
    time.sleep(1)

    # =================5.4 turn the right knob to the limited max angle (FTC control)=================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started! ")
    time.sleep(0.5)

    ftEnabled = [False,False,True,False,True,True]
    ftSet = [0,0,-3,0,0,-3]
    maxForce_1 = [0,0,50,0,0,1]
    ifDKStopOnMaxForce_1 = True
    B = [12000,12000,4000,1500,7500,1500]
    M = [1500,1500,400,150,750,150]
    ifNeedInit = False  # must use False, cause at this time, FTC has experienced force.
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=7, disAng6D_EndLimit=[0,0,0,0,150,0], maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1, B=B, M=M, ifNeedInit=ifNeedInit)
    if res.status_code == 200:
        print(f"Response:{res.text}, set FTC parameters successfully!")
    time.sleep(0.5)
    
    # set FTC program index
    FTC_program_index = 10    
    res = FTC_SetIndex(FTC_program_index)
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program Index is set to {FTC_program_index}!")
    time.sleep(0.5)

    # enable the program
    res = FTC_SetDKAssemFlag(1)   
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program is enabled! Arm is unlocking the right knob.")
    time.sleep(0.5)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]...")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC stopped!")
    time.sleep(1)

    # ==========================5.6 reverse rotate to release the knob=================================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [True,True,True,False,True,True]
    ftSet = [0,0,3,0,0,2]
    maxForce_1 = [20,20,20,0,1,1]
    ifDKStopOnMaxForce_1 = True
    B = [15000,15000,15000,1500,7500,1500]
    M = [1000,1000,1000,150,750,150]
    ifNeedInit = False  # must use False, cause at this time, FTC has experienced force.
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=7, disAng6D_EndLimit=[0,0,0,0,135,0], maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1, B=B, M=M, ifNeedInit=ifNeedInit)
    if res.status_code == 200:
        print(f"Response:{res.text}, set FTC parameters successfully!")
    time.sleep(0.5)
    
    # set FTC program index
    FTC_program_index = 10    
    res = FTC_SetIndex(FTC_program_index)
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program Index is set to {FTC_program_index}!")
    time.sleep(0.5)

    # enable the program
    res = FTC_SetDKAssemFlag(1)   
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program is enabled! Arm is releasing the right knob.")
    time.sleep(0.5)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC stopped!")
    time.sleep(1)

    # ===========================5.6 backward to avoid the handle=================================
    offset2 = [0/1000, 0/1000, -80/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset2, 0.2, 0.2, 0.0, '', True, op)
    time.sleep(1)

    Move2_taskunlockleft_startpoint(robot,op)

    return res

'''Task: step unlock handles in task extract server, based on TCP_move'''
def task_unlock2handle(robot,op):

    Move2_taskunlockright_startpoint(robot,op)

    # locate below the left handle
    offset = [0/1000, 150/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(1)  

    offset = [40/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(1)  

    offset = [0/1000, 0/1000, 60/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(1)  

    # up to unlock left
    offset = [-25/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(1)  


    offset = [25/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(1)

    # locate below the right handle
    offset = [0/1000, -100/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(1)   

    # up to unlock left
    offset = [-25/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(1)  

    offset = [25/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(1)

    # back 
    offset = [0/1000, 50/1000, -100/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(1)

    Move2_task_startpoint(robot,op)
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

    # 1. check the tools status (after opening the web) 
    status = get_tool_status()
    print(f"current tools status: {status}")
    # if stickP is not picked up, retrieve it, then execute FTC tasks.
    if status is not None and status[4]:
        res = switch_tool("stickR")
        print(f"switch stickR: {res}\n")
    elif status is None:
        print("Warning: Failed to get tool status, skipping tool switching...")

    # # 2. move to task start point
    # Move2_task_startpoint(robot,op)
    
    # # 3. task unlock server (finished)
    # FTC_task_unlockleftknob(robot,op)

    # FTC_task_unlockrightknob(robot,op)

    # task_unlock2handle(robot,op)


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
