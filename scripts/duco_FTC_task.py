import sys
import os
import requests
import math
import time
import numpy as np

try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'colcon_ws', 'src'))
    from common.workspace_utils import get_workspace_root, get_scripts_directory
    repo_root_path = get_workspace_root()
    current_file_path = get_scripts_directory()
except ImportError:
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

# ===========================execution functions for exchange tools========================
def get_tool_status():

    try:
        response = requests.get(f"{base_url}/api/tool_control/status", timeout=5)
        if response.status_code == 200:
            res = [response.json()['program'],response.json()['gripper'],response.json()['frame'],response.json()['stickP'],response.json()['stickR']]
            return res
        else:
            return None
    except Exception:
        return None

def switch_tool(tool_type):

    try:
        payload = {"tool_type": tool_type}
        response = requests.post(f"{base_url}/api/tool_control/execute", json=payload, timeout=300)
        return True
    except Exception:
        return False

# ===========================execution functions for robot arm moving=======================
def Move2_task_startpoint(robot,op):

    pose = [48.07, -21.83, -99.26, -56.87, -40.17, -91.59]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    print("Robot arm move to default starting point of all tasks.")
    return res

def Move2_taskpush_startpoint(robot,op):
    
    pose = [25.62, -41.70, -100.77, -36.02, -62.57, -90.69]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)
    print("Robot arm is moved to starting point of \"task push\"")

    return res

def Move2_taskcloseleft_startpoint(robot,op):
    
    Move2_taskpush_startpoint(robot,op)

    offset = [-150/1000, 0/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(1)  

    offset = [0/1000, 150/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.3, 0.2, 0.0, '', True, op)  
    time.sleep(1) 

    pose = [57.36, -41.06, -107.92, -28.49, -30.84, -92.22]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    print("Robot arm is moved to starting point of \"task closeleft\"")
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

'''FTC Task: step push server in task insert server'''
def FTC_task_pushserver(robot,op):

    # ==========================3.1 locate to the position before push the server============================
    Move2_taskpush_startpoint(robot,op)

    # 3.2 execute the task push server (FTC control)
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [False,False,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    maxForce_1 = [0,0,65,0,0,0]
    ifDKStopOnMaxForce_1 = True
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=3, timeEndLimit=60, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1)
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
        print(f"Response:{res.text}, FTC Program is enabled successfully! Arm is pushing the server...")
    time.sleep(0.5)

    # gradually increase the force
    target_force = [0,0,-60,0,0,0]
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

    # ==================================3.3 release the force=========================================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [False,False,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    ifNeedInit = False  # must use False, cause at this time, FTC has experienced force.
    B = [12000,12000,12000,1500,1500,1500]
    M = [1000,1000,1000,150,150,150]
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=3, timeEndLimit=2, ifNeedInit=ifNeedInit, B=B, M=M)
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
        print(f"Response:{res.text}, FTC Program is enabled successfully! Arm is releasing the force...")
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
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

    # 3.4 move to the task push starting point
    Move2_taskpush_startpoint(robot,op)

    return res

def FTC_task_lockserver(robot,op):

    # this task is executed after push server
    Move2_taskcloseleft_startpoint(robot,op)

    # 2.1 touch the handle
    res = FTC_start()
    if res.status_code == 200:
        print(f"FTC started! Response:{res.text}")
    time.sleep(1)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,-40,0,0,0]
    ftcEndType = 6
    maxForce_1 = [0,0,0.5,0,0,0]
    ifDKStopOnMaxForce_1 = True
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=ftcEndType, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1)
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
        print(f"Enable the FTC Program successfully! Response:{res.text}")
    time.sleep(1)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(1)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"FTC task hold knob is finished! Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

    # 2.2 push the handle to pre-close
    res = FTC_start()
    if res.status_code == 200:
        print(f"FTC started! Response:{res.text}")
    time.sleep(1)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,40,-40,0,0,0]
    ftcEndType = 7
    maxForce_1 = [50,50,50,0,0,0]
    ifDKStopOnMaxForce_1 = True
    disAng6D_EndLimit=[0,100,0,0,0,0]
    ifNeedInit=False
    res = FTC_setparams(ftEnabled=ftEnabled, ifNeedInit=ifNeedInit, ftSet=ftSet, ftcEndType=ftcEndType, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1,disAng6D_EndLimit=disAng6D_EndLimit)
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
        print(f"Enable the FTC Program successfully! Response:{res.text}")
    time.sleep(1)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(1)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"FTC task hold knob is finished! Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1) 

    # 2.5 slightly move the handle
    offset = [0/1000, -5/1000, -20/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.1, 0.1, 0.0, '', True, op)  
    time.sleep(1) 

    # 2.3 push the handle to the lock position
    res = FTC_start()
    if res.status_code == 200:
        print(f"FTC started! Response:{res.text}")
    time.sleep(1)

    ftEnabled = [False,False,True,False,False,False]
    ftSet = [0,0,-35,0,0,0]
    ftcEndType = 7
    maxForce_1 = [40,40,15,0,0,0]
    ifDKStopOnMaxForce_1 = True
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=ftcEndType, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1)
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
        print(f"Enable the FTC Program successfully! Response:{res.text}")
    time.sleep(1)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(1)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"FTC task hold knob is finished! Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

    # 2.9 release the force
    res = FTC_start()
    if res.status_code == 200:
        print(f"FTC started! Response:{res.text}")
    time.sleep(1)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    ftcEndType = 3
    timeEndLimit = 2
    ifNeedInit = False  # must use False, cause at this time, FTC has experienced force.
    B = [12000,12000,12000,1500,1500,1500]
    M = [1000,1000,1000,150,150,150]
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=ftcEndType, timeEndLimit=timeEndLimit, ifNeedInit=ifNeedInit, B=B, M=M)
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
        print(f"Enable the FTC Program successfully! Response:{res.text}")
    time.sleep(1)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"FTC task release force is finished! Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

    # # 2.4 alone the edge of handle to close the knob
    res = FTC_start()
    if res.status_code == 200:
        print(f"FTC started! Response:{res.text}")
    time.sleep(1)

    ftEnabled = [False,True,True,False,False,False]
    ftSet = [0,-6,-30,0,0,0]
    ftcEndType = 7
    maxForce_1 = [0,50,100,0,0,0]
    ifDKStopOnMaxForce_1 = True
    ifNeedInit=False
    disAng6D_EndLimit = [0,30,20,0,0,0]
    B = [2000,1500,4000,1500,1500,1500]
    M = [200,200,1000,150,150,150]
    res = FTC_setparams(ftEnabled=ftEnabled, ifNeedInit=ifNeedInit, ftSet=ftSet, ftcEndType=ftcEndType, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1, disAng6D_EndLimit=disAng6D_EndLimit, B=B, M=M)
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
        print(f"Enable the FTC Program successfully! Response:{res.text}")
    time.sleep(1)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(1)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"FTC task hold knob is finished! Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

    # 2.5 slightly move the handle
    offset = [0/1000, 0/1000, -100/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0, '', True, op)  
    time.sleep(1) 

    # 2.6 move to close right
    offset = [0/1000, -210/1000, 0/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0, '', True, op)  
    time.sleep(1) 

    # 2.7 hold the right handle
    res = FTC_start()
    if res.status_code == 200:
        print(f"FTC started! Response:{res.text}")
    time.sleep(1)

    ftEnabled = [True,False,True,False,False,False]
    ftSet = [0,0,-35,0,0,0]
    ftcEndType = 7
    maxForce_1 = [0,0,15,0,0,0]
    ifDKStopOnMaxForce_1 = True
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=ftcEndType, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1)
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
        print(f"Enable the FTC Program successfully! Response:{res.text}")
    time.sleep(1)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(1)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"FTC task hold knob is finished! Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

    # 2.9 release the force
    res = FTC_start()
    if res.status_code == 200:
        print(f"FTC started! Response:{res.text}")
    time.sleep(1)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    ftcEndType = 3
    timeEndLimit = 2
    ifNeedInit = False  # must use False, cause at this time, FTC has experienced force.
    B = [12000,12000,8000,1500,1500,1500]
    M = [1000,1000,1000,150,150,150]
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=ftcEndType, timeEndLimit=timeEndLimit, ifNeedInit=ifNeedInit, B=B, M=M)
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
        print(f"Enable the FTC Program successfully! Response:{res.text}")
    time.sleep(1)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"FTC task release force is finished! Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

    # # 2.10 alone the edge of handle to close the knob
    res = FTC_start()
    if res.status_code == 200:
        print(f"FTC started! Response:{res.text}")
    time.sleep(1)

    ftEnabled = [False,True,True,False,False,False]
    ftSet = [0,6,-30,0,0,0]
    ftcEndType = 7
    maxForce_1 = [0,50,100,0,0,0]
    ifDKStopOnMaxForce_1 = True
    ifNeedInit=False
    disAng6D_EndLimit = [0,35,20,0,0,0]
    B = [2000,1500,4000,1500,1500,1500]
    M = [200,200,1000,150,150,150]
    res = FTC_setparams(ftEnabled=ftEnabled, ifNeedInit=ifNeedInit, ftSet=ftSet, ftcEndType=ftcEndType, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1, disAng6D_EndLimit=disAng6D_EndLimit, B=B, M=M)
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
        print(f"Enable the FTC Program successfully! Response:{res.text}")
    time.sleep(1)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(1)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"FTC task hold knob is finished! Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

    # 2.5 back to start point 
    offset = [0/1000, 0/1000, -80/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset, 0.2, 0.2, 0.0, '', True, op)  
    time.sleep(1) 

    Move2_task_startpoint(robot,op)

    return res

def FTC_task_LockRightHandle(robot,op):
    # this task is executed after push server
    Move2_taskcloseleft_startpoint(robot,op)

    # =============================2.1 touch the handle=================================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    maxForce_1 = [0,0,0.5,0,0,0]
    ifDKStopOnMaxForce_1 = True
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=3, timeEndLimit=60, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1)
    if res.status_code == 200:
        print(f"Response:{res.text}, set FTC parameters successfully! ")
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
        print(f"Response:{res.text}, FTC Program is enabled successfully! Arm is holding the left handle...")
    time.sleep(1)

    # gradually increase the force
    target_force = [0,0,-40,0,0,0]
    current_force = ftSet[:]  # create a copy
    ftSetRT = [0, 0, 0, 0, 0, 0]
    while current_force[2] > target_force[2]:
        current_force[2] = current_force[2] - 2
        if current_force[2] < target_force[2]:
            current_force[2] = target_force[2]
        ftSetRT[2] = current_force[2]
        FTC_setFTValueRT(ftSetRT)
        time.sleep(0.2)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

    # =============================2.2 push the handle to semi-close=================================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    maxForce_1 = [50,50,50,0,0,0]
    ifDKStopOnMaxForce_1 = True
    ifNeedInit=False
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=7, disAng6D_EndLimit=[0,100,0,0,0,0], maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1, ifNeedInit=ifNeedInit,)
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
        print(f"Response:{res.text}, FTC Program is enabled successfully! Arm is pushing the left handle to semi-close...")
    time.sleep(1)

    # gradually increase the force in both y and z directions
    target_force = [0,40,-40,0,0,0]  # y=40, z=-40
    current_force = ftSet[:]  # create a copy
    ftSetRT = [0, 0, 0, 0, 0, 0]
    
    while current_force[1] < target_force[1] or current_force[2] > target_force[2]:
        # Gradually increase y force 
        if current_force[1] < target_force[1]:
            current_force[1] = current_force[1] + 2
            if current_force[1] > target_force[1]:
                current_force[1] = target_force[1]
        
        # Gradually increase z force 
        if current_force[2] > target_force[2]:
            current_force[2] = current_force[2] - 2
            if current_force[2] < target_force[2]:
                current_force[2] = target_force[2]
        
        ftSetRT[1] = current_force[1]
        ftSetRT[2] = current_force[2]
        FTC_setFTValueRT(ftSetRT)
        time.sleep(0.2)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC stopped!")
    time.sleep(1) 

    # ===========================2.3 push left handle to the lock position============================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    maxForce_1 = [30,30,35,0,0,0]
    ifDKStopOnMaxForce_1 = True
    ifNeedInit = False
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=3, timeEndLimit=60, maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1, ifNeedInit=ifNeedInit)
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
        print(f"Response:{res.text}, FTC Program is enabled successfully! Arm is pushing the left handle to lock position...")
    time.sleep(1)

    # gradually increase the force
    target_force = [0,0,-30,0,0,0]
    current_force = ftSet[:]  # create a copy
    ftSetRT = [0, 0, 0, 0, 0, 0]
    while current_force[2] > target_force[2]:
        current_force[2] = current_force[2] - 2
        if current_force[2] < target_force[2]:
            current_force[2] = target_force[2]
        ftSetRT[2] = current_force[2]
        FTC_setFTValueRT(ftSetRT)
        time.sleep(0.2)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    
    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC stopped!")
    time.sleep(1)

    # =======================2.4 alone the edge of handle to close the knob==============================
    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    maxForce_1 = [30,30,100,0,0,0]
    ifDKStopOnMaxForce_1 = True
    ifNeedInit=False
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=7, disAng6D_EndLimit=[0,30,20,0,0,0], maxForce_1=maxForce_1, ifDKStopOnMaxForce_1=ifDKStopOnMaxForce_1, ifNeedInit=ifNeedInit)
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
        print(f"Response:{res.text}, FTC Program is enabled successfully! Arm is closing the left knob alone the edge...")
    time.sleep(0.5)

    # gradually increase the force in both y and z directions
    target_force = [0,-10,-30,0,0,0]  # y=40, z=-40
    current_force = ftSet[:]  # create a copy
    ftSetRT = [0, 0, 0, 0, 0, 0]

    while current_force[1] < target_force[1] or current_force[2] > target_force[2]:
        # Gradually increase y force 
        if current_force[1] < target_force[1]:
            current_force[1] = current_force[1] - 2
            if current_force[1] > target_force[1]:
                current_force[1] = target_force[1]
        
        # Gradually increase z force 
        if current_force[2] > target_force[2]:
            current_force[2] = current_force[2] - 2
            if current_force[2] < target_force[2]:
                current_force[2] = target_force[2]
        
        ftSetRT[1] = current_force[1]
        ftSetRT[2] = current_force[2]
        FTC_setFTValueRT(ftSetRT)
        time.sleep(0.2)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)
    # when reaching the threshold of max force 1, Flag_maxf1 will be True, Flag_ok also become False
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        # print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"Current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

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

    # # move arm to task start point
    # Move2_task_startpoint(robot,op)

    # # push server into the rack
    # FTC_task_pushserver(robot,op)

    # FTC_task_LockRightHandle(robot,op)

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
