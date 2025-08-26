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

    pose = [-97.709, -46.951, 131.34, -84.59, -81.33, 91.201]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)
    return res

def Move2_taskpush_startpoint(robot,op):
    
    pose = [-126.14, -16.43, 110.57, -94.40, -52.88, 91.38]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    offset1 = [0/1000, 0/1000, 120/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset1, 0.3, 0.2, 0.0, '', True, op)
    time.sleep(0.5)

    return res

# ===========================execution functions for FTC tasks===============================
def FTC_setparams(ftEnabled, ftSet, isProgram=False, ftcProgram=None, onlyMonitor=False, graCalcIndex=3, 
                  dead_zone=[1,1,1,0.1,0.1,0.1], disEndLimit=5000, timeEndLimit=60, ftEndLimit=[0,0,0,0,0,0], 
                  disAng6D_EndLimit=[0,0,0,0,0,0], ftcEndType=6, quickSetIndex=[0,0,0,0,0,0], 
                  B=[2000,2000,2000,1500,1500,1500], M=[20,20,20,25,25,25], vel_limit=[500,500,500,500,500,500], cor_pos_limit=[10,10,10,5,5,5], 
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

'''FTC Task: linear move'''
# ftSet = [0,0,-25,0,0,0]
# stop_type = 'distance'
# params = 100 # mm
# FTC_task_moveline(ftSet, stop_type, params)
def FTC_task_moveline(ftSet, stop_type, params):
    # stop_type = time, params = xx s; stop_type = distance, params = xx mm; stop_type = api, params = None
    # ftSet includes the direction information of the force or torque

    res = FTC_start()
    if res.status_code == 200:
        print(f"FTC started! Response:{res.text}")
    time.sleep(1)

    # Parsing input parameters
    if stop_type == 'distance':
        ftcEndType = 0
        res = FTC_setparams(ftEnabled=[True,True,True,True,True,True], ftSet=ftSet, ftcEndType=ftcEndType, disEndLimit=params)
        if res.status_code == 200:
            print(f"Set FTC parameters successfully! Response:{res.text}")
            print(f"FTC stop type is {stop_type}, ftcEndtype={ftcEndType}, disAng6D_EndLimit={params}, ftSet={ftSet}")
        time.sleep(1)
    # elif stop_type == 'force':
    #     ftcEndType = 1 
    #     res = FTC_setparams(ftEnabled=[True,True,True,True,True,True], ftSet=ftSet, ftcEndType=ftcEndType, ftEndLimit=params)
    #     if res.status_code == 200:
    #         print(f"Set FTC parameters successfully! Response:{res.text}")
    #         print(f"FTC stop type is {stop_type}, ftcEndtype={ftcEndType}, ftEndLimit={params}, ftSet={ftSet}")
    #     time.sleep(1)
    elif stop_type == 'distforce':
        ftcEndType = 2
        res = FTC_setparams(ftEnabled=[True,True,True,True,True,True], ftSet=ftSet, ftcEndType=ftcEndType, disEndLimit=params, dead_zone=[1,1,1,1,1,1])
        if res.status_code == 200:
            print(f"Set FTC parameters successfully! Response:{res.text}")
            print(f"FTC stop type is {stop_type}, ftcEndtype={ftcEndType}, disEndLimit={params}, ftSet={ftSet}")
        time.sleep(1)
    elif stop_type == 'time':
        ftcEndType = 3
        res = FTC_setparams(ftEnabled=[True,True,True,True,True,True], ftSet=ftSet, ftcEndType=ftcEndType, timeEndLimit=params)
        if res.status_code == 200:
            print(f"Set FTC parameters successfully! Response:{res.text}")
            print(f"FTC stop type is {stop_type}, ftcEndtype={ftcEndType}, timeEndLimit={params}, ftSet={ftSet}")
        time.sleep(1)
    elif stop_type == 'api':
        ftcEndType = 6
        res = FTC_setparams(ftEnabled=[True,True,True,True,True,True], ftSet=ftSet, ftcEndType=ftcEndType)
        if res.status_code == 200:
            print(f"Set FTC parameters successfully! Response:{res.text}")
            print(f"FTC stop type is {stop_type}, ftcEndtype={ftcEndType}, ftSet={ftSet}")
        time.sleep(1)
    else:
        print("Error: unsupported stop type!")
        return None
    
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

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(1)
    print(f"FTC task moveline is finished! Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    # stop FTC to avoid the move function of robot arm
    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(0.5)

    print("FTC task moveline executed successfully!")

    return res

'''FTC Task: rotation around an axis'''
# ftSet = [0,0,0,0,0,10]
# stop_type = 'angle'
# params = [0,0,0,0,90,0] # deg
# FTC_task_rotate(ftSet, stop_type, params)
def FTC_task_rotate(ftSet, stop_type, params):
    # stop_type = time, params = xx s; stop_type = angle, params = [0,0,0,xx,xx,xx] deg (must positive value in base frame); stop_type = api, params = None
    # so if we want the tool frame rotate alone Rz 90 deg, params = [0,0,0,0,90,0]
    # ftSet includes the direction information of the force or torque

    res = FTC_start()
    if res.status_code == 200:
        print(f"FTC started! Response:{res.text}")
    time.sleep(1)

    # Parsing input parameters
    if stop_type == 'angle':
        ftcEndType = 7
        res = FTC_setparams(ftEnabled=[False,False,False,False,False,True], ftSet=ftSet, ftcEndType=ftcEndType, disAng6D_EndLimit=params)
        if res.status_code == 200:
            print(f"Set FTC parameters successfully! Response:{res.text}")
            print(f"FTC stop type is {stop_type}, ftcEndtype={ftcEndType}, disAng6D_EndLimit={params}, ftSet={ftSet}")
        time.sleep(1)
    elif stop_type == 'time':
        ftcEndType = 3
        res = FTC_setparams(ftEnabled=[False,False,False,False,False,True], ftSet=ftSet, ftcEndType=ftcEndType, timeEndLimit=params)
        if res.status_code == 200:
            print(f"Set FTC parameters successfully! Response:{res.text}")
            print(f"FTC stop type is {stop_type}, ftcEndtype={ftcEndType}, timeEndLimit={params}, ftSet={ftSet}")
        time.sleep(1)
    elif stop_type == 'api':
        ftcEndType = 6
        res = FTC_setparams(ftEnabled=[False,False,False,False,False,True], ftSet=ftSet, ftcEndType=ftcEndType)
        if res.status_code == 200:
            print(f"Set FTC parameters successfully! Response:{res.text}")
            print(f"FTC stop type is {stop_type}, ftcEndtype={ftcEndType}, ftSet={ftSet}")
        time.sleep(1)
    else:
        print("Error: unsupported stop type!")
        return None
        
    FTC_program_index = 10    
    res = FTC_SetIndex(FTC_program_index)
    if res.status_code == 200:
        print(f"Set FTC Program Index to {FTC_program_index}! Response:{res.text}")
    time.sleep(1)

    res = FTC_SetDKAssemFlag(1)   # enable the program
    if res.status_code == 200:
        print(f"Enable the FTC Program Successfully! Response:{res.text}")
    time.sleep(1)

    # ensure the program finish
    flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
    print(f"Initial FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
    time.sleep(0.5)
    while flag_ok:
        flag_ok, flag_maxf1, flag_maxf2, flag_timedis = FTC_getFTFlag()
        print(f"current FTC Task Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")
        time.sleep(2)
    print(f"FTC task moveline is finished! Flag: [{flag_ok},{flag_maxf1},{flag_maxf2},{flag_timedis}]")

    res = FTC_stop()
    if res.status_code == 200:
        print(f"FTC stopped! Response:{res.text}")
    time.sleep(1)

    print("FTC task circular rotation executed successfully!")

    return res

'''FTC Task: step push server in task insert server'''
def FTC_task_pushserver(robot,op):

    # 3.1 locate to the position before push the server, then execute the task push server (FTC control)
    Move2_taskpush_startpoint(robot,op)

    # 3.2 FTC_task_pushbox
    ftSet = [0,0,-60,0,0,0]
    stop_type = 'distance'
    params = 155 # mm
    res = FTC_task_moveline(ftSet, stop_type, params)

    # 3.3 back to taskpush start point
    if res.status_code == 200:
        print(f"Task push server finished! Response:{res.text}")
        time.sleep(0.5)
        
    offset2 = [0/1000, 0/1000, -100/1000, np.radians(0), np.radians(0), np.radians(0)]  
    res = robot.tcp_move(offset2, 0.5, 0.5, 0.0, '', True, op)
    time.sleep(0.5)

    pose = [-126.14, -16.43, 110.57, -94.40, -52.88, 91.38]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

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

    # 2. when getting stickP, move to task start point
    Move2_task_startpoint(robot,op)

    # 3. task push server
    FTC_task_pushserver(robot,op)

    # locate to the position of rotating the knob
    # execute the FTC task



    
      

    # ======================================================
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
