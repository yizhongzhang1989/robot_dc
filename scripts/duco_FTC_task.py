import sys
import os
import requests
import math
import time

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

def main():  
    
    # create the DucoCobot instance 
    robot = DucoCobot(ip, port)  
    # open connection, power on and enable the robot arm
    res = robot.open()  
    print("Open connection:", res)  
    # res = robot.power_on(True)  
    # print("Power on:", res)  
    # res = robot.enable(True)  
    # print("Enable:", res)  
        
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

    # check the status of ForceMaster.exe
    status = get_FTCexe_status()['status']
    time.sleep(0.5)
    if status != 'running':
        res = start_FTCexe()
        print(f"start FTCexe: {res}\n")
        time.sleep(1.0)
    # Wait for status to become 'running'
    while True:
        current_status = get_FTCexe_status()['status']
        if current_status == 'running':
            print("ForceMaster.exe is 'running', continuing execution...")
            break
        else:
            # print(f"Current status: {current_status}, waiting...")
            time.sleep(1)  # Wait 1 second before checking again
    # ======================================================

    # check the tools status
    status = get_tool_status()
    print(f"current status: {status}\n")
    # if stickP is not picked up, retrieve it, then execute FTC tasks.
    if status[3]:
        res = switch_tool("stickP")
        print(f"switch stickP: {res}\n")

    # when getting stickP, move to task start point
    pose = [-97.709, -46.951, 131.34, -84.59, -81.33, 91.201]
    pose_rad = ConvertDeg2Rad(pose)
    res = robot.movej2(pose_rad, 2.0, 1.0, 0.0, True, op)
    time.sleep(0.5)

    
            
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
