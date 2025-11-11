#!/usr/bin/env python3
"""
TAB25 Control Script - Sequential Task Execution
"""

import sys
import time
import os
import math
import requests
import socket
import subprocess

# AMR
script_dir = os.path.dirname(os.path.abspath(__file__))
third_party_dir = os.path.join(script_dir, 'ThirdParty')
sys.path.insert(0, third_party_dir)

from seer_control.dc_demo_2025_webapi_controller import DCDemo2025WebAPIController

amr_controller = None
AMR_WEB_API_URL = "http://192.168.1.142:5000"

# DUCO
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

duco_cobot = None
duco_ip = '192.168.1.10'
duco_port = 7003


# Lift Robot Web API
LIFT_WEB_BASE = "http://192.168.1.3:8090"


def init():
    """
    Initialize the global DCDemo2025WebAPIController and DucoCobot.
    """
    global amr_controller, duco_cobot
    print("="*60)
    print("🤖 TAB25 Control Script - Initializing")
    print("="*60)
    
    # Initialize AMR controller
    print(f"🌐 AMR Web API URL: {AMR_WEB_API_URL}")
    amr_controller = DCDemo2025WebAPIController(AMR_WEB_API_URL)
    
    # Check AMR connection status
    if amr_controller.is_connected():
        print("✅ AMR is connected")
    else:
        print("⚠️  AMR is not connected")
        print("   Please ensure the web server is running and robot is connected")
    
    # Initialize DUCO Cobot
    print(f"\n🦾 DUCO Cobot IP: {duco_ip}:{duco_port}")
    duco_cobot = DucoCobot(duco_ip, duco_port)
    rlt = duco_cobot.open()
    print(f"DUCO Cobot open result: {rlt}")
    
    if rlt == 0:
        print("✅ DUCO Cobot is connected")
    else:
        print("⚠️  DUCO Cobot connection failed")
    
    print("-"*60)


def amr_home():
    """
    Function 1: Move AMR to home position (LM2).
    
    Calls goto(target_id="LM2") with wait=True (blocking).
    """
    print("\n📍 Function 1: AMR Home")
    print("   Moving to home position LM2...")
    
    result = amr_controller.goto(target_id="LM2", wait=True)
    
    if result.get('success', False):
        print(f"✅ Successfully reached home position")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to reach home position: {result.get('message', 'Unknown error')}")
    
    return result


def amr_smalltest():
    """
    Execute small test trajectory.
    
    Calls navigate(trajectory="smalltest") with wait=True (blocking).
    """
    print("\n🚀 AMR Small Test")
    print("   Executing 'smalltest' trajectory...")
    
    result = amr_controller.navigate(trajectory="smalltest", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed smalltest trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete smalltest trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def amr_looptest():
    """
    Execute loop test trajectory.
    
    Calls navigate(trajectory="looptest") with wait=True (blocking).
    """
    print("\n🔄 AMR Loop Test")
    print("   Executing 'looptest' trajectory...")
    
    result = amr_controller.navigate(trajectory="looptest", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed looptest trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete looptest trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def amr_arm_dock2rack():
    """
    Execute arm dock to rack trajectory.
    
    Calls navigate(trajectory="arm_dock2rack") with wait=True (blocking).
    """
    print("\n📦 AMR Arm Dock to Rack")
    print("   Executing 'arm_dock2rack' trajectory...")
    
    result = amr_controller.navigate(trajectory="arm_dock2rack", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed arm_dock2rack trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete arm_dock2rack trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def amr_arm_rack2side():
    """
    Execute arm rack to side trajectory.
    
    Calls navigate(trajectory="arm_rack2side") with wait=True (blocking).
    """
    print("\n� AMR Arm Rack to Side")
    print("   Executing 'arm_rack2side' trajectory...")
    
    result = amr_controller.navigate(trajectory="arm_rack2side", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed arm_rack2side trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete arm_rack2side trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def amr_arm_side2rack():
    """
    Execute arm side to rack trajectory.
    
    Calls navigate(trajectory="arm_side2rack") with wait=True (blocking).
    """
    print("\n📦 AMR Arm Side to Rack")
    print("   Executing 'arm_side2rack' trajectory...")
    
    result = amr_controller.navigate(trajectory="arm_side2rack", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed arm_side2rack trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete arm_side2rack trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def amr_arm_rack2dock():
    """
    Execute arm rack to dock trajectory.
    
    Calls navigate(trajectory="arm_rack2dock") with wait=True (blocking).
    """
    print("\n📦 AMR Arm Rack to Dock")
    print("   Executing 'arm_rack2dock' trajectory...")
    
    result = amr_controller.navigate(trajectory="arm_rack2dock", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed arm_rack2dock trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete arm_rack2dock trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def amr_arm_dock2side():
    """
    Execute arm dock to side trajectory.
    
    Calls navigate(trajectory="arm_dock2side") with wait=True (blocking).
    """
    print("\n📦 AMR Arm Dock to Side")
    print("   Executing 'arm_dock2side' trajectory...")
    
    result = amr_controller.navigate(trajectory="arm_dock2side", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed arm_dock2side trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete arm_dock2side trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def amr_arm_side2dock():
    """
    Execute arm side to dock trajectory.
    
    Calls navigate(trajectory="arm_side2dock") with wait=True (blocking).
    """
    print("\n📦 AMR Arm Side to Dock")
    print("   Executing 'arm_side2dock' trajectory...")
    
    result = amr_controller.navigate(trajectory="arm_side2dock", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed arm_side2dock trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete arm_side2dock trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def amr_courier_dock2rack1():
    """
    Execute courier dock to rack trajectory.
    
    Calls navigate(trajectory="courier_dock2rack") with wait=True (blocking).
    """
    print("\n📮 AMR Courier Dock to Rack")
    print("   Executing 'courier_dock2rack' trajectory...")
    
    result = amr_controller.navigate(trajectory="courier_dock2rack1", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed courier_dock2rack1 trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete courier_dock2rack trajectory: {result.get('message', 'Unknown error')}")
    
    return result

def amr_courier_dock2rack2():
    """
    Execute courier dock to rack trajectory.
    
    Calls navigate(trajectory="courier_dock2rack") with wait=True (blocking).
    """
    print("\n📮 AMR Courier Dock to Rack")
    print("   Executing 'courier_dock2rack2' trajectory...")
    
    result = amr_controller.navigate(trajectory="courier_dock2rack2", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed courier_dock2rack trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete courier_dock2rack trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def amr_courier_rack2dock1():
    """
    Execute courier rack to dock trajectory.
    
    Calls navigate(trajectory="courier_rack2dock") with wait=True (blocking).
    """
    print("\n📮 AMR Courier Rack to Dock")
    print("   Executing 'courier_rack2dock1' trajectory...")
    
    result = amr_controller.navigate(trajectory="courier_rack2dock1", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed courier_rack2dock1 trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete courier_rack2dock trajectory: {result.get('message', 'Unknown error')}")
    
    return result

def amr_courier_rack2dock2():
    """
    Execute courier rack to dock trajectory.

    Calls navigate(trajectory="courier_rack2dock2") with wait=True (blocking).
    """
    print("\n📮 AMR Courier Rack to Dock")
    print("   Executing 'courier_rack2dock2' trajectory...")
    
    result = amr_controller.navigate(trajectory="courier_rack2dock2", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed courier_rack2dock2 trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete courier_rack2dock trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def duco_jspf_fold():
    program_name = "Fold.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


def duco_jspf_unfold():
    program_name = "Unfold.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


def duco_jspf_get_pushbutton_tool():
    program_name = "Get_pushbutton_tool.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


def duco_jspf_return_pushbutton_tool():
    program_name = "Return_pushbutton_tool.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


def duco_jspf_get_rotate_tool():
    program_name = "Get_rotate_tool.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


def duco_jspf_return_rotate_tool():
    program_name = "Return_rotate_tool.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


# ========================================================================
# UR15 Robot Control Functions - Run Standalone Scripts
# ========================================================================

def ur15_fold():
    """
    Fold UR15 robot arm to compact position by running Fold.py script.
    """
    print("\n📦 UR15 Fold")
    print("   Running Fold.py script...")
    
    script_path = os.path.join(script_dir, "ur_state_fold.py")
    result = subprocess.run(["python3", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR15 fold completed successfully")
        return 0
    else:
        print(f"❌ UR15 fold failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1


def ur15_unfold():
    """
    Unfold UR15 robot arm from compact position by running Unfold.py script.
    """
    print("\n📦 UR15 Unfold")
    print("   Running Unfold.py script...")
    
    script_path = os.path.join(script_dir, "ur_state_unfold.py")
    result = subprocess.run(["python3", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR15 unfold completed successfully")
        return 0
    else:
        print(f"❌ UR15 unfold failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1


def ur15_get_pptool():
    """
    Pick up the push tool from storage location by running Get_push_tool.py script.
    """
    print("\n🔧 UR15 Get Push Tool")
    print("   Running Get_push_tool.py script...")
    
    script_path = os.path.join(script_dir, "ur_get_pptool.py")
    result = subprocess.run(["python3", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR15 get push tool completed successfully")
        return 0
    else:
        print(f"❌ UR15 get push tool failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1


def ur15_return_pptool():
    """
    Return the push tool to storage location by running Return_push_tool.py script.
    """
    print("\n🔧 UR15 Return Push Tool")
    print("   Running Return_push_tool.py script...")
    
    script_path = os.path.join(script_dir, "ur_return_pptool.py")
    result = subprocess.run(["python3", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR15 return push tool completed successfully")
        return 0
    else:
        print(f"❌ UR15 return push tool failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1


def ur15_get_frame():
    """
    Pick up a frame from its location by running Get_frame.py script.
    """
    print("\n🖼️  UR15 Get Frame")
    print("   Running Get_frame.py script...")
    
    script_path = os.path.join(script_dir, "ur_get_frame.py")
    result = subprocess.run(["python3", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR15 get frame completed successfully")
        return 0
    else:
        print(f"❌ UR15 get frame failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1


def ur15_locate_handle1():
    """
    Locate handle 1 by running ur_locate_handle1.py script.
    """
    print("\n🔍 UR Locate Handle 1")
    print("   Running ur_locate_handle1.py script...")
    
    script_path = os.path.join(script_dir, "ur_locate_handle1.py")
    result = subprocess.run(["python3", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR locate handle 1 completed successfully")
        return 0
    else:
        print(f"❌ UR locate handle 1 failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1

def ur15_locate_handle2():
    """
    Locate handle 2 by running ur_locate_handle2.py script.
    """
    print("\n🔍 UR Locate Handle 2")
    print("   Running ur_locate_handle2.py script...")
    
    script_path = os.path.join(script_dir, "ur_locate_handle2.py")
    result = subprocess.run(["python3", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR locate handle 2 completed successfully")
        return 0
    else:
        print(f"❌ UR locate handle 2 failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1

def ur15_locate_frame():
    """
    Locate frame by running ur_locate_frame.py script.
    """
    print("\n🔍 UR Locate Frame")
    print("   Running ur_locate_frame.py script...")
    
    script_path = os.path.join(script_dir, "ur_locate_frame.py")
    result = subprocess.run(["python3", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR locate frame completed successfully")
        return 0
    else:
        print(f"❌ UR locate frame failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1

def ur15_execute_handle1():
    """
    Execute handle 1 by running ur_execute_handle1.py script.
    """
    print("\n🔍 UR Execute Handle 1")
    print("   Running ur_execute_handle1.py script...")
    
    script_path = os.path.join(script_dir, "ur_execute_handle1.py")
    result = subprocess.run(["python3", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR execute handle 1 completed successfully")
        return 0
    else:
        print(f"❌ UR execute handle 1 failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1

def ur15_execute_handle2():
    """
    Execute handle 2 by running ur_execute_handle2.py script.
    """
    print("\n🔍 UR Execute Handle 2")
    print("   Running ur_execute_handle1.py script...")
    
    script_path = os.path.join(script_dir, "ur_execute_handle2.py")
    result = subprocess.run(["python3", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR execute handle 2 completed successfully")
        return 0
    else:
        print(f"❌ UR execute handle 2 failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1

def ur15_execute_frame():
    """
    Execute frame by running ur_execute_frame.py script.
    """
    print("\n🔍 UR Execute Frame")
    print("   Running ur_execute_frame.py script...")
    
    script_path = os.path.join(script_dir, "ur_execute_frame.py")
    result = subprocess.run(["python3", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR execute frame completed successfully")
        return 0
    else:
        print(f"❌ UR execute frame failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1

def run():
    """
    Main execution function.
    """
    print("\n" + "="*60)
    print("🏃 Starting Sequential Task Execution")
    print("="*60)
    
    try:

        # step1 : locate handle1 
        ur15_locate_handle1()
        time.sleep(0.5)

        # step2 : amr courier dock2rack
        amr_courier_dock2rack1()
        time.sleep(0.5)

        # step3 : execute handle1 to extract server
        ur15_execute_handle1()
        time.sleep(0.5)

        # step4 : execute amr courier rack2dock
        amr_courier_rack2dock1()
        time.sleep(0.5)
                
    
    except Exception as e:
        print(f"\n❌ Error during execution: {e}")
        import traceback
        traceback.print_exc()
        print("="*60)


def cleanup():
    """
    Cleanup function to properly close connections.
    """
    global duco_cobot
    print("\n" + "="*60)
    print("🧹 Cleaning up...")
    print("="*60)
    
    if duco_cobot is not None:
        try:
            rlt = duco_cobot.close()
            print(f"DUCO Cobot close result: {rlt}")
            if rlt == 0:
                print("✅ DUCO Cobot connection closed")
            else:
                print("⚠️  DUCO Cobot close returned non-zero")
        except Exception as e:
            print(f"❌ Error closing DUCO Cobot: {e}")
    
    print("-"*60)


def main():
    """
    Main entry point.
    """
    try:
        # Initialize controllers
        init()
        
        # Execute sequential tasks
        run()
    
    except KeyboardInterrupt:
        print("\n\n🛑 Interrupted by user")
    
    except Exception as e:
        print(f"\n❌ Unexpected error in main: {e}")
        import traceback
        traceback.print_exc()
    
    finally:
        # Cleanup
        cleanup()


if __name__ == "__main__":
    main()
