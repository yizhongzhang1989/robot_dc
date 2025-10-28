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

# UR15 Robot - using standalone scripts
import subprocess

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


def amr_courier_dock2rack():
    """
    Execute courier dock to rack trajectory.
    
    Calls navigate(trajectory="courier_dock2rack") with wait=True (blocking).
    """
    print("\n📮 AMR Courier Dock to Rack")
    print("   Executing 'courier_dock2rack' trajectory...")
    
    result = amr_controller.navigate(trajectory="courier_dock2rack", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed courier_dock2rack trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete courier_dock2rack trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def amr_courier_rack2dock():
    """
    Execute courier rack to dock trajectory.
    
    Calls navigate(trajectory="courier_rack2dock") with wait=True (blocking).
    """
    print("\n📮 AMR Courier Rack to Dock")
    print("   Executing 'courier_rack2dock' trajectory...")
    
    result = amr_controller.navigate(trajectory="courier_rack2dock", wait=True)
    
    if result.get('success', False):
        print("✅ Successfully completed courier_rack2dock trajectory")
        if 'task_id' in result:
            print(f"   Task ID: {result['task_id']}")
    else:
        print(f"❌ Failed to complete courier_rack2dock trajectory: {result.get('message', 'Unknown error')}")
    
    return result


def duco_jspf_Fold():
    program_name = "Fold.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


def duco_jspf_Unfold():
    program_name = "Unfold.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


def duco_jspf_Get_pushbutton_tool():
    program_name = "Get_pushbutton_tool.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


def duco_jspf_Return_pushbutton_tool():
    program_name = "Return_pushbutton_tool.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


def duco_jspf_Get_rotate_tool():
    program_name = "Get_rotate_tool.jspf"
    res = duco_cobot.run_program(program_name, True)
    print(f"Run program \"{program_name}\" result: {res}")
    time.sleep(0.5)
    return res


def duco_jspf_Return_rotate_tool():
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
    
    script_path = os.path.join(script_dir, "Fold.py")
    result = subprocess.run(["python", script_path], capture_output=True, text=True)
    
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
    
    script_path = os.path.join(script_dir, "Unfold.py")
    result = subprocess.run(["python", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR15 unfold completed successfully")
        return 0
    else:
        print(f"❌ UR15 unfold failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1


def ur15_get_push_tool():
    """
    Pick up the push tool from storage location by running Get_push_tool.py script.
    """
    print("\n🔧 UR15 Get Push Tool")
    print("   Running Get_push_tool.py script...")
    
    script_path = os.path.join(script_dir, "Get_push_tool.py")
    result = subprocess.run(["python", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR15 get push tool completed successfully")
        return 0
    else:
        print(f"❌ UR15 get push tool failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1


def ur15_return_push_tool():
    """
    Return the push tool to storage location by running Return_push_tool.py script.
    """
    print("\n🔧 UR15 Return Push Tool")
    print("   Running Return_push_tool.py script...")
    
    script_path = os.path.join(script_dir, "Return_push_tool.py")
    result = subprocess.run(["python", script_path], capture_output=True, text=True)
    
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
    
    script_path = os.path.join(script_dir, "Get_frame.py")
    result = subprocess.run(["python", script_path], capture_output=True, text=True)
    
    if result.returncode == 0:
        print("✅ UR15 get frame completed successfully")
        return 0
    else:
        print(f"❌ UR15 get frame failed")
        if result.stderr:
            print(f"   Error: {result.stderr}")
        return -1


# ========================================================================
# Lift Robot Platform and Pushrod Control Functions
# ========================================================================

def lift_platform_up():
    """
    Move the lift platform upward (pulse relay).
    
    Sends a POST request to the lift web service to trigger upward motion.
    """
    print("\n⬆️  Lift Platform Up")
    print("   Sending UP command to lift platform...")
    
    url = f"{LIFT_WEB_BASE}/api/cmd"
    payload = {"command": "up", "target": "platform"}
    headers = {"Content-Type": "application/json"}
    
    try:
        response = requests.post(url, json=payload, headers=headers, timeout=5)
        if response.ok:
            print("✅ Lift platform UP command sent successfully")
            return response.json()
        else:
            print(f"❌ Lift platform UP command failed: HTTP {response.status_code}")
            return {"success": False, "status_code": response.status_code}
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to lift platform web service")
        return {"success": False, "error": "Connection failed"}
    except requests.exceptions.Timeout:
        print("❌ Timeout sending lift platform UP command")
        return {"success": False, "error": "Timeout"}
    except Exception as e:
        print(f"❌ Error sending lift platform UP command: {e}")
        return {"success": False, "error": str(e)}


def lift_platform_down():
    """
    Move the lift platform downward (pulse relay).
    
    Sends a POST request to the lift web service to trigger downward motion.
    """
    print("\n⬇️  Lift Platform Down")
    print("   Sending DOWN command to lift platform...")
    
    url = f"{LIFT_WEB_BASE}/api/cmd"
    payload = {"command": "down", "target": "platform"}
    headers = {"Content-Type": "application/json"}
    
    try:
        response = requests.post(url, json=payload, headers=headers, timeout=5)
        if response.ok:
            print("✅ Lift platform DOWN command sent successfully")
            return response.json()
        else:
            print(f"❌ Lift platform DOWN command failed: HTTP {response.status_code}")
            return {"success": False, "status_code": response.status_code}
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to lift platform web service")
        return {"success": False, "error": "Connection failed"}
    except requests.exceptions.Timeout:
        print("❌ Timeout sending lift platform DOWN command")
        return {"success": False, "error": "Timeout"}
    except Exception as e:
        print(f"❌ Error sending lift platform DOWN command: {e}")
        return {"success": False, "error": str(e)}


def lift_platform_stop():
    """
    Stop the lift platform motion (pulse stop relay).
    
    Sends a POST request to halt vertical motion immediately.
    """
    print("\n🛑 Lift Platform Stop")
    print("   Sending STOP command to lift platform...")
    
    url = f"{LIFT_WEB_BASE}/api/cmd"
    payload = {"command": "stop", "target": "platform"}
    headers = {"Content-Type": "application/json"}
    
    try:
        response = requests.post(url, json=payload, headers=headers, timeout=5)
        if response.ok:
            print("✅ Lift platform STOP command sent successfully")
            return response.json()
        else:
            print(f"❌ Lift platform STOP command failed: HTTP {response.status_code}")
            return {"success": False, "status_code": response.status_code}
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to lift platform web service")
        return {"success": False, "error": "Connection failed"}
    except requests.exceptions.Timeout:
        print("❌ Timeout sending lift platform STOP command")
        return {"success": False, "error": "Timeout"}
    except Exception as e:
        print(f"❌ Error sending lift platform STOP command: {e}")
        return {"success": False, "error": str(e)}


def pushrod_goto_only_forward():
    """
    Move pushrod to 'only forward' position (preset point).
    
    Sends goto_point command with point='only forward' (3.5s movement).
    """
    print("\n🔧 Pushrod Go to 'Only Forward'")
    print("   Moving pushrod to 'only forward' position...")
    
    url = f"{LIFT_WEB_BASE}/api/cmd"
    payload = {"command": "goto_point", "target": "pushrod", "point": "only forward"}
    headers = {"Content-Type": "application/json"}
    
    try:
        response = requests.post(url, json=payload, headers=headers, timeout=10)
        if response.ok:
            print("✅ Pushrod 'only forward' command sent successfully")
            print("   (Movement will take ~3.5 seconds)")
            return response.json()
        else:
            print(f"❌ Pushrod goto 'only forward' failed: HTTP {response.status_code}")
            return {"success": False, "status_code": response.status_code}
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to pushrod web service")
        return {"success": False, "error": "Connection failed"}
    except requests.exceptions.Timeout:
        print("❌ Timeout sending pushrod goto command")
        return {"success": False, "error": "Timeout"}
    except Exception as e:
        print(f"❌ Error sending pushrod goto command: {e}")
        return {"success": False, "error": str(e)}


def pushrod_goto_base():
    """
    Move pushrod to 'base' position (home/retracted position).
    
    Sends goto_point command with point='base'.
    """
    print("\n🏠 Pushrod Go to Base")
    print("   Moving pushrod to base position...")
    
    url = f"{LIFT_WEB_BASE}/api/cmd"
    payload = {"command": "goto_point", "target": "pushrod", "point": "base"}
    headers = {"Content-Type": "application/json"}
    
    try:
        response = requests.post(url, json=payload, headers=headers, timeout=10)
        if response.ok:
            print("✅ Pushrod 'base' command sent successfully")
            return response.json()
        else:
            print(f"❌ Pushrod goto 'base' failed: HTTP {response.status_code}")
            return {"success": False, "status_code": response.status_code}
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to pushrod web service")
        return {"success": False, "error": "Connection failed"}
    except requests.exceptions.Timeout:
        print("❌ Timeout sending pushrod goto command")
        return {"success": False, "error": "Timeout"}
    except Exception as e:
        print(f"❌ Error sending pushrod goto command: {e}")
        return {"success": False, "error": str(e)}
    
def lift_platform_goto_height(target_height=900.0):
    """
    Move lift platform to a specific height using automatic control.
    
    Args:
        target_height: Target height in millimeters (default: 900.0mm)
    
    This function uses the closed-loop height control feature that:
    - Automatically moves the platform toward the target
    - Stops within ±3mm precision
    - Uses predictive stopping to compensate for system delays
    """
    print(f"\n🎯 Lift Platform Go to Height: {target_height}mm")
    print(f"   Sending goto_height command (target: {target_height}mm)...")
    
    url = f"{LIFT_WEB_BASE}/api/cmd"
    payload = {
        "command": "goto_height",
        "target": "platform",
        "target_height": target_height
    }
    headers = {"Content-Type": "application/json"}
    
    try:
        response = requests.post(url, json=payload, headers=headers, timeout=5)
        if response.ok:
            print(f"✅ Goto height {target_height}mm command sent successfully")
            print("   Platform will automatically move to target and stop")
            print("   Precision: ±3mm")
            return response.json()
        else:
            print(f"❌ Goto height command failed: HTTP {response.status_code}")
            return {"success": False, "status_code": response.status_code}
    except requests.exceptions.ConnectionError:
        print("❌ Cannot connect to lift platform web service")
        return {"success": False, "error": "Connection failed"}
    except requests.exceptions.Timeout:
        print("❌ Timeout sending goto height command")
        return {"success": False, "error": "Timeout"}
    except Exception as e:
        print(f"❌ Error sending goto height command: {e}")
        return {"success": False, "error": str(e)}


def run():
    """
    Main execution function.
    
    Calls all functions in sequence:
    1. amr_home() - Move to home position
    2. amr_smalltest() - Execute small test trajectory
    """
    print("\n" + "="*60)
    print("🏃 Starting Sequential Task Execution")
    print("="*60)
    
    try:
        # AMR Navigation Tests
        # amr_home()
        # amr_smalltest()

        # DUCO Robot Arm Tests
        # duco_jspf_Fold()
        

        # Lift Platform Tests (uncomment to use)
        # lift_platform_up()
        # time.sleep(2)  # Wait for platform to move
        # lift_platform_stop()
        
        # lift_platform_down()
        # time.sleep(2)
        # lift_platform_stop()

        # Pushrod Tests (uncomment to use)
        # pushrod_goto_only_forward()
        # time.sleep(4)  # Wait for pushrod movement (~3.5s)
        
        # pushrod_goto_base()
        # time.sleep(3)

        # DUCO Tool Change Tests (uncomment to use)
        # duco_jspf_Get_pushbutton_tool()
        # duco_jspf_Return_pushbutton_tool()
        # duco_jspf_Get_rotate_tool()
        # duco_jspf_Return_rotate_tool()
        # lift_platform_goto_height(target_height=900.0)

        # print("\n" + "="*60)
        # print("✅ All tasks completed successfully")
        # print("="*60)
        amr_courier_rack2dock()
        #pull detect
        duco_jspf_Unfold()

    
    except Exception as e:
        print(f"\n❌ Error during execution: {e}")
        import traceback
        traceback.print_exc()
        print("="*60)


def cleanup():
    """
    Cleanup function to properly close connections.
    """
    global duco_cobot, ur15_robot
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
    
    if ur15_robot is not None:
        try:
            rlt = ur15_robot.close()
            print(f"UR15 Robot close result: {rlt}")
            if rlt == 0:
                print("✅ UR15 Robot connection closed")
            else:
                print("⚠️  UR15 Robot close returned non-zero")
        except Exception as e:
            print(f"❌ Error closing UR15 Robot: {e}")
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
