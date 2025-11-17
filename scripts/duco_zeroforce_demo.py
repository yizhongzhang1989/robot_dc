import sys
import os
import requests
import math
import time
import numpy as np
import json

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

# ===========================execution functions for FTC tasks===============================
'''general function of FTC tasks'''
def FTC_setparams(ftEnabled, ftSet, isProgram=False, ftcProgram=None, onlyMonitor=False, graCalcIndex=3, 
                  dead_zone=[1,1,1,0.1,0.1,0.1], disEndLimit=5000, angleEndLimit=30, timeEndLimit=60, ftEndLimit=[0,0,0,0,0,0], 
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

    # =======================1. start robot arm system=============================
    # open connection, power on and enable the robot arm
    res = robot.open()  
    print("Open connection:", res)  
    # res = robot.power_on(True)  
    # print("Power on:", res)  
    # res = robot.enable(True)  
    # print("Enable:", res)  

    # ================2. ensure the ForceMaster.exe are running====================
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
    # ====================3. Functions of task execution===========================

    res = FTC_start()
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC started!")
    time.sleep(0.5)

    ftEnabled = [True,True,True,False,False,False]
    ftSet = [0,0,0,0,0,0]
    res = FTC_setparams(ftEnabled=ftEnabled, ftSet=ftSet, ftcEndType=6)
    if res.status_code == 200:
        print(f"Response:{res.text}, set FTC parameters successfully!")
    time.sleep(0.5)
    
    # set FTC program index
    FTC_program_index = 10    
    res = FTC_SetIndex(FTC_program_index)
    if res.status_code == 200:
        print(f"Response:{res.text}, FTC Program Index is set to {FTC_program_index}! ")
    time.sleep(0.5)

    # Keyboard control loop
    print("\n" + "="*60)
    print("Keyboard Control:")
    print("  Press 'q' to ENABLE the FTC program")
    print("  Press 'p' to DISABLE the FTC program")
    print("  Press 'Esc' or 'Ctrl+C' to exit")
    print("="*60 + "\n")
    
    program_enabled = False
    
    try:
        import tty
        import termios
        
        # Get terminal settings
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setcbreak(fd)
            
            while True:
                # Check if input is available
                import select
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    
                    if key == 'q':
                        if not program_enabled:
                            # Enable the program
                            res = FTC_SetDKAssemFlag(1)   
                            if res.status_code == 200:
                                print(f"\n✓ FTC Program ENABLED!")
                                program_enabled = True
                            else:
                                print(f"\n✗ Failed to enable FTC program: {res.text}")
                        else:
                            print("\n⚠ Program is already enabled")
                    
                    elif key == 'p':
                        if program_enabled:
                            # Disable the program
                            res = FTC_SetDKAssemFlag(0)   
                            if res.status_code == 200:
                                print(f"\n✓ FTC Program DISABLED!")
                                program_enabled = False
                            else:
                                print(f"\n✗ Failed to disable FTC program: {res.text}")
                        else:
                            print("\n⚠ Program is already disabled")
                    
                    elif key == '\x1b':  # ESC key
                        print("\nExiting...")
                        break
                        
        finally:
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            
    except ImportError:
        print("Warning: tty/termios not available, using simple input mode")
        print("Enter 'q' to enable, 'p' to disable, 'exit' to quit")
        
        while True:
            cmd = input("\nCommand: ").strip().lower()
            
            if cmd == 'q':
                if not program_enabled:
                    res = FTC_SetDKAssemFlag(1)   
                    if res.status_code == 200:
                        print(f"✓ FTC Program ENABLED!")
                        program_enabled = True
                    else:
                        print(f"✗ Failed to enable FTC program: {res.text}")
                else:
                    print("⚠ Program is already enabled")
            
            elif cmd == 'p':
                if program_enabled:
                    res = FTC_SetDKAssemFlag(0)   
                    if res.status_code == 200:
                        print(f"✓ FTC Program DISABLED!")
                        program_enabled = False
                    else:
                        print(f"✗ Failed to disable FTC program: {res.text}")
                else:
                    print("⚠ Program is already disabled")
            
            elif cmd == 'exit' or cmd == 'quit':
                print("Exiting...")
                break
            else:
                print("Invalid command. Use 'q' to enable, 'p' to disable, 'exit' to quit")
    
    except KeyboardInterrupt:
        print("\n\nInterrupted by user")
    
    finally:
        # Cleanup: Disable FTC program if it's still enabled, then stop FTC
        if program_enabled:
            print("\nDisabling FTC program before exit...")
            res = FTC_SetDKAssemFlag(0)
            if res.status_code == 200:
                print(f"Response:{res.text}, FTC Program disabled!")
            else:
                print(f"Failed to disable FTC program: {res.text}")
            time.sleep(0.5)
        
        print("\nStopping FTC...")
        res = FTC_stop()
        if res.status_code == 200:
            print(f"Response:{res.text}, FTC stopped!")
        else:
            print(f"Failed to stop FTC: {res.text}")
        time.sleep(1)



    # ======================4. close the robot arm================================
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
