import time  
from math import radians  
import sys
import os

# get current file path
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
# Add the required paths for thrift and generated code
sys.path.append(duco_script_path)
sys.path.append(gen_py_path)
sys.path.append(lib_path)

from DucoCobot import DucoCobot  
from gen_py.robot.ttypes import Op  
from thrift import Thrift  
   
# Robot connection parameters  
ip = '192.168.1.10'     # real robot
# ip = '192.168.70.128' # virtual machine  
port = 7003  
      
def main():  
    # Create the DucoCobot instance and open connection  
    robot = DucoCobot(ip, port)  
    res = robot.open()  
    print("Open connection:", res)  
        
    # Power on and enable the robot  
    res = robot.power_on(True)  
    print("Power on:", res)  
    res = robot.enable(True)  
    print("Enable:", res)  
        
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
        

    # ------------------------------            
    # Read and print the current end-effector (TCP) position.  
    tcp_pose = robot.get_tcp_pose()  
    print("\nCurrent end-effector (TCP) position:", tcp_pose)  

    default_tool = ""  # Use an empty string (or a valid tool name) if required  

    # Move negative 2cm along the current axis.  
    offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  
    offset[2] = -0.02  # -5cm  
    res = robot.tcp_move(offset, 1.0, 1.0, 0.0, default_tool, True, op)  
    print("Result:", res)  
    time.sleep(2)  

    # Move negative 2cm along the current axis.  
    offset = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]  
    offset[2] = 0.02  # -5cm  
    res = robot.tcp_move(offset, 1.0, 1.0, 0.0, default_tool, True, op)  
    print("Result:", res)  
    time.sleep(2)  

        
    # ------------------------------  
    # Clean up: disable, power off, and close connection  
    res = robot.disable(True)  
    print("\nDisable result:", res)  
    res = robot.power_off(True)  
    print("Power off result:", res)  
    res = robot.close()  
    print("Close connection result:", res)  
     
if __name__ == '__main__':  
    try:  
        main()  
    except Thrift.TException as tx:  
        print("Thrift Exception:", tx.message)  
