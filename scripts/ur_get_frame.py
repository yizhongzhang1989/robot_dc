#!/usr/bin/env python3
"""
Get Frame Script
This script picks up a frame from its location.
"""

import sys
import os

# Add the ur15_robot_arm module to path
try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'colcon_ws', 'src'))
    from common.workspace_utils import get_workspace_root
    repo_root = get_workspace_root()
except ImportError:
    current_dir = os.path.dirname(os.path.abspath(__file__))
    repo_root = os.path.abspath(os.path.join(current_dir, '..'))
ur15_path = os.path.join(repo_root, 'colcon_ws/src/ur15_robot_arm/ur15_robot_arm')
sys.path.append(ur15_path)

from ur15 import UR15Robot
import socket
import time


def main():
    # Robot connection
    robot = UR15Robot("192.168.1.15", 30002)
    
    if robot.open() != 0:
        print("Failed to connect to robot")
        return
    
    # RS485 connection for gripper control
    rs485_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rs485_socket.connect(("192.168.1.15", 54321))
    #get frame

    # unlock_command = [0x53, 0x26, 0x01, 0x01, 0x02, 0x7A, 0xD5]
    # rs485_socket.sendall(bytes(unlock_command))
    # time.sleep(2)
    # # position 1
    # robot.movej([1.4961, -1.2703, 1.8783, -0.4545, 0.2356, 3.1863], a=0.2, v=0.5)
    # time.sleep(0.5)

    # # position 2
    # robot.movej([1.3347, -2.1303, 2.4581, -1.1046, 0.2395, 3.1981], a=0.2, v=0.5)
    # time.sleep(0.5)

    # # position 3
    # robot.movej([1.3393, -1.7129, 0.3227, -1.1046, 0.2395, 3.1981], a=0.2, v=0.5)       
    # time.sleep(0.5)

    # # position 4
    # robot.movej([3.8619, -1.7129, 0.3227, -1.1046, 0.2395, 3.1981], a=0.2, v=0.5)
    # time.sleep(0.5)

    # # position 5        
    # robot.movej([4.1107, -1.4378, 2.1962, -2.3324, -1.5706, -0.0421], a=0.2, v=0.5)
    # time.sleep(0.5)

    # # position 6
    # robot.movel([0.19354, 0.59381, 0.14359, 3.019, -0.879, 0.009], a=0.2, v=0.3)
    # time.sleep(0.5)

    # # gripper close
    # lock_command = [0x53, 0x26, 0x01, 0x01, 0x01, 0x3A, 0xD4]
    # rs485_socket.sendall(bytes(lock_command))
    # time.sleep(2)

    # # position 7
    # robot.movel([0.19354, 0.73082, 0.14430, 3.019, -0.879, 0.009], a=0.2, v=0.3)
    # time.sleep(0.5)

    # # position 8
    # robot.movel([0.19354, 0.73082, 0.77569, 3.019, -0.879, 0.009], a=0.2, v=0.3)
    # time.sleep(0.5)

    # # position 9
    # robot.movej([4.2196, -1.5857, 0.0182, 0.0335, -1.5709, 0.0635], a=0.2, v=0.5)
    # time.sleep(0.5)

    # # position 10
    # robot.movej([4.2196, -1.5857, 0.0182, -3.1186, -1.5709, 0.0635], a=0.2, v=0.5)
    # time.sleep(0.5)

    # # position 11
    # robot.movej([1.5180, -1.5857, 0.0182, -3.1186, -1.5709, 0.0635], a=0.2, v=0.5)
    # time.sleep(0.5)

    # # position 12
    # robot.movej([1.5236, -1.4562, 1.2454, -1.3261, -1.6292, -0.9416], a=0.2, v=0.5)
    # time.sleep(0.5)

    # # position 13
    # robot.movel([0.28482, -0.73288, 0.67809, 1.536, -3.533, 1.982], a=0.2, v=0.3)
    # time.sleep(0.5)

    #return frame
    
    # position 13
    robot.movel([0.28482, -0.73288, 0.67809, 1.536, -3.533, 1.982], a=0.2, v=0.3)
    time.sleep(0.5)

    # position 12
    robot.movel([0.13333, -0.71960, 0.82161, 1.328, -2.803, -0.092], a=0.2, v=0.5)
    time.sleep(0.5)

    # position 11
    robot.movej([1.5180, -1.5857, 0.0182, -3.1186, -1.5709, 0.0635], a=0.2, v=0.5)
    time.sleep(0.5)

    # position 10
    robot.movej([4.2196, -1.5857, 0.0182, -3.1186, -1.5709, 0.0635], a=0.2, v=0.5)
    time.sleep(0.5)

    # position 9
    robot.movej([4.2196, -1.5857, 0.0182, 0.0335, -1.5709, 0.0635], a=0.2, v=0.5)
    time.sleep(0.5)

    # position 8
    robot.movej([4.2196, -1.4412, 1.3278, -1.4611, -1.5709, 0.0635], a=0.2, v=0.3)
    time.sleep(0.5)

    # position 7
    robot.movel([0.19354, 0.73082, 0.14430, 3.019, -0.879, 0.009], a=0.2, v=0.3)
    time.sleep(0.5)

    # position 6
    robot.movel([0.19354, 0.59381, 0.14359, 3.019, -0.879, 0.009], a=0.2, v=0.3)
    time.sleep(0.5)

    # gripper open
    unlock_command = [0x53, 0x26, 0x01, 0x01, 0x02, 0x7A, 0xD5]
    rs485_socket.sendall(bytes(unlock_command))
    time.sleep(2)

    # position 5        
    robot.movel([0.19354, 0.59381, 0.36216, 3.019, -0.879, 0.009], a=0.2, v=0.3)
    time.sleep(0.5)

    # position 4
    robot.movej([3.8619, -1.7129, 0.3227, -1.1046, 0.2395, 3.1981], a=0.2, v=0.5)
    time.sleep(0.5)

    # position 3
    robot.movej([1.3393, -1.7129, 0.3227, -1.1046, 0.2395, 3.1981], a=0.2, v=0.5)       
    time.sleep(0.5)

    # position 2
    robot.movej([1.3347, -2.1303, 2.4581, -1.1046, 0.2395, 3.1981], a=0.2, v=0.5)
    time.sleep(0.5)

    # position 1
    robot.movej([1.4961, -1.2703, 1.8783, -0.4545, 0.2356, 3.1863], a=0.2, v=0.5)
    time.sleep(0.5)


    # Close connections
    rs485_socket.close()
    robot.close()


if __name__ == "__main__":
    main()
