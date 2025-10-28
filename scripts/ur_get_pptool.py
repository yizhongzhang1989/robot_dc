#!/usr/bin/env python3
"""
Get Push Tool Script
This script picks up the push tool from its storage location.
"""

import sys
import os

# Add the ur15_robot_arm module to path
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
    
    # Unlock command
    unlock_command = [0x53, 0x26, 0x01, 0x01, 0x02, 0x7A, 0xD5]
    rs485_socket.sendall(bytes(unlock_command))
    time.sleep(2)
    
    # Move to target (blocking by default)
    robot.movej([-1.5214632193194788, -1.5912000141539515, -0.061849094927310944, 
                 0.06347672521557612, 1.4398412704467773, -1.2330482641803187], a=0.2, v=0.3)
    time.sleep(0.5)

    robot.movej([-2.5792327562915247, -1.5378490921906014, 1.08154803911318, 
                 1.5686568456837158, 1.6108360290527344, -1.1309283415423792], a=0.2, v=0.3)
    time.sleep(0.5)

    robot.movej([-2.591349188481466, -1.0929209154895325, 1.1843932310687464, 
                 1.4496001440235595, 1.5851942300796509, -0.012807671223775685], a=0.2, v=0.3)
    time.sleep(0.5)

    robot.movej([-2.591351334248678, -1.0929315847209473, 1.1843932310687464, 
                 1.4496113496967773, 1.5851889848709106, -3.650076452885763], a=0.2, v=0.3)
    time.sleep(0.5)

    # Move to TCP pose using movel
    robot.movel([0.49266409744901424, 0.506101731678641, 0.4992625241220323, 
                 3.0336147441777968, -0.8106052926271219, 0.0402666899148563], a=0.2, v=0.1)
    time.sleep(0.5)

    robot.movel([0.4926612471167627, 0.5060998172654458, 0.3917882046328716, 
                 3.0335967534078705, -0.8106173335077443, 0.04025982155618463], a=0.2, v=0.1)
    time.sleep(0.5)

    # Lock command
    lock_command = [0x53, 0x26, 0x01, 0x01, 0x01, 0x3A, 0xD4]
    rs485_socket.sendall(bytes(lock_command))
    time.sleep(3)
    
    robot.movel([0.49266283295781915, 0.637514409856154, 0.39179186546027234, 
                 3.0335983095124393, -0.8106332417773685, 0.04026824667349679], a=0.2, v=0.1)
    time.sleep(0.5)

    robot.movel([0.4926625263948576, 0.6375082039466028, 0.657304401098948, 
                 3.0335884301787868, -0.8106517732023341, 0.040279561648905615], a=0.2, v=0.1)
    time.sleep(0.5)

    robot.movej([-2.591349188481466, -1.0929209154895325, 1.1843932310687464, 
                 1.4496001440235595, 1.5851942300796509, -0.012807671223775685], a=0.2, v=0.3)
    time.sleep(0.5)
    
    robot.movej([-2.5792327562915247, -1.5378490921906014, 1.08154803911318, 
                 1.5686568456837158, 1.6108360290527344, -1.1309283415423792], a=0.2, v=0.3)
    time.sleep(0.5)
    
    robot.movej([-1.5214632193194788, -1.5912000141539515, -0.061849094927310944, 
                 0.06347672521557612, 1.4398412704467773, -1.2330482641803187], a=0.2, v=0.3)
    time.sleep(0.5)
    
    # Close connections
    rs485_socket.close()
    robot.close()


if __name__ == "__main__":
    main()
