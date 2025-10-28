#!/usr/bin/env python3
"""
Unfold Script
This script unfolds the robot arm from a compact position (reverse of Fold).
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
    
    # Move to target positions in reverse order
    robot.movej([-4.669361893330709, -2.424460073510641, 2.8666275183307093, 
                 -3.414271970788473, 1.4398540258407593, -1.2330215612994593], a=0.2, v=0.3)
    time.sleep(0.5)
    
    robot.movej([-4.335100833569662, -2.2389666042723597, 2.085423294697897, 
                 -3.4142643413939417, 1.4398565292358398, -1.2330229918109339], a=0.2, v=0.3)
    time.sleep(0.5)
    
    robot.movej([-3.8213418165790003, -2.2389828167357386, 1.1874364058123987, 
                 0.063479943866394, 1.4398581981658936, -1.23303729692568], a=0.2, v=0.3)
    time.sleep(0.5)
    
    robot.movej([-3.013846222554342, -1.5912028751769007, -0.061867859214544296, 
                 0.06348053991284175, 1.4398565292358398, -1.2330310980426233], a=0.2, v=0.3)
    time.sleep(0.5)
    
    robot.movej([-1.5214560667621058, -1.5912043056883753, -0.061858177185058594, 
                 0.06348852693524165, 1.4398598670959473, -1.2330897490130823], a=0.2, v=0.3)
    time.sleep(0.5)
   
    # Close connections
    rs485_socket.close()
    robot.close()


if __name__ == "__main__":
    main()
