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
    
    # unlock_command = [0x53, 0x26, 0x01, 0x01, 0x02, 0x7A, 0xD5]
    # rs485_socket.sendall(bytes(unlock_command))
    # time.sleep(2)
    # # Move to target positions
    # robot.movej([-1.5214603582965296, -1.5911976299681605, -0.061844415962696075, 
    #              0.0634651619144897, 1.4398401975631714, -1.233070198689596], a=0.2, v=0.5)
    # time.sleep(0.5)
    
    # robot.movej([0.2855508029460907, -1.5912038288512171, -0.061856381595134735, 
    #              0.0634731489368896, 1.439842939376831, -1.2330248991595667], a=0.2, v=0.5)
    # time.sleep(0.5)
    
    # robot.movej([0.28557083010673523, -1.591199060479635, -0.06185074895620346, 
    #              0.04871325075115962, -1.5685623327838343, -1.2330392042743128], a=0.2, v=0.5)
    # time.sleep(0.5)
    
    # robot.movej([0.23396389186382294, -1.2096487444690247, -0.9661047458648682, 
    #              0.6180705267139892, -0.8157733122455042, -1.3010566870318812], a=0.2, v=0.5)
    # time.sleep(0.5)
    
    robot.movej([3.3862, -1.6388,  1.3364, 
                 -1.2553, -1.5508, 0.1518], a=0.2, v=0.5)
    time.sleep(0.5)
    
    # Move to TCP poses using movel
    robot.movel([0.52894,0.34015,0.87329,-2.3180,2.0520,-0.0122], a=0.2, v=0.3)
    time.sleep(0.5)
    
    # robot.movel([0.29874553779036866, -0.15609230154577425, 0.1711676056642954, 
    #              0.8040429473038347, 3.0321342450541757, 0.028574094050588018], a=0.2, v=0.3)
    # time.sleep(0.5)
    
    # robot.movel([0.29873421445620796, -0.1560922368641443, 0.1422687183435678, 
    #              0.8040467847720677, 3.0321535550257472, 0.028590373396249223], a=0.2, v=0.025)
    # time.sleep(0.5)
    
    # lock_command = [0x53, 0x26, 0x01, 0x01, 0x01, 0x3A, 0xD4]
    # rs485_socket.sendall(bytes(lock_command))
    # time.sleep(2)
    
    # robot.movel([0.2987367462974777, -0.2846565309993494, 0.14226463512805376, 
    #              0.8040322299689223, 3.032150901970706, 0.02856671600270579], a=0.2, v=0.1)
    # time.sleep(0.5)
    
    # robot.movel([0.0679248991478309, -0.917977457822281, 0.18355923789567852, 
    #              1.6900325490248567, 1.8297514645801967, -0.4799504515824927], a=0.2, v=0.3)
    # time.sleep(0.5)
    
    # robot.movej([-1.2915375868426722, -1.5575635519674798, -0.06898311525583267, 
    #              2.9911085802265625, -1.580907169972555, -1.365657154713766], a=0.2, v=0.5)
    # time.sleep(0.5)
    
    # robot.movej([-4.884998265896932, -1.5659199755503614, -0.06898307800292969, 
    #              2.991100950832031, -1.5809772650348108, -1.365657154713766], a=0.2, v=0.5)
    # time.sleep(0.5)
    
    # robot.movej([-4.885191384946005, -1.2189367276481171, 1.9253881613360804, 
    #              3.4605280595966796, -1.580993954335348, -1.3657320181476038], a=0.2, v=0.5)
    # time.sleep(0.5)
    
    # robot.movej([-4.628247443829672, -0.596695141201355, 1.2688997427569788, 
    #              -1.9046498737730921, 0.12711143493652344, 3.7376646995544434], a=0.2, v=0.5)
    # time.sleep(0.5)

    
    # robot.movej([-4.628224555646078, -0.5939362210086365, 1.9152935186969202, 
    #              -1.9046393833556117, 0.1272939145565033, 3.737786054611206], a=0.2, v=0.5)
    # time.sleep(0.5)
    
    # Close connections
    rs485_socket.close()
    robot.close()


if __name__ == "__main__":
    main()
