from ur15_robot_arm.ur15 import UR15Robot
import time
import socket
import math
import numpy as np
import sys
import select
import tty
import termios
from scipy import signal

def main():
    # Definition of robot IP and port
    ur15_ip = "192.168.1.15"
    ur15_port = 30002
    rs485_port = 54321

    # Init the robot object
    robot = UR15Robot(ur15_ip, ur15_port)
    # Open connection of robot
    ur15_start_res = robot.open()

    # Open RS485 socket connection
    rs485_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    rs485_start_res = rs485_socket.connect((ur15_ip, rs485_port))
    print(f"RS485 socket connection result: {rs485_start_res}")

    # =====================================Program===========================
    if ur15_start_res == 0:
        
        robot.zero_ftsensor()
        time.sleep(0.5) 

        robot.force_mode_set_damping(0.1)
        time.sleep(0.5)

        # set force mode parameters
        task_frame = [0, 0, 0, 0, 0, 0]
        selection_vector = [0, 0, 1, 0, 0, 0] # enable force control in which direction
        wrench = [0, 0, 0, 0, 0, 0] # desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57] # force/torque limits
        # activate force mode
        robot.force_mode(task_frame, selection_vector, wrench, limits=limits)
        time.sleep(0.5)
        
        print("\n[INFO] Force mode activated. Press 'q' to exit force mode...")
        
        # Set terminal to raw mode for non-blocking keyboard input
        old_settings = termios.tcgetattr(sys.stdin)
        exit_requested = False
        try:
            tty.setraw(sys.stdin.fileno())
            
            while True:
                # Check if key is pressed (non-blocking)
                if select.select([sys.stdin], [], [], 0.1)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() == 'q':
                        exit_requested = True
                        break
                
                # Optional: print current force readings
                force = robot.get_tcp_force()
                if force:
                    print(f"\rTCP Force: Fx={force[0]:6.2f} Fy={force[1]:6.2f} Fz={force[2]:6.2f} "
                          f"Tx={force[3]:6.2f} Ty={force[4]:6.2f} Tz={force[5]:6.2f}", 
                          end='', flush=True)
                
                time.sleep(0.05)
        
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
            # Clear the current line and print exit message after terminal is restored
            if exit_requested:
                print("\r" + " " * 100 + "\r", end='')
                print("[INFO] 'q' pressed, exiting force mode...")
                robot.end_force_mode()
        
    else:
        print(f"Failed to connect to robot at {ur15_ip}:{ur15_port}")
    

    rs485_socket.close()

    # # Power down and close connection of the robot   
    # res = robot.powerdown()
    # print(f"Power down result: {res}")
    
    # # Close connection
    close_res = robot.close()
    print(f"Close result: {close_res}")

if __name__ == "__main__":
    main()