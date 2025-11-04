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

    # Init the robot object
    robot = UR15Robot(ur15_ip, ur15_port)
    # Open connection of robot
    ur15_start_res = robot.open()

    # =====================================Program===========================
    if ur15_start_res == 0:

        # Save original terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        
        try:
            # Set terminal to cbreak mode for key detection
            tty.setcbreak(sys.stdin.fileno())
            
            # Get current TCP pose to use as task frame (for force control in TCP coordinate system)
            tcp_pose = robot.get_actual_tcp_pose()
            print(f"[INFO] Current TCP pose: {tcp_pose}")
            
            # Set force mode parameters
            task_frame = tcp_pose  # Use TCP coordinate system instead of base
            selection_vector = [0, 0, 0, 0, 0, 1]  # Enable force control in Z direction (now relative to TCP)
            wrench = [0, 0, 0, 0, 0, -0.5]  # Desired force/torque in each direction
            limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
            
            print("[INFO] Press 'q' to stop force mode, or Ctrl+C")
            print("[INFO] Starting force control task...")
            
            # Execute force control task with manual termination (end_type=0)
            result = robot.force_control_task(
                task_frame=task_frame,
                selection_vector=selection_vector,
                wrench=wrench,
                limits=limits,
                damping=0.1,
                end_type=4,
                end_angle=90
            )
            
            if result == 0:
                print("[INFO] Monitoring for stop signal...")
                
                # Monitor for 'q' key press (runs indefinitely until 'q' or Ctrl+C)
                while True:
                    # Check for key press
                    if select.select([sys.stdin], [], [], 0.1)[0]:
                        key = sys.stdin.read(1)
                        if key.lower() == 'q':
                            print("\n[INFO] 'q' key pressed, stopping force mode...")
                            break
                    time.sleep(0.1)
                
                # End force mode
                robot.end_force_mode()
                print("[INFO] Force control task completed successfully")
            else:
                print("[ERROR] Force control task failed")
                
        except KeyboardInterrupt:
            print("\n[INFO] Ctrl+C detected, stopping force mode...")
            robot.end_force_mode()
            print("[INFO] Force mode stopped")
            
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
    else:
        print(f"Failed to connect to robot at {ur15_ip}:{ur15_port}")
    

    # # Power down and close connection of the robot   
    # res = robot.powerdown()
    # print(f"Power down result: {res}")
    
    # # Close connection
    close_res = robot.close()
    print(f"Close result: {close_res}")

if __name__ == "__main__":
    main()