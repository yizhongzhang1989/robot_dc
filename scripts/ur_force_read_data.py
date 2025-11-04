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
        print("Reading TCP force data... Press 'q' to quit.")
        
        # Second-order low-pass filter parameters
        fs = 100  # Sampling frequency (Hz)
        fc = 5    # Cutoff frequency (Hz)
        order = 2  # Second-order filter
        
        # Design Butterworth low-pass filter
        nyquist = 0.5 * fs
        normal_cutoff = fc / nyquist
        b, a = signal.butter(order, normal_cutoff, btype='low', analog=False)
        
        # Initialize filter states
        zi = None  # Filter state
        
        # Save terminal settings
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            tty.setcbreak(sys.stdin.fileno())
            
            while True:
                f_current = robot.get_tcp_force()
                
                # Convert to numpy array for filtering
                if isinstance(f_current, (list, tuple)):
                    f_current_array = np.array(f_current)
                else:
                    f_current_array = np.array([f_current])
                
                # Initialize filter state on first reading
                if zi is None:
                    # Create filter state for each dimension
                    zi_single = signal.lfilter_zi(b, a)
                    # Broadcast to match the number of dimensions in force data
                    zi = np.outer(zi_single, f_current_array).T
                
                # Apply second-order low-pass filter to each dimension
                f_filtered = np.zeros_like(f_current_array)
                for i in range(len(f_current_array)):
                    f_filtered[i], zi[i] = signal.lfilter(b, a, [f_current_array[i]], zi=zi[i])
                
                # Convert back to list if input was list/tuple
                if isinstance(f_current, (list, tuple)):
                    f_filtered = f_filtered.tolist()
                else:
                    f_filtered = float(f_filtered[0]) if len(f_filtered) == 1 else f_filtered.tolist()
                
                # Print on the same line with carriage return
                print(f"\rTCP force (filtered): {f_filtered}\r", end='', flush=True)
                
                # No need to update previous values as filter maintains its own state
                
                # Check if 'q' key is pressed
                if select.select([sys.stdin], [], [], 0)[0]:
                    key = sys.stdin.read(1)
                    if key.lower() == 'q':
                        print("\nExiting...")
                        break
                
                time.sleep(0.01)  # Small delay to avoid overwhelming output
        finally:
            # Restore terminal settings
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)

    else:
        print(f"Failed to connect to robot at {ur15_ip}:{ur15_port}")
    

    rs485_socket.close()

    # 10. Power down and close connection of the robot   
    # res = robot.powerdown()
    # print(f"Power down result: {res}")
    
    # 11. Close connection
    close_res = robot.close()
    print(f"Close result: {close_res}")

if __name__ == "__main__":
    main()