#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import os
import time
import math
import termios
import tty
import select

# Add duco_robot_arm directory to search path
try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'colcon_ws', 'src'))
    from common.workspace_utils import get_workspace_root
    workspace_root = get_workspace_root()
    duco_robot_arm_path = os.path.join(workspace_root, 'colcon_ws', 'src', 'duco_robot_arm', 'duco_robot_arm')
except ImportError:
    duco_robot_arm_path = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'colcon_ws', 'src', 'duco_robot_arm', 'duco_robot_arm'))
sys.path.append(duco_robot_arm_path)
sys.path.append(os.path.join(duco_robot_arm_path, 'gen_py'))
sys.path.append(os.path.join(duco_robot_arm_path, 'lib'))

from DucoCobot import DucoCobot
from thrift import Thrift
from gen_py.robot.ttypes import Op

def ConvertPose2Rad(pose):
    """Convert angles to radians"""
    result = []
    for val in pose:
        result.append(math.radians(val))
    return result

def ConvertRad2Pose(pose_rad):
    """Convert radians to angles"""
    result = []
    for val in pose_rad:
        result.append(math.degrees(val))
    return result

class RobotKeyboardController:
    def __init__(self, ip_robot='192.168.1.10', port_robot=7003):
        self.duco_cobot = DucoCobot(ip_robot, port_robot)
        self.current_pose = [0, 0, 0, 0, 0, 0]  # Current joint angles (degrees)
        self.angle_step = 1.0  # Angle increment per key press (degrees) - suitable for single key control
        self.v = 2.0  # Velocity
        self.a = 2.0  # Acceleration
        self.kp = 250  # Proportional gain - adjust for better response
        self.kd = 25   # Derivative gain - adjust to reduce oscillation
        
        # Key mappings
        self.key_mappings = {
            'q': (0, 1),   # Joint 1 increase
            'a': (0, -1),  # Joint 1 decrease
            'w': (1, 1),   # Joint 2 increase
            's': (1, -1),  # Joint 2 decrease
            'e': (2, 1),   # Joint 3 increase
            'd': (2, -1),  # Joint 3 decrease
            'r': (3, 1),   # Joint 4 increase
            'f': (3, -1),  # Joint 4 decrease
            't': (4, 1),   # Joint 5 increase
            'g': (4, -1),  # Joint 5 decrease
            'y': (5, 1),   # Joint 6 increase
            'h': (5, -1),  # Joint 6 decrease
        }
        
        # Joint angle limits (degrees)
        self.joint_limits = [
            (-180, 180),  # Joint 1
            (-180, 180),  # Joint 2
            (-180, 180),  # Joint 3
            (-180, 180),  # Joint 4
            (-180, 180),  # Joint 5
            (-180, 180),  # Joint 6
        ]
        
    def initialize_robot(self):
        """Initialize robot connection"""
        try:
            rlt = self.duco_cobot.open()
            if rlt == 0:
                print("Robot connection successful")
                return True
            else:
                print("Robot connection failed")
                return False
        except Exception as e:
            print(f"Failed to initialize robot: {e}")
            return False
            
    def close_robot(self):
        """Close robot connection"""
        try:
            rlt = self.duco_cobot.close()
            if rlt == 0:
                print("Robot connection closed")
            else:
                print("Failed to close robot connection")
        except Exception as e:
            print(f"Error closing robot connection: {e}")
            
    def get_current_position(self):
        """Get current robot joint positions"""
        try:
            joints_rad = self.duco_cobot.get_actual_joints_position()
            self.current_pose = ConvertRad2Pose(joints_rad)
            return True
        except Exception as e:
            print(f"Failed to get current position: {e}")
            return False
            
    def clamp_angle(self, angle, joint_index):
        """Limit joint angles within safe range"""
        min_angle, max_angle = self.joint_limits[joint_index]
        return max(min_angle, min(max_angle, angle))
        
    def getch(self):
        """Get single character input (no Enter required)"""
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        try:
            tty.setraw(sys.stdin.fileno())
            ch = sys.stdin.read(1)
        finally:
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
        return ch
        
    def kbhit(self):
        """Check if keyboard input is available"""
        return select.select([sys.stdin], [], [], 0) == ([sys.stdin], [], [])
        
    def move_joint(self, joint_index, direction):
        """Move specified joint"""
        try:
            # Calculate new angle
            angle_change = direction * self.angle_step
            new_angle = self.current_pose[joint_index] + angle_change
            
            # Limit angle range
            new_angle = self.clamp_angle(new_angle, joint_index)
            
            # Update current position
            self.current_pose[joint_index] = new_angle
            
            # Convert to radians
            pose_rad = ConvertPose2Rad(self.current_pose)
            
            # Send servo command
            self.duco_cobot.servoj(pose_rad, self.v, self.a, 
                                  False, kp=self.kp, kd=self.kd)
            
            # Display current position with minimal screen refresh
            print(f"\rJ{joint_index + 1}: {new_angle:6.1f}° | Position: [{', '.join([f'{x:5.1f}' for x in self.current_pose])}]", end='', flush=True)
            
        except Exception as e:
            print(f"Error moving joint {joint_index + 1}: {e}")
            
    def set_angle_step(self, step):
        """Set angle step size"""
        if 0.1 <= step <= 10.0:
            self.angle_step = step
            print(f"Angle step set to: {self.angle_step}°")
        else:
            print("Angle step must be between 0.1 and 10.0 degrees")
                
    def print_instructions(self):
        """Print operation instructions"""
        print("\n" + "="*60)
        print("Robot Arm Joint Control Instructions (keys respond immediately, no Enter required):")
        print("="*60)
        print("q/a - Joint 1 increase/decrease")
        print("w/s - Joint 2 increase/decrease")
        print("e/d - Joint 3 increase/decrease")
        print("r/f - Joint 4 increase/decrease")
        print("t/g - Joint 5 increase/decrease")
        print("y/h - Joint 6 increase/decrease")
        print("p - Get current position")
        print("+ - Increase step size   - - Decrease step size")
        print("x - Exit program")
        print(f"Current angle step: {self.angle_step}°")
        print("="*60)
        
        
    def run(self):
        """Run main program"""
        print("Initializing robot...")
        if not self.initialize_robot():
            return
            
        # Get robot current position
        print("Getting robot current position...")
        if not self.get_current_position():
            print("Unable to get robot current position, exiting program")
            self.close_robot()
            return
            
        self.print_instructions()
        
        # Set terminal to raw mode to capture single characters
        fd = sys.stdin.fileno()
        old_settings = termios.tcgetattr(fd)
        
        try:
            tty.setraw(sys.stdin.fileno())
            print("\nProgram running... Press corresponding keys to control robot arm (press 'x' to exit)")
            
            while True:
                if self.kbhit():
                    key = sys.stdin.read(1).lower()
                    
                    if key == 'x':
                        print("\nPreparing to exit...")
                        break
                    elif key == 'p':
                        # Refresh current position
                        if self.get_current_position():
                            print(f"\nCurrent joint positions: [{', '.join([f'{x:5.1f}' for x in self.current_pose])}]")
                        else:
                            print("\nFailed to get position")
                    elif key == '+':
                        new_step = min(10.0, self.angle_step + 0.1)
                        self.set_angle_step(new_step)
                    elif key == '-':
                        new_step = max(0.1, self.angle_step - 0.1)
                        self.set_angle_step(new_step)
                    elif key in self.key_mappings:
                        joint_index, direction = self.key_mappings[key]
                        self.move_joint(joint_index, direction)
                    elif key == '\x03':  # Ctrl+C
                        print("\nDetected Ctrl+C, preparing to exit...")
                        break
                        
        except KeyboardInterrupt:
            print("\nProgram interrupted by user")
        except Exception as e:
            print(f"\nProgram error: {e}")
        finally:
            # Restore terminal settings
            termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
            self.close_robot()
            print("Program exited")

def main():
    """Main function"""
    # You can modify robot IP and port here
    ip_robot = '192.168.1.10'
    port_robot = 7003
    
    controller = RobotKeyboardController(ip_robot, port_robot)
    controller.run()

if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"Program startup failed: {e}")
        input("Press any key to exit...")
