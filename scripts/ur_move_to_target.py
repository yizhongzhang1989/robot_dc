#!/usr/bin/env python3
"""
UR Robot Move to Target Script

This script implements functionality to move a UR robot to target joint positions.
The movement is executed in a specific order from J6 to J1 (reverse joint order).

Features:
- Move to home position using predefined joint angles
- Execute movements in reverse order (J6 -> J5 -> J4 -> J3 -> J2 -> J1)
- Safe and controlled joint movements

Usage:
    python3 ur_move_to_target.py

Author: Robot DC Team
Date: 2025-12-25
"""

import os
import sys
import numpy as np
import time
import argparse

# Add the robot arm directory to the Python path
try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'colcon_ws', 'src'))
    from common.workspace_utils import get_workspace_root
    project_root = get_workspace_root()
except ImportError:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)

ur_robot_arm_dir = os.path.join(project_root, 'colcon_ws', 'src', 'ur15_robot_arm')
sys.path.insert(0, ur_robot_arm_dir)

from ur15_robot_arm.ur15 import UR15Robot


class URMoveToTarget:
    """
    Class for controlling UR robot movements to target positions.
    Supports moving joints in reverse order (J6 to J1).
    """
    
    def __init__(self, robot_ip=None, robot_port=None):
        """
        Initialize the UR robot controller.
        
        Args:
            robot_ip (str): IP address of the UR robot (if None, loads from config)
            robot_port (int): Port number for robot communication (if None, loads from config)
        """
        # Load config parameters if not provided
        config_params = self._load_robot_parameters_from_config()
        
        self.robot_ip = robot_ip if robot_ip is not None else config_params['robot_ip']
        self.robot_port = robot_port if robot_port is not None else config_params['robot_port']
        self.robot = None
        
        # Movement parameters
        self.a_movej = 1.0  # Acceleration for joint movements (rad/s²)
        self.v_movej = 1.0  # Velocity for joint movements (rad/s)
        
        # Home position in degrees
        self.home_position_deg = [96.3, -92.3, 4.0, -98.1, -4.3, 178.0]
        
        # Task position in degrees
        self.task_position_deg = [92.0, -57.5, 102.9, -47.9, 9.6, 187.1]
        
        # Initialize robot connection
        self._initialize_robot()
    
    def _load_robot_parameters_from_config(self):
        """
        Load robot IP and port from robot_config.yaml
        
        Returns:
            dict: Dictionary with robot_ip and robot_port
        """
        # Default values
        defaults = {
            'robot_ip': '192.168.1.15',
            'robot_port': 30002
        }
        
        try:
            import yaml
            from common.workspace_utils import get_workspace_root
            
            # Get workspace root
            workspace_root = get_workspace_root()
            if workspace_root is None:
                workspace_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
            
            # Path to config file
            config_path = os.path.join(workspace_root, 'config', 'robot_config.yaml')
            
            if not os.path.exists(config_path):
                print(f"Config file not found: {config_path}")
                print(f"Using default values: IP={defaults['robot_ip']}, Port={defaults['robot_port']}")
                return defaults
            
            # Load config file
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            ur15_config = config.get('ur15', {})
            
            # Get robot IP from ur15.robot.ip
            robot_ip = ur15_config.get('robot', {}).get('ip', defaults['robot_ip'])
            
            # Get robot port from ur15.robot.ports.control
            robot_port = ur15_config.get('robot', {}).get('ports', {}).get('control', defaults['robot_port'])
            
            print(f"✓ Loaded config: IP={robot_ip}, Port={robot_port}")
            
            return {
                'robot_ip': robot_ip,
                'robot_port': robot_port
            }
            
        except Exception as e:
            print(f"Error loading config: {e}")
            print(f"Using default values: IP={defaults['robot_ip']}, Port={defaults['robot_port']}")
            return defaults
    
    def _initialize_robot(self):
        """
        Initialize the connection to the UR robot.
        """
        try:
            print(f"Initializing UR robot at {self.robot_ip}:{self.robot_port}...")
            self.robot = UR15Robot(ip=self.robot_ip, port=self.robot_port)
            
            # Attempt to connect
            res = self.robot.open()
            if res == 0:
                print('✓ UR15 robot connected successfully')
                return True
            else:
                print(f'✗ Failed to connect to UR15 robot (error code: {res})')
                self.robot = None
                return False
        except Exception as e:
            print(f"✗ Failed to initialize robot: {e}")
            self.robot = None
            return False
    
    def get_current_joint_positions(self):
        """
        Get current joint positions from the robot.
        
        Returns:
            list: Current joint positions in degrees, or None if failed
        """
        if not self.robot:
            print("✗ Robot not connected")
            return None
        
        try:
            # Get actual joint positions in radians
            joint_positions_rad = self.robot.get_actual_joint_positions()
            
            # Convert to degrees
            joint_positions_deg = [np.rad2deg(angle) for angle in joint_positions_rad]
            
            return joint_positions_deg
        except Exception as e:
            print(f"✗ Failed to get joint positions: {e}")
            return None
    
    def movej_to_home_position(self):
        """
        Move robot to home position by moving each joint in reverse order (J6 to J1).
        
        The robot will move:
        1. Joint 6 (J6) first
        2. Joint 5 (J5)
        3. Joint 4 (J4)
        4. Joint 3 (J3)
        5. Joint 2 (J2)
        6. Joint 1 (J1) last
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_to_home_position")
            return False
        
        print("=" * 60)
        print("Starting movej_to_home_position - Moving joints in order J6 -> J1")
        print("=" * 60)
        
        # Target home position in degrees
        target_position_deg = self.home_position_deg.copy()
        print(f"Target home position (degrees): {target_position_deg}")
        
        # Get current joint positions
        current_position_deg = self.get_current_joint_positions()
        if current_position_deg is None:
            print("✗ Failed to get current joint positions")
            return False
        
        print(f"Current position (degrees): {[f'{x:.2f}' for x in current_position_deg]}")
        print()
        
        try:
            # Move each joint in reverse order (J6 to J1)
            # Joint indices: J1=0, J2=1, J3=2, J4=3, J5=4, J6=5
            joint_order = [5, 4, 3, 2, 1, 0]  # J6, J5, J4, J3, J2, J1
            joint_names = ['J6', 'J5', 'J4', 'J3', 'J2', 'J1']
            
            # Create intermediate position starting from current position
            intermediate_position_deg = current_position_deg.copy()
            
            for idx, (joint_idx, joint_name) in enumerate(zip(joint_order, joint_names), 1):
                # Update the target joint position
                intermediate_position_deg[joint_idx] = target_position_deg[joint_idx]
                
                # Convert to radians for robot command
                intermediate_position_rad = [np.deg2rad(angle) for angle in intermediate_position_deg]
                
                print(f"Step {idx}/6: Moving {joint_name} to {target_position_deg[joint_idx]:.2f}°")
                print(f"  Target position: {[f'{x:.2f}' for x in intermediate_position_deg]}")
                
                # Execute joint movement
                result = self.robot.movej(intermediate_position_rad, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move {joint_name}, error code: {result}")
                    return False
                
                print(f"✓ Successfully moved {joint_name}")
                print()
                
                # Add small delay between movements for stability
                time.sleep(0.3)
            
            # Verify final position
            final_position_deg = self.get_current_joint_positions()
            if final_position_deg is not None:
                print("=" * 60)
                print("Movement complete!")
                print(f"Final position (degrees): {[f'{x:.2f}' for x in final_position_deg]}")
                print(f"Target position (degrees): {[f'{x:.2f}' for x in target_position_deg]}")
                
                # Calculate position error
                position_error = [abs(final - target) for final, target in zip(final_position_deg, target_position_deg)]
                print(f"Position error (degrees): {[f'{x:.2f}' for x in position_error]}")
                print("=" * 60)
            
            print("✓ Successfully completed movej_to_home_position")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_to_home_position: {e}")
            return False
    
    def movej_to_task_position(self):
        """
        Move robot to task position by moving each joint in reverse order (J6 to J1).
        
        The robot will move:
        1. Joint 6 (J6) first
        2. Joint 5 (J5)
        3. Joint 4 (J4)
        4. Joint 3 (J3)
        5. Joint 2 (J2)
        6. Joint 1 (J1) last
        
        Returns:
            bool: True if successful, False otherwise
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_to_task_position")
            return False
        
        print("=" * 60)
        print("Starting movej_to_task_position - Moving joints in order J6 -> J1")
        print("=" * 60)
        
        # Target task position in degrees
        target_position_deg = self.task_position_deg.copy()
        print(f"Target task position (degrees): {target_position_deg}")
        
        # Get current joint positions
        current_position_deg = self.get_current_joint_positions()
        if current_position_deg is None:
            print("✗ Failed to get current joint positions")
            return False
        
        print(f"Current position (degrees): {[f'{x:.2f}' for x in current_position_deg]}")
        print()
        
        try:
            # Move each joint in reverse order (J6 to J1)
            # Joint indices: J1=0, J2=1, J3=2, J4=3, J5=4, J6=5
            joint_order = [5, 4, 3, 2, 1, 0]  # J6, J5, J4, J3, J2, J1
            joint_names = ['J6', 'J5', 'J4', 'J3', 'J2', 'J1']
            
            # Create intermediate position starting from current position
            intermediate_position_deg = current_position_deg.copy()
            
            for idx, (joint_idx, joint_name) in enumerate(zip(joint_order, joint_names), 1):
                # Update the target joint position
                intermediate_position_deg[joint_idx] = target_position_deg[joint_idx]
                
                # Convert to radians for robot command
                intermediate_position_rad = [np.deg2rad(angle) for angle in intermediate_position_deg]
                
                print(f"Step {idx}/6: Moving {joint_name} to {target_position_deg[joint_idx]:.2f}°")
                print(f"  Target position: {[f'{x:.2f}' for x in intermediate_position_deg]}")
                
                # Execute joint movement
                result = self.robot.movej(intermediate_position_rad, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move {joint_name}, error code: {result}")
                    return False
                
                print(f"✓ Successfully moved {joint_name}")
                print()
                
                # Add small delay between movements for stability
                time.sleep(0.3)
            
            # Verify final position
            final_position_deg = self.get_current_joint_positions()
            if final_position_deg is not None:
                print("=" * 60)
                print("Movement complete!")
                print(f"Final position (degrees): {[f'{x:.2f}' for x in final_position_deg]}")
                print(f"Target position (degrees): {[f'{x:.2f}' for x in target_position_deg]}")
                
                # Calculate position error
                position_error = [abs(final - target) for final, target in zip(final_position_deg, target_position_deg)]
                print(f"Position error (degrees): {[f'{x:.2f}' for x in position_error]}")
                print("=" * 60)
            
            print("✓ Successfully completed movej_to_task_position")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_to_task_position: {e}")
            return False


def main():
    """
    Main function to test the URMoveToTarget class.
    """
    parser = argparse.ArgumentParser(description='UR Robot Move to Target Position')
    parser.add_argument('--robot-ip', type=str, default=None,
                        help='IP address of the UR robot (default: from robot_config.yaml)')
    parser.add_argument('--robot-port', type=int, default=None,
                        help='Port number for robot communication (default: from robot_config.yaml)')
    parser.add_argument('--action', type=str, default='home', choices=['home', 'task'],
                        help='Action to perform: home (move to home position) or task (move to task position)')
    
    args = parser.parse_args()
    
    # Create controller instance (will load from config if robot_ip/port are None)
    controller = URMoveToTarget(robot_ip=args.robot_ip, robot_port=args.robot_port)
    
    if not controller.robot:
        print("✗ Failed to initialize robot controller")
        return
    
    # Display current position
    current_pos = controller.get_current_joint_positions()
    if current_pos is not None:
        print(f"\nCurrent joint positions (degrees): {[f'{x:.2f}' for x in current_pos]}")
        print()
    
    # Execute requested action
    if args.action == 'home':
        print("Moving to home position...")
        success = controller.movej_to_home_position()
    elif args.action == 'task':
        print("Moving to task position...")
        success = controller.movej_to_task_position()
    
    if success:
        print("\n" + "=" * 60)
        print("✓ Movement completed successfully!")
        print("=" * 60)
    else:
        print("\n" + "=" * 60)
        print("✗ Movement failed")
        print("=" * 60)


if __name__ == '__main__':
    main()
