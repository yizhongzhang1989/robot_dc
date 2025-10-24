from ur15_robot_arm.ur15 import UR15Robot
import time
import socket
import math
import numpy as np
import json
import os


class URExtractTask:
    def __init__(self, log_file_path=None):
        """
        Initialize URExtractTask with log file path
        
        Args:
            log_file_path (str): Path to the location log file. 
                                Defaults to robot_dc/temp/ur15_precise_location_result/location_log_file.json
        """
        if log_file_path is None:
            # Get the script directory and construct default path
            script_dir = os.path.dirname(os.path.abspath(__file__))
            self.log_file_path = os.path.join(script_dir, '..', 'temp', 
                                            'ur15_precise_location_result', 
                                            'location_log_file.json')
        else:
            self.log_file_path = log_file_path
        
        self.estimated_position = None
        self.target_start_pose = None
        self.joint_positions = None
    
    def load_data(self):
        """
        Load data from the log file and extract estimated_position and tcp_pose
        
        Returns:
            bool: True if data loaded successfully, False otherwise
        """
        try:
            if not os.path.exists(self.log_file_path):
                print(f"Log file not found: {self.log_file_path}")
                return False
            
            with open(self.log_file_path, 'r') as f:
                data = json.load(f)
            
            # Extract estimated_position
            if 'estimated_position' in data:
                self.estimated_position = data['estimated_position']
            else:
                print("Warning: 'estimated_position' not found in log file")
            
            # Extract tcp_pose as target_start_pose
            if 'current_robot_state' in data and 'tcp_pose' in data['current_robot_state']:
                self.target_start_pose = data['current_robot_state']['tcp_pose']
            else:
                print("Warning: 'tcp_pose' not found in log file")
            
            # Extract joint_positions
            if 'current_robot_state' in data and 'joint_positions' in data['current_robot_state']:
                self.joint_positions = data['current_robot_state']['joint_positions']
            else:
                print("Warning: 'joint_positions' not found in log file")
            
            return True
            
        except FileNotFoundError:
            print(f"Error: Log file not found at {self.log_file_path}")
            return False
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON format in log file - {e}")
            return False
        except Exception as e:
            print(f"Error loading data: {e}")
            return False
    
    def get_estimated_position(self):
        """
        Get the estimated position
        
        Returns:
            dict: Dictionary containing x, y, z coordinates or None if not loaded
        """
        return self.estimated_position
    
    def get_target_start_pose(self):
        """
        Get the target start pose (tcp_pose)
        
        Returns:
            dict: Dictionary containing x, y, z, rx, ry, rz or None if not loaded
        """
        return self.target_start_pose
    
    def get_joint_positions(self):
        """
        Get the joint positions
        
        Returns:
            dict: Dictionary containing joint_1 to joint_6 positions in radians or None if not loaded
        """
        return self.joint_positions
    
    def print_data(self):
        """
        Print the loaded data in a readable format
        """
        print("=" * 50)
        print("UR Extract Task Data")
        print("=" * 50)
        
        if self.estimated_position:
            print("Estimated Position:")
            print(f"  x: {self.estimated_position.get('x', 'N/A')}")
            print(f"  y: {self.estimated_position.get('y', 'N/A')}")
            print(f"  z: {self.estimated_position.get('z', 'N/A')}")
        else:
            print("Estimated Position: Not available")
        
        print()
        
        if self.target_start_pose:
            print("Target Start Pose (TCP Pose):")
            print(f"  x: {self.target_start_pose.get('x', 'N/A')}")
            print(f"  y: {self.target_start_pose.get('y', 'N/A')}")
            print(f"  z: {self.target_start_pose.get('z', 'N/A')}")
            print(f"  rx: {self.target_start_pose.get('rx', 'N/A')}")
            print(f"  ry: {self.target_start_pose.get('ry', 'N/A')}")
            print(f"  rz: {self.target_start_pose.get('rz', 'N/A')}")
        else:
            print("Target Start Pose: Not available")
        
        print()
        
        if self.joint_positions:
            print("Joint Positions (radians):")
            print(f"  joint_1: {self.joint_positions.get('joint_1', 'N/A')}")
            print(f"  joint_2: {self.joint_positions.get('joint_2', 'N/A')}")
            print(f"  joint_3: {self.joint_positions.get('joint_3', 'N/A')}")
            print(f"  joint_4: {self.joint_positions.get('joint_4', 'N/A')}")
            print(f"  joint_5: {self.joint_positions.get('joint_5', 'N/A')}")
            print(f"  joint_6: {self.joint_positions.get('joint_6', 'N/A')}")
        else:
            print("Joint Positions: Not available")
        
        print("=" * 50)
    
    def Movel2ExecutionPosition(self, robot):
        """
        Move robot to execution position based on estimated position
        
        Args:
            robot: UR15Robot instance
            
        Returns:
            bool: True if movement successful, False otherwise
        """
        if not self.estimated_position or not self.target_start_pose:
            print("Error: Position data not loaded. Call load_data() first.")
            return False
        
        try:
            # Get estimated position coordinates
            estimated_x = self.estimated_position.get('x', 0)
            estimated_y = self.estimated_position.get('y', 0)
            estimated_z = self.estimated_position.get('z', 0)
            
            print(f"\nMoving to estimated position:")
            print(f"Target X: {estimated_x:.6f}")
            print(f"Target Y: {estimated_y:.6f}")
            print(f"Target Z: {estimated_z:.6f}")
            
            # Create target pose using estimated position and current orientation
            target_pose = [
                estimated_x,
                estimated_y, 
                estimated_z+0.2265-0.015,
                self.target_start_pose['rx'],  # Keep current orientation
                self.target_start_pose['ry'],
                self.target_start_pose['rz']
            ]
            
            print(f"Target pose: {target_pose}")
            
            # Move robot using movel function
            # Parameters: pose, acceleration, velocity, time, radius
            move_result = robot.movel(target_pose, a=0.1, v=0.05, t=0, r=0)
            
            if move_result == 0:
                print("Movement to execution position completed successfully!")
                return True
            else:
                print(f"Movement to execution position failed! (result: {move_result})")
                return False
                
        except Exception as e:
            print(f"Error controlling robot: {e}")
            return False

    def Movel2ExtractPosition(self, robot):
        """
        Move robot to extract position by moving 80cm in negative X direction from execution position
        
        Args:
            robot: UR15Robot instance
            
        Returns:
            bool: True if movement successful, False otherwise
        """
        if not self.estimated_position or not self.target_start_pose:
            print("Error: Position data not loaded. Call load_data() first.")
            return False
        
        try:
            # Get current TCP pose to use as base for extraction movement
            current_pose = robot.get_actual_tcp_pose()
            if current_pose is None:
                print("Error: Failed to get current TCP pose")
                return False
            
            print(f"\nCurrent TCP pose: {[f'{p:.6f}' for p in current_pose]}")
            
            # Create extract position by moving 80cm (0.8m) in negative X direction
            extract_distance = -0.85  # 80cm in negative X direction
            
            extract_pose = [
                current_pose[0] + extract_distance,  # X: move 80cm in negative direction
                current_pose[1],                     # Y: keep current
                current_pose[2],                     # Z: keep current  
                current_pose[3],                     # Rx: keep current orientation
                current_pose[4],                     # Ry: keep current orientation
                current_pose[5]                      # Rz: keep current orientation
            ]
            
            print(f"Moving to extract position:")
            print(f"X displacement: {extract_distance:.3f}m (negative direction)")
            print(f"Target extract pose: {[f'{p:.6f}' for p in extract_pose]}")
            
            # Move robot using movel function
            # Parameters: pose, acceleration, velocity, time, radius
            move_result = robot.movel(extract_pose, a=0.1, v=0.05, t=0, r=0)
            
            if move_result == 0:
                print("Movement to extract position completed successfully!")
                return True
            else:
                print(f"Movement to extract position failed! (result: {move_result})")
                return False
                
        except Exception as e:
            print(f"Error controlling robot during extraction movement: {e}")
            return False

    def Movej2LoggedPosition(self, robot):
        """
        Move robot to logged joint positions using movej
        
        Args:
            robot: UR15Robot instance
            
        Returns:
            bool: True if movement successful, False otherwise
        """
        if not self.joint_positions:
            print("Error: Joint positions data not loaded. Call load_data() first.")
            return False
        
        try:
            # Extract joint positions from the loaded data
            target_joints = [
                self.joint_positions.get('joint_1', 0),
                self.joint_positions.get('joint_2', 0),
                self.joint_positions.get('joint_3', 0),
                self.joint_positions.get('joint_4', 0),
                self.joint_positions.get('joint_5', 0),
                self.joint_positions.get('joint_6', 0)
            ]
            
            print(f"\nMoving to logged joint positions:")
            print(f"Target joints (rad): {[f'{j:.6f}' for j in target_joints]}")
            print(f"Target joints (deg): {[f'{math.degrees(j):.2f}°' for j in target_joints]}")
            
            # Move robot using movej function (joint movement)
            # Parameters: joint_positions, acceleration, velocity, time, radius
            move_result = robot.movej(target_joints, a=0.5, v=0.5)
            
            if move_result == 0:
                print("Movement to logged joint positions completed successfully!")
                return True
            else:
                print(f"Movement to logged joint positions failed! (result: {move_result})")
                return False
                
        except Exception as e:
            print(f"Error controlling robot during joint movement: {e}")
            return False

    def Movel2ExitPosition(self, robot):
        """
        Move robot to exit position by moving in Z direction to target_start_pose Z component
        
        Args:
            robot: UR15Robot instance
            
        Returns:
            bool: True if movement successful, False otherwise
        """
        if not self.target_start_pose:
            print("Error: Target start pose data not loaded. Call load_data() first.")
            return False
        
        try:
            # Get current TCP pose to use as base for exit movement
            current_pose = robot.get_actual_tcp_pose()
            if current_pose is None:
                print("Error: Failed to get current TCP pose")
                return False
            
            print(f"\nCurrent TCP pose: {[f'{p:.6f}' for p in current_pose]}")
            
            # Get target Z from target_start_pose
            target_z = self.target_start_pose['z']
            
            # Create exit position by moving to target_start_pose Z component
            exit_pose = [
                current_pose[0],  # X: keep current
                current_pose[1],  # Y: keep current
                target_z,         # Z: move to target_start_pose Z
                current_pose[3],  # Rx: keep current orientation
                current_pose[4],  # Ry: keep current orientation
                current_pose[5]   # Rz: keep current orientation
            ]
            
            print(f"Moving to exit position:")
            print(f"Current Z: {current_pose[2]:.6f}")
            print(f"Target Z: {target_z:.6f}")
            print(f"Z displacement: {target_z - current_pose[2]:.6f}")
            print(f"Target exit pose: {[f'{p:.6f}' for p in exit_pose]}")
            
            # Move robot using movel function
            # Parameters: pose, acceleration, velocity, time, radius
            move_result = robot.movel(exit_pose, a=0.1, v=0.05, t=0, r=0)
            
            if move_result == 0:
                print("Movement to exit position completed successfully!")
                return True
            else:
                print(f"Movement to exit position failed! (result: {move_result})")
                return False
                
        except Exception as e:
            print(f"Error controlling robot during exit movement: {e}")
            return False


# Example usage
if __name__ == "__main__":
    # Initialize UR15 robot at the beginning
    try:
        robot_ip = "192.168.1.15"
        robot_port = 30002
        robot = UR15Robot(ip=robot_ip, port=robot_port)
        
        # Open connection to robot
        print(f"Connecting to UR robot at {robot_ip}:{robot_port}...")
        result = robot.open()
        if result == 0:
            print("Successfully connected to UR robot")
        else:
            print(f"Failed to connect to UR robot (result: {result})")
            exit(1)
    except Exception as e:
        print(f"Error initializing robot: {e}")
        exit(1)
    
    # Create an instance of URExtractTask
    extract_task = URExtractTask()
    
    # Load data from the log file
    if extract_task.load_data():
        print("Data loaded successfully!")
        
        # Print the loaded data
        extract_task.print_data()
        
        # Access individual components
        estimated_pos = extract_task.get_estimated_position()
        target_pose = extract_task.get_target_start_pose()
        joint_positions = extract_task.get_joint_positions()
        
        if estimated_pos and target_pose and joint_positions:
            # First move to logged joint positions
            print("Step 1: Moving to logged joint positions...")
            joint_success = extract_task.Movej2LoggedPosition(robot)
            time.sleep(5)

            if joint_success:
                print("Successfully moved to logged joint positions!")
                
                # Execute movement to target position
                success = extract_task.Movel2ExecutionPosition(robot)
                time.sleep(10)
                
                if success:
                    print("Execution position reached successfully!")
                                        
                    # Move to extract position
                    extract_success = extract_task.Movel2ExtractPosition(robot)
                    time.sleep(20)

                    if extract_success:
                        print("Extract position reached successfully!")
                        
                        # Move to exit position (Z direction to target_start_pose Z component)
                        exit_success = extract_task.Movel2ExitPosition(robot)
                        time.sleep(10)

                        if exit_success:
                            print("Exit position reached successfully!")
                        else:
                            print("Failed to reach exit position!")
                        
                        
                        # Move back to logged joint positions
                        final_joint_success = extract_task.Movej2LoggedPosition(robot)
                        
                        if final_joint_success:
                            print("Successfully returned to logged joint positions!")
                        else:
                            print("Failed to return to logged joint positions!")
                    else:
                        print("Failed to reach extract position!")
                else:
                    print("Movement to execution position failed!")
            else:
                print("Failed to move to initial logged joint positions!")
        else:
            print("Error: Could not retrieve position data for movement calculation")

    else:
        print("Failed to load data from log file")

