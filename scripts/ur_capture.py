#!/usr/bin/env python3
"""
URCapture - Simplified robot camera capture system
"""

import os
import sys
import cv2
import json
import time
import math
import numpy as np
import threading
import argparse
from datetime import datetime
from scipy.spatial.transform import Rotation as R

# Robot control imports
from ur15_robot_arm.ur15 import UR15Robot

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge


class URCapture(Node):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, camera_topic="/ur15_camera/image_raw", operation_name="test_operation", camera_params_path="../temp/ur15_cam_calibration_result/ur15_camera_parameters"):
        """
        Initialize URCapture class for basic robot camera capture
        
        Args:
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
            camera_topic (str): ROS topic name for camera images
            operation_name (str): Name of the operation for data directory
            camera_params_path (str): Path to camera calibration parameters directory
        """
        # Initialize ROS node
        super().__init__('ur_capture')
        # Update ROS node name for URCapture
        self.get_logger().info("URCapture node initialized")
        
        # =========================== Configurable Parameters ===========================
        # Get the script directory for relative paths
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # IP address and port for UR15 robot
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        
        # Camera topic name
        self.camera_topic = camera_topic
        
        # Operation name for data organization
        self.operation_name = operation_name
        
        # ============================= Instance variables ==============================
        # robot arm instance
        self.robot = None
        # Camera-related attributes
        self.cv_bridge = CvBridge()
        self.latest_image = None      

        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        # =============================== Other variables ==============================      
        # Default fallback values if config file not found
        self.collect_start_position = None
        
        # Collection movement offsets (in tcp coordinate system, unit: meters)
        self.movements = {
            "movement1": [0, 0, 0, 0, 0, 0],         # No offset
            "movement2": [0.01, 0, 0, 0, 0, 0],      # X+1cm
            "movement3": [-0.01, 0, 0, 0, 0, 0],     # X-1cm
            "movement4": [0, 0.01, 0, 0, 0, 0],      # Y+1cm
            "movement5": [0, -0.01, 0, 0, 0, 0]      # Y-1cm
        }
        
        # ============================ Initialization =================================
        # Initialize the robot connection
        self._initialize_robot()
        # Setup paths for data and camera parameters
        self._setup_paths(operation_name, camera_params_path)
        # Load collect start position from config file
        self._load_collect_position_from_config()

        # Subscribe to camera topic
        self.image_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
    
    def image_callback(self, msg):
        """
        Callback function for camera image subscription
        
        Args:
            msg: ROS Image message from camera
        """
        self.latest_image = msg
    
    def _setup_paths(self, operation_name, camera_params_path):
        """
        Setup data directory and camera parameter paths
        
        Args:
            operation_name (str): Name of the operation for data directory
            camera_params_path (str): Path to camera calibration parameters directory
        """
        # Data directory path (for storing collected data)
        self.data_dir = os.path.join(self.script_dir, '..', 'dataset', operation_name, 'test')
        
        # Parent directory of data_dir (operation directory)
        self.data_parent_dir = os.path.dirname(self.data_dir)
        
        # Camera calibration parameters path
        if not os.path.isabs(camera_params_path):
            camera_params_dir = os.path.join(self.script_dir, camera_params_path)
        else:
            camera_params_dir = camera_params_path
        
        # Camera intrinsic parameters file path
        self.camera_intrinsic_path = os.path.join(camera_params_dir, 'ur15_cam_calibration_result.json')
        
        # Camera extrinsic parameters file path (eye-in-hand calibration)
        self.camera_extrinsic_path = os.path.join(camera_params_dir, 'ur15_cam_eye_in_hand_result.json')
    
    def _load_collect_position_from_config(self):
        """
        Load collect start position from ref_img_1_pose.json file
        If file doesn't exist or key is missing, collect_start_position will remain None
        """
        try:
            ref_pose_path = os.path.join(self.data_parent_dir, 'ref_img_1_pose.json')
            if os.path.exists(ref_pose_path):
                with open(ref_pose_path, 'r') as f:
                    ref_pose = json.load(f)
                    
                if 'joint_angles' in ref_pose:
                    position = ref_pose['joint_angles']
                    if isinstance(position, list) and len(position) == 6:
                        self.collect_start_position = position
                        self.get_logger().info(f"✓ Loaded collect_start_position from ref_img_1_pose: {ref_pose_path}")
                        self.get_logger().info(f"  Position: {[f'{j:.4f}' for j in position]}")
                    else:
                        self.get_logger().error(f"✗ Invalid joint_angles format in ref_img_1_pose (expected list of 6 values)")
                else:
                    self.get_logger().error(f"✗ joint_angles not found in ref_img_1_pose file: {ref_pose_path}")
            else:
                self.get_logger().error(f"✗ Reference pose file not found: {ref_pose_path}")
        except Exception as e:
            self.get_logger().error(f"✗ Error loading collect position from ref_img_1_pose: {e}")
    
    def _initialize_robot(self):
        """Initialize UR15 robot instance and establish connection"""
        try:
            print(f'>>> Initializing UR15 robot at {self.robot_ip}:{self.robot_port}...')
            self.robot = UR15Robot(ip=self.robot_ip, port=self.robot_port)
            
            # Attempt to connect
            res = self.robot.open()
            if res == 0:
                print('✓ UR15 robot connected successfully')
            else:
                print(f'✗ Failed to connect to UR15 robot (error code: {res})')
                self.robot = None
        except Exception as e:
            print(f'Failed to initialize robot: {e}')
            self.robot = None
    
    def _load_camera_parameters(self):
        """
        Load camera intrinsic and extrinsic parameters from JSON files
        
        Returns:
            dict: Dictionary containing camera parameters, or None if loading fails
        """
        try:
            camera_params = {}
            
            # Load intrinsic parameters
            if os.path.exists(self.camera_intrinsic_path):
                with open(self.camera_intrinsic_path, 'r') as f:
                    intrinsic_data = json.load(f)
                    camera_params['camera_matrix'] = intrinsic_data.get('camera_matrix', None)
                    camera_params['distortion_coefficients'] = intrinsic_data.get('distortion_coefficients', None)
            else:
                print(f"✗ Camera intrinsic file not found: {self.camera_intrinsic_path}")
                camera_params['camera_matrix'] = None
                camera_params['distortion_coefficients'] = None
            
            # Load extrinsic parameters (cam2end matrix)
            if os.path.exists(self.camera_extrinsic_path):
                with open(self.camera_extrinsic_path, 'r') as f:
                    extrinsic_data = json.load(f)
                    camera_params['cam2end_matrix'] = extrinsic_data.get('cam2end_matrix', None)
            else:
                print(f"✗ Camera extrinsic file not found: {self.camera_extrinsic_path}")
                camera_params['cam2end_matrix'] = None
            
            return camera_params
            
        except Exception as e:
            print(f"Error loading camera parameters: {e}")
            return {
                'camera_matrix': None,
                'distortion_coefficients': None,
                'cam2end_matrix': None
            }
    
    def _create_session_directory(self):
        """
        Create a new session directory with auto-incremented number
        
        Returns:
            str: Path to the created session directory
        """
        base_session_dir = self.data_dir
        
        # Find the next available session number
        session_num = 1
        while True:
            session_dir = os.path.join(base_session_dir, f"session_{session_num:03d}")
            if not os.path.exists(session_dir):
                break
            session_num += 1
        
        # Create the session directory
        os.makedirs(session_dir, exist_ok=True)
        print(f"📁 Created new session directory: {session_dir}")
        
        return session_dir
    
    def set_collect_position(self, joint_angles):
        """
        Set custom collect start position
        
        Args:
            joint_angles: List of 6 joint angles in radians
        """
        if len(joint_angles) != 6:
            print("Error: joint_angles must contain exactly 6 values")
            return False
        
        self.collect_start_position = joint_angles.copy()
        print(f"✓ Updated collect position: {[f'{j:.4f}' for j in joint_angles]}")
        return True
    
    def _movej_to_collect_position(self, robot=None):
        """
        Move robot to predefined collect position using movej function
        """
        if robot is None:
            print("Error: Robot not initialized")
            return False
        
        if self.collect_start_position is None:
            print("Error: collect_start_position not loaded from config file")
            return False
        
        try:
            print(">>> Moving robot to collect start position...")
            print(f"    Target position: {[f'{j:.4f}' for j in self.collect_start_position]}")
                
            # Move robot using movej function (joint movement)
            # Parameters: joint_positions, acceleration, velocity
            move_result = robot.movej(self.collect_start_position, a=0.5, v=0.5)
            time.sleep(0.5)  # Wait for movement to complete
            
            if move_result == 0:
                print("Movement to collect position completed successfully!")
                return True
            else:
                print(f"Movement to collect position failed! (result: {move_result})")
                return False
                
        except Exception as e:
            print(f"Error controlling robot during movement to collect position: {e}")
            return False
    
    def capture_image_and_pose(self, save_dir, img_filename, pose_filename, robot=None):
        """
        Capture current camera image and robot pose, save to specified directory
        Args:
            save_dir: Directory to save the files
            img_filename: Filename for the image
            pose_filename: Filename for the pose JSON
            robot: Optional UR15Robot instance (if None, uses self.robot)
        """
        robot = robot or self.robot
        if robot is None:
            print("Error: Robot not initialized")
            return False
        
        print(f'>>> Capturing image and pose to {save_dir}...')
        
        # Create save directory if it doesn't exist
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            print(f'Created save directory: {save_dir}')
        
        try:
            # Read actual joint positions
            joint_positions = robot.get_actual_joint_positions()
            if joint_positions is None:
                print('Failed to read joint positions')
                return False
            
            # Read actual TCP pose
            tcp_pose = robot.get_actual_tcp_pose()
            if tcp_pose is None:
                print('Failed to read TCP pose')
                return False
            
            # Calculate end2base transformation matrix
            end2base = np.eye(4)
            end2base[:3, :3] = R.from_rotvec(tcp_pose[3:6]).as_matrix()
            end2base[:3, 3] = tcp_pose[:3]
            
            # Load camera parameters
            camera_params = self._load_camera_parameters()
            
            # Prepare data structure
            data = {
                "joint_angles": list(joint_positions),
                "end_xyzrpy": {
                    "x": tcp_pose[0],
                    "y": tcp_pose[1],
                    "z": tcp_pose[2],
                    "rx": tcp_pose[3],
                    "ry": tcp_pose[4],
                    "rz": tcp_pose[5]
                },
                "end2base": end2base.tolist(),
                "camera_matrix": camera_params['camera_matrix'],
                "distortion_coefficients": camera_params['distortion_coefficients'],
                "cam2end_matrix": camera_params['cam2end_matrix'],
                "timestamp": datetime.now().isoformat()
            }
            
            # Save to JSON file
            json_filename = os.path.join(save_dir, pose_filename)
            with open(json_filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            print(f'✓ Saved pose to: {json_filename}')
            
            # Save current camera image if available
            if self.latest_image is not None:
                try:
                    # Convert ROS Image to OpenCV format
                    cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
                    
                    # Save image
                    img_path = os.path.join(save_dir, img_filename)
                    cv2.imwrite(img_path, cv_image)
                    print(f'✓ Saved image to: {img_path}')
                    
                except Exception as e:
                    print(f'Error saving image: {e}')
                    return False
            else:
                print('No camera image available to save')
                return False
            
            print('<<< Capture completed successfully!')
            return True
            
        except Exception as e:
            print(f'Error capturing image and pose: {e}')
            return False
    
    def auto_collect_data(self, save_dir=None, robot=None):
        """
        Automatically collect data by moving robot by predefined offsets in base coordinate system.
        """
        robot = robot or self.robot
        if robot is None:
            print("Error: Robot not initialized")
            return False
        
        if save_dir is None:
            # Create session directory with auto-incremented number
            session_dir = self._create_session_directory()
            save_dir = session_dir

        print(">>> Starting Auto Data Collection..")

        try:
            if not self._movej_to_collect_position(robot):
                print("✗ Failed to move to collect position, aborting data collection")
                return False
            
            # Get current TCP pose as reference
            current_tcp_pose = robot.get_actual_tcp_pose()
            if current_tcp_pose is None:
                print("Failed to get current TCP pose")
                return False
            
            # Use movements from class variable, convert to list format for processing
            movements = []
            for idx, (movement_name, offset) in enumerate(self.movements.items()):
                movements.append({
                    "name": movement_name,
                    "offset": offset,
                    "index": idx
                })
            
            # Create save directory
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
                print(f"Created save directory: {save_dir}")
            
            # Wait for camera image to be available
            print("Waiting for camera image...")
            max_wait_time = 10.0  # Maximum wait time in seconds
            start_time = time.time()
            
            while self.latest_image is None and (time.time() - start_time) < max_wait_time:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_image is None:
                print("Error: No camera image received within timeout. Make sure camera is running.")
                return False
            
            print("✓ Camera image available, starting data collection...")
            
            success_count = 0
            
            for movement in movements:
                try:
                    print(f"\n--- {movement['name']} ---")
                    
                    # Use offset directly for move_tcp
                    offset = movement['offset']
                    
                    print(f"Move TCP by offset: {[f'{p:.4f}' for p in offset]}")
                    
                    # Move robot to target position using relative offset
                    move_result = robot.move_tcp(offset, a=0.1, v=0.05)
                    
                    if move_result != 0:
                        print(f"Movement failed for {movement['name']} (result: {move_result})")
                        continue
                    
                    # Wait for robot to settle
                    time.sleep(0.5)
                    
                    # Capture image and pose
                    img_filename = f"{movement['index']}.jpg"
                    pose_filename = f"{movement['index']}_pose.json"
                    
                    if self.capture_image_and_pose(save_dir, img_filename, 
                                                 pose_filename, robot=robot):
                        print(f"✓ Successfully collected data for {movement['name']}")
                        success_count += 1
                        time.sleep(0.2)  
                    else:
                        print(f"✗ Failed to capture data for {movement['name']}")
                    
                except Exception as e:
                    print(f"Error during {movement['name']} movement: {e}")
                    continue
            
            # Return to original position
            print(f"\n--- Returning to original position ---")
            return_result = robot.movel(current_tcp_pose, a=0.1, v=0.05)
            
            time.sleep(0.5)  # Wait for movement to complete

            if return_result == 0:
                print("✓ Successfully returned to original position")
            else:
                print(f"✗ Failed to return to original position (result: {return_result})")
            
            print(f"\nData collection completed: {success_count}/{len(movements)} successful")
            print("="*60)
            
            # Return save_dir if all movements were successful, False otherwise
            return save_dir if success_count == len(movements) else False
            
        except Exception as e:
            print(f"Error during auto data collection: {e}")
            return False

def main():
    """
    Main function for testing URCapture class
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='URCapture - Robot camera capture system')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot')
    parser.add_argument('--camera-topic', type=str, default='/ur15_camera/image_raw',
                       help='ROS topic name for camera images')
    parser.add_argument('--operation-name', type=str, default='test_operation',
                       help='Name of the operation for data directory')
    parser.add_argument('--camera-params-path', type=str, default='../temp/ur15_cam_calibration_result/ur15_camera_parameters',
                       help='Path to camera calibration parameters directory')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Initialize URCapture instance with command line arguments
        ur_capture = URCapture(
            robot_ip=args.robot_ip,
            robot_port=args.robot_port,
            camera_topic=args.camera_topic,
            operation_name=args.operation_name,
            camera_params_path=args.camera_params_path
        )
        
        # Check if robot was initialized successfully
        if ur_capture.robot is None or not ur_capture.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_capture)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print('URCapture node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        try:
            # Perform auto data collection (includes moving to collect position)
            result = ur_capture.auto_collect_data()
            if result:
                print("\n🎉 Data collection completed successfully!")
            else:
                print("\n❌ Data collection failed!")
                
        except KeyboardInterrupt:
            print("\n🛑 Data collection interrupted by user")
        except Exception as e:
            print(f"\n❌ Error during execution: {e}")
        
        finally:
            # Always disconnect robot in finally block
            if ur_capture.robot is not None:
                print("Disconnecting robot...")
                ur_capture.robot.close()
                print("✓ Robot disconnected")
                
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_capture.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()


if __name__ == "__main__":
    main()