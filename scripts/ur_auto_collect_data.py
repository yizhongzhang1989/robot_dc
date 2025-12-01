#!/usr/bin/env python3

"""
UR Auto Collect Data Script
This script automatically collects camera calibration data by:
1. Reading joint angles from collect_positions.json
2. Moving robot to each position using movej
3. Capturing images from /ur15_camera/image_raw topic
4. Saving images and pose data in JSON format

Usage:
    python3 ur_auto_collect_data.py [options]
    
Options:
    --robot-ip IP           IP address of UR15 robot (default: 192.168.1.15)
    --positions-file FILE   Path to collect_positions.json file (optional, if not provided, will use existing JSON files in data-dir)
    --data-dir DIR          Directory to save collected data (default: ../temp/ur15_cam_calibration_data)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import json
import os
import time
import argparse
from ur15_robot_arm.ur15 import UR15Robot
from cv_bridge import CvBridge
import cv2
import sys


def parse_arguments():
    """Parse command line arguments."""
    parser = argparse.ArgumentParser(
        description="UR15 Auto Collect Calibration Data",
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )
    
    parser.add_argument(
        '--robot-ip',
        type=str,
        default='192.168.1.15',
        help='IP address of UR15 robot'
    )
    
    parser.add_argument(
        '--positions-file',
        type=str,
        default=None,
        help='Path to collect_positions.json file (optional, if not provided, will use existing JSON files in data-dir)'
    )
    
    parser.add_argument(
        '--data-dir',
        type=str,
        default='../temp/ur15_cam_calibration_data',
        help='Directory to save collected data'
    )
    
    return parser.parse_args()


class URAutoCollectData(Node):
    def __init__(self, robot_ip="192.168.1.15", 
                 collect_positions_file=None,
                 data_save_dir="../temp/ur15_cam_calibration_data"):
        super().__init__('ur_auto_collect_data')
        
        # UR15 Robot connection
        self.robot_ip = robot_ip
        self.robot_port = 30002  # Fixed port for UR15
        self.ur_robot = None
        
        # File paths - handle relative paths relative to script directory
        try:
            from common.workspace_utils import get_scripts_directory
            self.script_dir = get_scripts_directory() if get_scripts_directory() else os.path.dirname(os.path.abspath(__file__))
        except ImportError:
            self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Handle data save directory
        if not os.path.isabs(data_save_dir):
            self.save_dir = os.path.abspath(os.path.join(self.script_dir, data_save_dir))
        else:
            self.save_dir = os.path.abspath(data_save_dir)
        
        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f'Created save directory: {self.save_dir}')
        
        # Handle positions file - can be None
        self.positions_file = None
        if collect_positions_file is not None:
            if not os.path.isabs(collect_positions_file):
                self.positions_file = os.path.abspath(os.path.join(self.script_dir, collect_positions_file))
            else:
                self.positions_file = os.path.abspath(collect_positions_file)
            
            # Verify positions file exists
            if not os.path.exists(self.positions_file):
                self.get_logger().error(f'Positions file not found: {self.positions_file}')
                raise FileNotFoundError(f'Positions file not found: {self.positions_file}')
        
        # Load positions from JSON file or existing data directory
        self.positions = self._load_positions()
        if self.positions_file:
            self.get_logger().info(f'Loaded {len(self.positions)} positions from {self.positions_file}')
        else:
            self.get_logger().info(f'Loaded {len(self.positions)} positions from existing JSON files in {self.save_dir}')
        
        # Current camera image
        self.latest_image = None
        self.cv_bridge = CvBridge()
        self.image_received = False
        
        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribe to camera image
        self.image_subscriber = self.create_subscription(
            Image,
            '/ur15_camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('UR Auto Collect Data Node started')
        self.get_logger().info(f'Save directory: {self.save_dir}')
        
        # Connect to UR robot
        self._connect_to_robot()
    
    def _load_positions(self):
        """
        Load joint positions from either:
        1. positions_file (collect_positions.json) if provided
        2. existing JSON files in save_dir if positions_file is None
        """
        try:
            if self.positions_file is not None:
                # Mode 1: Load from collect_positions.json
                with open(self.positions_file, 'r') as f:
                    data = json.load(f)
                
                # Convert to sorted list of (index, joint_angles)
                positions = []
                for key, value in data.items():
                    if key.startswith('position_'):
                        idx = int(key.replace('position_', ''))
                        positions.append((idx, value))
                
                # Sort by index
                positions.sort(key=lambda x: x[0])
                
                return positions
            else:
                # Mode 2: Load from existing JSON files in save_dir
                self.get_logger().info(f'No positions file provided, loading from existing JSON files in {self.save_dir}')
                
                # Find all JSON files with numeric names (0.json, 1.json, etc.)
                json_files = []
                for filename in os.listdir(self.save_dir):
                    if filename.endswith('.json'):
                        try:
                            # Extract index from filename (e.g., "0.json" -> 0)
                            idx = int(os.path.splitext(filename)[0])
                            json_files.append((idx, os.path.join(self.save_dir, filename)))
                        except ValueError:
                            # Skip non-numeric JSON files
                            continue
                
                if not json_files:
                    raise FileNotFoundError(f'No existing JSON files found in {self.save_dir}')
                
                # Sort by index
                json_files.sort(key=lambda x: x[0])
                
                # Load joint_angles from each JSON file
                positions = []
                for idx, filepath in json_files:
                    with open(filepath, 'r') as f:
                        pose_data = json.load(f)
                    
                    if 'joint_angles' not in pose_data:
                        self.get_logger().warn(f'No joint_angles found in {filepath}, skipping')
                        continue
                    
                    joint_angles = pose_data['joint_angles']
                    positions.append((idx, joint_angles))
                    self.get_logger().info(f'Loaded position {idx} from {filepath}')
                
                if not positions:
                    raise ValueError(f'No valid joint_angles found in JSON files in {self.save_dir}')
                
                return positions
                
        except Exception as e:
            self.get_logger().error(f'Error loading positions: {e}')
            raise
    
    def _connect_to_robot(self):
        """Connect to UR robot"""
        try:
            self.get_logger().info(f'Connecting to UR robot at {self.robot_ip}:{self.robot_port}...')
            self.ur_robot = UR15Robot(self.robot_ip, self.robot_port)
            result = self.ur_robot.open()
            if result == 0:
                self.get_logger().info('Successfully connected to UR robot')
            else:
                self.get_logger().error('Failed to connect to UR robot')
                self.ur_robot = None
        except Exception as e:
            self.get_logger().error(f'Error connecting to UR robot: {e}')
            self.ur_robot = None
    
    def image_callback(self, msg):
        """Callback for camera image"""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.cv_bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            self.latest_image = cv_image
            self.image_received = True
        except Exception as e:
            self.get_logger().error(f'Error converting image: {e}')
    
    def _rotvec_to_matrix(self, rx, ry, rz):
        """Convert rotation vector to rotation matrix"""
        import math
        angle = math.sqrt(rx**2 + ry**2 + rz**2)
        if angle < 1e-10:
            return np.eye(3)
        
        # Normalize axis
        kx, ky, kz = rx/angle, ry/angle, rz/angle
        
        # Rodrigues' rotation formula
        c = math.cos(angle)
        s = math.sin(angle)
        v = 1 - c
        
        R = np.array([
            [kx*kx*v + c,    kx*ky*v - kz*s, kx*kz*v + ky*s],
            [ky*kx*v + kz*s, ky*ky*v + c,    ky*kz*v - kx*s],
            [kz*kx*v - ky*s, kz*ky*v + kx*s, kz*kz*v + c]
        ])
        return R
    
    def _pose_to_matrix(self, pose):
        """Convert pose [X,Y,Z,Rx,Ry,Rz] to 4x4 homogeneous transformation matrix"""
        T = np.eye(4)
        T[0:3, 0:3] = self._rotvec_to_matrix(pose[3], pose[4], pose[5])
        T[0:3, 3] = [pose[0], pose[1], pose[2]]
        return T
    
    def wait_for_image(self, timeout=5.0):
        """Wait for camera image with timeout"""
        self.image_received = False
        start_time = time.time()
        
        while not self.image_received:
            rclpy.spin_once(self, timeout_sec=0.1)
            if time.time() - start_time > timeout:
                self.get_logger().warn('Timeout waiting for camera image')
                return False
        
        return True
    
    def save_pose_and_image(self, idx, joint_angles):
        """
        Save current robot pose and camera image
        
        Args:
            idx: Index number for the file name
            joint_angles: Joint angles to verify current position
        """
        try:
            # Wait for camera image
            if not self.wait_for_image(timeout=5.0):
                self.get_logger().error(f'Failed to get camera image for position {idx}')
                return False
            
            # Read actual joint positions from robot
            actual_joint_positions = self.ur_robot.get_actual_joint_positions()
            
            # Read actual TCP pose from robot
            actual_tcp_pose = self.ur_robot.get_actual_tcp_pose()
            
            # Convert TCP pose to transformation matrix
            end2base_matrix = self._pose_to_matrix(actual_tcp_pose)
            
            # Prepare pose data in the format of 0.json
            pose_data = {
                "joint_angles": actual_joint_positions,
                "end_xyzrpy": {
                    "x": actual_tcp_pose[0],
                    "y": actual_tcp_pose[1],
                    "z": actual_tcp_pose[2],
                    "rx": actual_tcp_pose[3],
                    "ry": actual_tcp_pose[4],
                    "rz": actual_tcp_pose[5]
                },
                "end2base": end2base_matrix.tolist()
            }
            
            # Save JSON file
            json_filename = os.path.join(self.save_dir, f'{idx}.json')
            with open(json_filename, 'w') as f:
                json.dump(pose_data, f, indent=2)
            
            # Save image file
            img_filename = os.path.join(self.save_dir, f'{idx}.jpg')
            cv2.imwrite(img_filename, self.latest_image)
            
            self.get_logger().info(f'✓ Saved position {idx}: {json_filename}, {img_filename}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error saving pose and image for position {idx}: {e}')
            return False
    
    def run_collection(self):
        """
        Main collection loop:
        1. Move to each position from collect_positions.json
        2. Wait for robot to reach position
        3. Capture image and save pose data
        """
        if self.ur_robot is None:
            self.get_logger().error('Robot not connected. Cannot run collection.')
            return False
        
        self.get_logger().info(f'Starting data collection for {len(self.positions)} positions...')
        
        success_count = 0
        
        for idx, joint_angles in self.positions:
            self.get_logger().info(f'\n--- Position {idx}/{len(self.positions)-1} ---')
            self.get_logger().info(f'Target joint angles: {[f"{j:.4f}" for j in joint_angles]}')
            
            # Move robot to position using movej
            try:
                self.get_logger().info(f'Moving to position {idx}...')
                result = self.ur_robot.movej(joint_angles, a=0.3, v=0.3)
                
                if result != 0:
                    self.get_logger().error(f'Failed to move to position {idx}')
                    continue
                
                # Wait for robot to stabilize
                self.get_logger().info('Waiting for robot to stabilize...')
                time.sleep(1.0)
                
                # Save pose and image
                if self.save_pose_and_image(idx, joint_angles):
                    success_count += 1
                else:
                    self.get_logger().warn(f'Failed to save data for position {idx}')
                
                # Short delay before next position
                time.sleep(0.5)
                
            except Exception as e:
                self.get_logger().error(f'Error processing position {idx}: {e}')
                continue
        
        self.get_logger().info(f'\n=== Collection Complete ===')
        self.get_logger().info(f'Successfully collected: {success_count}/{len(self.positions)} positions')
        self.get_logger().info(f'Data saved to: {self.save_dir}')
        
        return success_count == len(self.positions)


def main():
    """Main function"""
    
    # Parse command line arguments
    args = parse_arguments()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Create node
        collector = URAutoCollectData(
            robot_ip=args.robot_ip,
            collect_positions_file=args.positions_file,
            data_save_dir=args.data_dir
        )
        
        # Create executor for multithreaded spinning
        executor = MultiThreadedExecutor()
        executor.add_node(collector)
        
        # Start spinning in a separate thread
        import threading
        spin_thread = threading.Thread(target=executor.spin, daemon=True)
        spin_thread.start()
        
        # Wait for initial image
        collector.get_logger().info('Waiting for camera image...')
        time.sleep(2.0)
        
        # Run collection
        success = collector.run_collection()
        
        # Cleanup
        collector.get_logger().info('Shutting down...')
        executor.shutdown()
        collector.destroy_node()
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print('\nInterrupted by user')
        sys.exit(1)
    except Exception as e:
        print(f'Error: {e}')
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
