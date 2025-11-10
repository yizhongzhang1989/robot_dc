#!/usr/bin/env python3
"""
UR Locate Get Script
Extends URLocateBase with custom configuration for 'get' task
"""

import os
import sys
import time
import json
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor
from ur_locate_base import URLocateBase


class URLocateGet(URLocateBase):
    def __init__(self, api_url="http://10.172.100.34:8001", robot_ip="192.168.1.15", robot_port=30002):
        """
        Initialize URLocateGet class - extends URLocateBase with custom settings
        
        Args:
            api_url (str): URL for the FlowFormer++ Web API service
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
        """
        # Call parent constructor
        super().__init__(api_url=api_url, robot_ip=robot_ip, robot_port=robot_port)
        
        # Override movements for 'get' task
        # Format: {movement_name: [delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz]}
        self.movements = {
            "movement1": [0, 0, 0.03, 0, 0, 0],      # movement0
            "movement2": [0, 0, -0.03, 0, 0, 0],      # movement1
            "movement3": [0.03, 0, 0, 0, 0, 0],     # movement2
            "movement4": [-0.03, 0, 0, 0, 0, 0],      # movement3
            "movement5": [-0.03, 0, 0.05, 0, 0, 0]      # movement4
        }
        
        # Override data directory path
        self.data_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_get_data')
        
        # Override result directory path
        self.result_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_get_result')
        
        # Update reference data paths to use new data directory
        self.ref_img_path = os.path.join(self.data_dir, 'ref_img.jpg')
        self.ref_keypoints_path = os.path.join(self.data_dir, 'ref_keypoints.json')
        self.ref_pose_path = os.path.join(self.data_dir, 'ref_pose.json')
        
        # Try to load collect_start_position from ref_pose.json if it exists
        self._set_new_collect_start_position()
        
        print("\n" + "="*60)
        print("URLocateGet Configuration")
        print("="*60)
        print(f"Data directory: {self.data_dir}")
        print(f"Result directory: {self.result_dir}")
        print(f"Collect position: {self.collect_start_position}")
        print(f"Number of movements: {len(self.movements)}")
        print("="*60 + "\n")

    def _set_new_collect_start_position(self):
        """
        Load collect_start_position from ref_pose.json.
        This function requires the reference pose file to exist and be valid.
        If the file doesn't exist or is invalid, it will raise an exception to terminate the script.
        """
        if not os.path.exists(self.ref_pose_path):
            raise FileNotFoundError(f'✗ Reference pose file not found: {self.ref_pose_path}\n'
                                   f'  Please run data collection first to generate reference pose data.')
        
        try:
            with open(self.ref_pose_path, 'r') as f:
                ref_pose_data = json.load(f)
            
            if 'joint_angles' not in ref_pose_data:
                raise KeyError(f'✗ No "joint_angles" field found in {self.ref_pose_path}\n'
                              f'  The reference pose file is corrupted or incomplete.')
            
            self.collect_start_position = ref_pose_data['joint_angles']
            self.get_logger().info(f'✓ Loaded collect_start_position from {self.ref_pose_path}')
            self.get_logger().info(f'  Joint angles: {[f"{j:.4f}" for j in self.collect_start_position]}')
                
        except json.JSONDecodeError as e:
            raise ValueError(f'✗ Invalid JSON format in {self.ref_pose_path}: {e}\n'
                           f'  The reference pose file is corrupted.')
        except Exception as e:
            raise RuntimeError(f'✗ Error loading collect_start_position from {self.ref_pose_path}: {e}')


def main():
    """
    Main function to run the URLocateGet workflow
    """
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Initialize URLocateGet instance (robot connection is handled internally)
        ur_locate_get = URLocateGet()
        
        # Check if robot was initialized successfully
        if ur_locate_get.robot is None or not ur_locate_get.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_locate_get)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print('Node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        # Load camera parameters
        print("Loading camera parameters...")
        if not ur_locate_get.load_camera_parameters():
            print("Failed to load camera parameters!")
            return
        
        try:
            # Perform auto data collection (includes moving to collect position)
            if ur_locate_get.auto_collect_data():
                print("\n✅ Data collection completed successfully!")
                
                # Perform 3D keypoint estimation after data collection
                if ur_locate_get.estimate_3d_position():
                    print("✅ 3D estimation completed successfully!")
                    
                    # Validate 3D estimation with reprojection
                    print("\n" + "="*60)
                    print("Validating 3D Estimation with Reprojection...")
                    print("="*60)
                    if ur_locate_get.validate_keypoints_3d_estimate_result():
                        print("✅ 3D estimation validation completed!")
                    else:
                        print("⚠ 3D estimation validation failed!")
                    
                    # Build keypoint coordinate system
                    coord_system = ur_locate_get.build_local_coordinate_system()
                    if coord_system:
                        print("✅ Coordinate system built successfully!")
                        
                        # Validate and visualize coordinate system
                        print("\n" + "="*60)
                        print("Validating Coordinate System...")
                        print("="*60)
                        if ur_locate_get.validate_local_coordinate_system(coord_system):
                            print("✅ Coordinate system validation completed!")

                        else:
                            print("⚠ Coordinate system validation failed!")
                    else:
                        print("⚠ Failed to build coordinate system!")
                else:
                    print("⚠ 3D estimation failed!")
            else:
                print("\n✗ Data collection failed!")
                
        except Exception as e:
            print(f"Error during execution: {e}")
        
        finally:
            # Always disconnect robot in finally block
            if ur_locate_get.robot is not None:
                try:
                    ur_locate_get.robot.close()
                    print("Robot disconnected successfully")
                except Exception as e:
                    print(f"Error disconnecting robot: {e}")
                
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_locate_get.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()


if __name__ == '__main__':
    main()
