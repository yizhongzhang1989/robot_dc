#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR Locate Handle2 Task
This script inherits from URLocateBase and customizes parameters for handle2 location task
"""

import os
import time
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor

from ur_locate_base import URLocateBase


class URLocateHandle2(URLocateBase):
    """
    UR Locate Handle2 class - specialized configuration for handle2 location
    Inherits from URLocateBase and overrides specific parameters
    """
    
    def __init__(self, api_url="http://10.172.151.12:8001", robot_ip="192.168.1.15", robot_port=30002):
        """
        Initialize URLocateHandle2 with customized parameters
        
        Args:
            api_url (str): URL for the FlowFormer++ Web API service
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
        """
        # Call parent class constructor
        super().__init__(api_url=api_url, robot_ip=robot_ip, robot_port=robot_port)
        
        # Override data directory path for handle2 task
        self.data_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_handle2_data')
        
        # Override result directory path for handle2 task
        self.result_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_handle2_result')
        
        # Override reference data paths
        self.ref_img_path = os.path.join(self.data_dir, 'ref_img.jpg')
        self.ref_keypoints_path = os.path.join(self.data_dir, 'ref_keypoints.json')
        self.ref_pose_path = os.path.join(self.data_dir, 'ref_pose.json')
        
        # Override collect start position for handle2 task (radians)
        self.collect_start_position = [
            -1.4736245314227503,
            -1.6785484753050746,
            -1.7203681468963623,
            -1.3410038512996216,
            1.5090150833129883,
            1.6812989711761475
        ]
        
        # Override movement offsets for handle2 task (in base coordinate system, unit: meters)
        # Format: {movement_name: [delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz]}
        self.movements = {
            "movement1": [0.03, 0, 0, 0, 0, 0],
            "movement2": [-0.03, 0, 0, 0, 0, 0],
            "movement3": [0, 0, 0.05, 0, 0, 0],
            "movement4": [-0.03, 0, -0.05, 0, 0, 0],
            "movement5": [0, 0, -0.05, 0, 0, 0]
        }
        
        self.local_x_kp_index = [0, 2]

        print(f"\n{'='*60}")
        print("URLocateHandle2 Initialized")
        print(f"{'='*60}")
        print(f"Data directory: {self.data_dir}")
        print(f"Result directory: {self.result_dir}")
        print(f"Collect start position: {self.collect_start_position}")
        print(f"Number of movements: {len(self.movements)}")
        print(f"{'='*60}\n")


def main():
    """
    Main function for URLocateHandle2 task
    Executes the complete workflow: data collection, tracking, 3D estimation, and coordinate system building
    """
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Initialize URLocateHandle2 instance (robot connection is handled internally)
        print("Initializing URLocateHandle2...")
        ur_handle2 = URLocateHandle2()
        
        # Check if robot was initialized successfully
        if ur_handle2.robot is None or not ur_handle2.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_handle2)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print('Node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        # Load camera parameters
        print("Loading camera parameters...")
        if not ur_handle2.load_camera_parameters():
            print("Failed to load camera parameters!")
            return
        
        try:
            # Perform auto data collection (includes moving to collect position)
            if ur_handle2.auto_collect_data():
                print("\n✅ Data collection completed successfully!")
                
                # Perform 3D keypoint estimation after data collection
                if ur_handle2.estimate_3d_position():
                    print("✅ 3D estimation completed successfully!")
                    
                    # Validate 3D estimation with reprojection
                    if ur_handle2.validate_keypoints_3d_estimate_result():
                        print("✅ 3D estimation validation completed!")
                    else:
                        print("⚠ 3D estimation validation failed!")
                    
                    # Build keypoint coordinate system
                    coord_system = ur_handle2.build_local_coordinate_system()
                    if coord_system:
                        print("✅ Coordinate system built successfully!")
                        
                        # Validate and visualize coordinate system
                        if ur_handle2.validate_local_coordinate_system(coord_system):
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
            import traceback
            traceback.print_exc()
        
        finally:
            # Always disconnect robot in finally block
            if ur_handle2.robot is not None:
                try:
                    ur_handle2.robot.close()
                    print("\nRobot disconnected successfully")
                except Exception as e:
                    print(f"Error disconnecting robot: {e}")
                
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_handle2.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()
        print("\n" + "="*60)
        print("URLocateHandle2 Task Completed")
        print("="*60)


if __name__ == "__main__":
    main()
