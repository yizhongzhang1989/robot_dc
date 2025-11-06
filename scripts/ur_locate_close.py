#!/usr/bin/env python3
"""
UR Locate Close Script
This script inherits from URLocateBase and customizes it for close location tasks.
"""

import os
import sys
import time
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

# Import the base class
from ur_locate_base import URLocateBase


class URLocateClose(URLocateBase):
    def __init__(self, api_url="http://10.172.100.34:8001", robot_ip="192.168.1.15", robot_port=30002):
        """
        Initialize URLocateClose class for UR robot close location tasks
        
        Args:
            api_url (str): URL for the FlowFormer++ Web API service
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
        """
        # Initialize the base class
        super().__init__(api_url=api_url, robot_ip=robot_ip, robot_port=robot_port)
        
        # Override ROS node name
        self.get_logger().info('URLocateClose initialized')
        
        # Override collect position joint angles (radians)
        self.collect_start_position = [
            1.6349868774414062,
            -1.2644238334945221,
            2.0044129530536097,
            -2.4834019146361292,
            -0.592684570943014,
            -1.3681491057025355
        ]
        
        # Override movement offsets (in base coordinate system, unit: meters)
        # Format: {movement_name: [delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz]}
        self.movements = {
            "movement1": [0.03, 0, 0, 0, 0, 0],
            "movement2": [0.05, 0.03, 0, 0, 0, 0],
            "movement3": [0.05, -0.03, 0, 0, 0, 0],
            "movement4": [0.05, 0, -0.03, 0, 0, 0],
            "movement5": [0.05, 0, -0.03, 0, 0, 0]
        }
        
        # Override data directory path (for storing collected data)
        self.data_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_close_data')
        
        # Override result directory path
        self.result_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_close_result')
        
        # Update reference data paths to use new data directory
        self.ref_img_path = os.path.join(self.data_dir, 'ref_img.jpg')
        self.ref_keypoints_path = os.path.join(self.data_dir, 'ref_keypoints.json')
        self.ref_pose_path = os.path.join(self.data_dir, 'ref_pose.json')
        
        # Override local coordinate system X-axis keypoint indices
        # For close location, use keypoint 0 to keypoint 1 to define X-axis
        self.local_x_kp_index = [0, 1]
        
        print(f"URLocateClose initialized with custom settings:")
        print(f"  Data directory: {self.data_dir}")
        print(f"  Result directory: {self.result_dir}")
        print(f"  Collect position: {[f'{j:.4f}' for j in self.collect_start_position]}")
        print(f"  Number of movements: {len(self.movements)}")


def main():
    """
    Main function for URLocateClose
    """
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Initialize URLocateClose instance (robot connection is handled internally)
        ur_close = URLocateClose()
        
        # Check if robot was initialized successfully
        if ur_close.robot is None or not ur_close.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_close)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print('Node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        # Load camera parameters
        print("Loading camera parameters...")
        if not ur_close.load_camera_parameters():
            print("Failed to load camera parameters!")
            return

        try:
            # Perform auto data collection (includes moving to collect position)
            if ur_close.auto_collect_data():
                print("\n✅ Data collection completed successfully!")
                
                # Perform 3D keypoint estimation after data collection
                if ur_close.estimate_3d_position():
                    print("✅ 3D estimation completed successfully!")
                    
                    # Validate 3D estimation with reprojection
                    print("\n" + "="*60)
                    print("Validating 3D Estimation with Reprojection...")
                    print("="*60)
                    if ur_close.validate_keypoints_3d_estimate_result():
                        print("✅ 3D estimation validation completed!")
                    else:
                        print("⚠ 3D estimation validation failed!")
                    
                    # Build keypoint coordinate system
                    coord_system = ur_close.build_local_coordinate_system()
                    if coord_system:
                        print("✅ Coordinate system built successfully!")
                        
                        # Validate and visualize coordinate system
                        print("\n" + "="*60)
                        print("Validating Coordinate System...")
                        print("="*60)
                        if ur_close.validate_local_coordinate_system(coord_system):
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
            if ur_close.robot is not None:
                try:
                    ur_close.robot.close()
                    print("Robot disconnected successfully")
                except Exception as e:
                    print(f"Error disconnecting robot: {e}")
                
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_close.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()


if __name__ == "__main__":
    main()
