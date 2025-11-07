#!/usr/bin/env python3
"""
UR Locate Push2End Script
This script inherits from URLocateBase and customizes it for push2end location tasks.
"""

import os
import sys
import time
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

# Import the base class
from ur_locate_base import URLocateBase


class URLocatePush2End(URLocateBase):
    def __init__(self, api_url="http://10.172.100.34:8001", robot_ip="192.168.1.15", robot_port=30002):
        """
        Initialize URLocatePush2End class for UR robot push2end location tasks
        
        Args:
            api_url (str): URL for the FlowFormer++ Web API service
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
        """
        # Initialize the base class
        super().__init__(api_url=api_url, robot_ip=robot_ip, robot_port=robot_port)
        
        # Override ROS node name
        self.get_logger().info('URLocatePush2End initialized')
        
        # Override collect position joint angles (radians)
        self.collect_start_position = [
            1.6157532930374146,
            -0.6971209806254883,
            2.0771897474872034,
            -1.51478514940057,
            0.24884912371635437,
            -2.985495392476217
        ]
        
        # Override movement offsets (in base coordinate system, unit: meters)
        # Format: {movement_name: [delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz]}
        self.movements = {
            "movement1": [0.01, 0, 0, 0, 0, 0],
            "movement2": [0.03, 0.01, 0, 0, 0, 0],
            "movement3": [0.03, -0.01, 0, 0, 0, 0],
            "movement4": [0.03, 0, -0.01, 0, 0, 0],
            "movement5": [0.03, 0, -0.01, 0, 0, 0]
        }
        
        # Override data directory path (for storing collected data)
        self.data_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_push2end_data')
        
        # Override result directory path
        self.result_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_push2end_result')
        
        # Update reference data paths to use new data directory
        self.ref_img_path = os.path.join(self.data_dir, 'ref_img.jpg')
        self.ref_keypoints_path = os.path.join(self.data_dir, 'ref_keypoints.json')
        self.ref_pose_path = os.path.join(self.data_dir, 'ref_pose.json')
        
        # Override local coordinate system X-axis keypoint indices
        # For push2end location, use keypoint 0 to keypoint 1 to define X-axis
        self.local_x_kp_index = [0, 1]
        
        print(f"URLocatePush2End initialized with custom settings:")
        print(f"  Data directory: {self.data_dir}")
        print(f"  Result directory: {self.result_dir}")
        print(f"  Collect position: {[f'{j:.4f}' for j in self.collect_start_position]}")
        print(f"  Number of movements: {len(self.movements)}")


def main():
    """
    Main function for URLocatePush2End
    """
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Initialize URLocatePush2End instance (robot connection is handled internally)
        ur_push2end = URLocatePush2End()
        
        # Check if robot was initialized successfully
        if ur_push2end.robot is None or not ur_push2end.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_push2end)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print('Node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        # Load camera parameters
        print("Loading camera parameters...")
        if not ur_push2end.load_camera_parameters():
            print("Failed to load camera parameters!")
            return

        try:
            # Perform auto data collection (includes moving to collect position)
            if ur_push2end.auto_collect_data():
                print("\n✅ Data collection completed successfully!")
                
                # Perform 3D keypoint estimation after data collection
                if ur_push2end.estimate_3d_position():
                    print("✅ 3D estimation completed successfully!")
                    
                    # Validate 3D estimation with reprojection
                    print("\n" + "="*60)
                    print("Validating 3D Estimation with Reprojection...")
                    print("="*60)
                    if ur_push2end.validate_keypoints_3d_estimate_result():
                        print("✅ 3D estimation validation completed!")
                    else:
                        print("⚠ 3D estimation validation failed!")
                    
                    # Build keypoint coordinate system
                    coord_system = ur_push2end.build_local_coordinate_system()
                    if coord_system:
                        print("✅ Coordinate system built successfully!")
                        
                        # Validate and visualize coordinate system
                        print("\n" + "="*60)
                        print("Validating Coordinate System...")
                        print("="*60)
                        if ur_push2end.validate_local_coordinate_system(coord_system):
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
            if ur_push2end.robot is not None:
                try:
                    ur_push2end.robot.close()
                    print("Robot disconnected successfully")
                except Exception as e:
                    print(f"Error disconnecting robot: {e}")
                
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_push2end.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()


if __name__ == "__main__":
    main()
