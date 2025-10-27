#!/usr/bin/env python3
"""
UR Locate Handle1 Script
Based on URLocateBase class with custom configurations
"""

import os
import sys
import time
import threading
import rclpy
from rclpy.executors import MultiThreadedExecutor

# Import the base class
from ur_locate_base import URLocateBase


class URLocateHandle1(URLocateBase):
    """
    Custom location test class for handle1 task
    Inherits from URLocateBase and overrides specific configurations
    """
    
    def __init__(self, api_url="http://10.172.151.12:8001", robot_ip="192.168.1.15", robot_port=30002):
        """
        Initialize URLocateHandle1 class
        
        Args:
            api_url (str): URL for the FlowFormer++ Web API service
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
        """
        # Call parent class constructor
        super().__init__(api_url=api_url, robot_ip=robot_ip, robot_port=robot_port)
        
        # Override collect start position
        # Joint angles in radians for handle1 task
        self.collect_start_position = [
            -0.7684944311725062,
            -1.872650762597555,
            -1.5162721872329712,
            -0.962266282444336,
            1.8500767946243286,
            -0.6623833815204065
        ]
        
        # Override movements with custom offsets
        # Format: movement name -> [delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz]
        self.movements = {
            "movement1": [-0.03, 0, 0, 0, 0, 0],
            "movement2": [0.03, 0, 0, 0, 0, 0],
            "movement3": [-0.05, 0, 0.05, 0, 0, 0],
            "movement4": [0, 0, -0.03, 0, 0, 0],
            "movement5": [0, 0, 0.03, 0, 0, 0]
        }
        
        # Override data directory
        self.data_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_handle1_data')
        
        # Override result directory
        self.result_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_handle1_result')
        
        # Update reference data paths based on new data_dir
        self.ref_img_path = os.path.join(self.data_dir, 'ref_img.jpg')
        self.ref_keypoints_path = os.path.join(self.data_dir, 'ref_keypoints.json')
        self.ref_pose_path = os.path.join(self.data_dir, 'ref_pose.json')
        
        self.local_x_kp_index = [0, 2]

        print(f"✓ URLocateHandle1 initialized")
        print(f"  Data directory: {self.data_dir}")
        print(f"  Result directory: {self.result_dir}")


def main():
    """
    Main function for URLocateHandle1 task
    Executes the complete workflow: data collection, 3D estimation, and validation
    """
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Initialize URLocateHandle1 instance
        print("="*60)
        print("UR Locate Handle1 Task")
        print("="*60)
        
        ur_handle1 = URLocateHandle1()
        
        # Check if robot was initialized successfully
        if ur_handle1.robot is None or not ur_handle1.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_handle1)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print('Node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        # Load camera parameters
        print("\n" + "="*60)
        print("Loading camera parameters...")
        print("="*60)
        if not ur_handle1.load_camera_parameters():
            print("✗ Failed to load camera parameters!")
            return
        print("✓ Camera parameters loaded successfully")
        
        try:
            # Step 1: Perform auto data collection

            if ur_handle1.auto_collect_data():
                print("\n✅ Data collection completed successfully!")
                
                # Step 2: Perform 3D keypoint estimation
                if ur_handle1.estimate_3d_position():
                    print("✅ 3D estimation completed successfully!")
                    
                    # Step 3: Validate 3D estimation with reprojection
                    if ur_handle1.validate_keypoints_3d_estimate_result():
                        print("✅ 3D estimation validation completed!")
                    else:
                        print("⚠ 3D estimation validation failed!")
                    
                    # Step 4: Build keypoint coordinate system
                    coord_system = ur_handle1.build_local_coordinate_system()
                    if coord_system:
                        print("✅ Coordinate system built successfully!")
                        
                        # Step 5: Validate and visualize coordinate system
                        print("\n" + "="*60)
                        print("Step 5: Validating Coordinate System")
                        print("="*60)
                        if ur_handle1.validate_local_coordinate_system(coord_system):
                            print("✅ Coordinate system validation completed!")
                        else:
                            print("⚠ Coordinate system validation failed!")
                    else:
                        print("⚠ Failed to build coordinate system!")
                else:
                    print("⚠ 3D estimation failed!")
            else:
                print("\n✗ Data collection failed!")
                
        except KeyboardInterrupt:
            print("\n\nTask interrupted by user")
        except Exception as e:
            print(f"\n✗ Error during execution: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # Always disconnect robot in finally block
            if ur_handle1.robot is not None:
                try:
                    ur_handle1.robot.close()
                    print("\n✓ Robot disconnected successfully")
                except Exception as e:
                    print(f"✗ Error disconnecting robot: {e}")
            
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_handle1.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()
        print("\n" + "="*60)
        print("UR Locate Handle1 Task Finished")
        print("="*60)


if __name__ == "__main__":
    main()
