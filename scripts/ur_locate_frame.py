#!/usr/bin/env python3
"""
UR Locate Frame Script
This script inherits from URLocateBase and customizes it for frame location tasks.
"""

import os
import sys
import time
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

# Import the base class
from ur_locate_base import URLocateBase


class URLocateFrame(URLocateBase):
    def __init__(self, api_url="http://10.172.151.12:8001", robot_ip="192.168.1.15", robot_port=30002):
        """
        Initialize URLocateFrame class for UR robot frame location tasks
        
        Args:
            api_url (str): URL for the FlowFormer++ Web API service
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
        """
        # Initialize the base class
        super().__init__(api_url=api_url, robot_ip=robot_ip, robot_port=robot_port)
        
        # Override ROS node name
        self.get_logger().info('URLocateFrame initialized')
        
        # Override collect position joint angles (radians)
        self.collect_start_position = [
            -4.628224555646078,
            -0.5939362210086365,
            1.9152935186969202,
            -1.9046393833556117,
            0.1272939145565033,
            3.737786054611206
        ]
        
        # Override movement offsets (in base coordinate system, unit: meters)
        # Format: {movement_name: [delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz]}
        self.movements = {
            "movement1": [0.03, 0, 0, 0, 0, 0],
            "movement2": [0.08, 0.03, 0, 0, 0, 0],
            "movement3": [0.08, -0.03, 0, 0, 0, 0],
            "movement4": [0.05, 0, -0.03, 0, 0, 0],
            "movement5": [0.08, 0, -0.03, 0, 0, 0]
        }
        
        # Override data directory path (for storing collected data)
        self.data_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_frame_data')
        
        # Override result directory path
        self.result_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_frame_result')
        
        # Update reference data paths to use new data directory
        self.ref_img_path = os.path.join(self.data_dir, 'ref_img.jpg')
        self.ref_keypoints_path = os.path.join(self.data_dir, 'ref_keypoints.json')
        self.ref_pose_path = os.path.join(self.data_dir, 'ref_pose.json')
        
        # Override local coordinate system X-axis keypoint indices
        # For frame location, use keypoint 0 to keypoint 2 to define X-axis
        self.local_x_kp_index = [0, 2]
        
        print(f"URLocateFrame initialized with custom settings:")
        print(f"  Data directory: {self.data_dir}")
        print(f"  Result directory: {self.result_dir}")
        print(f"  Collect position: {[f'{j:.4f}' for j in self.collect_start_position]}")
        print(f"  Number of movements: {len(self.movements)}")

    def movej_to_safe_position_before_execution(self):
        """
        Move robot to get tool start position after process
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1

        pose = [-1.5214632193194788, -1.5912000141539515, -0.061849094927310944, 
                 0.06347672521557612, 1.4398412704467773, -1.2330482641803187]
        print("Moving robot to zero state position...")
        res = self.robot.movej(pose, a=0.5, v=0.3)
        time.sleep(0.5)

        pose = [-4.628224555646078, -1.5912000141539515, -0.061849094927310944, 
                 0.06347672521557612, 1.4398412704467773, -1.2330482641803187]
        print("Moving robot to zero state position...")
        res = self.robot.movej(pose, a=0.5, v=0.3)
        time.sleep(0.5)

        pose = [-4.628224555646078, -0.5939362210086365, 1.9152935186969202, 
                 0.06347672521557612, 1.4398412704467773, -1.2330482641803187]
        print("Moving robot to zero state position...")
        res = self.robot.movej(pose, a=0.5, v=0.3)
        time.sleep(0.5)

        pose = [-4.628224555646078, -0.5939362210086365, 1.9152935186969202, 
                 -1.9046393833556117, 1.4398412704467773, -1.2330482641803187]
        print("Moving robot to zero state position...")
        res = self.robot.movej(pose, a=0.5, v=0.3)
        time.sleep(0.5)

        pose = [-4.628224555646078, -0.5939362210086365, 1.9152935186969202,
                -1.9046393833556117, 0.1272939145565033, 3.737786054611206]
        print("Moving robot to zero state position...")
        res = self.robot.movej(pose, a=0.5, v=0.3)
        time.sleep(0.5)
        
        if res == 0:
            print("Robot moved to zero state successfully")
        else:
            print(f"Failed to move robot to zero state (error code: {res})")
        
        return res

def main():
    """
    Main function for URLocateFrame
    """
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Initialize URLocateFrame instance (robot connection is handled internally)
        ur_frame = URLocateFrame()
        
        # Check if robot was initialized successfully
        if ur_frame.robot is None or not ur_frame.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_frame)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print('Node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        # Load camera parameters
        print("Loading camera parameters...")
        if not ur_frame.load_camera_parameters():
            print("Failed to load camera parameters!")
            return

        ur_frame.movej_to_safe_position_before_execution()
        time.sleep(0.5)

        try:
            # Perform auto data collection (includes moving to collect position)
            if ur_frame.auto_collect_data():
                print("\n✅ Data collection completed successfully!")
                
                # Perform 3D keypoint estimation after data collection
                if ur_frame.estimate_3d_position():
                    print("✅ 3D estimation completed successfully!")
                    
                    # Validate 3D estimation with reprojection
                    print("\n" + "="*60)
                    print("Validating 3D Estimation with Reprojection...")
                    print("="*60)
                    if ur_frame.validate_keypoints_3d_estimate_result():
                        print("✅ 3D estimation validation completed!")
                    else:
                        print("⚠ 3D estimation validation failed!")
                    
                    # Build keypoint coordinate system
                    coord_system = ur_frame.build_local_coordinate_system()
                    if coord_system:
                        print("✅ Coordinate system built successfully!")
                        
                        # Validate and visualize coordinate system
                        print("\n" + "="*60)
                        print("Validating Coordinate System...")
                        print("="*60)
                        if ur_frame.validate_local_coordinate_system(coord_system):
                            print("✅ Coordinate system validation completed!")

                            # move to position to get the frame
                            pose2 = [-4.6480483452426355, -0.9079412978938599, 1.5085294882403772, 0.0630008417316894, 1.43977689743042, -1.2330697218524378]
                            ur_frame.robot.movej(pose2, a=0.5, v=0.3)
                            time.sleep(0.5)

                            pose1 = [-4.648319784794943,-1.5912381611266078, -0.06179070472717285,  0.06347481786694331, 1.439825415611267, -1.2331050078021448]

                            ur_frame.robot.movej(pose1, a=0.5, v=0.3)
                            time.sleep(0.5)

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
            if ur_frame.robot is not None:
                try:
                    ur_frame.robot.close()
                    print("Robot disconnected successfully")
                except Exception as e:
                    print(f"Error disconnecting robot: {e}")
                
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_frame.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()


if __name__ == "__main__":
    main()
