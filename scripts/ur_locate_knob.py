#!/usr/bin/env python3
"""
UR Locate Knob Script
This script inherits from URLocateBase and customizes it for knob location tasks.
"""

import os
import sys
import time
import rclpy
import threading
from rclpy.executors import MultiThreadedExecutor

# Import the base class
from ur_locate_base import URLocateBase


class URLocateKnob(URLocateBase):
    def __init__(self, api_url="http://10.172.151.12:8001", robot_ip="192.168.1.15", robot_port=30002):
        """
        Initialize URLocateKnob class for UR robot knob location tasks
        
        Args:
            api_url (str): URL for the FlowFormer++ Web API service
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
        """
        # Initialize the base class
        super().__init__(api_url=api_url, robot_ip=robot_ip, robot_port=robot_port)
        
        # Override ROS node name
        self.get_logger().info('URLocateKnob initialized')
        
        # Override collect position joint angles (radians)
        self.collect_start_position = [
            1.7843583822250366,
            -0.5837817353061219,
            1.7311351935016077,
            -1.2094109815410157,
            0.27201223373413086,
            -3.0829160849200647
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
        self.data_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_knob_data')
        
        # Override result directory path
        self.result_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_knob_result')
        
        # Update reference data paths to use new data directory
        self.ref_img_path = os.path.join(self.data_dir, 'ref_img.jpg')
        self.ref_keypoints_path = os.path.join(self.data_dir, 'ref_keypoints.json')
        self.ref_pose_path = os.path.join(self.data_dir, 'ref_pose.json')
        
        # Override local coordinate system X-axis keypoint indices
        # For knob location, use keypoint 0 to keypoint 1 to define X-axis
        self.local_x_kp_index = [0, 2]
        
        print(f"URLocateKnob initialized with custom settings:")
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
        res = self.robot.movej(pose, a=0.5, v=0.5)
        time.sleep(0.5)

        pose = [-4.628224555646078, -1.5912000141539515, -0.061849094927310944, 
                 0.06347672521557612, 1.4398412704467773, -1.2330482641803187]
        print("Moving robot to zero state position...")
        res = self.robot.movej(pose, a=0.5, v=0.5)
        time.sleep(0.5)

        pose = [-4.628224555646078, -0.5939362210086365, 1.9152935186969202, 
                 0.06347672521557612, 1.4398412704467773, -1.2330482641803187]
        print("Moving robot to zero state position...")
        res = self.robot.movej(pose, a=0.5, v=0.5)
        time.sleep(0.5)

        pose = [-4.628224555646078, -0.5939362210086365, 1.9152935186969202, 
                 -1.9046393833556117, 1.4398412704467773, -1.2330482641803187]
        print("Moving robot to zero state position...")
        res = self.robot.movej(pose, a=0.5, v=0.5)
        time.sleep(0.5)

        pose = [-4.628224555646078, -0.5939362210086365, 1.9152935186969202,
                -1.9046393833556117, 0.1272939145565033, 3.737786054611206]
        print("Moving robot to zero state position...")
        res = self.robot.movej(pose, a=0.5, v=0.5)
        time.sleep(0.5)
        
        if res == 0:
            print("Robot moved to zero state successfully")
        else:
            print(f"Failed to move robot to zero state (error code: {res})")
        
        return res

def main():
    """
    Main function for URLocateKnob
    """
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Initialize URLocateKnob instance (robot connection is handled internally)
        ur_knob = URLocateKnob()
        
        # Check if robot was initialized successfully
        if ur_knob.robot is None or not ur_knob.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_knob)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print('Node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        # Load camera parameters
        print("Loading camera parameters...")
        if not ur_knob.load_camera_parameters():
            print("Failed to load camera parameters!")
            return

        # ur_knob.movej_to_safe_position_before_execution()
        # time.sleep(0.5)

        try:
            # Perform auto data collection (includes moving to collect position)
            if ur_knob.auto_collect_data():
                print("\n✅ Data collection completed successfully!")
                
                # Perform 3D keypoint estimation after data collection
                if ur_knob.estimate_3d_position():
                    print("✅ 3D estimation completed successfully!")
                    
                    # Validate 3D estimation with reprojection
                    print("\n" + "="*60)
                    print("Validating 3D Estimation with Reprojection...")
                    print("="*60)
                    if ur_knob.validate_keypoints_3d_estimate_result():
                        print("✅ 3D estimation validation completed!")
                    else:
                        print("⚠ 3D estimation validation failed!")
                    
                    # Build keypoint coordinate system
                    coord_system = ur_knob.build_local_coordinate_system()
                    if coord_system:
                        print("✅ Coordinate system built successfully!")
                        
                        # Validate and visualize coordinate system
                        print("\n" + "="*60)
                        print("Validating Coordinate System...")
                        print("="*60)
                        if ur_knob.validate_local_coordinate_system(coord_system):
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
            if ur_knob.robot is not None:
                try:
                    ur_knob.robot.close()
                    print("Robot disconnected successfully")
                except Exception as e:
                    print(f"Error disconnecting robot: {e}")
                
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_knob.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()


if __name__ == "__main__":
    main()
