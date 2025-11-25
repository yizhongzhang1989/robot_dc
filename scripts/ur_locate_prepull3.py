#!/usr/bin/env python3
"""
UR Locate PrePull3 Script
This script inherits from URLocateBase and customizes it for prepull3 location tasks.
"""

import os
import sys
import time
import json
import numpy as np
import rclpy
import threading
from datetime import datetime
from rclpy.executors import MultiThreadedExecutor

# Import the base class
from ur_locate_base import URLocateBase


class URLocatePrePull3(URLocateBase):
    def __init__(self, ffpp_web_url="http://10.172.100.34:8001", robot_ip="192.168.1.15", robot_port=30002):
        """
        Initialize URLocatePrepull2 class
        
        Args:
            ffpp_web_url (str): URL for the FlowFormer++ Web API service
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
        """
        # Call parent class constructor
        super().__init__(ffpp_web_url=ffpp_web_url, robot_ip=robot_ip, robot_port=robot_port)
        
        # Override ROS node name
        self.get_logger().info('URLocatePrePull3 initialized')
        
        # Override data directory path (for storing collected data)
        self.data_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_prepull3_data')
        
        # Override result directory path
        self.result_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_prepull3_result')
        
        # Update reference data paths to use new data directory
        self.ref_img_path = os.path.join(self.data_dir, 'ref_img.jpg')
        self.ref_keypoints_path = os.path.join(self.data_dir, 'ref_keypoints.json')
        self.ref_pose_path = os.path.join(self.data_dir, 'ref_pose.json')
        
        # Override local coordinate system X-axis keypoint indices
        # For prepull3 location, use keypoint 0 to keypoint 1 to define X-axis
        self.local_x_kp_index = [0, 1]
        
        # Try to load collect_start_position from ref_pose.json if it exists
        self._set_new_collect_start_position()
        
        # Load crack local coordinate system from log file
        crack_coord_file = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_crack_result', 'log_local_coordinate_system_result.json')
        self.load_crack_local_coordinate_system(crack_coord_file)

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

    # def calculate_kp3_kp4_rotation_angle(self, estimation_result_path=None, coord_system_path=None):
    #     """
    #     Calculate the rotation angle of the line connecting keypoint 3 and keypoint 4 relative to the local coordinate system X-axis.

    #     """
    #     print("\n" + "="*60)
    #     print("Calculating KP3-KP4 Rotation Angle Relative to Local X-Axis")
    #     print("="*60)
        
    #     try:
    #         # Define default paths if not provided
    #         if estimation_result_path is None:
    #             estimation_result_path = os.path.join(self.result_dir, self.ESTIMATION_RESULT_FILENAME)
            
    #         if coord_system_path is None:
    #             coord_system_path = os.path.join(self.result_dir, self.COORD_SYSTEM_RESULT_FILENAME)
            
    #         # Check if files exist
    #         if not os.path.exists(estimation_result_path):
    #             print(f"✗ Estimation result file not found: {estimation_result_path}")
    #             return None
                
    #         if not os.path.exists(coord_system_path):
    #             print(f"✗ Coordinate system file not found: {coord_system_path}")
    #             return None
            
    #         # Load 3D estimation results
    #         print("📖 Loading 3D estimation results...")
    #         with open(estimation_result_path, 'r') as f:
    #             estimation_data = json.load(f)
            
    #         estimated_keypoints = estimation_data.get('estimated_keypoints', [])
            
    #         # Load coordinate system data
    #         print("📖 Loading coordinate system data...")
    #         with open(coord_system_path, 'r') as f:
    #             coord_system = json.load(f)
            
    #         # Show available keypoints first
    #         available_keypoints = [kp.get('keypoint_index') for kp in estimated_keypoints]
    #         print(f"📋 Available keypoints: {sorted(available_keypoints)}")
            
    #         # Find keypoint 3 and keypoint 4 (using indices 2 and 3)
    #         kp3 = None
    #         kp4 = None
            
    #         for kp in estimated_keypoints:
    #             if kp.get('keypoint_index') == 2:  # kp3 corresponds to index 2
    #                 kp3 = np.array([kp['x'], kp['y'], kp['z']])
    #             elif kp.get('keypoint_index') == 3:  # kp4 corresponds to index 3
    #                 kp4 = np.array([kp['x'], kp['y'], kp['z']])
            
    #         if kp3 is None:
    #             print(f"✗ Error: Could not find keypoint at index 2 (kp3) in estimation results")
    #             print(f"   Available keypoints: {sorted(available_keypoints)}")
    #             return None
                
    #         if kp4 is None:
    #             print(f"✗ Error: Could not find keypoint at index 3 (kp4) in estimation results")
    #             print(f"   Available keypoints: {sorted(available_keypoints)}")
    #             print(f"   This function requires both kp3 (index 2) and kp4 (index 3) to be detected.")
    #             return None
            
    #         print(f"✓ Found keypoints:")
    #         print(f"  KP3: ({kp3[0]:.6f}, {kp3[1]:.6f}, {kp3[2]:.6f})")
    #         print(f"  KP4: ({kp4[0]:.6f}, {kp4[1]:.6f}, {kp4[2]:.6f})")
            
    #         # Calculate KP3-KP4 vector in world coordinates
    #         kp3_to_kp4_world = kp4 - kp3
    #         kp3_to_kp4_length = np.linalg.norm(kp3_to_kp4_world)
    #         kp3_to_kp4_world_unit = kp3_to_kp4_world / kp3_to_kp4_length
            
    #         print(f"\n📐 KP3→KP4 vector in world coordinates:")
    #         print(f"  Vector: [{kp3_to_kp4_world[0]:.6f}, {kp3_to_kp4_world[1]:.6f}, {kp3_to_kp4_world[2]:.6f}]")
    #         print(f"  Length: {kp3_to_kp4_length:.6f} m")
    #         print(f"  Unit vector: [{kp3_to_kp4_world_unit[0]:.6f}, {kp3_to_kp4_world_unit[1]:.6f}, {kp3_to_kp4_world_unit[2]:.6f}]")
            
    #         # Extract local coordinate system X-axis from coordinate system data
    #         local_x_axis = np.array(coord_system['axes']['x_axis']['vector'])
            
    #         print(f"📐 Local coordinate system X-axis:")
    #         print(f"  X-axis: [{local_x_axis[0]:.6f}, {local_x_axis[1]:.6f}, {local_x_axis[2]:.6f}]")
            
    #         # Calculate the angle between KP3-KP4 vector and local X-axis
    #         # Use dot product to find angle: cos(θ) = (a·b) / (|a|*|b|)
    #         dot_product = np.dot(kp3_to_kp4_world_unit, local_x_axis)
            
    #         # Clamp dot product to valid range for arccos to handle numerical errors
    #         dot_product = np.clip(dot_product, -1.0, 1.0)
            
    #         # Calculate angle in radians
    #         angle_radians = np.arccos(dot_product)
            
    #         # Convert to degrees
    #         angle_degrees = np.degrees(angle_radians)
            
    #         # Also calculate signed angle in the local coordinate XY plane
    #         # Project both vectors onto local XY plane
    #         local_y_axis = np.array(coord_system['axes']['y_axis']['vector'])
    #         local_z_axis = np.array(coord_system['axes']['z_axis']['vector'])
            
    #         # Project KP3-KP4 vector onto local XY plane
    #         kp3_to_kp4_proj_xy = (kp3_to_kp4_world_unit - 
    #                               np.dot(kp3_to_kp4_world_unit, local_z_axis) * local_z_axis)
            
    #         # Normalize projected vector
    #         if np.linalg.norm(kp3_to_kp4_proj_xy) > 1e-6:
    #             kp3_to_kp4_proj_xy_unit = kp3_to_kp4_proj_xy / np.linalg.norm(kp3_to_kp4_proj_xy)
                
    #             # Calculate signed angle in XY plane using atan2
    #             x_component = np.dot(kp3_to_kp4_proj_xy_unit, local_x_axis)
    #             y_component = np.dot(kp3_to_kp4_proj_xy_unit, local_y_axis)
                
    #             signed_angle_radians = np.arctan2(y_component, x_component)
    #             signed_angle_degrees = np.degrees(signed_angle_radians)
                
    #             print(f"\n🔍 Angle calculation results:")
    #             print(f"  Dot product: {dot_product:.6f}")
    #             print(f"  3D angle between KP3→KP4 and local X-axis: {angle_degrees:.2f}° ({angle_radians:.6f} rad)")
    #             print(f"  Signed angle in local XY plane: {signed_angle_degrees:.2f}° ({signed_angle_radians:.6f} rad)")
    #             print(f"  X component in local frame: {x_component:.6f}")
    #             print(f"  Y component in local frame: {y_component:.6f}")
                
    #             xy_plane_valid = True
                
    #         else:
    #             print(f"\n⚠ Warning: KP3→KP4 vector is nearly perpendicular to local XY plane")
    #             print(f"  3D angle between KP3→KP4 and local X-axis: {angle_degrees:.2f}° ({angle_radians:.6f} rad)")
    #             print(f"  Cannot calculate meaningful signed angle in XY plane")
                
    #             signed_angle_degrees = None
    #             signed_angle_radians = None
    #             x_component = None
    #             y_component = None
    #             xy_plane_valid = False
            
    #         # Calculate point at 12cm from kp3 along kp3->kp4 direction
    #         # Project KP3->KP4 vector to xy plane to avoid z-direction issues
    #         target_distance = 0.12  # 12cm in meters
            
    #         # Create xy-plane projection of the vector (set z component to 0)
    #         kp3_to_kp4_xy = np.array([kp3_to_kp4_world[0], kp3_to_kp4_world[1], 0])
    #         kp3_to_kp4_xy_length = np.linalg.norm(kp3_to_kp4_xy)
            
    #         if kp3_to_kp4_xy_length > 1e-6:
    #             kp3_to_kp4_xy_unit = kp3_to_kp4_xy / kp3_to_kp4_xy_length
    #             # Calculate target point using xy projection, keep original z coordinate
    #             target_point_12cm_xy = kp3 + target_distance * kp3_to_kp4_xy_unit
    #             target_point_12cm = np.array([target_point_12cm_xy[0], target_point_12cm_xy[1], kp3[2]])
    #         else:
    #             # If xy projection is too small, use original calculation with warning
    #             target_point_12cm = kp3 + target_distance * kp3_to_kp4_world_unit
    #             print(f"⚠ Warning: KP3->KP4 vector has very small xy component, using 3D calculation")
            
    #         print(f"\n📍 Target point calculation:")
    #         print(f"  Distance from KP3: {target_distance:.3f} m (12 cm)")
    #         print(f"  Original 3D vector: [{kp3_to_kp4_world[0]:.6f}, {kp3_to_kp4_world[1]:.6f}, {kp3_to_kp4_world[2]:.6f}]")
    #         if kp3_to_kp4_xy_length > 1e-6:
    #             print(f"  XY projection vector: [{kp3_to_kp4_xy[0]:.6f}, {kp3_to_kp4_xy[1]:.6f}, 0.000000]")
    #             print(f"  XY projection length: {kp3_to_kp4_xy_length:.6f} m")
    #         print(f"  Target point coordinates: ({target_point_12cm[0]:.6f}, {target_point_12cm[1]:.6f}, {target_point_12cm[2]:.6f})")
            
    #         # Create result dictionary
    #         result = {
    #             "keypoints": {
    #                 "kp3_coordinates": [float(kp3[0]), float(kp3[1]), float(kp3[2])],
    #                 "kp4_coordinates": [float(kp4[0]), float(kp4[1]), float(kp4[2])]
    #             },
    #             "vectors": {
    #                 "kp3_to_kp4_unit": [float(kp3_to_kp4_world_unit[0]), float(kp3_to_kp4_world_unit[1]), float(kp3_to_kp4_world_unit[2])]
    #             },
    #             "angles": {
    #                 "signed_angle_degrees": float(signed_angle_degrees) if signed_angle_degrees is not None else None,
    #                 "signed_angle_radians": float(signed_angle_radians) if signed_angle_radians is not None else None
    #             },
    #             "target_point": {
    #                 "coordinates_12cm": [float(target_point_12cm[0]), float(target_point_12cm[1]), float(target_point_12cm[2])],
    #                 "distance_from_kp3": float(target_distance),
    #                 "description": "Point at 12cm from KP3 along KP3->KP4 direction"
    #             },
    #             "timestamp": datetime.now().isoformat(),
    #             "method": "kp3_kp4_vector_angle_relative_to_local_x_axis"
    #         }
            
    #         # Save result to file
    #         os.makedirs(self.result_dir, exist_ok=True)
    #         angle_result_path = os.path.join(self.result_dir, 'kp3_kp4_rotation_angle_result.json')
            
    #         with open(angle_result_path, 'w') as f:
    #             json.dump(result, f, indent=2)
            
    #         print(f"\n💾 Angle calculation result saved to: {angle_result_path}")
    #         print(f"✅ KP3-KP4 rotation angle calculation completed successfully!")
            
    #         if xy_plane_valid:
    #             print(f"🎯 Main result: KP3→KP4 rotated {signed_angle_degrees:.2f}° relative to local X-axis in XY plane")
    #         else:
    #             print(f"🎯 Main result: KP3→KP4 has {angle_degrees:.2f}° 3D angle with local X-axis")
            
    #         return result
            
    #     except FileNotFoundError as e:
    #         print(f"✗ File not found: {e}")
    #         return None
    #     except json.JSONDecodeError as e:
    #         print(f"✗ JSON decode error: {e}")
    #         return None
    #     except Exception as e:
    #         print(f"✗ Error calculating KP3-KP4 rotation angle: {e}")
    #         return None


def main():
    """
    Main function for URLocatePrePull3
    """
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Initialize URLocatePrePull3 instance (robot connection is handled internally)
        ur_prepull3 = URLocatePrePull3()
        
        # Check if robot was initialized successfully
        if ur_prepull3.robot is None or not ur_prepull3.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_prepull3)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print('Node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        # Load camera parameters
        print("Loading camera parameters...")
        if not ur_prepull3.load_camera_parameters():
            print("Failed to load camera parameters!")
            return

        try:
            # Perform auto data collection (includes moving to collect position)
            if ur_prepull3.auto_collect_data():
                print("\n✅ Data collection completed successfully!")
                
                # Perform 3D keypoint estimation after data collection
                if ur_prepull3.estimate_3d_position():
                    print("✅ 3D estimation completed successfully!")
                    
                    # Validate 3D estimation with reprojection
                    print("\n" + "="*60)
                    print("Validating 3D Estimation with Reprojection...")
                    print("="*60)
                    if ur_prepull3.validate_keypoints_3d_estimate_result():
                        print("✅ 3D estimation validation completed!")
                    else:
                        print("⚠ 3D estimation validation failed!")
                    
                    # Build keypoint coordinate system
                    coord_system = ur_prepull3.build_local_coordinate_system()
                    if coord_system:
                        print("✅ Coordinate system built successfully!")
                        
                        # Validate and visualize coordinate system
                        print("\n" + "="*60)
                        print("Validating Coordinate System...")
                        print("="*60)
                        if ur_prepull3.validate_local_coordinate_system(coord_system):
                            print("✅ Coordinate system validation completed!")
                            
                            # # Calculate KP3-KP4 rotation angle relative to local X-axis
                            # print("\n" + "="*60)
                            # print("Calculating KP3-KP4 Rotation Angle...")
                            # print("="*60)
                            # angle_result = ur_prepull3.calculate_kp3_kp4_rotation_angle()
                            # if angle_result:
                            #     print("✅ KP3-KP4 rotation angle calculation completed!")
                            # else:
                            #     print("⚠ KP3-KP4 rotation angle calculation failed!")
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
            if ur_prepull3.robot is not None:
                try:
                    ur_prepull3.robot.close()
                    print("Robot disconnected successfully")
                except Exception as e:
                    print(f"Error disconnecting robot: {e}")
                
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_prepull3.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()


if __name__ == "__main__":
    main()
