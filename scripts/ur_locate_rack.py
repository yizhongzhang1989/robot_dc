#!/usr/bin/env python3
"""
URLocateRack - Automated rack location capture system
Sequentially executes location operations for rack1, rack2, and rack3
"""

import os
import sys
import time
import json
import subprocess
import numpy as np
import cv2
import matplotlib.pyplot as plt
from pathlib import Path
from datetime import datetime
from scipy.spatial.transform import Rotation as R
import rclpy
from rclpy.node import Node

# Add ThirdParty robot_vision to path for Web API client
sys.path.append(os.path.join(os.path.dirname(__file__), 'ThirdParty', 'robot_vision'))
from core.positioning_3d_webapi import load_camera_params_from_json
from robot_status.client_utils import RobotStatusClient


class URLocateRack:
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, 
                 camera_topic="/ur15_camera/image_raw",
                 camera_params_path="../temp/ur15_cam_calibration_result/ur15_camera_parameters",
                 verbose=True,
                 ros_node=None):
        """
        Initialize URLocateRack class for automated rack location operations
        
        Args:
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
            camera_topic (str): ROS topic name for camera images
            camera_params_path (str): Path to camera calibration parameters directory
            verbose (bool): If True, saves validation images and error logs to disk
            ros_node (Node): ROS2 node instance for RobotStatusClient (optional)
        """
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.camera_topic = camera_topic
        self.camera_params_path = camera_params_path
        self.verbose = verbose
        self.ros_node = ros_node
        
        # Get the directory where this script is located
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        self.ur_3d_positioning_path = os.path.join(self.script_dir, "ur_3d_positioning.py")
        
        # Verify ur_3d_positioning.py exists
        if not os.path.exists(self.ur_3d_positioning_path):
            raise FileNotFoundError(f"ur_3d_positioning.py not found at: {self.ur_3d_positioning_path}")
        
        # Define rack operations in order
        self.rack_operations = ["rack1", "rack2", "rack3", "rack4"]
        
        # Initialize RobotStatusClient
        self.robot_status_client = None
        if self.ros_node:
            try:
                self.robot_status_client = RobotStatusClient(self.ros_node, timeout_sec=5.0)
                print("✓ RobotStatusClient initialized successfully")
            except Exception as e:
                print(f"⚠ Failed to initialize RobotStatusClient: {e}")
                self.robot_status_client = None
    
    def auto_capture(self):
        """
        Automatically execute location operations for rack1, rack2, and rack3 in sequence
        
        Returns:
            dict: Summary of execution results for each rack operation
        """
        results = {}
                
        # Execute each rack operation sequentially
        for idx, operation_name in enumerate(self.rack_operations, 1):
            print(f"\n{'='*70}")
            print(f" [{idx}/{len(self.rack_operations)}] Executing operation: {operation_name}")
            print(f"{'='*70}\n")
            
            try:
                # Build command to execute ur_3d_positioning.py
                cmd = [
                    "python3",
                    self.ur_3d_positioning_path,
                    "--robot-ip", self.robot_ip,
                    "--robot-port", str(self.robot_port),
                    "--camera-topic", self.camera_topic,
                    "--operation-name", operation_name,
                    "--camera-params-path", self.camera_params_path
                ]
                
                # Add verbose flag if disabled
                if not self.verbose:
                    cmd.append("--no-verbose")
                
                # Execute the command
                print(f"Running command: {' '.join(cmd)}\n")
                start_time = time.time()
                
                result = subprocess.run(
                    cmd,
                    cwd=self.script_dir,
                    capture_output=False,  # Show output in real-time
                    text=True
                )
                
                elapsed_time = time.time() - start_time
                
                # Record result
                results[operation_name] = {
                    "success": result.returncode == 0,
                    "return_code": result.returncode,
                    "elapsed_time": elapsed_time
                }
                
                if result.returncode == 0:
                    print(f"\n✓ {operation_name} completed successfully in {elapsed_time:.2f} seconds")
                else:
                    print(f"\n✗ {operation_name} failed with return code {result.returncode}")
                    print(f"  Elapsed time: {elapsed_time:.2f} seconds")
                
            except KeyboardInterrupt:
                print(f"\n\n🛑 Operation interrupted by user during {operation_name}")
                results[operation_name] = {
                    "success": False,
                    "return_code": -1,
                    "error": "Interrupted by user"
                }
                break
                
            except Exception as e:
                print(f"\n✗ Error executing {operation_name}: {e}")
                results[operation_name] = {
                    "success": False,
                    "return_code": -1,
                    "error": str(e)
                }
                
            # Add a short delay between operations
            if idx < len(self.rack_operations):
                print(f"\nWaiting 2 seconds before next operation...\n")
                time.sleep(2)
        
        # Print summary
        self._print_summary(results)
        
        return results
    
    def perform_wobj_frame_building(self, rack1_result_path=None, rack2_result_path=None, rack3_result_path=None, rack4_result_path=None):
        """
        Build a wobj coordinate system based on 3D positioning keypoints from rack1, rack2, and rack3.
        
        The coordinate system is defined as follows:
        - Origin: at rack1 feature point (keypoint[0])
        - X-axis: rack1_point → rack2_point direction (rack1_KP0 → rack2_KP0)
        - Z-axis: rack1_point → rack3_point direction (rack1_KP0 → rack3_KP0)
        - Y-axis: follows right-hand rule (Y = Z × X)
        
        Args:
            rack1_result_path (str): Path to rack1 3d_positioning_result.json file
            rack2_result_path (str): Path to rack2 3d_positioning_result.json file
            rack3_result_path (str): Path to rack3 3d_positioning_result.json file
            rack4_result_path (str): Path to rack4 3d_positioning_result.json file
            
        Returns:
            dict: Coordinate system information or None if failed
        """
        print("\n" + "="*70)
        print(" Building Wobj Coordinate System")
        print("="*70)
        
        try:
            # ===================== Find result files =====================
            dataset_dir = os.path.join(self.script_dir, "..", "dataset")
            
            # Helper function to find most recent session result
            def find_latest_result(operation_name, provided_path=None):
                if provided_path and os.path.exists(provided_path):
                    print(f"✓ Using provided {operation_name} result: {provided_path}")
                    return provided_path
                
                # Look for most recent session in dataset/{operation_name}/result/
                result_dir = os.path.join(dataset_dir, operation_name, "result")
                
                if not os.path.exists(result_dir):
                    print(f"✗ Result directory not found: {result_dir}")
                    return None
                
                # Find all session directories
                sessions = [d for d in os.listdir(result_dir) 
                           if os.path.isdir(os.path.join(result_dir, d))]
                
                if not sessions:
                    print(f"✗ No session directories found in {result_dir}")
                    return None
                
                # Sort to get most recent
                sessions.sort(reverse=True)
                latest_session = sessions[0]
                result_file = os.path.join(result_dir, latest_session, "3d_positioning_result.json")
                
                if os.path.exists(result_file):
                    print(f"✓ Using latest {operation_name} result: {result_file}")
                    return result_file
                else:
                    print(f"✗ 3d_positioning_result.json not found in {os.path.join(result_dir, latest_session)}")
                    return None
            
            # Find all four result files
            rack1_file = find_latest_result("rack1", rack1_result_path)
            rack2_file = find_latest_result("rack2", rack2_result_path)
            rack3_file = find_latest_result("rack3", rack3_result_path)
            rack4_file = find_latest_result("rack4", rack4_result_path)
            
            if not all([rack1_file, rack2_file, rack3_file, rack4_file]):
                print("\n✗ Failed to locate all required result files")
                return None
            
            # ===================== Load data =====================
            print("\n📖 Loading 3D positioning results...")
            
            with open(rack1_file, 'r') as f:
                rack1_data = json.load(f)
            rack1_points_3d = rack1_data.get('points_3d', [])
            
            with open(rack2_file, 'r') as f:
                rack2_data = json.load(f)
            rack2_points_3d = rack2_data.get('points_3d', [])
            
            with open(rack3_file, 'r') as f:
                rack3_data = json.load(f)
            rack3_points_3d = rack3_data.get('points_3d', [])
            
            with open(rack4_file, 'r') as f:
                rack4_data = json.load(f)
            rack4_points_3d = rack4_data.get('points_3d', [])
            
            # Validate we have enough points
            if not rack1_points_3d:
                print("✗ No keypoints found in rack1")
                return None
            if not rack2_points_3d:
                print("✗ No keypoints found in rack2")
                return None
            if not rack3_points_3d:
                print("✗ No keypoints found in rack3")
                return None
            if not rack4_points_3d:
                print("✗ No keypoints found in rack4")
                return None
            
            print(f"✓ Loaded {len(rack1_points_3d)} rack1 points")
            print(f"✓ Loaded {len(rack2_points_3d)} rack2 points")
            print(f"✓ Loaded {len(rack3_points_3d)} rack3 points")
            print(f"✓ Loaded {len(rack4_points_3d)} rack4 points")
            
            # ===================== Build wobj coordinate system =====================
            print("\n🔧 Building coordinate system...")
            
            # Origin: rack1 keypoint[0]
            origin = np.array(rack1_points_3d[0])
            print(f"✓ Origin at rack1_point (KP0): ({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f})")
            
            # X-axis: rack1_keypoint[0] → rack2_keypoint[0]
            rack1_kp = np.array(rack1_points_3d[0])
            rack2_kp = np.array(rack2_points_3d[0])
            x_vec_raw = rack2_kp - rack1_kp
            x_axis_length = np.linalg.norm(x_vec_raw)
            x_vec = x_vec_raw / x_axis_length
            
            print(f"✓ X-axis: rack1_point → rack2_point (rack1_KP0 → rack2_KP0)")
            print(f"  rack1_point: ({rack1_kp[0]:.6f}, {rack1_kp[1]:.6f}, {rack1_kp[2]:.6f})")
            print(f"  rack2_point: ({rack2_kp[0]:.6f}, {rack2_kp[1]:.6f}, {rack2_kp[2]:.6f})")
            print(f"  X = [{x_vec[0]:.6f}, {x_vec[1]:.6f}, {x_vec[2]:.6f}], length = {x_axis_length:.6f} m")
            
            # Z-axis: rack1_keypoint[0] → rack3_keypoint[0]
            rack3_kp = np.array(rack3_points_3d[0])
            z_vec_raw = rack3_kp - rack1_kp
            z_axis_length = np.linalg.norm(z_vec_raw)
            z_vec = z_vec_raw / z_axis_length
            
            print(f"✓ Z-axis: rack1_point → rack3_point (rack1_KP0 → rack3_KP0)")
            print(f"  rack3_point: ({rack3_kp[0]:.6f}, {rack3_kp[1]:.6f}, {rack3_kp[2]:.6f})")
            print(f"  Z = [{z_vec[0]:.6f}, {z_vec[1]:.6f}, {z_vec[2]:.6f}], length = {z_axis_length:.6f} m")
            
            # Check if X and Z are nearly parallel
            x_dot_z = np.dot(x_vec, z_vec)
            if abs(x_dot_z) > 0.95:
                print(f"\n✗ X-axis and Z-axis are nearly parallel (dot product = {x_dot_z:.6f})")
                print("  Cannot build a valid coordinate system. Please check keypoint positions.")
                return None
            
            # Y-axis: right-hand rule (Y = Z × X)
            y_vec = np.cross(z_vec, x_vec)
            y_vec = y_vec / np.linalg.norm(y_vec)
            
            print(f"✓ Y-axis: right-hand rule (Y = Z × X)")
            print(f"  Y = [{y_vec[0]:.6f}, {y_vec[1]:.6f}, {y_vec[2]:.6f}]")
            
            # Recalculate Z-axis to ensure perfect orthogonality (Z = X × Y)
            z_vec_orthogonal = np.cross(x_vec, y_vec)
            z_vec_orthogonal = z_vec_orthogonal / np.linalg.norm(z_vec_orthogonal)
            
            # Check how much Z-axis changed
            z_deviation = np.linalg.norm(z_vec - z_vec_orthogonal)
            if z_deviation > 0.01:
                print(f"⚠ Z-axis adjusted for orthogonality (deviation: {z_deviation:.6f})")
                z_vec = z_vec_orthogonal
                print(f"  Z = [{z_vec[0]:.6f}, {z_vec[1]:.6f}, {z_vec[2]:.6f}] (orthogonal)")
            
            # Verify orthogonality
            dot_xy = np.dot(x_vec, y_vec)
            dot_xz = np.dot(x_vec, z_vec)
            dot_yz = np.dot(y_vec, z_vec)
            
            print(f"\n🔍 Orthogonality check:")
            print(f"  X·Y = {dot_xy:.8f} (should be ~0)")
            print(f"  X·Z = {dot_xz:.8f} (should be ~0)")
            print(f"  Y·Z = {dot_yz:.8f} (should be ~0)")
            
            # Build rotation matrix (columns are the axis vectors)
            rot_matrix = np.column_stack([x_vec, y_vec, z_vec])
            
            # Verify it's a valid rotation matrix
            det_R = np.linalg.det(rot_matrix)
            print(f"  det(R) = {det_R:.8f} (should be ~1)")
            
            if abs(det_R - 1.0) > 0.01:
                print(f"⚠ Warning: Rotation matrix determinant is not 1, may indicate numerical issues")
            
            # Build 4x4 transformation matrix
            T = np.eye(4)
            T[:3, :3] = rot_matrix  # Rotation part
            T[:3, 3] = origin  # Translation part
            
            # Convert rotation matrix to axis-angle representation for robot use
            rotation_scipy = R.from_matrix(rot_matrix)
            rotvec = rotation_scipy.as_rotvec()
            rx, ry, rz = rotvec[0], rotvec[1], rotvec[2]
            
            # Create coordinate system information
            coord_system = {
                "origin": {
                    "x": float(origin[0]),
                    "y": float(origin[1]), 
                    "z": float(origin[2]),
                    "source": "rack1_point (KP0)"
                },
                "axes": {
                    "x_axis": {
                        "vector": [float(x_vec[0]), float(x_vec[1]), float(x_vec[2])],
                        "source": "rack1_point→rack2_point (rack1_KP0→rack2_KP0)",
                        "length": float(x_axis_length)
                    },
                    "y_axis": {
                        "vector": [float(y_vec[0]), float(y_vec[1]), float(y_vec[2])],
                        "source": "right_hand_rule_Z_cross_X"
                    },
                    "z_axis": {
                        "vector": [float(z_vec[0]), float(z_vec[1]), float(z_vec[2])],
                        "source": "rack1_point→rack3_point (rack1_KP0→rack3_KP0)",
                        "length": float(z_axis_length)
                    }
                },
                "transformation_matrix": T.tolist(),
                "pose_representation": {
                    "x": float(origin[0]),
                    "y": float(origin[1]),
                    "z": float(origin[2]),
                    "rx": float(rx),
                    "ry": float(ry),
                    "rz": float(rz)
                },
                "orthogonality_check": {
                    "x_dot_y": float(dot_xy),
                    "x_dot_z": float(dot_xz),
                    "y_dot_z": float(dot_yz),
                    "determinant": float(det_R)
                },
                "keypoints_used": {
                    "rack1": [
                        {"index": 0, "name": "point", "coordinates": [float(rack1_kp[0]), float(rack1_kp[1]), float(rack1_kp[2])]}
                    ],
                    "rack2": [
                        {"index": 0, "name": "point", "coordinates": [float(rack2_kp[0]), float(rack2_kp[1]), float(rack2_kp[2])]}
                    ],
                    "rack3": [
                        {"index": 0, "name": "point", "coordinates": [float(rack3_kp[0]), float(rack3_kp[1]), float(rack3_kp[2])]}
                    ],
                    "rack4": [
                        {"index": 0, "name": "point", "coordinates": [float(rack4_points_3d[0][0]), float(rack4_points_3d[0][1]), float(rack4_points_3d[0][2])]}
                    ]
                },
                "timestamp": datetime.now().isoformat(),
                "method": "rack1_rack2_rack3_based_coordinate_system",
                "source_files": {
                    "rack1": rack1_file,
                    "rack2": rack2_file,
                    "rack3": rack3_file,
                    "rack4": rack4_file
                }
            }
            
            # Save coordinate system to temp/wobj_coordinate_system/wobj_result.json
            temp_dir = os.path.join(self.script_dir, "..", "temp")
            wobj_dir = os.path.join(temp_dir, "wobj_coordinate_system")
            os.makedirs(wobj_dir, exist_ok=True)
            
            coord_system_path = os.path.join(wobj_dir, 'wobj_result.json')
            
            with open(coord_system_path, 'w') as f:
                json.dump(coord_system, f, indent=2)
            
            print(f"\n💾 Wobj coordinate system saved to: {coord_system_path}")
            
            # Upload wobj coordinate system to robot_status
            if self.robot_status_client:
                try:
                    print("\n📤 Uploading wobj coordinate system to robot_status...")
                    
                    # Save origin
                    if self.robot_status_client.set_status('wobj', 'wobj_origin', [float(origin[0]), float(origin[1]), float(origin[2])]):
                        print(f"  ✓ wobj_origin saved to robot_status")
                    
                    # Save x_axis
                    if self.robot_status_client.set_status('wobj', 'wobj_x', [float(x_vec[0]), float(x_vec[1]), float(x_vec[2])]):
                        print(f"  ✓ wobj_x saved to robot_status")
                    
                    # Save y_axis
                    if self.robot_status_client.set_status('wobj', 'wobj_y', [float(y_vec[0]), float(y_vec[1]), float(y_vec[2])]):
                        print(f"  ✓ wobj_y saved to robot_status")
                    
                    # Save z_axis
                    if self.robot_status_client.set_status('wobj', 'wobj_z', [float(z_vec[0]), float(z_vec[1]), float(z_vec[2])]):
                        print(f"  ✓ wobj_z saved to robot_status")
                    
                    print(f"  ✓ Wobj coordinate system uploaded to robot_status (namespace: wobj)")
                except Exception as e:
                    print(f"  ⚠ Error uploading wobj coordinate system to robot_status: {e}")
            else:
                print("\n⚠ RobotStatusClient not available, skipping upload to robot_status")
            print("\n✓ Wobj coordinate system established successfully!")
            
            return coord_system
            
        except Exception as e:
            print(f"\n✗ Error building wobj coordinate system: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def validate_wobj_frame_building(self, coord_system, rack1_result_path=None, 
                                      rack2_result_path=None, rack3_result_path=None,
                                      rack4_result_path=None, verbose=True):
        """
        Validate workpiece coordinate system by drawing it on rack1, rack2, rack3, and rack4 images.
        
        Draws the coordinate system axes on captured images:
        - For rack1 images: Origin at rack1_point (wobj origin)
        - For rack2 images: Origin at rack2_point, but axes follow wobj orientation
        - For rack3 images: Origin at rack3_point, but axes follow wobj orientation
        - For rack4 images: Origin at rack4_point, but axes follow wobj orientation
        
        Args:
            coord_system (dict): Dictionary containing coordinate system information
            rack1_result_path (str): Path to rack1 3d_positioning_result.json file
            rack2_result_path (str): Path to rack2 3d_positioning_result.json file
            rack3_result_path (str): Path to rack3 3d_positioning_result.json file
            rack4_result_path (str): Path to rack4 3d_positioning_result.json file
            verbose (bool): If True, saves validation images to disk
            
        Returns:
            bool: True if validation successful, False otherwise
        """
        print("\n" + "="*70)
        print(" Validating Wobj Coordinate System")
        print("="*70)
        
        try:
            if coord_system is None:
                print("✗ Error: coord_system is None")
                return False
            
            # Extract wobj coordinate system information
            wobj_origin_3d = np.array([
                coord_system['origin']['x'],
                coord_system['origin']['y'],
                coord_system['origin']['z']
            ])
            
            x_axis = np.array(coord_system['axes']['x_axis']['vector'])
            y_axis = np.array(coord_system['axes']['y_axis']['vector'])
            z_axis = np.array(coord_system['axes']['z_axis']['vector'])
            
            # Define arrow length in 3D space (meters)
            arrow_length = 0.10  # 10cm arrows
            
            # Find session directories for each rack
            dataset_dir = os.path.join(self.script_dir, "..", "dataset")
            
            def find_latest_session_dir(operation_name, result_file_path):
                """Find the session directory based on result file path"""
                if result_file_path:
                    # Extract session from result file path
                    # result_file_path: dataset/{operation}/result/{session}/3d_positioning_result.json
                    result_dir = os.path.dirname(result_file_path)
                    session_name = os.path.basename(result_dir)
                    operation_dir = os.path.dirname(os.path.dirname(result_dir))
                    session_dir = os.path.join(operation_dir, 'test', session_name)
                    return session_dir, result_dir
                
                # Find most recent session
                result_base_dir = os.path.join(dataset_dir, operation_name, "result")
                if not os.path.exists(result_base_dir):
                    return None, None
                
                sessions = [d for d in os.listdir(result_base_dir) 
                           if os.path.isdir(os.path.join(result_base_dir, d))]
                if not sessions:
                    return None, None
                
                sessions.sort(reverse=True)
                latest_session = sessions[0]
                
                operation_dir = os.path.join(dataset_dir, operation_name)
                session_dir = os.path.join(operation_dir, 'test', latest_session)
                result_dir = os.path.join(result_base_dir, latest_session)
                
                return session_dir, result_dir
            
            # Process each rack
            validation_results = {}
            
            for rack_name, result_path in [('rack1', rack1_result_path), 
                                           ('rack2', rack2_result_path),
                                           ('rack3', rack3_result_path),
                                           ('rack4', rack4_result_path)]:
                
                print(f"\n>>> Processing {rack_name} images...")
                
                session_dir, result_dir = find_latest_session_dir(rack_name, result_path)
                
                if not session_dir or not os.path.exists(session_dir):
                    print(f"✗ {rack_name} session directory not found")
                    validation_results[rack_name] = False
                    continue
                
                print(f"  Session directory: {session_dir}")
                
                # Get the appropriate origin for this rack
                keypoints_for_rack = coord_system['keypoints_used'].get(rack_name, [])
                if keypoints_for_rack:
                    origin_coords = keypoints_for_rack[0]['coordinates']
                    origin_3d = np.array(origin_coords)
                    print(f"  Using {rack_name}_point as origin: ({origin_3d[0]:.6f}, {origin_3d[1]:.6f}, {origin_3d[2]:.6f})")
                else:
                    # Fallback to wobj origin
                    origin_3d = wobj_origin_3d.copy()
                    print(f"  Using wobj origin: ({origin_3d[0]:.6f}, {origin_3d[1]:.6f}, {origin_3d[2]:.6f})")
                
                # Calculate 3D endpoints of axes from the origin
                x_end_3d = origin_3d + x_axis * arrow_length
                y_end_3d = origin_3d + y_axis * arrow_length
                z_end_3d = origin_3d + z_axis * arrow_length
                
                # Find all test images
                test_img_dir = Path(session_dir)
                image_files = sorted(test_img_dir.glob("*.jpg"))
                
                if len(image_files) == 0:
                    print(f"✗ No images found in {session_dir}")
                    validation_results[rack_name] = False
                    continue
                
                print(f"  📖 Found {len(image_files)} images")
                
                # Create visualization for each image
                num_images = len(image_files)
                cols = min(3, num_images)
                rows = (num_images + cols - 1) // cols
                
                fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 5*rows))
                if num_images == 1:
                    axes = np.array([axes])
                axes = axes.flatten() if num_images > 1 else axes
                
                for idx, img_file in enumerate(image_files):
                    if idx >= len(axes):
                        break
                    
                    ax = axes[idx]
                    
                    # Load image
                    img = cv2.imread(str(img_file))
                    if img is None:
                        print(f"  ⚠ Failed to load image: {img_file}")
                        ax.axis('off')
                        continue
                    
                    # Convert BGR to RGB for matplotlib
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    
                    # Load camera parameters for this view
                    pose_file = img_file.parent / f"{img_file.stem}_pose.json"
                    if not pose_file.exists():
                        print(f"  ⚠ Pose file not found: {pose_file}")
                        ax.axis('off')
                        continue
                    
                    try:
                        intrinsic, distortion, extrinsic = load_camera_params_from_json(str(pose_file))
                    except Exception as e:
                        print(f"  ⚠ Failed to load pose: {e}")
                        ax.axis('off')
                        continue
                    
                    # Convert to rvec and tvec for cv2.projectPoints
                    base2cam = extrinsic
                    rotation_matrix = base2cam[:3, :3]
                    translation_vector = base2cam[:3, 3]
                    rvec, _ = cv2.Rodrigues(rotation_matrix)
                    tvec = translation_vector.reshape(3, 1)
                    
                    # Prepare 3D points for projection
                    points_3d_to_project = np.array([
                        origin_3d,
                        x_end_3d,
                        y_end_3d,
                        z_end_3d
                    ], dtype=np.float32)
                    
                    # Project all points
                    dist_coeffs = distortion if distortion is not None else np.zeros(5, dtype=np.float32)
                    projected_points, _ = cv2.projectPoints(
                        points_3d_to_project,
                        rvec,
                        tvec,
                        intrinsic,
                        dist_coeffs
                    )
                    projected_points = projected_points.reshape(-1, 2)
                    
                    # Extract projected 2D points
                    origin_2d = tuple(projected_points[0])
                    x_end_2d = tuple(projected_points[1])
                    y_end_2d = tuple(projected_points[2])
                    z_end_2d = tuple(projected_points[3])
                    
                    # Draw on image
                    img_draw = img_rgb.copy()
                    arrow_thickness = 3
                    circle_radius = 5
                    
                    if origin_2d is not None:
                        # Draw origin as black circle
                        cv2.circle(img_draw, (int(origin_2d[0]), int(origin_2d[1])), 
                                  circle_radius, (0, 0, 0), -1)
                        
                        # Draw X-axis (red arrow)
                        if x_end_2d is not None:
                            cv2.arrowedLine(img_draw, 
                                           (int(origin_2d[0]), int(origin_2d[1])),
                                           (int(x_end_2d[0]), int(x_end_2d[1])),
                                           (255, 0, 0), arrow_thickness, tipLength=0.3)
                            cv2.putText(img_draw, 'X', 
                                       (int(x_end_2d[0]) + 10, int(x_end_2d[1])),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 0, 0), 2)
                        
                        # Draw Y-axis (green arrow)
                        if y_end_2d is not None:
                            cv2.arrowedLine(img_draw, 
                                           (int(origin_2d[0]), int(origin_2d[1])),
                                           (int(y_end_2d[0]), int(y_end_2d[1])),
                                           (0, 255, 0), arrow_thickness, tipLength=0.3)
                            cv2.putText(img_draw, 'Y', 
                                       (int(y_end_2d[0]) + 10, int(y_end_2d[1])),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 255, 0), 2)
                        
                        # Draw Z-axis (blue arrow)
                        if z_end_2d is not None:
                            cv2.arrowedLine(img_draw, 
                                           (int(origin_2d[0]), int(origin_2d[1])),
                                           (int(z_end_2d[0]), int(z_end_2d[1])),
                                           (0, 0, 255), arrow_thickness, tipLength=0.3)
                            cv2.putText(img_draw, 'Z', 
                                       (int(z_end_2d[0]) + 10, int(z_end_2d[1])),
                                       cv2.FONT_HERSHEY_SIMPLEX, 0.8, (0, 0, 255), 2)
                    
                    # Display image with coordinate system
                    ax.imshow(img_draw)
                    ax.set_title(f'View {idx}\n{img_file.name}', fontsize=10)
                    ax.axis('off')
                    
                    print(f"    ✓ Drew coordinate system on {img_file.name}")
                
                # Hide unused subplots
                for idx in range(num_images, len(axes)):
                    axes[idx].axis('off')
                
                # Add overall title
                origin_label = f"{rack_name}_point"
                fig.suptitle(f'Wobj Coordinate System Validation - {rack_name.upper()}\n'
                            f'Origin: {origin_label} | Red: X-axis | Green: Y-axis | Blue: Z-axis',
                            fontsize=14, fontweight='bold', y=0.98)
                
                # Add coordinate system information as text
                x_axis_length = coord_system['axes']['x_axis']['length']
                z_axis_length = coord_system['axes']['z_axis']['length']
                info_text = (
                    f"Wobj Coordinate System Properties:\n"
                    f"Origin (display): ({origin_3d[0]:.4f}, {origin_3d[1]:.4f}, {origin_3d[2]:.4f}) m\n"
                    f"X-axis: rack1_point→rack2_point, length: {x_axis_length:.4f}m\n"
                    f"Z-axis: rack1_point→rack3_point, length: {z_axis_length:.4f}m\n"
                    f"Arrow length: {arrow_length:.3f}m"
                )
                
                fig.text(0.02, 0.02, info_text, fontsize=9,
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                        verticalalignment='bottom')
                
                # Save figure
                if verbose:
                    # Save to temp/wobj_coordinate_system/
                    temp_dir = os.path.join(self.script_dir, "..", "temp")
                    wobj_dir = os.path.join(temp_dir, "wobj_coordinate_system")
                    os.makedirs(wobj_dir, exist_ok=True)
                    
                    output_path = os.path.join(wobj_dir, f'wobj_validation_{rack_name}.jpg')
                    plt.tight_layout()
                    plt.savefig(output_path, dpi=150, bbox_inches='tight')
                    print(f"  💾 Saved validation image to: {output_path}")
                    
                    # Also save to session result directory
                    if result_dir and os.path.exists(result_dir):
                        session_output_path = os.path.join(result_dir, f'wobj_validation_{rack_name}.jpg')
                        plt.savefig(session_output_path, dpi=150, bbox_inches='tight')
                        print(f"  💾 Copy saved to session: {session_output_path}")
                
                # Close plot to free memory
                plt.close(fig)
                
                validation_results[rack_name] = True
            
            # Summary
            print("\n" + "="*70)
            print(" Validation Summary")
            print("="*70)
            for rack_name, success in validation_results.items():
                status = "✓ SUCCESS" if success else "✗ FAILED"
                print(f"  {rack_name}: {status}")
            print("="*70)
            
            all_success = all(validation_results.values())
            if all_success:
                print("\n✓ Wobj coordinate system validation completed successfully!")
            else:
                print("\n⚠ Wobj coordinate system validation completed with some failures")
            
            return all_success
            
        except Exception as e:
            print(f"\n✗ Validation error: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _print_summary(self, results):
        """
        Print execution summary for all rack operations
        
        Args:
            results (dict): Results dictionary containing execution status for each operation
        """
        print("\n" + "="*70)
        print(" Execution Summary")
        print("="*70)
        
        total_operations = len(self.rack_operations)
        completed_operations = sum(1 for r in results.values() if r.get("success", False))
        failed_operations = total_operations - completed_operations
        
        print(f"Total Operations: {total_operations}")
        print(f"Completed Successfully: {completed_operations}")
        print(f"Failed: {failed_operations}")
        print()
        
        for operation_name in self.rack_operations:
            if operation_name in results:
                result = results[operation_name]
                status = "✓ SUCCESS" if result.get("success", False) else "✗ FAILED"
                elapsed = result.get("elapsed_time", 0)
                
                print(f"  {operation_name}: {status} ({elapsed:.2f}s)")
                
                if "error" in result:
                    print(f"    Error: {result['error']}")
            else:
                print(f"  {operation_name}: ⊘ NOT EXECUTED")
        
        print("="*70)


def main():
    """
    Main function to run automated rack location capture
    """
    import argparse
    
    parser = argparse.ArgumentParser(description='URLocateRack - Automated rack location capture')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot')
    parser.add_argument('--camera-topic', type=str, default='/ur15_camera/image_raw',
                       help='ROS topic name for camera images')
    parser.add_argument('--camera-params-path', type=str, 
                       default='../temp/ur15_cam_calibration_result/ur15_camera_parameters',
                       help='Path to camera calibration parameters directory')
    parser.add_argument('--no-verbose', dest='verbose', action='store_false', default=True,
                       help='Do not save validation results to disk (default: save results)')
    
    args = parser.parse_args()
    
    try:
        # Initialize ROS2
        rclpy.init()
        
        # Create a simple ROS2 node for RobotStatusClient
        class URLocateRackNode(Node):
            def __init__(self):
                super().__init__('ur_locate_rack_node')
        
        ros_node = URLocateRackNode()
        
        # Create URLocateRack instance
        ur_locate_rack = URLocateRack(
            robot_ip=args.robot_ip,
            robot_port=args.robot_port,
            camera_topic=args.camera_topic,
            camera_params_path=args.camera_params_path,
            verbose=args.verbose,
            ros_node=ros_node
        )
        
        # Execute auto capture
        results = ur_locate_rack.auto_capture()
        
        # Check if all operations completed successfully
        if not all(r.get("success", False) for r in results.values()):
            print("\n✗ Some operations failed. Skipping wobj coordinate system building.")
            sys.exit(1)
        
        # Build wobj coordinate system
        print("\n" + "="*70)
        print(" Building Wobj Coordinate System")
        print("="*70)
        
        coord_system = ur_locate_rack.perform_wobj_frame_building()
        
        if coord_system:
            print("\n✓ Wobj coordinate system built successfully!")
            
            # Validate wobj coordinate system
            print("\n" + "="*70)
            print(" Validating Wobj Coordinate System")
            print("="*70)
            
            validation_success = ur_locate_rack.validate_wobj_frame_building(
                coord_system=coord_system,
                verbose=args.verbose
            )
            
            if validation_success:
                print("\n✓ Wobj coordinate system validation completed successfully!")
            else:
                print("\n⚠ Wobj coordinate system validation completed with warnings")
        else:
            print("\n✗ Failed to build wobj coordinate system")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Program interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\n❌ Fatal error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        # Shutdown ROS2
        try:
            rclpy.shutdown()
        except:
            pass


if __name__ == "__main__":
    main()
