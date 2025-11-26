#!/usr/bin/env python3
"""
URLocateRack - Automated rack location capture system
"""

import os
import sys
import time
import json
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
from core.positioning_3d_webapi import Positioning3DWebAPIClient, load_camera_params_from_json
from robot_status.client_utils import RobotStatusClient

# Import URCapture base class
from ur_capture import URCapture


class URLocateRack(URCapture):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, 
                 camera_topic="/ur15_camera/image_raw",
                 camera_params_path="../temp/ur15_cam_calibration_result/ur15_camera_parameters",
                 verbose=True,
                 operation_name="rack_bottom_left"):
        """
        Initialize URLocateRack class for automated rack location operations
        
        Args:
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
            camera_topic (str): ROS topic name for camera images
            camera_params_path (str): Path to camera calibration parameters directory
            verbose (bool): If True, saves validation images and error logs to disk
            operation_name (str): Name of the operation for data directory
        """
        # Call parent class constructor
        super().__init__(
            robot_ip=robot_ip,
            robot_port=robot_port,
            camera_topic=camera_topic,
            operation_name=operation_name,
            camera_params_path=camera_params_path
        )
        
        self.verbose = verbose
        
        # ================================= Configuration Parameters =================================
        self.operation_names = []   # Initialize operation names list (will be populated from JSON file)
        self.recorded_positions = {}
        # Store temp directory path for reuse
        self.temp_dir = os.path.join(self.script_dir, "..", "temp")
        # Store dataset directory path for reuse
        self.dataset_dir = os.path.join(self.script_dir, "..", "dataset")
        
        # Define all rack corner points in wobj coordinate system (for positioning based on fitting method)
        self.template_points = [
            {
                "name": "GB200_Rack_Bottom_Left_Corner",
                "x": 0,
                "y": 0,
                "z": 0.0
            },
            {
                "name": "GB200_Rack_Bottom_Right_Corner",
                "x": 0.55,
                "y": 0,
                "z": 0.0
            },
            {
                "name": "GB200_Rack_Top_Left_Corner",
                "x": 0,
                "y": 0,
                "z": 2.145
            },
            {
                "name": "GB200_Rack_Top_Right_Corner",
                "x": 0.55,
                "y": 0,
                "z": 2.145
            }
        ]
        # ============================ Instance Paramters ==================================
        self.positioning_client = None
        self.robot_status_client = None
        
        # ============================ Initialization ================================
        self._init_positioning_client()
        self._init_robot_status_client()
        
    def _init_positioning_client(self, service_url="http://localhost:8004", max_retries=3, retry_delay=2):
        """
        Initialize the 3D Positioning Web API Client
        """
        # Check if the Web API client is available
        if Positioning3DWebAPIClient is None:
            print("✗ Positioning Web API client not available (import failed)")
            self.positioning_client = None
            return
        
        for attempt in range(max_retries):
            try:
                self.positioning_client = Positioning3DWebAPIClient(service_url=service_url)
                
                # Check service health
                health = self.positioning_client.check_health()
                if health.get('success'):
                    print(f"✓ Positioning service connected to: {service_url}")
                    ffpp_connected = health.get('status', {}).get('ffpp_server', {}).get('connected', False)
                    refs_loaded = health.get('status', {}).get('references', {}).get('loaded', 0)
                    print(f"  FFPP server is in {ffpp_connected} status")
                    return  # Successfully connected
                else:
                    raise Exception(health.get('error', 'Service health check failed'))
                    
            except Exception as e:
                if attempt < max_retries - 1:
                    print(f"✗ Positioning service not ready (attempt {attempt + 1}/{max_retries}): {e}")
                    print(f"  Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                else:
                    print(f"✗ Failed to connect to positioning service after {max_retries} attempts: {e}")
                    self.positioning_client = None
    
    def _init_robot_status_client(self, timeout_sec=5.0):
        """
        Initialize the RobotStatusClient
        
        Args:
            timeout_sec (float): Timeout in seconds for the client
        """
        try:
            self.robot_status_client = RobotStatusClient(self, timeout_sec=timeout_sec)
            print("✓ RobotStatusClient initialized successfully")
        except Exception as e:
            print(f"✗ Failed to initialize RobotStatusClient: {e}")
            self.robot_status_client = None
    
    def _load_recorded_positions(self):
        """
        Load recorded positions from robot_dc/temp/recorded_GB200_locate_positions.json, and convert from degrees to radians.
        """
        try:
            # Construct path to recorded positions file
            recorded_positions_file_path = os.path.join(self.temp_dir, "recorded_GB200_locate_positions.json")
            
            if not os.path.exists(recorded_positions_file_path):
                print(f"✗ Recorded positions file not found in: {recorded_positions_file_path}")
                return False
            
            # Load JSON file
            with open(recorded_positions_file_path, 'r') as f:
                recorded_positions_data = json.load(f)
            
            # Convert degrees to radians and store
            for key, position_deg in recorded_positions_data.items():
                if isinstance(position_deg, list) and len(position_deg) == 6:
                    # Convert from degrees to radians
                    position_rad = [np.deg2rad(angle) for angle in position_deg]
                    self.recorded_positions[key] = position_rad
                    self.operation_names.append(key)
                else:
                    print(f"✗ Invalid position format for '{key}': {position_deg}")
            
            print(f"\n✓ Successfully loaded {len(self.recorded_positions)} recorded positions: {self.operation_names}")
            return True
            
        except json.JSONDecodeError as e:
            print(f"✗ Failed to parse JSON file: {e}")
            return False
        except Exception as e:
            print(f"✗ Error loading recorded positions: {e}")
            import traceback
            traceback.print_exc()
            return False
        
    def auto_capture_and_positioning(self):
        """
        Automatically execute location operations for all configured rack positions in sequence.
        For each operation:
        1. Load the corresponding position from recorded_GB200_locate_positions.json
        2. Move robot to that position
        3. Capture images using self.movements offsets
        4. Execute 3D positioning on the captured images
        
        Returns:
            dict: Summary of execution results for each operation
        """
        results = {}
        
        print("\n" + "="*70)
        print(" Starting Capture and Positioning!")
        print("="*70)

        # Load recorded positions
        print(">>> 1. Loading Recorded Positions")
        if not self._load_recorded_positions():
            print("✗ Failed to load recorded positions. Cannot proceed.")
            return results
        
        # Upload references to positioning service (if available)
        if self.positioning_client is not None:
            print(">>> 2. Uploading Reference Data onto server")
            
            # Check service health first
            print("\n=== Checking positioning service health ===")
            health = self.positioning_client.check_health()
            if not health.get('success'):
                print(f"✗ Positioning service not available: {health.get('error')}")
                print("✗ Will skip 3D positioning for all operations")
            else:
                print(f"✓ Positioning service is running")
                
                # List current references
                print("\n=== Listing current references ===")
                refs = self.positioning_client.list_references()
                if refs.get('success'):
                    ref_count = refs.get('count', 0)
                    print(f"✓ Currently loaded: {ref_count} references")
                    if ref_count > 0:
                        for name in refs.get('references', {}).keys():
                            print(f"  - {name}")
                
                # Upload references
                print("\n=== Uploading references to FFPP server ===")
                upload_result = self.positioning_client.upload_references()
                
                if upload_result.get('success'):
                    refs_loaded = upload_result.get('references_loaded', 0)
                    refs_found = upload_result.get('references_found', 0)
                    print(f"✓ Reference upload {refs_loaded}/{refs_found}!")
                else:
                    print(f"✗ Failed to upload references: {upload_result.get('error')}")
                    print("✗ Will skip 3D positioning for all operations")
        else:
            print("\n✗ Positioning client not available, will skip 3D positioning")
        
        # Initialize a shared session for all rack operations
        shared_session_id = None
        all_session_dirs = {}  # Store session directories for each rack
        
        if self.positioning_client is not None:
            print(">>> 3. Initializing a 3D Positioning Session")
            
            session_result = self.positioning_client.init_session()
            if session_result.get('success'):
                shared_session_id = session_result['session_id']
                print(f"✓ Session {shared_session_id} is created successfully!")
            else:
                print(f"✗ Failed to create shared session: {session_result.get('error')}")
                print("✗ Will skip 3D positioning for all operations")
        
        print(">>> 4. Executing Capture and Upload Sequentially")
        # Execute each rack operation sequentially (capture and upload only)
        for idx, operation_name in enumerate(self.operation_names, 1):
            print(f"=== [{idx}/{len(self.operation_names)}] Capturing: {operation_name} ===")
            try:
                # Update operation_name and data paths for this operation
                self.operation_name = operation_name
                self._setup_paths(operation_name, self.camera_params_path)
                print(f"Current Data directory is: {self.data_parent_dir}")
                
                # Check if position exists for this operation
                if operation_name not in self.recorded_positions:
                    print(f"✗ No recorded position found for {operation_name}")
                    results[operation_name] = {
                        "success": False,
                        "capture_success": False,
                        "error": f"No recorded position for {operation_name}"
                    }
                    continue
                
                # Get the position for this rack
                target_position = self.recorded_positions[operation_name]
                
                # Move robot to the target position
                print(f"🤖 Moving robot to {operation_name} position...")
                if self.robot is None:
                    print("✗ Robot not initialized")
                    results[operation_name] = {
                        "success": False,
                        "capture_success": False,
                        "error": "Robot not initialized"
                    }
                    continue
                
                move_result = self.robot.movej(target_position, a=0.5, v=0.5, t=0, r=0)
                if move_result != 0:
                    print(f"✗ Failed to move robot to {operation_name} position (error code: {move_result})")
                    results[operation_name] = {
                        "success": False,
                        "capture_success": False,
                        "error": f"Robot movement failed with code {move_result}"
                    }
                    continue
                
                print(f"✓ Robot moved to {operation_name} position successfully")
                time.sleep(1.0)  # Wait for robot to stabilize
                
                # Create session directory in {operation_name}/test
                session_dir = self._create_session_directory()
                all_session_dirs[operation_name] = session_dir
                
                # Check if reference exists
                if shared_session_id is not None:
                    print(f"\n🔍 Checking reference for {operation_name}...")
                    refs = self.positioning_client.list_references()
                    reference_exists = refs.get('success') and operation_name in refs.get('references', {})
                    
                    if not reference_exists:
                        print(f"  ✗ Reference '{operation_name}' not found on server")
                        print(f"  ✗ Please ensure reference data exists in dataset/{operation_name}/ref_img_1.jpg and ref_img_1.json")
                        results[operation_name] = {
                            "success": False,
                            "capture_success": False,
                            "error": f"Reference '{operation_name}' not found"
                        }
                        continue
                    
                    print(f"  ✓ Reference '{operation_name}' is available")
                
                # Capture images and upload views to shared session
                print(f"\n📸 Capturing images and uploading views for {operation_name}...")
                capture_success = True
                upload_count = 0
                
                for move_idx, (movement_name, movement_offset) in enumerate(self.movements.items()):
                    try:
                        print(f"\n  → Capturing Image {move_idx+1} for {operation_name}...")
                        
                        # Apply movement offset using move_tcp (relative movement in tool coordinate)
                        if any(movement_offset):  # If there's any non-zero offset
                            move_result = self.robot.move_tcp(movement_offset, a=0.1, v=0.1)
                            if move_result != 0:
                                print(f"    ✗ Failed to apply movement offset (error code: {move_result})")
                                capture_success = False
                                continue
                            time.sleep(1.0)  # Wait for stabilization
                        
                        # Use parent class method to capture image and pose
                        if not self.capture_image_and_pose(
                            save_dir=session_dir, 
                            img_filename=f"{move_idx}.jpg",
                            pose_filename=f"{move_idx}.json"
                        ):
                            print(f"    ✗ Failed to capture {movement_name}")
                            capture_success = False
                            continue
                        
                        # Upload this view to shared session
                        if shared_session_id is not None:
                            img_path = os.path.join(session_dir, f"{move_idx}.jpg")
                            pose_path = os.path.join(session_dir, f"{move_idx}.json")
                            
                            try:
                                # Load image
                                image = cv2.imread(img_path)
                                if image is None:
                                    print(f"    ✗ Failed to load {move_idx}.jpg for upload")
                                    capture_success = False
                                    continue
                                
                                # Load camera parameters
                                intrinsic, distortion, extrinsic = load_camera_params_from_json(pose_path)
                                
                                # Upload view to shared session
                                upload_result = self.positioning_client.upload_view(
                                    session_id=shared_session_id,
                                    reference_name=operation_name,
                                    image=image,
                                    intrinsic=intrinsic,
                                    distortion=distortion,
                                    extrinsic=extrinsic
                                )
                                
                                if upload_result.get('success'):
                                    print(f"    ✓ Uploaded view {move_idx+1} to shared session")
                                    upload_count += 1
                                else:
                                    print(f"    ✗ Failed to upload view {move_idx+1}: {upload_result.get('error')}")
                                    capture_success = False
                                    
                            except Exception as e:
                                print(f"    ✗ Error uploading view {move_idx+1}: {e}")
                                capture_success = False
                        
                        # Move back to original position (reverse the offset)
                        if any(movement_offset):
                            reverse_offset = [-x for x in movement_offset]
                            move_result = self.robot.move_tcp(reverse_offset, a=0.1, v=0.05)
                            if move_result != 0:
                                print(f"    ✗ Warning: Failed to return to original position (error code: {move_result})")
                            time.sleep(0.5)
                        
                    except Exception as e:
                        print(f"    ✗ Error capturing {movement_name}: {e}")
                        capture_success = False
                        continue
                
                # Record capture result for this rack
                results[operation_name] = {
                    "capture_success": capture_success,
                    "upload_count": upload_count,
                    "session_dir": session_dir
                }
                
                if capture_success and upload_count > 0:
                    print(f"\n✓ Captured and uploaded {upload_count} views successfully for {operation_name}")
                else:
                    print(f"\n✗ Warning: Only captured {upload_count} views for {operation_name}")
                    results[operation_name]["success"] = False
                
            except KeyboardInterrupt:
                print(f"\n\n🛑 Operation interrupted by user during {operation_name}")
                results[operation_name] = {
                    "success": False,
                    "capture_success": False,
                    "error": "Interrupted by user"
                }
                break
                
            except Exception as e:
                print(f"\n✗ Error capturing {operation_name}: {e}")
                import traceback
                traceback.print_exc()
                results[operation_name] = {
                    "success": False,
                    "capture_success": False,
                    "error": str(e)
                }
                
            # Add a short delay between operations
            if idx < len(self.operation_names):
                print(f"\nWaiting 2 seconds before next operation...\n")
                time.sleep(2)
        
        # ============ Now get positioning result for all racks together ============
        if shared_session_id is not None:
            print(">>> 5. Getting 3D Positioning Result Using Fitting Method")
            
            try:
                # Use all 4 template points - now we have views from all racks
                print(f"\n🔍 Computing 3D positions using all captured views and template points...")
                
                result = self.positioning_client.get_result(
                    shared_session_id,
                    template_points=self.template_points,
                    timeout=10000
                )
                
                if not result.get('success'):
                    print(f"✗ Failed to get result: {result.get('error')}")
                else:
                    if 'result' not in result:
                        print(f"✗ No result returned (timeout or failed)")
                    else:
                        # Extract results
                        positioning_result = result['result']
                        points_3d = np.array(positioning_result['points_3d'])
                        mean_error = positioning_result['mean_error']
                        
                        print(f"✓ Positioning completed!")
                        print(f"  Number of 3D points: {len(points_3d)}")
                        print(f"  Mean reprojection error: {mean_error:.3f} pixels")
                        
                        # Save results for each rack
                        for operation_name in self.operation_names:
                            if operation_name not in results or not results[operation_name].get('capture_success', False):
                                continue
                            
                            session_dir = results[operation_name].get('session_dir')
                            if not session_dir:
                                continue
                            
                            print(f"\n=== 💾 Saving positioning results for {operation_name} ===")
                            session_name = os.path.basename(session_dir)
                            
                            # Update operation paths
                            self.operation_name = operation_name
                            self._setup_paths(operation_name, self.camera_params_path)
                            
                            result_dir = os.path.join(self.data_parent_dir, "result")
                            os.makedirs(result_dir, exist_ok=True)
                            session_result_dir = os.path.join(result_dir, session_name)
                            os.makedirs(session_result_dir, exist_ok=True)
                            
                            result_data = {
                                'timestamp': datetime.now().isoformat(),
                                'points_3d': positioning_result['points_3d'],
                                'mean_error': positioning_result['mean_error'],
                                'processing_time': positioning_result.get('processing_time', 0),
                                'views': result.get('views', []),
                                'fitting_mode': True,
                                'shared_session': True  # Mark this as from shared session
                            }
                            
                            if 'local2world' in positioning_result:
                                result_data['local2world'] = positioning_result['local2world']
                            
                            result_file = os.path.join(session_result_dir, '3d_positioning_result.json')
                            with open(result_file, 'w') as f:
                                json.dump(result_data, f, indent=2)
                            
                            print(f"  ✓ Results saved to: {result_file}")
                            
                            # Validate positioning results
                            validation_success = self.validate_positioning_results(
                                session_dir=session_dir,
                                result_data=result_data,
                                operation_name=operation_name,
                                verbose=self.verbose
                            )
                            
                            if validation_success:
                                print(f"  ✓ Validation completed successfully")
                            else:
                                print(f"  ✗ Validation completed with warnings")
                            
                            # Save to robot_status
                            if self.robot_status_client:
                                try:
                                    if self.robot_status_client.set_status(operation_name, 'points_3d', positioning_result['points_3d']):
                                        print(f"  ✓ points_3d saved to robot_status (namespace: {operation_name})")
                                except Exception as e:
                                    print(f"  ✗ Error saving to robot_status: {e}")
                            
                            # Update results
                            results[operation_name]['success'] = True
                            results[operation_name]['positioning_success'] = True
                            results[operation_name]['positioning_result'] = result_data
                
            except Exception as e:
                print(f"✗ Error getting positioning result: {e}")
                import traceback
                traceback.print_exc()
            finally:
                # Terminate shared session
                print(f"\n>>> 6. Terminating session {shared_session_id}, positioning is ended...")
                self.positioning_client.terminate_session(shared_session_id)
                print(f"✓ Shared session terminated")
        
        return results
    
    def validate_positioning_results(self, session_dir, result_data, operation_name, verbose=True):
        """
        Validate 3D positioning results by visualizing tracked and reprojected keypoints. Only validates keypoints defined in the reference (ref_img_1.json) for this operation.
        - Red X markers: Reprojected 3D points back to each view
        - Green circles: Original tracked 2D keypoints from views

        """
        try:
            print("  📊 Validating 3D Positioning Results...")
            
            # Load reference keypoints for this operation
            ref_json_path = os.path.join(self.data_parent_dir, "ref_img_1.json")
            if not os.path.exists(ref_json_path):
                print(f"  ✗ Reference file not found: {ref_json_path}")
                return False
            
            with open(ref_json_path, 'r') as f:
                ref_data = json.load(f)
            
            ref_keypoints = ref_data.get('keypoints', [])
            if not ref_keypoints:
                print(f"  ✗ No keypoints found in reference file")
                return False
            
            # Create mapping from template point names to indices
            template_name_to_index = {
                "GB200_Rack_Bottom_Left_Corner": 0,
                "GB200_Rack_Bottom_Right_Corner": 1,
                "GB200_Rack_Top_Left_Corner": 2,
                "GB200_Rack_Top_Right_Corner": 3
            }
            
            # Extract data from result
            points_3d = np.array(result_data['points_3d'])
            all_views_data = result_data.get('views', [])
            
            if len(all_views_data) == 0:
                print("  ✗ No view data found in results")
                return False
            
            # Filter views to only include those from this operation (matching reference_name)
            # In shared session mode, views from all racks are included
            views_data = [v for v in all_views_data if v.get('reference_name') == operation_name]
            
            if len(views_data) == 0:
                print(f"  ✗ No views found for {operation_name} in results")
                print(f"  Total views in result: {len(all_views_data)}")
                return False
            
            # Map reference keypoints to their corresponding 3D points by name
            points_3d_to_validate = []
            keypoint_names = []
            template_indices = []

            for idx, kp in enumerate(ref_keypoints):
                kp_name = kp['name']
                keypoint_names.append(kp_name)
                
                # Find the template index for this keypoint
                if kp_name in template_name_to_index:
                    template_idx = template_name_to_index[kp_name]
                    template_indices.append(template_idx)
                    
                    # Check if we have this point in the results
                    if template_idx < len(points_3d):
                        points_3d_to_validate.append(points_3d[template_idx])
                        print(f"    [{idx}] {kp_name} -> template[{template_idx}]")
                    else:
                        print(f"    [{idx}] {kp_name} -> template[{template_idx}] ✗ NOT FOUND in points_3d")
                else:
                    print(f"    [{idx}] {kp_name} ✗ Unknown template point name")
            
            if len(points_3d_to_validate) == 0:
                print("  ✗ No valid 3D points found for validation")
                return False
            
            points_3d_to_validate = np.array(points_3d_to_validate)
            
            print(f"  ✓ Processing {len(views_data)} views and {len(points_3d_to_validate)} 3D points")
            
            # Find all test images
            test_img_dir = Path(session_dir)
            image_files = sorted(test_img_dir.glob("*.jpg"))
            
            if len(image_files) == 0:
                print(f"  ✗ No images found in {session_dir}")
                return False
            
            # Create figure with subplots for each view
            num_views = min(len(views_data), len(image_files))
            cols = min(3, num_views)
            rows = (num_views + cols - 1) // cols
            
            fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 5*rows))
            if num_views == 1:
                axes = np.array([axes])
            axes = axes.flatten() if num_views > 1 else axes
            
            reprojection_errors = []
            
            # Process each view
            for view_idx, (view_data, img_file) in enumerate(zip(views_data, image_files)):
                if view_idx >= len(axes):
                    break
                
                ax = axes[view_idx]
                
                # Load image
                img = cv2.imread(str(img_file))
                if img is None:
                    print(f"  ✗ Failed to load image: {img_file}")
                    ax.axis('off')
                    continue
                
                # Convert BGR to RGB for matplotlib
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                ax.imshow(img_rgb)
                
                # Get tracked 2D keypoints (green circles)
                keypoints_2d = view_data.get('keypoints_2d', [])
                
                # Load camera parameters for this view
                pose_file = img_file.parent / f"{img_file.stem}.json"
                if not pose_file.exists():
                    print(f"  ✗ Pose file not found: {pose_file}")
                    ax.axis('off')
                    continue
                
                try:
                    intrinsic, distortion, extrinsic = load_camera_params_from_json(str(pose_file))
                except Exception as e:
                    print(f"  ✗ Failed to load pose: {e}")
                    ax.axis('off')
                    continue
                
                # extrinsic is already base2cam (world to camera transformation)
                # Convert to rvec and tvec for cv2.projectPoints
                base2cam = extrinsic
                rotation_matrix = base2cam[:3, :3]
                translation_vector = base2cam[:3, 3]
                rvec, _ = cv2.Rodrigues(rotation_matrix)
                tvec = translation_vector.reshape(3, 1)
                
                # Reproject only the 3D points corresponding to reference keypoints
                dist_coeffs = distortion if distortion is not None else np.zeros(5, dtype=np.float32)
                projected_points, _ = cv2.projectPoints(
                    points_3d_to_validate.astype(np.float32),
                    rvec,
                    tvec,
                    intrinsic,
                    dist_coeffs
                )
                projected_points = projected_points.reshape(-1, 2)
                
                # Plot reprojected points and calculate errors (only for reference keypoints)
                for pt_idx, (u_reproj, v_reproj) in enumerate(projected_points):
                    if pt_idx >= len(ref_keypoints):
                        break
                    
                    # Get keypoint name
                    kp_name = ref_keypoints[pt_idx]['name']
                    
                    # Plot reprojected point (red x marker)
                    ax.plot(u_reproj, v_reproj, 'rx', markersize=8,
                           markeredgewidth=2,
                           label='Reprojected' if pt_idx == 0 else '')
                    
                    # Calculate reprojection error
                    if pt_idx < len(keypoints_2d):
                        tracked_kp = keypoints_2d[pt_idx]
                        u_track = tracked_kp.get('x')
                        v_track = tracked_kp.get('y')
                        
                        if u_track is not None and v_track is not None:
                            error = np.sqrt((u_reproj - u_track)**2 + (v_reproj - v_track)**2)
                            reprojection_errors.append({
                                'view': view_idx,
                                'image': img_file.name,
                                'point_index': pt_idx,
                                'point_name': kp_name,
                                'error_pixels': float(error)
                            })
                
                # Plot tracked keypoints (green filled circles, only for reference keypoints)
                for pt_idx, kp in enumerate(keypoints_2d):
                    if pt_idx >= len(ref_keypoints):
                        break
                    
                    u_track = kp.get('x')
                    v_track = kp.get('y')
                    
                    if u_track is not None and v_track is not None:
                        ax.plot(u_track, v_track, 'go', markersize=4,
                               markerfacecolor='green', markeredgewidth=2,
                               label='Tracked' if pt_idx == 0 else '')
                
                # Set title
                ax.set_title(f'View {view_idx}\n{img_file.name}', fontsize=10)
                ax.axis('off')
                
                # Add legend only to first subplot
                if view_idx == 0:
                    ax.legend(loc='upper right', fontsize=8)
            
            # Hide unused subplots
            for idx in range(num_views, len(axes)):
                axes[idx].axis('off')
            
            # Add overall title
            fig.suptitle(f'3D Positioning Validation - {operation_name.upper()}\n'
                        f'Green: Tracked 2D | Red X: Reprojected from 3D | {len(ref_keypoints)} keypoints',
                        fontsize=14, fontweight='bold', y=0.98)
            
            # Calculate and display statistics
            if reprojection_errors:
                errors = [e['error_pixels'] for e in reprojection_errors]
                mean_error = np.mean(errors)
                max_error = np.max(errors)
                min_error = np.min(errors)
                std_error = np.std(errors)
                
                stats_text = (
                    f"Reprojection Error Statistics:\n"
                    f"Operation: {operation_name}\n"
                    f"Keypoints: {len(ref_keypoints)} ({', '.join([kp['name'].split('_')[-1] for kp in ref_keypoints])})\n"
                    f"Mean: {mean_error:.2f} pixels\n"
                    f"Std: {std_error:.2f} pixels\n"
                    f"Min: {min_error:.2f} pixels\n"
                    f"Max: {max_error:.2f} pixels\n"
                    f"Total samples: {len(errors)}"
                )
                
                fig.text(0.02, 0.02, stats_text, fontsize=10,
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                        verticalalignment='bottom')
                
                print(f"  📊 Reprojection Error Statistics:")
                print(f"    Mean error: {mean_error:.2f} pixels")
                print(f"    Std error: {std_error:.2f} pixels")
                print(f"    Min error: {min_error:.2f} pixels")
                print(f"    Max error: {max_error:.2f} pixels")
            
            # Save figure to session result directory (if verbose)
            if verbose:
                session_name = os.path.basename(session_dir)
                result_dir = os.path.join(self.data_parent_dir, "result", session_name)
                
                output_path = os.path.join(result_dir, 'positioning_result_reprojection.jpg')
                plt.tight_layout()
                plt.savefig(output_path, dpi=150, bbox_inches='tight')
                print(f"  💾 Validation visualization saved to: {output_path}")
            
            # Close plot to free memory
            plt.close(fig)
            
            # Save detailed reprojection errors to JSON (if verbose)
            if verbose and reprojection_errors:
                error_log_path = os.path.join(result_dir, 'positioning_result_reprojection_report.json')
                error_data = {
                    'timestamp': datetime.now().isoformat(),
                    'operation_name': operation_name,
                    'reference_keypoints': [{'name': kp['name'], 'id': kp.get('id', idx)} 
                                           for idx, kp in enumerate(ref_keypoints)],
                    'num_views': len(views_data),
                    'num_points_validated': len(points_3d_to_validate),
                    'total_samples': len(reprojection_errors),
                    'statistics': {
                        'mean_error_pixels': float(mean_error),
                        'std_error_pixels': float(std_error),
                        'min_error_pixels': float(min_error),
                        'max_error_pixels': float(max_error)
                    },
                    'reprojection_errors': reprojection_errors
                }
                
                with open(error_log_path, 'w') as f:
                    json.dump(error_data, f, indent=2)
                
                print(f"  💾 Reprojection report saved to: {error_log_path}")
            
            return True
            
        except Exception as e:
            print(f"  ✗ Error during validation: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def _find_latest_result(self, operation_name):
        """
        Find the most recent 3d_positioning_result.json file for a given operation.
        """
        # Look for most recent session in dataset/{operation_name}/result/
        result_dir = os.path.join(self.dataset_dir, operation_name, "result")
        
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
    
    def _find_latest_session_dir(self, operation_name):
        """
        Find the most recent session directory and result directory for a given operation.
        
        Args:
            operation_name (str): Name of the operation to find session for
            
        Returns:
            tuple: (session_dir, result_dir) or (None, None) if not found
        """
        # Find most recent session
        result_base_dir = os.path.join(self.dataset_dir, operation_name, "result")
        if not os.path.exists(result_base_dir):
            return None, None
        
        sessions = [d for d in os.listdir(result_base_dir) 
                   if os.path.isdir(os.path.join(result_base_dir, d))]
        if not sessions:
            return None, None
        
        sessions.sort(reverse=True)
        latest_session = sessions[0]
        
        operation_dir = os.path.join(self.dataset_dir, operation_name)
        session_dir = os.path.join(operation_dir, 'test', latest_session)
        result_dir = os.path.join(result_base_dir, latest_session)
        
        return session_dir, result_dir
    
    def perform_wobj_frame_building(self):
        """
        Build a wobj coordinate system based on 3D positioning keypoints from 4 rack corners.
        
        The coordinate system is defined as follows:
        - Origin: GB200_Rack_Bottom_Left_Corner
        - X-axis: GB200_Rack_Bottom_Left_Corner → GB200_Rack_Bottom_Right_Corner
        - Z-axis: GB200_Rack_Bottom_Left_Corner → GB200_Rack_Top_Left_Corner
        - Y-axis: follows right-hand rule (Y = Z × X)
        """
        print(">>> 7. Building Wobj Coordinate System...")
        
        try:
            # ===================== Find result files =====================
            # Find result files for all operations
            rack_files = {}
            for operation_name in self.operation_names:
                result_file = self._find_latest_result(operation_name)
                if result_file:
                    rack_files[operation_name] = result_file
                else:
                    print(f"\n✗ Failed to locate result file for {operation_name}")
                    return None
            
            if len(rack_files) != len(self.operation_names):
                print("\n✗ Failed to locate all required result files")
                return None
            
            # ===================== Load data =====================
            print("\n📖 Loading 3D positioning results...")
            
            rack_points_data = {}
            for operation_name, result_file in rack_files.items():
                with open(result_file, 'r') as f:
                    rack_data = json.load(f)
                points_3d = rack_data.get('points_3d', [])
                
                if not points_3d:
                    print(f"✗ No keypoints found in {operation_name}")
                    return None
                
                rack_points_data[operation_name] = points_3d
            
            # ===================== Build wobj coordinate system =====================
            print("\n🔧 Perform building wobj coordinate system...")
            
            # Based on template points definition:
            # Index 0: GB200_Rack_Bottom_Left_Corner
            # Index 1: GB200_Rack_Bottom_Right_Corner
            # Index 2: GB200_Rack_Top_Left_Corner
            # Index 3: GB200_Rack_Top_Right_Corner
            
            # Get first operation for bottom corners (typically rack1)
            first_operation = self.operation_names[0]
            first_operation_points = rack_points_data[first_operation]
            
            # Origin: GB200_Rack_Bottom_Left_Corner
            lower_left = np.array(first_operation_points[0])  # KP0 = Lower_Left
            origin = lower_left
            print(f"✓ Origin at GB200_Rack_Bottom_Left_Corner: ({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f})")
            
            # X-axis: Lower_Left → Lower_Right
            lower_right = np.array(first_operation_points[1])  # KP1 = Lower_Right
            x_vec_raw = lower_right - lower_left
            x_axis_length = np.linalg.norm(x_vec_raw)
            x_vec = x_vec_raw / x_axis_length
            
            print(f"✓ X-axis: From 'GB200_Rack_Bottom_Left_Corner' To 'GB200_Rack_Bottom_Right_Corner'")
            print(f"  X = [{x_vec[0]:.6f}, {x_vec[1]:.6f}, {x_vec[2]:.6f}], length = {x_axis_length:.6f} m")
            
            # Z-axis: Lower_Left → Top_Left
            # Find operation that has top corners (typically rack3, index 2)
            # Assuming operation order matches: rack1=bottom_left, rack2=bottom_right, rack3=top_left, rack4=top_right
            top_operation = self.operation_names[2] if len(self.operation_names) > 2 else self.operation_names[0]
            top_operation_points = rack_points_data[top_operation]
            top_left = np.array(top_operation_points[2])  # Template index 2 = Top_Left
            z_vec_raw = top_left - lower_left
            z_axis_length = np.linalg.norm(z_vec_raw)
            z_vec = z_vec_raw / z_axis_length
            
            print(f"✓ Z-axis: From 'GB200_Rack_Bottom_Left_Corner' To 'GB200_Rack_Top_Left_Corner'")
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
            
            print(f"✓ Y-axis: follow the right-hand rule (Y = Z × X)")
            print(f"  Y = [{y_vec[0]:.6f}, {y_vec[1]:.6f}, {y_vec[2]:.6f}]")
            
            # Recalculate Z-axis to ensure perfect orthogonality (Z = X × Y)
            z_vec_orthogonal = np.cross(x_vec, y_vec)
            z_vec_orthogonal = z_vec_orthogonal / np.linalg.norm(z_vec_orthogonal)
            
            # Check how much Z-axis changed
            z_deviation = np.linalg.norm(z_vec - z_vec_orthogonal)
            if z_deviation > 0.01:
                print(f"✗ Z-axis adjusted for orthogonality (deviation: {z_deviation:.6f})")
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
                print(f"✗ Warning: Rotation matrix determinant is not 1, may indicate numerical issues")
            
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
                    "source": "GB200_Rack_Bottom_Left_Corner"
                },
                "axes": {
                    "x_axis": {
                        "vector": [float(x_vec[0]), float(x_vec[1]), float(x_vec[2])],
                        "source": "GB200_Rack_Bottom_Left_Corner → GB200_Rack_Bottom_Right_Corner",
                        "length": float(x_axis_length)
                    },
                    "y_axis": {
                        "vector": [float(y_vec[0]), float(y_vec[1]), float(y_vec[2])],
                        "source": "right_hand_rule (Y = Z × X)"
                    },
                    "z_axis": {
                        "vector": [float(z_vec[0]), float(z_vec[1]), float(z_vec[2])],
                        "source": "GB200_Rack_Bottom_Left_Corner → GB200_Rack_Top_Left_Corner",
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
                "keypoints_used": {op_name: [
                    {"index": idx, "name": self.template_points[idx]['name'], 
                     "coordinates": [float(pt[0]), float(pt[1]), float(pt[2])]}
                    for idx, pt in enumerate(points)
                ] for op_name, points in rack_points_data.items()},
                "timestamp": datetime.now().isoformat(),
                "method": "GB200_rack_corner_based_coordinate_system",
                "description": "X: Lower_Left→Lower_Right, Z: Lower_Left→Top_Left, Y: right-hand rule",
                "source_files": rack_files
            }
            
            # Save coordinate system to temp/wobj_coordinate_system/wobj_result.json
            wobj_dir = os.path.join(self.temp_dir, "wobj_coordinate_system")
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
                    
                    # Save transformation_matrix
                    if self.robot_status_client.set_status('wobj', 'transformation_matrix', T.tolist()):
                        print(f"  ✓ transformation_matrix saved to robot_status")
                    
                    print(f"  ✓ Wobj coordinate system uploaded to robot_status (namespace: wobj)")
                except Exception as e:
                    print(f"  ✗ Error uploading wobj coordinate system to robot_status: {e}")
            else:
                print("\n✗ RobotStatusClient not available, skipping upload to robot_status")
                        
            return coord_system
            
        except Exception as e:
            print(f"\n✗ Error building wobj coordinate system: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def validate_wobj_frame_building(self, coord_system, verbose=True):
        """
        Validate wobj coordinate system by drawing it on all operation images.
        """
        print(">>> 8. Validating Wobj Coordinate System")
        
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
            
            # Process each operation
            validation_results = {}
            
            for operation_name in self.operation_names:
                
                print(f"\n>>> Processing images for {operation_name}...")
                
                session_dir, result_dir = self._find_latest_session_dir(operation_name)
                
                if not session_dir or not os.path.exists(session_dir):
                    print(f"✗ {operation_name} session directory not found")
                    validation_results[operation_name] = False
                    continue
                
                print(f"  Session directory: {session_dir}")
                
                # Get the appropriate origin for this operation
                keypoints_for_rack = coord_system['keypoints_used'].get(operation_name, [])
                if keypoints_for_rack:
                    origin_coords = keypoints_for_rack[0]['coordinates']
                    origin_3d = np.array(origin_coords)
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
                    validation_results[operation_name] = False
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
                        print(f"  ✗ Failed to load image: {img_file}")
                        ax.axis('off')
                        continue
                    
                    # Convert BGR to RGB for matplotlib
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    
                    # Load camera parameters for this view
                    pose_file = img_file.parent / f"{img_file.stem}.json"
                    if not pose_file.exists():
                        print(f"  ✗ Pose file not found for {img_file.name}, skipping")
                        ax.axis('off')
                        continue
                    
                    try:
                        intrinsic, distortion, extrinsic = load_camera_params_from_json(str(pose_file))
                    except Exception as e:
                        print(f"  ✗ Failed to load pose: {e}")
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
                
                # Hide unused subplots
                for idx in range(num_images, len(axes)):
                    axes[idx].axis('off')
                
                # Add overall title
                origin_label = f"{operation_name}_point"
                fig.suptitle(f'Wobj Coordinate System Validation - {operation_name.upper()}\n'
                            f'Origin: {origin_label} | Red: X-axis | Green: Y-axis | Blue: Z-axis',
                            fontsize=14, fontweight='bold', y=0.98)
                
                # Add coordinate system information as text
                x_axis_length = coord_system['axes']['x_axis']['length']
                z_axis_length = coord_system['axes']['z_axis']['length']
                info_text = (
                    f"Wobj Coordinate System Properties:\n"
                    f"Origin (display): ({origin_3d[0]:.4f}, {origin_3d[1]:.4f}, {origin_3d[2]:.4f}) m\n"
                    f"X-axis: Bottom_Left→Bottom_Right, length: {x_axis_length:.4f}m\n"
                    f"Z-axis: Bottom_Left→Top_Left, length: {z_axis_length:.4f}m\n"
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
                    
                    output_path = os.path.join(wobj_dir, f'wobj_validation_{operation_name}.jpg')
                    plt.tight_layout()
                    plt.savefig(output_path, dpi=150, bbox_inches='tight')
                    print(f"  💾 Saved validation image to: {output_path}")
                    
                    # Also save to session result directory
                    if result_dir and os.path.exists(result_dir):
                        session_output_path = os.path.join(result_dir, f'wobj_validation_{operation_name}.jpg')
                        plt.savefig(session_output_path, dpi=150, bbox_inches='tight')
                        print(f"  💾 Copy saved to session: {session_output_path}")
                
                # Close plot to free memory
                plt.close(fig)
                
                validation_results[operation_name] = True
            
            all_success = all(validation_results.values())
            if all_success:
                print("\n✓ Wobj coordinate system validation successfully!")
            else:
                print("\n✗ Wobj coordinate system validation completed with some failures")
            
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
        
        total_operations = len(self.operation_names)
        completed_operations = sum(1 for r in results.values() if r.get("success", False))
        failed_operations = total_operations - completed_operations
        
        print(f"Total Operations: {total_operations}")
        print(f"Completed Successfully: {completed_operations}")
        print(f"Failed: {failed_operations}")
        print()
        
        for operation_name in self.operation_names:
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
        
        # Create URLocateRack instance (inherits from URCapture which is a Node)
        ur_locate_rack = URLocateRack(
            robot_ip=args.robot_ip,
            robot_port=args.robot_port,
            camera_topic=args.camera_topic,
            camera_params_path=args.camera_params_path,
            verbose=args.verbose,
            operation_name='rack_bottom_left'
        )
        
        # Execute auto capture and positioning
        results = ur_locate_rack.auto_capture_and_positioning()
        
        # Check if all positioning operations completed successfully
        all_positioning_success = all(
            r.get('positioning_success', False) for r in results.values()
        )
        
        if not all_positioning_success:
            print("\n✗ Some positioning operations failed.")
            print("✗ Skipping wobj coordinate system building")
            sys.exit(1)
        
        print("\n✓ All positioning operations completed successfully!")
        
        # Build wobj coordinate system        
        coord_system = ur_locate_rack.perform_wobj_frame_building()
        
        if coord_system:
            print("\n✓ Wobj coordinate system built successfully!")
            
            # Validate wobj coordinate system
            validation_success = ur_locate_rack.validate_wobj_frame_building(
                coord_system=coord_system,
                verbose=ur_locate_rack.verbose
            )
            
        else:
            print("\n✗ Failed to build wobj coordinate system")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n\n🛑 Program interrupted by user")
        sys.exit(130)
    except Exception as e:
        print(f"\n✗ Fatal error: {e}")
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
