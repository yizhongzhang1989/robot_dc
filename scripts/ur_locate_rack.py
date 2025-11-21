#!/usr/bin/env python3
"""
URLocateRack - Specialized class for rack-related robot operations
Inherits from URCapture and adds rack-specific functionality
"""

import os
import sys
import json
import time
import cv2
import numpy as np
import argparse
import traceback
from datetime import datetime
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# Import the parent class
from ur_capture import URCapture

# Add ThirdParty robot_vision to path for Web API client
sys.path.append(os.path.join(os.path.dirname(__file__), 'ThirdParty', 'robot_vision'))

# Import Web API client (with error handling)
from core.positioning_3d_webapi import Positioning3DWebAPIClient, load_camera_params_from_json


# ROS2 imports
import rclpy

# Robot status imports
from robot_status.client_utils import RobotStatusClient


class URLocateRack(URCapture):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, 
                 camera_topic="/ur15_camera/image_raw", operation_name="rack1", 
                 camera_params_path="../temp/ur15_cam_calibration_result/ur15_camera_parameters",
                 verbose=True):
        """
        Initialize URLocateRack class for rack-specific robot operations
        
        Args:
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
            camera_topic (str): ROS topic name for camera images
            operation_name (str): Name of the operation for data directory (e.g., 'rack1', 'rack2')
            camera_params_path (str): Path to camera calibration parameters directory
            verbose (bool): If True, saves validation images and error logs to disk
        """
        # Store camera_params_path before calling parent constructor
        # (parent class doesn't save it as an instance attribute)
        self.camera_params_path = camera_params_path
        
        # Call parent class constructor (without verbose parameter)
        super().__init__(
            robot_ip=robot_ip,
            robot_port=robot_port,
            camera_topic=camera_topic,
            operation_name=operation_name,
            camera_params_path=camera_params_path
        )
        
        # Update ROS node name for URLocateRack
        self.get_logger().info("URLocateRack node initialized")
        
        # ===================== Rack-specific attributes =====================
        # Store verbose flag as instance attribute
        self.verbose = verbose
        
        # Initialize positioning client
        self.positioning_client = None
        self._init_positioning_client()
        
        # Initialize robot status client
        self.robot_status_client = None
        self._init_robot_status_client()
        
        # Session directory for storing results
        self.session_dir = None
        self.session_result_dir = None
    
    def _init_positioning_client(self, service_url="http://localhost:8004"):
        """
        Initialize the 3D Positioning Web API Client
        
        Args:
            service_url (str): URL of the positioning service
        """
        # Check if the Web API client is available
        if Positioning3DWebAPIClient is None:
            self.get_logger().warn("✗ Positioning Web API client not available (import failed)")
            self.positioning_client = None
            return
        
        try:
            self.positioning_client = Positioning3DWebAPIClient(service_url=service_url)
            
            # Check service health
            health = self.positioning_client.check_health()
            if health.get('success'):
                self.get_logger().info(f"✓ Positioning service connected to: {service_url}")
                ffpp_connected = health.get('status', {}).get('ffpp_server', {}).get('connected', False)
                refs_loaded = health.get('status', {}).get('references', {}).get('loaded', 0)
                self.get_logger().info(f"  FFPP server connected: {ffpp_connected}")
                self.get_logger().info(f"  {refs_loaded} reference data loaded")
            else:
                self.get_logger().warn(f"✗ Positioning service not available: {health.get('error')}")
                self.positioning_client = None
                
        except Exception as e:
            self.get_logger().error(f"Failed to initialize positioning client: {e}")
            self.positioning_client = None
    
    def _init_robot_status_client(self):
        """
        Initialize the RobotStatusClient for data persistence
        """
        try:
            self.robot_status_client = RobotStatusClient(self, timeout_sec=5.0)
            self.get_logger().info("RobotStatusClient initialized successfully")
        except Exception as e:
            self.get_logger().warning(f"Failed to initialize RobotStatusClient: {e}")
            self.get_logger().warning("Continuing without robot status functionality")
            self.robot_status_client = None
    
    def capture_image_and_pose(self, save_dir, img_filename, pose_filename, robot=None):
        """
        Override parent's method to ensure fresh camera image for each capture.
        Clears old image and waits for new image by spinning ROS multiple times.
        """
        # Clear the old image to force waiting for a fresh one
        old_image = self.latest_image
        self.latest_image = None
        
        # Spin multiple times to get fresh camera image
        max_attempts = 20
        for i in range(max_attempts):
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_image is not None and self.latest_image != old_image:
                # Got a new image
                break
        
        # If we couldn't get a new image, restore the old one
        if self.latest_image is None:
            self.get_logger().warn("Failed to get fresh camera image, using previous image")
            self.latest_image = old_image
        
        # Call parent's implementation
        return super().capture_image_and_pose(save_dir, img_filename, pose_filename, robot)
    
    def _setup_result_directory(self, operation_name):
        """
        Setup result directory for storing positioning results
        
        Args:
            operation_name (str): Name of the operation (rack1, rack2, or wobj)
        """
        # Special handling for wobj coordinate system
        if operation_name == 'wobj':
            result_dir = os.path.join(self.script_dir, '..', 'temp', 'wobj_coordinate_system')
        else:
            # Result directory path for storing location results
            data_parent_dir = os.path.join(self.script_dir, '..', 'dataset', operation_name)
            result_dir = os.path.join(data_parent_dir, "result")
        
        # Create result directory if it doesn't exist
        if not os.path.exists(result_dir):
            os.makedirs(result_dir, exist_ok=True)
            self.get_logger().info(f"✓ Created result directory: {result_dir}")
        
        return result_dir
    
    def _upload_reference_data(self, operation_name):
        """
        Upload reference data to FFPP web service
        
        Args:
            operation_name (str): Name of the operation (rack1 or rack2)
            
        Returns:
            bool: True if upload successful, False otherwise
        """
        if self.positioning_client is None:
            self.get_logger().error("Positioning client not initialized")
            return False
        
        try:
            self.get_logger().info(f">>> Uploading reference data for {operation_name} to FFPP web...")
            result = self.positioning_client.upload_references()
            
            if result.get('success'):
                refs_loaded = result.get('references_loaded', 0)
                refs_found = result.get('references_found', 0)
                self.get_logger().info(f"✓ Reference data uploaded: {refs_loaded}/{refs_found}")
                return True
            else:
                self.get_logger().error(f"✗ Failed to upload references: {result.get('error')}")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error uploading references: {e}")
            return False
    
    def _collect_and_upload_data(self, operation_name, session_id):
        """
        Collect camera images and upload them in real-time
        
        Args:
            operation_name (str): Name of the operation (rack1 or rack2)
            session_id (str): Session ID for uploading images
            
        Returns:
            str: Path to session directory if successful, False otherwise
        """
        robot = self.robot
        if robot is None:
            self.get_logger().error("Error: Robot not initialized")
            return False
        
        # Create session directory
        session_dir = self._create_session_directory()
        
        self.get_logger().info(f">>> Starting data collection for {operation_name}...")
        
        try:
            if not self._movej_to_collect_position(robot):
                self.get_logger().error("✗ Failed to move to collect position")
                return False
            
            # Get current TCP pose
            current_tcp_pose = robot.get_actual_tcp_pose()
            if current_tcp_pose is None:
                self.get_logger().error("Failed to get current TCP pose")
                return False
            
            # Use movements from class variable
            movements = []
            for idx, (movement_name, offset) in enumerate(self.movements.items()):
                movements.append({
                    "name": movement_name,
                    "offset": offset,
                    "index": idx
                })
            
            # Create save directory
            if not os.path.exists(session_dir):
                os.makedirs(session_dir)
            
            # Wait for camera image
            self.get_logger().info("Waiting for camera image...")
            max_wait_time = 10.0
            start_time = time.time()
            
            while self.latest_image is None and (time.time() - start_time) < max_wait_time:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_image is None:
                self.get_logger().error("Error: No camera image received")
                return False
            
            self.get_logger().info("✓ Camera image available, starting data collection...")
            
            success_count = 0
            upload_count = 0
            
            for movement in movements:
                try:
                    self.get_logger().info(f"\n--- {movement['name']} ---")
                    
                    offset = movement['offset']
                    self.get_logger().info(f"Move TCP by offset: {[f'{p:.4f}' for p in offset]}")
                    
                    # Move robot
                    move_result = robot.move_tcp(offset, a=0.1, v=0.05)
                    if move_result != 0:
                        self.get_logger().warn(f"Movement failed for {movement['name']}")
                        continue
                    
                    time.sleep(0.5)
                    
                    # Capture image and pose
                    img_filename = f"{movement['index']}.jpg"
                    pose_filename = f"{movement['index']}_pose.json"
                    
                    if self.capture_image_and_pose(session_dir, img_filename, pose_filename, robot=robot):
                        self.get_logger().info(f"✓ Successfully captured data for {movement['name']}")
                        success_count += 1
                        
                        # Upload immediately
                        if session_id is not None and self.positioning_client is not None:
                            img_path = os.path.join(session_dir, img_filename)
                            pose_path = os.path.join(session_dir, pose_filename)
                            
                            try:
                                image = cv2.imread(img_path)
                                if image is None:
                                    self.get_logger().warn(f"  ⚠️  Failed to load image: {img_path}")
                                else:
                                    intrinsic, distortion, extrinsic = load_camera_params_from_json(pose_path)
                                    
                                    result = self.positioning_client.upload_view(
                                        session_id=session_id,
                                        image=image,
                                        intrinsic=intrinsic,
                                        distortion=distortion,
                                        extrinsic=extrinsic
                                    )
                                    
                                    if result.get('success'):
                                        upload_count += 1
                                        self.get_logger().info(f"   ✓ View {upload_count} uploaded")
                                    else:
                                        self.get_logger().error(f"   ✗ Upload failed: {result.get('error')}")
                            except Exception as e:
                                self.get_logger().error(f"   ✗ Error uploading: {e}")
                        
                        time.sleep(0.5)
                    else:
                        self.get_logger().warn(f"✗ Failed to capture data for {movement['name']}")
                    
                    # Return to original position
                    return_result = robot.movel(current_tcp_pose, a=0.1, v=0.05)
                    if return_result == 0:
                        self.get_logger().info("✓ Returned to start position")
                    
                    time.sleep(0.5)
                    
                except Exception as e:
                    self.get_logger().error(f"Error during {movement['name']}: {e}")
                    continue
            
            # Final return
            self.get_logger().info(f"\n--- Final return to original position ---")
            robot.movel(current_tcp_pose, a=0.1, v=0.05)
            time.sleep(0.5)
            
            self.get_logger().info(f"\nData collection: {success_count}/{len(movements)} successful")
            self.get_logger().info(f"Upload: {upload_count}/{success_count} uploaded")
            
            return session_dir if success_count == len(movements) else False
            
        except Exception as e:
            self.get_logger().error(f"Error during data collection: {e}")
            return False
    
    def capture_data(self):
        """
        Complete 3D positioning workflow for both rack1 and rack2.
        
        For each operation, this method:
        1. Upload reference data to FFPP web service
        2. Initialize session for real-time upload
        3. Collect camera images and upload in real-time
        4. Wait for triangulation results
        5. Save results to result directory
        6. Validate positioning results
        
        Returns:
            dict: Results for both rack1 and rack2
                  {'rack1': result_data or None, 'rack2': result_data or None}
        """
        if self.positioning_client is None:
            self.get_logger().error("✗ Positioning client not initialized. Cannot perform 3D positioning.")
            return {'rack1': None, 'rack2': None}
        
        results = {}
        operations = ['rack1', 'rack2']
        # Store session directories for each operation
        self.session_dirs = {}
        
        for operation in operations:
            self.get_logger().info(f"\n{'='*80}")
            self.get_logger().info(f"Starting 3D positioning workflow for {operation}")
            self.get_logger().info(f"{'='*80}")
            
            try:
                # Update operation name
                original_operation_name = self.operation_name
                self.operation_name = operation
                
                # Update paths for this operation
                self._setup_paths(operation, self.camera_params_path)
                self._load_collect_position_from_config()
                
                # Step 1: Upload reference data
                self.get_logger().info(f"Step 1: Uploading reference data for {operation}...")
                upload_success = self._upload_reference_data(operation)
                if not upload_success:
                    self.get_logger().error(f"✗ Failed to upload reference data for {operation}")
                    results[operation] = None
                    self.operation_name = original_operation_name
                    continue
                
                self.get_logger().info(f"✓ Reference data uploaded successfully!")
                
                # Step 2: Initialize session
                self.get_logger().info(f"Step 2: Initializing session for {operation}...")
                session_result = self.positioning_client.init_session(reference_name=operation)
                if not session_result.get('success'):
                    self.get_logger().error(f"✗ Failed to initialize session: {session_result.get('error')}")
                    results[operation] = None
                    self.operation_name = original_operation_name
                    continue
                
                session_id = session_result['session_id']
                self.get_logger().info(f"✓ Web session created: {session_id}")
                
                # Step 3: Collect and upload data
                self.get_logger().info(f"Step 3: Collecting and uploading camera images for {operation}...")
                session_dir = self._collect_and_upload_data(operation, session_id)
                if not session_dir:
                    self.get_logger().error(f"✗ Failed to collect data for {operation}")
                    self.positioning_client.terminate_session(session_id)
                    results[operation] = None
                    self.operation_name = original_operation_name
                    continue
                
                self.get_logger().info(f"✓ Data collection and upload completed!")
                
                # Step 4: Wait for triangulation result
                self.get_logger().info(f"Step 4: Waiting for positioning results (timeout: 30s)...")
                result = self.positioning_client.get_result(session_id, timeout=30000)
                
                if not result.get('success'):
                    self.get_logger().error(f"✗ Failed to get result: {result.get('error')}")
                    self.positioning_client.terminate_session(session_id)
                    results[operation] = None
                    self.operation_name = original_operation_name
                    continue
                
                # Check if we got the final result
                if 'result' not in result:
                    if result.get('timeout'):
                        self.get_logger().error(f"✗ Timeout waiting for triangulation")
                    else:
                        session_info = result.get('session', {})
                        session_status = session_info.get('status')
                        if session_status == 'failed':
                            self.get_logger().error(f"✗ Session failed: {session_info.get('error_message', 'Unknown error')}")
                        else:
                            self.get_logger().error(f"✗ Triangulation not completed (status: {session_status})")
                    self.positioning_client.terminate_session(session_id)
                    results[operation] = None
                    self.operation_name = original_operation_name
                    continue
                
                self.get_logger().info(f"✓ Triangulation completed!")
                
                triangulation_result = result['result']
                points_3d = np.array(triangulation_result['points_3d'])
                mean_error = triangulation_result['mean_error']
                processing_time = triangulation_result.get('processing_time', 0)
                views_data = result.get('views', [])
                
                self.get_logger().info(f"   Number of 3D points: {len(points_3d)}")
                self.get_logger().info(f"   Mean reprojection error: {mean_error:.3f} pixels")
                self.get_logger().info(f"   Processing time: {processing_time:.2f} seconds")
                self.get_logger().info(f"   Number of views: {len(views_data)}")
                
                # Step 5: Terminate session
                self.get_logger().info(f"Step 5: Terminating session...")
                term_result = self.positioning_client.terminate_session(session_id)
                if term_result.get('success'):
                    self.get_logger().info(f"✓ Session {session_id} terminated")
                else:
                    self.get_logger().warn(f"⚠️  Failed to terminate session: {term_result.get('error')}")
                
                # Step 6: Save results to result directory
                self.get_logger().info(f"Step 6: Saving 3D positioning results...")
                
                # Setup result directory
                result_dir = self._setup_result_directory(operation)
                
                # Get session name and create result directory
                session_name = os.path.basename(session_dir)
                session_result_dir = os.path.join(result_dir, session_name)
                os.makedirs(session_result_dir, exist_ok=True)
                
                # Store session_dir for this operation
                self.session_dirs[operation] = session_dir
                
                # Prepare result data
                result_data = {
                    'timestamp': datetime.now().isoformat(),
                    'operation': operation,
                    'points_3d': triangulation_result['points_3d'],
                    'mean_error': triangulation_result['mean_error'],
                    'processing_time': triangulation_result.get('processing_time', 0),
                    'views': result.get('views', [])
                }
                
                # Save to JSON file
                positioning_result_file_path = os.path.join(session_result_dir, '3d_positioning_result.json')
                with open(positioning_result_file_path, 'w') as f:
                    json.dump(result_data, f, indent=2)
                
                self.get_logger().info(f"✓ Results saved to: {positioning_result_file_path}")
                
                # Step 7: Validate positioning results
                self.get_logger().info(f"Step 7: Validating positioning results...")
                
                try:
                    # Import validate function from ur_locate if available
                    validation_success = self._validate_positioning_results(session_dir, result, session_result_dir)
                    if validation_success:
                        self.get_logger().info("✓ Validation completed successfully!")
                    else:
                        self.get_logger().warn("⚠ Validation completed with warnings")
                except Exception as e:
                    self.get_logger().warn(f"⚠ Validation failed: {e}")
                
                results[operation] = result_data
                
                # Restore original operation name
                self.operation_name = original_operation_name
                
                self.get_logger().info(f"✓ 3D positioning workflow for {operation} completed successfully!")
                
            except Exception as e:
                self.get_logger().error(f"Error during {operation} 3D positioning: {e}")
                traceback.print_exc()
                results[operation] = None
                # Restore original operation name
                if 'original_operation_name' in locals():
                    self.operation_name = original_operation_name
            
            # Add delay between operations
            if operation != operations[-1]:
                self.get_logger().info(f"\nWaiting 3 seconds before next operation...")
                time.sleep(3)
        
        # Summary
        self.get_logger().info(f"\n{'='*80}")
        self.get_logger().info("3D Positioning Summary:")
        self.get_logger().info(f"{'='*80}")
        for operation, result_data in results.items():
            if result_data:
                status = "✓ SUCCESS"
                self.get_logger().info(f"{operation}: {status}")
                self.get_logger().info(f"  - 3D points: {len(result_data['points_3d'])}")
                self.get_logger().info(f"  - Mean error: {result_data['mean_error']:.3f} pixels")
            else:
                status = "✗ FAILED"
                self.get_logger().info(f"{operation}: {status}")
        self.get_logger().info(f"{'='*80}")
        
        return results
    
    def _validate_positioning_results(self, session_dir, result_data, output_dir):
        """
        Validate 3D positioning results by visualizing tracked and reprojected keypoints.
        
        Creates a visualization showing:
        - Green circles: Original tracked 2D keypoints from views
        - Red X markers: Reprojected 3D points back to each view
        
        Args:
            session_dir (str): Path to session directory containing test images
            result_data (dict): Result data containing 3D points and views information
            output_dir (str): Directory to save validation results
            
        Returns:
            bool: True if validation successful, False otherwise
        """
        try:
            self.get_logger().info("Validating 3D Positioning Results")
            
            # Extract data from result
            points_3d = np.array(result_data['result']['points_3d'])
            views_data = result_data.get('views', [])
            
            if len(views_data) == 0:
                self.get_logger().error("No view data found in results")
                return False
            
            self.get_logger().info(f"✓ Loaded {len(points_3d)} 3D points")
            self.get_logger().info(f"✓ Loaded {len(views_data)} views")
            
            # Find all test images
            test_img_dir = Path(session_dir)
            image_files = sorted(test_img_dir.glob("*.jpg"))
            
            if len(image_files) == 0:
                self.get_logger().error(f"No images found in {session_dir}")
                return False
            
            # Create figure with subplots for each view
            num_views = min(len(views_data), len(image_files))
            cols = min(3, num_views)
            rows = (num_views + cols - 1) // cols
            
            import matplotlib.pyplot as plt
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
                    self.get_logger().warn(f"⚠ Failed to load image: {img_file}")
                    ax.axis('off')
                    continue
                
                # Convert BGR to RGB for matplotlib
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                ax.imshow(img_rgb)
                
                # Get tracked 2D keypoints (green circles)
                keypoints_2d = view_data.get('keypoints_2d', [])
                
                # Load camera parameters for this view
                pose_file = img_file.parent / f"{img_file.stem}_pose.json"
                if not pose_file.exists():
                    self.get_logger().warn(f"⚠ Pose file not found: {pose_file}")
                    ax.axis('off')
                    continue
                
                try:
                    intrinsic, distortion, extrinsic = load_camera_params_from_json(str(pose_file))
                except Exception as e:
                    self.get_logger().warn(f"⚠ Failed to load pose: {e}")
                    ax.axis('off')
                    continue
                
                # Extract camera parameters
                fx = intrinsic[0, 0]
                fy = intrinsic[1, 1]
                cx = intrinsic[0, 2]
                cy = intrinsic[1, 2]
                
                # extrinsic is already base2cam (world to camera transformation)
                base2cam = extrinsic
                
                # Reproject 3D points back to this view (red X markers)
                for pt_idx, point_3d in enumerate(points_3d):
                    # Transform 3D point to camera frame
                    # points_3d are in base coordinate system
                    point_3d_base = np.array([point_3d[0], point_3d[1], point_3d[2], 1.0])
                    point_3d_cam = base2cam @ point_3d_base
                    
                    x_cam = point_3d_cam[0]
                    y_cam = point_3d_cam[1]
                    z_cam = point_3d_cam[2]
                    
                    if z_cam <= 0:
                        continue  # Point behind camera
                    
                    # Project to image plane with distortion
                    if distortion is not None and np.any(distortion != 0):
                        x_norm = x_cam / z_cam
                        y_norm = y_cam / z_cam
                        
                        # Apply radial and tangential distortion
                        k1, k2, p1, p2, k3 = distortion.flatten()[:5]
                        r2 = x_norm**2 + y_norm**2
                        
                        radial = 1 + k1*r2 + k2*r2**2 + k3*r2**3
                        x_distorted = x_norm * radial + 2*p1*x_norm*y_norm + p2*(r2 + 2*x_norm**2)
                        y_distorted = y_norm * radial + p1*(r2 + 2*y_norm**2) + 2*p2*x_norm*y_norm
                        
                        u_reproj = fx * x_distorted + cx
                        v_reproj = fy * y_distorted + cy
                    else:
                        u_reproj = fx * (x_cam / z_cam) + cx
                        v_reproj = fy * (y_cam / z_cam) + cy
                    
                    # Plot reprojected point (red x marker)
                    ax.plot(u_reproj, v_reproj, 'rx', markersize=6,
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
                                'error_pixels': float(error)
                            })
                            
                            # Add point index label (only once, at midpoint between tracked and reprojected)
                            mid_u = (u_track + u_reproj) / 2
                            mid_v = (v_track + v_reproj) / 2
                            ax.text(mid_u, mid_v + 50, f'{pt_idx}',
                                   color='red', fontsize=8, fontweight='bold', ha='center')
                
                # Plot tracked keypoints (green filled circles)
                for pt_idx, kp in enumerate(keypoints_2d):
                    u_track = kp.get('x')
                    v_track = kp.get('y')
                    
                    if u_track is not None and v_track is not None:
                        ax.plot(u_track, v_track, 'go', markersize=2,
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
            fig.suptitle('3D Positioning Validation\nGreen: Tracked 2D | Red X: Reprojected from 3D',
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
                    f"Mean: {mean_error:.2f} pixels\n"
                    f"Std: {std_error:.2f} pixels\n"
                    f"Min: {min_error:.2f} pixels\n"
                    f"Max: {max_error:.2f} pixels\n"
                    f"Total samples: {len(errors)}"
                )
                
                fig.text(0.02, 0.02, stats_text, fontsize=10,
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                        verticalalignment='bottom')
                
                self.get_logger().info(f"  Reprojection Error Statistics:")
                self.get_logger().info(f"  Mean error: {mean_error:.2f} pixels")
                self.get_logger().info(f"  Std error: {std_error:.2f} pixels")
                self.get_logger().info(f"  Min error: {min_error:.2f} pixels")
                self.get_logger().info(f"  Max error: {max_error:.2f} pixels")
                self.get_logger().info(f"  Total samples: {len(errors)}")
            
            # Save figure to output directory (if verbose)
            if self.verbose:
                output_path = os.path.join(output_dir, 'positioning_result_reprojection.jpg')
                plt.tight_layout()
                plt.savefig(output_path, dpi=150, bbox_inches='tight')
                self.get_logger().info(f"\n💾 Validation visualization saved to: {output_path}")
            
            # Close plot to free memory
            plt.close(fig)
            
            # Save detailed reprojection errors to JSON (if verbose)
            if self.verbose and reprojection_errors:
                error_log_path = os.path.join(output_dir, 'positioning_result_reprojection_report.json')
                error_data = {
                    'timestamp': datetime.now().isoformat(),
                    'num_views': len(views_data),
                    'num_points': len(points_3d),
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
                
                self.get_logger().info(f"💾 Positioning result reprojection report saved to: {error_log_path}")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Validation error: {e}")
            traceback.print_exc()
            return False
    
    # =================================== functions for wobj frame building ===================================
    def perform_wobj_frame_building(self, rack1_session_result_dir=None, rack2_session_result_dir=None):
        """
        Build a wobj coordinate system based on 3D positioning keypoints from both rack1 and rack2.
        
        The coordinate system is defined as follows:
        - Origin: at rack1 point1 (keypoint[0])
        - X-axis: rack1_point1 → rack1_point2 direction (keypoint[0] → keypoint[1])
        - Z-axis: rack1_point1 → rack2_point1 direction (rack1_keypoint[0] → rack2_keypoint[0])
        - Y-axis: follows right-hand rule (Y = Z × X)
        
        Args:
            rack1_session_result_dir (str): Path to rack1 session result directory
                                           If None, uses the most recent rack1 session
            rack2_session_result_dir (str): Path to rack2 session result directory
                                           If None, uses the most recent rack2 session
            
        Returns:
            dict: Coordinate system information or None if failed
        """
        self.get_logger().info(">>> Building wobj Coordinate System from rack1 and rack2")
        
        try:
            # ===================== Load rack1 data =====================
            if rack1_session_result_dir is None:
                # Find most recent rack1 session
                rack1_result_dir = self._setup_result_directory('rack1')
                if not os.path.exists(rack1_result_dir):
                    self.get_logger().error(f"rack1 result directory not found: {rack1_result_dir}")
                    return None
                
                sessions = [d for d in os.listdir(rack1_result_dir) 
                           if os.path.isdir(os.path.join(rack1_result_dir, d))]
                
                if not sessions:
                    self.get_logger().error("No session directories found in rack1 result directory")
                    return None
                
                sessions.sort(reverse=True)
                rack1_session_result_dir = os.path.join(rack1_result_dir, sessions[0])
                self.get_logger().info(f"Using most recent rack1 session: {os.path.basename(rack1_session_result_dir)}")
            
            # Verify rack1 session result directory exists
            if not os.path.exists(rack1_session_result_dir):
                self.get_logger().error(f"rack1 session result directory not found: {rack1_session_result_dir}")
                return None
            
            # Load rack1 3D positioning result
            rack1_positioning_file = os.path.join(rack1_session_result_dir, '3d_positioning_result.json')
            if not os.path.exists(rack1_positioning_file):
                self.get_logger().error(f"rack1 3D positioning result file not found: {rack1_positioning_file}")
                return None
            
            self.get_logger().info(f"📖 Loading rack1 3D positioning results from: {rack1_positioning_file}")
            
            with open(rack1_positioning_file, 'r') as f:
                rack1_data = json.load(f)
            
            rack1_points_3d = rack1_data.get('points_3d', [])
            
            if len(rack1_points_3d) < 2:
                self.get_logger().error(f"Insufficient rack1 keypoints (need at least 2, got {len(rack1_points_3d)})")
                return None
            
            self.get_logger().info(f"✓ Loaded {len(rack1_points_3d)} rack1 3D points")
            
            # ===================== Load rack2 data =====================
            if rack2_session_result_dir is None:
                # Find most recent rack2 session
                rack2_result_dir = self._setup_result_directory('rack2')
                if not os.path.exists(rack2_result_dir):
                    self.get_logger().error(f"rack2 result directory not found: {rack2_result_dir}")
                    return None
                
                sessions = [d for d in os.listdir(rack2_result_dir) 
                           if os.path.isdir(os.path.join(rack2_result_dir, d))]
                
                if not sessions:
                    self.get_logger().error("No session directories found in rack2 result directory")
                    return None
                
                sessions.sort(reverse=True)
                rack2_session_result_dir = os.path.join(rack2_result_dir, sessions[0])
                self.get_logger().info(f"Using most recent rack2 session: {os.path.basename(rack2_session_result_dir)}")
            
            # Verify rack2 session result directory exists
            if not os.path.exists(rack2_session_result_dir):
                self.get_logger().error(f"rack2 session result directory not found: {rack2_session_result_dir}")
                return None
            
            # Load rack2 3D positioning result
            rack2_positioning_file = os.path.join(rack2_session_result_dir, '3d_positioning_result.json')
            if not os.path.exists(rack2_positioning_file):
                self.get_logger().error(f"rack2 3D positioning result file not found: {rack2_positioning_file}")
                return None
            
            self.get_logger().info(f"📖 Loading rack2 3D positioning results from: {rack2_positioning_file}")
            
            with open(rack2_positioning_file, 'r') as f:
                rack2_data = json.load(f)
            
            rack2_points_3d = rack2_data.get('points_3d', [])
            
            if len(rack2_points_3d) < 2:
                self.get_logger().error(f"Insufficient rack2 keypoints (need at least 2, got {len(rack2_points_3d)})")
                return None
            
            self.get_logger().info(f"✓ Loaded {len(rack2_points_3d)} rack2 3D points")
            
            # ===================== Build wobj coordinate system =====================
            # Origin: rack1 keypoint[0] (点1)
            origin = np.array(rack1_points_3d[0])
            self.get_logger().info(f"✓ Origin defined at rack1_point1 (KP0): ({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f})")
            
            # X-axis: rack1_keypoint[0] → rack1_keypoint[1] (点1→点2)
            rack1_kp1 = np.array(rack1_points_3d[0])
            rack1_kp2 = np.array(rack1_points_3d[1])
            x_vec_raw = rack1_kp2 - rack1_kp1
            x_axis_length = np.linalg.norm(x_vec_raw)
            x_vec = x_vec_raw / x_axis_length
            
            self.get_logger().info(f"✓ X-axis defined as rack1_point1→rack1_point2 (KP0→KP1):")
            self.get_logger().info(f"  rack1_point1 (KP0): ({rack1_kp1[0]:.6f}, {rack1_kp1[1]:.6f}, {rack1_kp1[2]:.6f})")
            self.get_logger().info(f"  rack1_point2 (KP1): ({rack1_kp2[0]:.6f}, {rack1_kp2[1]:.6f}, {rack1_kp2[2]:.6f})")
            self.get_logger().info(f"  X = [{x_vec[0]:.6f}, {x_vec[1]:.6f}, {x_vec[2]:.6f}], length = {x_axis_length:.6f}")
            
            # Z-axis: rack1_keypoint[0] → rack2_keypoint[0] (rack1点1→rack2点1)
            rack2_kp1 = np.array(rack2_points_3d[0])
            z_vec_raw = rack2_kp1 - rack1_kp1
            z_axis_length = np.linalg.norm(z_vec_raw)
            z_vec = z_vec_raw / z_axis_length
            
            self.get_logger().info(f"✓ Z-axis defined as rack1_point1→rack2_point1 (rack1_KP0→rack2_KP0):")
            self.get_logger().info(f"  rack2_point1 (KP0): ({rack2_kp1[0]:.6f}, {rack2_kp1[1]:.6f}, {rack2_kp1[2]:.6f})")
            self.get_logger().info(f"  Z = [{z_vec[0]:.6f}, {z_vec[1]:.6f}, {z_vec[2]:.6f}], length = {z_axis_length:.6f}")
            
            # Check if X and Z are nearly parallel
            x_dot_z = np.dot(x_vec, z_vec)
            if abs(x_dot_z) > 0.95:
                self.get_logger().error(f"✗ X-axis and Z-axis are nearly parallel (dot product = {x_dot_z:.6f})")
                self.get_logger().error("  Cannot build a valid coordinate system. Please check keypoint positions.")
                return None
            
            # Y-axis: right-hand rule (Y = Z × X)
            y_vec = np.cross(z_vec, x_vec)
            y_vec = y_vec / np.linalg.norm(y_vec)
            
            self.get_logger().info(f"✓ Y-axis defined from right-hand rule (Y = Z × X):")
            self.get_logger().info(f"  Y = [{y_vec[0]:.6f}, {y_vec[1]:.6f}, {y_vec[2]:.6f}]")
            
            # Recalculate Z-axis to ensure perfect orthogonality (Z = X × Y)
            z_vec_orthogonal = np.cross(x_vec, y_vec)
            z_vec_orthogonal = z_vec_orthogonal / np.linalg.norm(z_vec_orthogonal)
            
            # Check how much Z-axis changed
            z_deviation = np.linalg.norm(z_vec - z_vec_orthogonal)
            if z_deviation > 0.01:
                self.get_logger().warn(f"⚠ Z-axis adjusted for orthogonality (deviation: {z_deviation:.6f})")
                z_vec = z_vec_orthogonal
                self.get_logger().info(f"  Z = [{z_vec[0]:.6f}, {z_vec[1]:.6f}, {z_vec[2]:.6f}] (orthogonal)")
            
            # Verify orthogonality
            dot_xy = np.dot(x_vec, y_vec)
            dot_xz = np.dot(x_vec, z_vec)
            dot_yz = np.dot(y_vec, z_vec)
            
            self.get_logger().info(f"\n🔍 Orthogonality check:")
            self.get_logger().info(f"  X·Y = {dot_xy:.8f} (should be ~0)")
            self.get_logger().info(f"  X·Z = {dot_xz:.8f} (should be ~0)")
            self.get_logger().info(f"  Y·Z = {dot_yz:.8f} (should be ~0)")
            
            # Build rotation matrix (columns are the axis vectors)
            rot_matrix = np.column_stack([x_vec, y_vec, z_vec])
            
            # Verify it's a valid rotation matrix
            det_R = np.linalg.det(rot_matrix)
            self.get_logger().info(f"  det(R) = {det_R:.8f} (should be ~1)")
            
            if abs(det_R - 1.0) > 0.01:
                self.get_logger().warn(f"⚠ Warning: Rotation matrix determinant is not 1, may indicate numerical issues")
            
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
                    "source": "rack1_point1 (KP0)"
                },
                "axes": {
                    "x_axis": {
                        "vector": [float(x_vec[0]), float(x_vec[1]), float(x_vec[2])],
                        "source": "rack1_point1→rack1_point2 (KP0→KP1)",
                        "length": float(x_axis_length)
                    },
                    "y_axis": {
                        "vector": [float(y_vec[0]), float(y_vec[1]), float(y_vec[2])],
                        "source": "right_hand_rule_Z_cross_X"
                    },
                    "z_axis": {
                        "vector": [float(z_vec[0]), float(z_vec[1]), float(z_vec[2])],
                        "source": "rack1_point1→rack2_point1 (rack1_KP0→rack2_KP0)",
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
                        {"index": 0, "name": "point1", "coordinates": [float(rack1_kp1[0]), float(rack1_kp1[1]), float(rack1_kp1[2])]},
                        {"index": 1, "name": "point2", "coordinates": [float(rack1_kp2[0]), float(rack1_kp2[1]), float(rack1_kp2[2])]}
                    ],
                    "rack2": [
                        {"index": 0, "name": "point1", "coordinates": [float(rack2_kp1[0]), float(rack2_kp1[1]), float(rack2_kp1[2])]}
                    ]
                },
                "timestamp": datetime.now().isoformat(),
                "method": "rack1_rack2_based_coordinate_system",
                "source_files": {
                    "rack1": rack1_positioning_file,
                    "rack2": rack2_positioning_file
                }
            }
            
            # Save coordinate system to temp/wobj_coordinate_system/wobj_result.json
            wobj_result_dir = self._setup_result_directory('wobj')
            coord_system_path = os.path.join(wobj_result_dir, 'wobj_result.json')
            
            with open(coord_system_path, 'w') as f:
                json.dump(coord_system, f, indent=2)
            
            self.get_logger().info(f"💾 Wobj coordinate system saved to: {coord_system_path}")
            
            # Also save to both session result directories for reference
            for session_dir, rack_name in [(rack1_session_result_dir, 'rack1'), 
                                          (rack2_session_result_dir, 'rack2')]:
                session_coord_path = os.path.join(session_dir, 'wobj_frame_building_result.json')
                with open(session_coord_path, 'w') as f:
                    json.dump(coord_system, f, indent=2)
                self.get_logger().info(f"💾 Wobj coordinate system also saved to {rack_name} session: {session_coord_path}")
            
            self.get_logger().info(f"🎯 Wobj coordinate system established successfully!")
            
            # Display summary
            self.get_logger().info("Wobj Coordinate System Summary:")
            self.get_logger().info(f"Origin: rack1_point1 (KP0) = ({origin[0]:.4f}, {origin[1]:.4f}, {origin[2]:.4f})")
            self.get_logger().info(f"X-axis: rack1_point1→rack1_point2 (KP0→KP1), length = {x_axis_length:.4f} m")
            self.get_logger().info(f"Z-axis: rack1_point1→rack2_point1 (rack1_KP0→rack2_KP0), length = {z_axis_length:.4f} m")
            self.get_logger().info(f"Y-axis: right-hand rule (Y = Z × X)")
            self.get_logger().info(f"Pose (x, y, z, rx, ry, rz):")
            self.get_logger().info(f"  ({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f}, {rx:.6f}, {ry:.6f}, {rz:.6f})")

            
            # Validate wobj frame building results
            self.get_logger().info("\n>>> Validating wobj frame building results...")
            validation_success = self.validate_wobj_frame_building_results(
                rack1_session_result_dir,
                rack2_session_result_dir,
                coord_system,
                verbose=self.verbose
            )
            
            if validation_success:
                self.get_logger().info("✓ Wobj validation completed successfully!")
            else:
                self.get_logger().warn("⚠ Wobj validation completed with warnings")
            
            # Save wobj coordinate system to robot_status
            if self.robot_status_client:
                try:
                    # Save origin
                    if self.robot_status_client.set_status('rack', 'wobj_origin', [float(origin[0]), float(origin[1]), float(origin[2])]):
                        self.get_logger().info(f"✓ wobj_origin saved to robot_status")
                    
                    # Save x_axis
                    if self.robot_status_client.set_status('rack', 'wobj_x', [float(x_vec[0]), float(x_vec[1]), float(x_vec[2])]):
                        self.get_logger().info(f"✓ wobj_x saved to robot_status")
                    
                    # Save y_axis
                    if self.robot_status_client.set_status('rack', 'wobj_y', [float(y_vec[0]), float(y_vec[1]), float(y_vec[2])]):
                        self.get_logger().info(f"✓ wobj_y saved to robot_status")
                    
                    # Save z_axis
                    if self.robot_status_client.set_status('rack', 'wobj_z', [float(z_vec[0]), float(z_vec[1]), float(z_vec[2])]):
                        self.get_logger().info(f"✓ wobj_z saved to robot_status")
                    
                    self.get_logger().info(f"✓ Wobj coordinate system saved to robot_status (namespace: rack)")
                except Exception as e:
                    self.get_logger().warning(f"Error saving wobj coordinate system to robot_status: {e}")
            
            return coord_system
            
        except Exception as e:
            self.get_logger().error(f"Error building wobj coordinate system: {e}")
            traceback.print_exc()
            return None
    
    def validate_wobj_frame_building_results(self, rack1_session_result_dir, rack2_session_result_dir, 
                                            coord_system, verbose=True):
        """
        Validate workpiece coordinate system by drawing it on rack1 and rack2 images.
        
        Draws the coordinate system axes on captured images:
        - For rack1 images: Origin at rack1_point1 (wobj origin)
        - For rack2 images: Origin at rack2_point1, but axes follow wobj orientation
        
        Args:
            rack1_session_result_dir (str): Path to rack1 session result directory
            rack2_session_result_dir (str): Path to rack2 session result directory
            coord_system (dict): Dictionary containing coordinate system information
            verbose (bool): If True, saves validation images to disk
            
        Returns:
            bool: True if validation successful, False otherwise
        """
        self.get_logger().info("Drawing Wobj Coordinate System on rack1 and rack2 Images")
        
        try:
            if coord_system is None:
                self.get_logger().error("Error: coord_system is None")
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
            
            # Process rack1 and rack2 separately
            validation_results = {}
            
            for rack_name, session_result_dir in [('rack1', rack1_session_result_dir), 
                                                   ('rack2', rack2_session_result_dir)]:
                
                self.get_logger().info(f"\n>>> Processing {rack_name} images...")
                
                # For rack2, use rack2_point1 as origin for display
                if rack_name == 'rack2':
                    # Get rack2_point1 from coord_system
                    rack2_kp1_coords = coord_system['keypoints_used']['rack2'][0]['coordinates']
                    origin_3d = np.array(rack2_kp1_coords)
                    self.get_logger().info(f"  Using rack2_point1 as origin: ({origin_3d[0]:.6f}, {origin_3d[1]:.6f}, {origin_3d[2]:.6f})")
                else:
                    # For rack1, use wobj origin (rack1_point1)
                    origin_3d = wobj_origin_3d.copy()
                    self.get_logger().info(f"  Using wobj origin (rack1_point1): ({origin_3d[0]:.6f}, {origin_3d[1]:.6f}, {origin_3d[2]:.6f})")
                
                # Calculate 3D endpoints of axes from the origin
                x_end_3d = origin_3d + x_axis * arrow_length
                y_end_3d = origin_3d + y_axis * arrow_length
                z_end_3d = origin_3d + z_axis * arrow_length
                
                # Derive session_dir from session_result_dir
                # session_result_dir: dataset/{operation}/result/{session}
                # session_dir: dataset/{operation}/test/{session}
                result_parent = os.path.dirname(session_result_dir)  # dataset/{operation}/result
                operation_dir = os.path.dirname(result_parent)  # dataset/{operation}
                session_name = os.path.basename(session_result_dir)  # {session}
                session_dir = os.path.join(operation_dir, 'test', session_name)
                
                self.get_logger().info(f"  Looking for images in: {session_dir}")
                
                if not os.path.exists(session_dir):
                    self.get_logger().error(f"{rack_name} session data directory not found: {session_dir}")
                    validation_results[rack_name] = False
                    continue
                
                # Find all test images and their pose files
                test_img_dir = Path(session_dir)
                image_files = sorted(test_img_dir.glob("*.jpg"))
                
                if len(image_files) == 0:
                    self.get_logger().error(f"No images found in {session_dir}")
                    validation_results[rack_name] = False
                    continue
                
                self.get_logger().info(f"  📖 Found {len(image_files)} images for {rack_name}")
                
                # Create visualization for each image
                num_images = len(image_files)
                cols = min(3, num_images)
                rows = (num_images + cols - 1) // cols
                
                import matplotlib.pyplot as plt
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
                        self.get_logger().warn(f"  ⚠ Failed to load image: {img_file}")
                        ax.axis('off')
                        continue
                    
                    # Convert BGR to RGB for matplotlib
                    img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                    
                    # Load camera parameters for this view
                    pose_file = img_file.parent / f"{img_file.stem}_pose.json"
                    if not pose_file.exists():
                        self.get_logger().warn(f"  ⚠ Pose file not found: {pose_file}")
                        ax.axis('off')
                        continue
                    
                    try:
                        intrinsic, distortion, extrinsic = load_camera_params_from_json(str(pose_file))
                    except Exception as e:
                        self.get_logger().warn(f"  ⚠ Failed to load pose: {e}")
                        ax.axis('off')
                        continue
                    
                    # Extract camera parameters
                    fx = intrinsic[0, 0]
                    fy = intrinsic[1, 1]
                    cx = intrinsic[0, 2]
                    cy = intrinsic[1, 2]
                    
                    # extrinsic is already base2cam (world to camera transformation)
                    base2cam = extrinsic
                    
                    # Function to project 3D point to 2D
                    def project_3d_to_2d(point_3d_base):
                        """Project a 3D point in base frame to 2D image coordinates"""
                        point_3d_homog = np.array([point_3d_base[0], point_3d_base[1], point_3d_base[2], 1.0])
                        point_3d_cam = base2cam @ point_3d_homog
                        
                        x_cam = point_3d_cam[0]
                        y_cam = point_3d_cam[1]
                        z_cam = point_3d_cam[2]
                        
                        if z_cam <= 0:
                            return None  # Point is behind camera
                        
                        # Apply distortion and projection
                        if distortion is not None and np.any(distortion != 0):
                            # Normalize coordinates
                            x_norm = x_cam / z_cam
                            y_norm = y_cam / z_cam
                            
                            # Apply distortion
                            k1, k2, p1, p2, k3 = distortion.flatten()[:5]
                            r2 = x_norm**2 + y_norm**2
                            
                            radial = 1 + k1*r2 + k2*r2**2 + k3*r2**3
                            x_distorted = x_norm * radial + 2*p1*x_norm*y_norm + p2*(r2 + 2*x_norm**2)
                            y_distorted = y_norm * radial + p1*(r2 + 2*y_norm**2) + 2*p2*x_norm*y_norm
                            
                            # Convert to pixel coordinates
                            u = fx * x_distorted + cx
                            v = fy * y_distorted + cy
                        else:
                            # No distortion
                            u = fx * (x_cam / z_cam) + cx
                            v = fy * (y_cam / z_cam) + cy
                        
                        return (u, v)
                    
                    # Project origin and axis endpoints
                    origin_2d = project_3d_to_2d(origin_3d)
                    x_end_2d = project_3d_to_2d(x_end_3d)
                    y_end_2d = project_3d_to_2d(y_end_3d)
                    z_end_2d = project_3d_to_2d(z_end_3d)
                    
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
                    
                    self.get_logger().info(f"    ✓ Drawn coordinate system on {img_file.name}")
                
                # Hide unused subplots
                for idx in range(num_images, len(axes)):
                    axes[idx].axis('off')
                
                # Add overall title
                origin_label = f"{rack_name}_point1" if rack_name == 'rack2' else "wobj origin (rack1_point1)"
                fig.suptitle(f'Wobj Coordinate System Validation - {rack_name.upper()}\n'
                            f'Origin: {origin_label} | Red: X-axis | Green: Y-axis | Blue: Z-axis',
                            fontsize=14, fontweight='bold', y=0.98)
                
                # Add coordinate system information as text
                x_axis_length = coord_system['axes']['x_axis']['length']
                z_axis_length = coord_system['axes']['z_axis']['length']
                info_text = (
                    f"Wobj Coordinate System Properties:\n"
                    f"Origin (display): ({origin_3d[0]:.4f}, {origin_3d[1]:.4f}, {origin_3d[2]:.4f}) m\n"
                    f"X-axis: rack1_point1→rack1_point2, length: {x_axis_length:.4f}m\n"
                    f"Z-axis: rack1_point1→rack2_point1, length: {z_axis_length:.4f}m\n"
                    f"Arrow length: {arrow_length:.3f}m"
                )
                
                fig.text(0.02, 0.02, info_text, fontsize=9,
                        bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                        verticalalignment='bottom')
                
                # Save figure to wobj result directory (if verbose)
                if verbose:
                    wobj_result_dir = self._setup_result_directory('wobj')
                    output_path = os.path.join(wobj_result_dir, f'wobj_coordinate_system_validation_{rack_name}.jpg')
                    plt.tight_layout()
                    plt.savefig(output_path, dpi=150, bbox_inches='tight')
                    self.get_logger().info(f"  💾 {rack_name} wobj frame visualization saved to: {output_path}")
                    
                    # Also save a copy to session result directory for reference
                    session_output_path = os.path.join(session_result_dir, f'wobj_coordinate_system_validation_{rack_name}.jpg')
                    plt.savefig(session_output_path, dpi=150, bbox_inches='tight')
                    self.get_logger().info(f"  💾 Copy also saved to {rack_name} session: {session_output_path}")
                
                # Close plot to free memory
                plt.close(fig)
                
                validation_results[rack_name] = True
            
            # Summary
            self.get_logger().info(f"\n✓ Wobj coordinate system validation completed!")
            all_success = all(validation_results.values())
            return all_success
            
        except Exception as e:
            self.get_logger().error(f"Validation error: {e}")
            traceback.print_exc()
            return False


def main():
    """
    Main function for testing URLocateRack class
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='URLocateRack - Specialized class for rack-related robot operations')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot')
    parser.add_argument('--camera-topic', type=str, default='/ur15_camera/image_raw',
                       help='ROS topic name for camera images')
    parser.add_argument('--operation-name', type=str, default='rack1',
                       help='Name of the operation for data directory')
    parser.add_argument('--camera-params-path', type=str, default='../temp/ur15_cam_calibration_result/ur15_camera_parameters',
                       help='Path to camera calibration parameters directory')
    parser.add_argument('--verbose', action='store_true', default=True,
                       help='Save validation images and error logs to disk')
    
    args = parser.parse_args()
    
    # Initialize rclpy
    import rclpy
    rclpy.init()
    
    try:
        # Create URLocateRack instance
        ur_locate_rack = URLocateRack(
            robot_ip=args.robot_ip,
            robot_port=args.robot_port,
            camera_topic=args.camera_topic,
            operation_name=args.operation_name,
            camera_params_path=args.camera_params_path,
            verbose=args.verbose
        )
        
        ur_locate_rack.get_logger().info("URLocateRack initialized successfully!")
        
        # Execute capture_data method
        ur_locate_rack.get_logger().info("Starting data capture for rack1 and rack2")
        
        results = ur_locate_rack.capture_data()
        
        # Check if all operations completed successfully
        if results and all(results.values()):
            ur_locate_rack.get_logger().info("✓ All operations completed successfully!")
            
            # Build wobj coordinate system from rack1 and rack2 results
            ur_locate_rack.get_logger().info("\n" + "="*80)
            ur_locate_rack.get_logger().info("Building wobj coordinate system...")
            ur_locate_rack.get_logger().info("="*80)
            
            coord_system = ur_locate_rack.perform_wobj_frame_building()
            
            if coord_system:
                ur_locate_rack.get_logger().info("✓ Wobj coordinate system built successfully!")
            else:
                ur_locate_rack.get_logger().error("✗ Failed to build wobj coordinate system")
        else:
            ur_locate_rack.get_logger().error("✗ Some operations failed")
        
    except KeyboardInterrupt:
        print("\nShutting down URLocateRack...")
    except Exception as e:
        print(f"Error in main: {e}")
        traceback.print_exc()
    finally:
        # Cleanup
        if 'ur_locate_rack' in locals():
            ur_locate_rack.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
