#!/usr/bin/env python3
"""
URLocate - Extended robot camera capture and location system
Inherits from URCapture and adds location-specific functionality
"""

import os
import sys
import cv2
import json
import time
import math
import numpy as np
import threading
import argparse
from datetime import datetime
from pathlib import Path
from scipy.spatial.transform import Rotation as R

# Import the parent class
from ur_capture import URCapture

# Add ThirdParty robot_vision to path for Web API client
sys.path.append(os.path.join(os.path.dirname(__file__), 'ThirdParty', 'robot_vision'))

# Import Web API client (with error handling)
from core.positioning_3d_webapi import Positioning3DWebAPIClient, load_camera_params_from_json


# Robot control imports
from ur15_robot_arm.ur15 import UR15Robot

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Robot status imports
from robot_status.client_utils import RobotStatusClient


class URLocate(URCapture):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, 
                 camera_topic="/ur15_camera/image_raw", operation_name="test_operation", 
                 camera_params_path="../temp/ur15_cam_calibration_result/ur15_camera_parameters",
                 verbose=True):
        """
        Initialize URLocate class for robot camera capture and location tasks
        
        Args:
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
            camera_topic (str): ROS topic name for camera images
            operation_name (str): Name of the operation for data directory
            camera_params_path (str): Path to camera calibration parameters directory
            verbose (bool): If True, saves validation images and error logs to disk
        """
        # Call parent class constructor
        super().__init__(
            robot_ip=robot_ip,
            robot_port=robot_port,
            camera_topic=camera_topic,
            operation_name=operation_name,
            camera_params_path=camera_params_path
        )
        
        # Update ROS node name for URLocate
        self.get_logger().info("URLocate node initialized")
        
        # ===================== Other variables =====================
        # Additional URLocate-specific attributes
        self.ref_img = None
        self.ref_keypoints = None 
        self.ref_pose = None
        
        # Template points for Fitting mode (local coordinates)
        # Define all rack corner points in local coordinate system
        self.all_template_points = {
            "rack1": [
                {
                    "name": "GB200_Rack_Lower_Left_Corner",
                    "x": 0,
                    "y": 0,
                    "z": 0.0
                }
            ],
            "rack2": [
                {
                    "name": "GB200_Rack_Lower_Right_Corner",
                    "x": 0.55,
                    "y": 0,
                    "z": 0.0
                }
            ],
            "rack3": [
                {
                    "name": "GB200_Rack_Top_Left_Corner",
                    "x": 0,
                    "y": 0,
                    "z": 2.145
                }
            ],
            "rack4": [
                {
                    "name": "GB200_Rack_Top_Right_Corner",
                    "x": 0.55,
                    "y": 0,
                    "z": 2.145
                }
            ]
        }
        
        # Select template points based on operation_name
        self.template_points = self.all_template_points.get(self.operation_name, None)
        
        # Store verbose flag
        self.verbose = verbose

        # Collection movement offsets (in tcp coordinate system, unit: meters)
        self.movements = {
            "movement1": [0, 0, 0, 0, 0, 0],         # No offset
            "movement2": [0.05, 0, 0, 0, 0, 0],      # X+1cm
            "movement3": [-0.05, 0, 0, 0, 0, 0],     # X-1cm
            "movement4": [0, 0.05, 0, 0, 0, 0],      # Y+1cm
            "movement5": [0, -0.05, 0, 0, 0, 0]      # Y-1cm
        }

        # ===================== Instances =====================
        self.positioning_client = None
        self.robot_status_client = None

        # ===================== Initialize =====================
        # Set up directories and reference data paths
        self._setup_directories()
        # Load reference data automatically
        self._load_reference_data()      
        self._init_positioning_client()
        self._init_robot_status_client()

    def _setup_directories(self):
        """
        Set up required directories for URLocate operations
        """
        # Result directory path for storing location results
        self.result_dir = os.path.join(self.data_parent_dir, "result")
        
        # Create result directory if it doesn't exist
        if not os.path.exists(self.result_dir):
            os.makedirs(self.result_dir, exist_ok=True)
            self.get_logger().info(f"✓ Created result directory: {self.result_dir}")
        else:
            self.get_logger().info(f"✓ Result directory exists: {self.result_dir}")
        
        # Reference data file paths with specified names
        self.ref_img_path = os.path.join(self.data_parent_dir, 'ref_img_1.jpg')
        self.ref_keypoints_path = os.path.join(self.data_parent_dir, 'ref_img_1.json') 
        self.ref_pose_path = os.path.join(self.data_parent_dir, 'ref_img_1_pose.json')

    def _load_reference_data(self):
        """
        Load reference data (image, keypoints, and pose) automatically during initialization
        """
        try:
            # Load reference image
            if os.path.exists(self.ref_img_path):
                self.ref_img = cv2.imread(self.ref_img_path)
                if self.ref_img is not None:
                    self.get_logger().info(f"✓ Reference image loaded: {self.ref_img_path}")
                else:
                    self.get_logger().warn(f"✗ Failed to load reference image: {self.ref_img_path}")
            else:
                self.get_logger().warn(f"✗ Reference image not found: {self.ref_img_path}")
            
            # Load reference keypoints
            if os.path.exists(self.ref_keypoints_path):
                with open(self.ref_keypoints_path, 'r') as f:
                    self.ref_keypoints = json.load(f)
                self.get_logger().info(f"✓ Reference keypoints loaded: {self.ref_keypoints_path}")
            else:
                self.get_logger().warn(f"✗ Reference keypoints not found: {self.ref_keypoints_path}")
            
            # Load reference pose
            if os.path.exists(self.ref_pose_path):
                with open(self.ref_pose_path, 'r') as f:
                    self.ref_pose = json.load(f)
                self.get_logger().info(f"✓ Reference pose loaded: {self.ref_pose_path}")
            else:
                self.get_logger().warn(f"✗ Reference pose not found: {self.ref_pose_path}")
                
        except Exception as e:
            self.get_logger().error(f"Error loading reference data: {e}")

    def _init_positioning_client(self, service_url="http://localhost:8004", max_retries=5, retry_delay=2):
        """
        Initialize the 3D Positioning Web API Client with retry logic
        
        Args:
            service_url (str): URL of the positioning service
            max_retries (int): Maximum number of connection attempts
            retry_delay (int): Seconds to wait between retries
        """
        # Check if the Web API client is available
        if Positioning3DWebAPIClient is None:
            self.get_logger().warn("✗ Positioning Web API client not available (import failed)")
            self.positioning_client = None
            return
        
        for attempt in range(max_retries):
            try:
                self.positioning_client = Positioning3DWebAPIClient(service_url=service_url)
                
                # Check service health
                health = self.positioning_client.check_health()
                if health.get('success'):
                    self.get_logger().info(f"Positioning service connected to: {service_url}")
                    ffpp_connected = health.get('status', {}).get('ffpp_server', {}).get('connected', False)
                    refs_loaded = health.get('status', {}).get('references', {}).get('loaded', 0)
                    self.get_logger().info(f"FFPP server connected: {ffpp_connected}")
                    self.get_logger().info(f"{refs_loaded} reference data has been loaded")
                    
                    # List all loaded references
                    refs = self.positioning_client.list_references()
                    if refs.get('success'):
                        ref_list = list(refs.get('references', {}).keys())
                        if ref_list:
                            self.get_logger().info(f"Available reference data: {ref_list}")
                        else:
                            self.get_logger().info("No reference data currently loaded")
                    return  # Successfully connected
                else:
                    raise Exception(health.get('error', 'Service health check failed'))
                    
            except Exception as e:
                if attempt < max_retries - 1:
                    self.get_logger().warn(f"Positioning service not ready (attempt {attempt + 1}/{max_retries}): {e}")
                    self.get_logger().info(f"Retrying in {retry_delay} seconds...")
                    time.sleep(retry_delay)
                else:
                    self.get_logger().error(f"Failed to connect to positioning service after {max_retries} attempts: {e}")
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

    # =================================== functions for 3d positioning ===================================
    def auto_collect_data(self, save_dir=None, robot=None, session_id=None):
        """
        Override parent's auto_collect_data to add real-time upload functionality.
        
        Args:
            save_dir: Directory to save captured images
            robot: Robot instance to use
            session_id: Optional session ID for uploading images in real-time
            
        Returns:
            str: Path to session directory if successful, False otherwise
        """
        robot = robot or self.robot
        if robot is None:
            self.get_logger().error("Error: Robot not initialized")
            return False
        
        if save_dir is None:
            # Create session directory with auto-incremented number
            session_dir = self._create_session_directory()
            save_dir = session_dir

        self.get_logger().info(">>> Starting Auto Data Collection with Real-time Upload..")

        try:
            if not self._movej_to_collect_position(robot):
                self.get_logger().error("✗ Failed to move to collect position, aborting data collection")
                return False
            
            # Get current TCP pose as reference
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
            if not os.path.exists(save_dir):
                os.makedirs(save_dir)
                self.get_logger().info(f"Created save directory: {save_dir}")
            
            # Wait for camera image to be available
            self.get_logger().info("Waiting for camera image...")
            max_wait_time = 10.0
            start_time = time.time()
            
            while self.latest_image is None and (time.time() - start_time) < max_wait_time:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_image is None:
                self.get_logger().error("Error: No camera image received within timeout.")
                return False
            
            self.get_logger().info("✓ Camera image available, starting data collection...")
            
            success_count = 0
            upload_count = 0
            
            for movement in movements:
                try:
                    self.get_logger().info(f"\n--- {movement['name']} ---")
                    
                    # Use offset directly for move_tcp
                    offset = movement['offset']
                    self.get_logger().info(f"Move TCP by offset: {[f'{p:.4f}' for p in offset]}")
                    
                    # Move robot to target position
                    move_result = robot.move_tcp(offset, a=0.1, v=0.05)
                    
                    if move_result != 0:
                        self.get_logger().warn(f"Movement failed for {movement['name']} (result: {move_result})")
                        continue
                    
                    # Wait for robot to settle
                    time.sleep(2.0)
                    
                    # Capture image and pose
                    img_filename = f"{movement['index']}.jpg"
                    pose_filename = f"{movement['index']}_pose.json"
                    
                    if self.capture_image_and_pose(save_dir, img_filename, pose_filename, robot=robot):
                        self.get_logger().info(f"✓ Successfully captured data for {movement['name']}")
                        success_count += 1
                        
                        # If session_id is provided, upload immediately
                        if session_id is not None and self.positioning_client is not None:
                            img_path = os.path.join(save_dir, img_filename)
                            pose_path = os.path.join(save_dir, pose_filename)
                            
                            try:
                                # Load image
                                image = cv2.imread(img_path)
                                if image is None:
                                    self.get_logger().warn(f"  ⚠️  Failed to load image for upload: {img_path}")
                                else:
                                    # Load camera parameters
                                    intrinsic, distortion, extrinsic = load_camera_params_from_json(pose_path)
                                    
                                    # Upload view with reference_name
                                    result = self.positioning_client.upload_view(
                                        session_id=session_id,
                                        reference_name=self.operation_name,
                                        image=image,
                                        intrinsic=intrinsic,
                                        distortion=distortion,
                                        extrinsic=extrinsic
                                    )
                                    
                                    if result.get('success'):
                                        upload_count += 1
                                        self.get_logger().info(f"   ✓ View {upload_count} uploaded, queue position: {result.get('queue_position', 'N/A')}")
                                    else:
                                        self.get_logger().error(f"   ✗ Upload failed: {result.get('error')}")
                            except Exception as e:
                                self.get_logger().error(f"   ✗ Error uploading image: {e}")
                        
                        time.sleep(0.5)
                    else:
                        self.get_logger().warn(f"✗ Failed to capture data for {movement['name']}")
                    
                    # Return to original position before next movement
                    self.get_logger().info(f"Returning to start position...")
                    return_result = robot.movel(current_tcp_pose, a=0.1, v=0.05)
                    
                    if return_result == 0:
                        self.get_logger().info("✓ Returned to start position")
                    else:
                        self.get_logger().warn(f"✗ Failed to return to start position (result: {return_result})")
                    
                    time.sleep(0.5)  # Wait for movement to complete
                    
                except Exception as e:
                    self.get_logger().error(f"Error during {movement['name']} movement: {e}")
                    continue
            
            # Final return to original position (for safety)
            self.get_logger().info(f"\n--- Final return to original position ---")
            return_result = robot.movel(current_tcp_pose, a=0.1, v=0.05)
            
            time.sleep(0.5)

            if return_result == 0:
                self.get_logger().info("✓ Successfully returned to original position")
            else:
                self.get_logger().warn(f"✗ Failed to return to original position (result: {return_result})")
            
            self.get_logger().info(f"\nData collection completed: {success_count}/{len(movements)} successful")
            if session_id is not None:
                self.get_logger().info(f"Upload completed: {upload_count}/{success_count} uploaded")
            self.get_logger().info("="*60)
            
            # Return save_dir if all movements were successful, False otherwise
            return save_dir if success_count == len(movements) else False
            
        except Exception as e:
            self.get_logger().error(f"Error during auto data collection: {e}")
            return False
    
    def upload_reference_data_to_ffpp_web(self):
        """
        Upload reference images toffppweb, which is the first step before performing 3D triangulation.
        
        Returns:
            bool: True if upload successful, False otherwise
        """
        if self.positioning_client is None:
            self.get_logger().error("Positioning client not initialized")
            return False
        
        try:
            self.get_logger().info(">>> Uploading reference data to ffpp web...")
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

    def perform_3d_positioning(self):
        """
        Complete 3D positioning workflow: upload references, collect data, and estimate position.
        
        This method orchestrates the full positioning process:
        1. Check and upload reference data if needed (smart upload)
        2. Automatically collect camera images from multiple viewpoints
        3. Estimate 3D position using triangulation (with minimal waiting)
        
        Returns:
            dict: Triangulation result containing 3D points and errors, or None if failed
        """
        try:
            # Step 1: Check if reference exists, upload only if needed
            msg = "Step 1: Checking reference availability..."
            self.get_logger().info(msg)
            print(msg)
            
            refs = self.positioning_client.list_references()
            reference_exists = refs.get('success') and self.operation_name in refs.get('references', {})
            
            if not reference_exists:
                msg = f"Reference '{self.operation_name}' not found, uploading to FFPP server..."
                self.get_logger().info(msg)
                print(msg)
                
                upload_success = self.upload_reference_data_to_ffpp_web()
                if not upload_success:
                    msg = "✗ Failed to upload reference data. Aborting positioning."
                    self.get_logger().error(msg)
                    print(msg)
                    return None
                
                msg = "✓ Reference data uploaded successfully!"
                self.get_logger().info(msg)
                print(msg)
            else:
                msg = f"✓ Reference '{self.operation_name}' already loaded, skipping upload"
                self.get_logger().info(msg)
                print(msg)
            
            # Step 2: Initialize session for real-time upload
            msg = "Step 2: Initializing session for data collection..."
            self.get_logger().info(msg)
            print(msg)
            
            session_result = self.positioning_client.init_session()
            if not session_result.get('success'):
                msg = f"✗ Failed to initialize session: {session_result.get('error')}"
                self.get_logger().error(msg)
                print(msg)
                return None
            
            session_id = session_result['session_id']
            msg = f"✓ Web session created: {session_id}"
            self.get_logger().info(msg)
            print(msg)
            
            # Step 3: Collect images and upload in real-time
            msg = "Step 3: Collecting and uploading camera images..."
            self.get_logger().info(msg)
            print(msg)
            
            # Call overridden auto_collect_data with session_id for real-time upload
            self.session_dir = self.auto_collect_data(session_id=session_id)
            if self.session_dir is False or self.session_dir is None:
                msg = "✗ Failed to collect camera data. Aborting positioning."
                self.get_logger().error(msg)
                print(msg)
                # Terminate session on failure
                self.positioning_client.terminate_session(session_id)
                return None
            
            msg = "✓ Data collection and upload completed successfully!"
            self.get_logger().info(msg)
            print(msg)
            
            # Step 4: Get triangulation/fitting result (minimal wait due to parallel tracking)
            if self.template_points:
                msg = f"Step 4: Getting fitting results with {len(self.template_points)} template points..."
                self.get_logger().info(msg)
                print(msg)
                msg = f"  → Fitting mode: estimating local2world transformation"
                self.get_logger().info(msg)
                print(msg)
            else:
                msg = "Step 4: Getting triangulation results..."
                self.get_logger().info(msg)
                print(msg)
            
            # Most views should already be tracked due to parallel processing
            # Only need short timeout for any remaining views
            result = self.positioning_client.get_result(
                session_id, 
                template_points=self.template_points,  # None for standard mode, list for fitting mode
                timeout=10000
            )
            
            if not result.get('success'):
                msg = f"✗ Failed to get result: {result.get('error')}"
                self.get_logger().error(msg)
                print(msg)
                # Terminate session
                self.positioning_client.terminate_session(session_id)
                return None
            
            # Check if we got the final result or timed out
            if 'result' not in result:
                if result.get('timeout'):
                    msg = "\n✗ Timeout waiting for triangulation"
                    self.get_logger().error(msg)
                    print(msg)
                else:
                    session_info = result.get('session', {})
                    session_status = session_info.get('status')
                    if session_status == 'failed':
                        msg = f"\n✗ Session failed: {session_info.get('error_message', 'Unknown error')}"
                        self.get_logger().error(msg)
                        print(msg)
                    else:
                        msg = f"\n✗ Triangulation not completed (status: {session_status})"
                        self.get_logger().error(msg)
                        print(msg)
                # Terminate session
                self.positioning_client.terminate_session(session_id)
                return None
            
            # Display results based on mode
            if self.template_points:
                msg = "✓ Fitting completed!"
                self.get_logger().info(msg)
                print(msg)
            else:
                msg = "✓ Triangulation completed!"
                self.get_logger().info(msg)
                print(msg)
            
            triangulation_result = result['result']
            points_3d = np.array(triangulation_result['points_3d'])
            mean_error = triangulation_result['mean_error']
            processing_time = triangulation_result.get('processing_time', 0)
            views_data = result.get('views', [])
            
            msg = f"   Number of 3D points: {len(points_3d)}"
            self.get_logger().info(msg)
            print(msg)
            msg = f"   Mean reprojection error: {mean_error:.3f} pixels"
            self.get_logger().info(msg)
            print(msg)
            msg = f"   Processing time: {processing_time:.2f} seconds"
            self.get_logger().info(msg)
            print(msg)
            msg = f"   Number of views: {len(views_data)}"
            self.get_logger().info(msg)
            print(msg)
            
            # Display local2world transformation if in fitting mode
            if self.template_points and 'local2world' in triangulation_result:
                local2world = triangulation_result['local2world']
                msg = "\n   Local-to-World Transformation Matrix:"
                self.get_logger().info(msg)
                print(msg)
                for i, row in enumerate(local2world):
                    row_str = f"    [{row[0]:8.4f}  {row[1]:8.4f}  {row[2]:8.4f}  {row[3]:8.4f}]"
                    self.get_logger().info(row_str)
                    print(row_str)
            
            # Terminate session
            msg = "Step 5: Terminating session..."
            self.get_logger().info(msg)
            print(msg)
            term_result = self.positioning_client.terminate_session(session_id)
            if term_result.get('success'):
                msg = f"✓ Session {session_id} terminated and cleaned up"
                self.get_logger().info(msg)
                print(msg)
            else:
                msg = f"⚠️  Failed to terminate session: {term_result.get('error')}"
                self.get_logger().warn(msg)
                print(msg)
            
            msg = "✓ 3D positioning completed successfully!"
            self.get_logger().info(msg)
            print(msg)
            
            # Step 6: Save results to result directory
            msg = "Step 6: Saving 3D positioning results..."
            self.get_logger().info(msg)
            print(msg)
            
            # Get session name and create result directory for this session
            session_name = os.path.basename(self.session_dir)
            self.session_result_dir = os.path.join(self.result_dir, session_name)
            os.makedirs(self.session_result_dir, exist_ok=True)
            
            # Prepare result data to save
            result_data = {
                'timestamp': datetime.now().isoformat(),
                'points_3d': result['result']['points_3d'],
                'mean_error': result['result']['mean_error'],
                'processing_time': result['result'].get('processing_time', 0),
                'views': result.get('views', []),
                'fitting_mode': self.template_points is not None
            }
            
            # Save local2world transformation if in fitting mode
            if self.template_points and 'local2world' in result['result']:
                result_data['local2world'] = result['result']['local2world']
            
            # Save to JSON file
            positioning_result_file_path = os.path.join(self.session_result_dir, '3d_positioning_result.json')
            with open(positioning_result_file_path, 'w') as f:
                json.dump(result_data, f, indent=2)
            
            msg = f"✓ Results saved to: {positioning_result_file_path}"
            self.get_logger().info(msg)
            print(msg)
            
            # Save points_3d to robot_status
            if self.robot_status_client:
                try:
                    if self.robot_status_client.set_status(self.operation_name, 'points_3d', result['result']['points_3d']):
                        msg = f"✓ points_3d saved to robot_status (namespace: {self.operation_name})"
                        self.get_logger().info(msg)
                        print(msg)
                    else:
                        msg = "Failed to save points_3d to robot_status"
                        self.get_logger().warning(msg)
                        print(msg)
                except Exception as e:
                    msg = f"Error saving points_3d to robot_status: {e}"
                    self.get_logger().warning(msg)
                    print(msg)
            
            # Step 7: Validate positioning results
            msg = "Step 7: Validating positioning results..."
            self.get_logger().info(msg)
            print(msg)
            
            validation_success = self.validate_positioning_results(self.session_dir, result, verbose=self.verbose)
            if validation_success:
                msg = "✓ wobj validation completed successfully!"
                self.get_logger().info(msg)
                print(msg)
            else:
                msg = "⚠ wobj validation completed with warnings"
                self.get_logger().warn(msg)
                print(msg)
            
            return result
            
        except Exception as e:
            msg = f"Error during 3D positioning: {e}"
            self.get_logger().error(msg)
            print(msg)
            return None

    def validate_positioning_results(self, session_dir, result_data, verbose=True):
        """
        Validate 3D positioning results by visualizing tracked and reprojected keypoints.
        
        Creates a visualization showing:
        - Red circles: Original tracked 2D keypoints from views
        - Green circles: Reprojected 3D points back to each view
        
        Args:
            session_dir (str): Path to session directory containing test images
            result_data (dict): Result data containing 3D points and views information
            verbose (bool): If True, saves validation images and error logs to disk
            
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
                
                # Get tracked 2D keypoints (red circles)
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
                
                # extrinsic is already base2cam (world to camera transformation)
                # Convert to rvec and tvec for cv2.projectPoints
                base2cam = extrinsic
                rotation_matrix = base2cam[:3, :3]
                translation_vector = base2cam[:3, 3]
                rvec, _ = cv2.Rodrigues(rotation_matrix)
                tvec = translation_vector.reshape(3, 1)
                
                # Reproject 3D points using cv2.projectPoints
                dist_coeffs = distortion if distortion is not None else np.zeros(5, dtype=np.float32)
                projected_points, _ = cv2.projectPoints(
                    points_3d.astype(np.float32),
                    rvec,
                    tvec,
                    intrinsic,
                    dist_coeffs
                )
                projected_points = projected_points.reshape(-1, 2)
                
                # Plot reprojected points and calculate errors
                for pt_idx, (u_reproj, v_reproj) in enumerate(projected_points):
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
            
            # Save figure to session result directory (if verbose)
            if verbose:
                output_path = os.path.join(self.session_result_dir, 'positioning_result_reprojection.jpg')
                plt.tight_layout()
                plt.savefig(output_path, dpi=150, bbox_inches='tight')
                self.get_logger().info(f"\n💾 Validation visualization saved to: {output_path}")
            
            # Close plot to free memory
            plt.close(fig)
            
            # Save detailed reprojection errors to JSON (if verbose)
            if verbose and reprojection_errors:
                error_log_path = os.path.join(self.session_result_dir, 'positioning_result_reprojection_report.json')
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
            self.get_logger().error(f"Error during validation: {e}")
            import traceback
            traceback.print_exc()
            return False


def main():
    """
    Main function for testing URLocate class
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='URLocate - Extended robot camera capture and location system')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot')
    parser.add_argument('--camera-topic', type=str, default='/ur15_camera/image_raw',
                       help='ROS topic name for camera images')
    parser.add_argument('--operation-name', type=str, default='test_operation',
                       help='Name of the operation for data directory')
    parser.add_argument('--camera-params-path', type=str, default='../temp/ur15_cam_calibration_result/ur15_camera_parameters',
                       help='Path to camera calibration parameters directory')
    parser.add_argument('--no-verbose', dest='verbose', action='store_false', default=True,
                       help='Do not save validation results to disk (default: save results)')
    
    args = parser.parse_args()
    
    # Initialize ROS2 (only if not already initialized)
    if not rclpy.ok():
        rclpy.init()
    
    try:
        # Initialize URLocate instance with command line arguments
        ur_locate = URLocate(
            robot_ip=args.robot_ip,
            robot_port=args.robot_port,
            camera_topic=args.camera_topic,
            operation_name=args.operation_name,
            camera_params_path=args.camera_params_path,
            verbose=args.verbose
        )
        
        # Check if robot was initialized successfully
        if ur_locate.robot is None or not ur_locate.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_locate)
        
        # Flag to control executor spinning
        stop_spinning = threading.Event()
        
        def spin_executor():
            """Spin executor until stop signal is received."""
            while not stop_spinning.is_set() and rclpy.ok():
                executor.spin_once(timeout_sec=0.1)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=spin_executor, daemon=False)
        executor_thread.start()
        
        print('URLocate node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        try:
            # Perform complete 3D positioning workflow
            result = ur_locate.perform_3d_positioning()
            
            if result:
                # Send points_3d to draw_points workspace
                if ur_locate.robot_status_client:
                    try:
                        points_3d = result['result']['points_3d']
                        if ur_locate.robot_status_client.set_status('ur15', 'last_points_3d', points_3d):
                            print("✓ points_3d sent to ur15 workspace")
                        else:
                            print("⚠️  Failed to send last_points_3d to ur15 workspace")
                    except Exception as e:
                        print(f"⚠️  Error sending last_points_3d to ur15 workspace: {e}")
                
            else:
                print("\n✗ 3D Positioning failed!")
                
        except KeyboardInterrupt:
            print("\n🛑 Operation interrupted by user")
        except Exception as e:
            print(f"\n❌ Error during execution: {e}")
        
        finally:
            # Always disconnect robot in finally block
            if ur_locate.robot is not None:
                print("Disconnecting robot...")
                ur_locate.robot.close()
                print("✓ Robot disconnected")
            
            # Proper cleanup sequence to avoid threading issues
            try:
                # Signal executor thread to stop
                stop_spinning.set()
                
                # Wait for executor thread to finish
                if executor_thread.is_alive():
                    executor_thread.join(timeout=3.0)
                    if executor_thread.is_alive():
                        print("⚠️  Executor thread did not finish in time")
                
                # Shutdown executor
                executor.shutdown()
                
                # Destroy the node
                try:
                    ur_locate.destroy_node()
                except Exception as e:
                    print(f"⚠️  Warning during node destruction: {e}")
            except Exception as e:
                print(f"⚠️  Warning during executor cleanup: {e}")
    
    finally:
        # Shutdown ROS2 (only if we initialized it)
        if rclpy.ok():
            try:
                rclpy.shutdown()
            except Exception as e:
                print(f"⚠️  Warning during ROS2 shutdown: {e}")


if __name__ == "__main__":
    main()