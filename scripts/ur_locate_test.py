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
        
        # Store verbose flag
        self.verbose = verbose
        
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
                                    
                                    # Upload view
                                    result = self.positioning_client.upload_view(
                                        session_id=session_id,
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

    def estimate_3d_position_of_test_image(self, test_image_path, operation_name):
        """
        Perform 3D triangulation using multiple camera views from /test/session folder in operation directory.
        
        This function follows the standard workflow:
        1. Find and validate test images
        2. Check service health
        3. Verify reference image exists
        4. Initialize session
        5. Upload camera views
        6. Wait for triangulation result
        7. Terminate session and cleanup
        
        Args:
            test_image_path (str): Path to directory containing test images and pose files
                                   Expected files: image.jpg and image_pose.json for each view
            operation_name (str): Name of the reference to use for triangulation
            
        Returns:
            dict: Triangulation result or None if failed
        """
        if self.positioning_client is None:
            self.get_logger().error("Positioning client not initialized")
            return None
        
        try:
            # Step 1: Find and validate test images
            self.get_logger().info(">>> I: Finding test images...")
            
            test_img_dir = Path(test_image_path)
            
            if not test_img_dir.exists():
                self.get_logger().error(f"✗ Test image directory not found: {test_image_path}")
                return None
            
            image_files = sorted(test_img_dir.glob("*.jpg"))
            if not image_files:
                self.get_logger().error(f"✗ No images found in {test_image_path}")
                return None
            
            self.get_logger().info(f"✓ Found {len(image_files)} images in test image directory: {test_image_path}")
            
            # Step 2: Check service health
            self.get_logger().info(">>> II: Checking service health...")
            
            health = self.positioning_client.check_health()
            if not health.get('success'):
                self.get_logger().error(f"✗ Service not available: {health.get('error')}")
                return None
            
            self.get_logger().info("✓ Positioning service is running normally")
            
            # Step 3: Check if reference exists
            self.get_logger().info(f">>> III: Checking reference data of '{operation_name}'...")
            
            refs = self.positioning_client.list_references()
            if not refs.get('success') or operation_name not in refs.get('references', {}):
                self.get_logger().error(f"✗ Reference '{operation_name}' not found")
                available_refs = list(refs.get('references', {}).keys())
                self.get_logger().error(f" Available references: {available_refs}")
                return None
            
            self.get_logger().info(f"✓ Successfully, reference data to '{operation_name}' is loaded")
            
            session_id = None
            try:
                # Step 4: Initialize session
                self.get_logger().info(f">>> IV: Initializing a web session to execute...")
                
                session_result = self.positioning_client.init_session(reference_name=operation_name)
                
                if not session_result.get('success'):
                    self.get_logger().error(f"✗ Failed to initialize session: {session_result.get('error')}")
                    return None
                
                session_id = session_result['session_id']
                self.get_logger().info(f"✓ Web session created: {session_id}")
                
                # Step 5: Upload camera views
                self.get_logger().info(f">>> V: Uploading {len(image_files)} camera views from {test_image_path}...")
                
                for idx, img_file in enumerate(image_files):
                    # Find corresponding pose file
                    pose_file = img_file.parent / f"{img_file.stem}_pose.json"
                    
                    if not pose_file.exists():
                        self.get_logger().warn(f"  ⚠️  Skipping {img_file.name}: pose file not found")
                        continue
                    
                    # Load image
                    image = cv2.imread(str(img_file))
                    if image is None:
                        self.get_logger().warn(f"  ⚠️  Skipping {img_file.name}: failed to load image")
                        continue
                    
                    # Load camera parameters
                    try:
                        intrinsic, distortion, extrinsic = load_camera_params_from_json(str(pose_file))
                    except Exception as e:
                        self.get_logger().warn(f"  ⚠️  Skipping {img_file.name}: failed to load pose - {e}")
                        continue
                    
                    # Upload view              
                    result = self.positioning_client.upload_view(
                        session_id=session_id,
                        image=image,
                        intrinsic=intrinsic,
                        distortion=distortion,
                        extrinsic=extrinsic
                    )
                    
                    if result.get('success'):
                        self.get_logger().info(f"   ✓ View {idx+1}/{len(image_files)} uploaded, queue position: {result.get('queue_position', 'N/A')}")
                    else:
                        self.get_logger().error(f"   ✗ {result.get('error')}")
                
                # Step 6: Wait for triangulation result
                self.get_logger().info(">>> VI: Waiting for positioning results (timeout: 30s)...")
                
                result = self.positioning_client.get_result(session_id, timeout=30000)  # 30 seconds
                
                if not result.get('success'):
                    self.get_logger().error(f"✗ Failed to get result: {result.get('error')}")
                    return None
                
                # Check if we got the final result or timed out
                if 'result' not in result:
                    if result.get('timeout'):
                        self.get_logger().error("\n✗ Timeout waiting for triangulation")
                    else:
                        session_info = result.get('session', {})
                        session_status = session_info.get('status')
                        if session_status == 'failed':
                            self.get_logger().error(f"\n✗ Session failed: {session_info.get('error_message', 'Unknown error')}")
                        else:
                            self.get_logger().error(f"\n✗ Triangulation not completed (status: {session_status})")
                    return None
                
                self.get_logger().info("✓ Triangulation completed!")
                
                triangulation_result = result['result']
                points_3d = np.array(triangulation_result['points_3d'])
                mean_error = triangulation_result['mean_error']
                processing_time = triangulation_result.get('processing_time', 0)
                views_data = result.get('views', [])
                
                self.get_logger().info(f"   Number of 3D points: {len(points_3d)}")
                self.get_logger().info(f"   Mean reprojection error: {mean_error:.3f} pixels")
                self.get_logger().info(f"   Processing time: {processing_time:.2f} seconds")
                self.get_logger().info(f"   Number of views: {len(views_data)}")
                
                return result
                
            finally:
                # Step 7: Terminate session and cleanup
                if session_id is not None:
                    self.get_logger().info(">>> VII: Terminating session...")
                    
                    term_result = self.positioning_client.terminate_session(session_id)
                    if term_result.get('success'):
                        self.get_logger().info(f"✓ Session {session_id} terminated and cleaned up")
                    else:
                        self.get_logger().warn(f"⚠️  Failed to terminate session: {term_result.get('error')}")
                
        except Exception as e:
            self.get_logger().error(f"Error during triangulation: {e}")
            return None

    def perform_3d_positioning(self):
        """
        Complete 3D positioning workflow: upload references, collect data, and estimate position.
        
        This method orchestrates the full positioning process:
        1. Upload reference data to FFPP web service
        2. Automatically collect camera images from multiple viewpoints
        3. Estimate 3D position using triangulation
        
        Returns:
            dict: Triangulation result containing 3D points and errors, or None if failed
        """
        try:
            # Step 1: Upload reference data
            self.get_logger().info("Step 1: Uploading reference data to FFPP web service...")
            
            upload_success = self.upload_reference_data_to_ffpp_web()
            if not upload_success:
                self.get_logger().error("✗ Failed to upload reference data. Aborting positioning.")
                return None
            
            self.get_logger().info("✓ Reference data uploaded successfully!")
            
            # Step 2: Initialize session for real-time upload
            self.get_logger().info("Step 2: Initializing session for data collection...")
            
            session_result = self.positioning_client.init_session(reference_name=self.operation_name)
            if not session_result.get('success'):
                self.get_logger().error(f"✗ Failed to initialize session: {session_result.get('error')}")
                return None
            
            session_id = session_result['session_id']
            self.get_logger().info(f"✓ Web session created: {session_id}")
            
            # Step 3: Collect images and upload in real-time
            self.get_logger().info("Step 3: Collecting and uploading camera images...")
            
            # Call overridden auto_collect_data with session_id for real-time upload
            self.session_dir = self.auto_collect_data(session_id=session_id)
            if self.session_dir is False or self.session_dir is None:
                self.get_logger().error("✗ Failed to collect camera data. Aborting positioning.")
                # Terminate session on failure
                self.positioning_client.terminate_session(session_id)
                return None
            
            self.get_logger().info(f"✓ Data collection and upload completed successfully!")
            
            # Step 4: Wait for triangulation result
            self.get_logger().info("Step 4: Waiting for positioning results (timeout: 30s)...")
            
            result = self.positioning_client.get_result(session_id, timeout=30000)
            
            if not result.get('success'):
                self.get_logger().error(f"✗ Failed to get result: {result.get('error')}")
                # Terminate session
                self.positioning_client.terminate_session(session_id)
                return None
            
            # Check if we got the final result or timed out
            if 'result' not in result:
                if result.get('timeout'):
                    self.get_logger().error("\n✗ Timeout waiting for triangulation")
                else:
                    session_info = result.get('session', {})
                    session_status = session_info.get('status')
                    if session_status == 'failed':
                        self.get_logger().error(f"\n✗ Session failed: {session_info.get('error_message', 'Unknown error')}")
                    else:
                        self.get_logger().error(f"\n✗ Triangulation not completed (status: {session_status})")
                # Terminate session
                self.positioning_client.terminate_session(session_id)
                return None
            
            self.get_logger().info("✓ Triangulation completed!")
            
            triangulation_result = result['result']
            points_3d = np.array(triangulation_result['points_3d'])
            mean_error = triangulation_result['mean_error']
            processing_time = triangulation_result.get('processing_time', 0)
            views_data = result.get('views', [])
            
            self.get_logger().info(f"   Number of 3D points: {len(points_3d)}")
            self.get_logger().info(f"   Mean reprojection error: {mean_error:.3f} pixels")
            self.get_logger().info(f"   Processing time: {processing_time:.2f} seconds")
            self.get_logger().info(f"   Number of views: {len(views_data)}")
            
            # Terminate session
            self.get_logger().info("Step 5: Terminating session...")
            term_result = self.positioning_client.terminate_session(session_id)
            if term_result.get('success'):
                self.get_logger().info(f"✓ Session {session_id} terminated and cleaned up")
            else:
                self.get_logger().warn(f"⚠️  Failed to terminate session: {term_result.get('error')}")
            
            self.get_logger().info("✓ 3D positioning completed successfully!")
            
            # Step 6: Save results to result directory
            self.get_logger().info("Step 6: Saving 3D positioning results...")
            
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
                'views': result.get('views', [])
            }
            
            # Save to JSON file
            positioning_result_file_path = os.path.join(self.session_result_dir, '3d_positioning_result.json')
            with open(positioning_result_file_path, 'w') as f:
                json.dump(result_data, f, indent=2)
            
            self.get_logger().info(f"✓ Results saved to: {positioning_result_file_path}")
            
            # Save points_3d to robot_status
            if self.robot_status_client:
                try:
                    if self.robot_status_client.set_status(self.operation_name, 'points_3d', result['result']['points_3d']):
                        self.get_logger().info(f"✓ points_3d saved to robot_status (namespace: {self.operation_name})")
                    else:
                        self.get_logger().warning("Failed to save points_3d to robot_status")
                except Exception as e:
                    self.get_logger().warning(f"Error saving points_3d to robot_status: {e}")
            
            # Step 7: Validate positioning results
            self.get_logger().info("Step 7: Validating positioning results...")
            
            validation_success = self.validate_positioning_results(self.session_dir, result, verbose=self.verbose)
            if validation_success:
                self.get_logger().info("✓ wobj validation completed successfully!")
            else:
                self.get_logger().warn("⚠ wobj validation completed with warnings")
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error during 3D positioning: {e}")
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

    # =================================== functions for wobj frame building ===================================
    def perform_wobj_frame_building(self, session_result_dir=None, kp_index_for_wobj_x_axis=[0, 1]):
        """
        Build a wobj coordinate system based on 3D positioning keypoints.
        
        The coordinate system is defined as follows:
        - Origin: at keypoint[kp_index_for_wobj_x_axis[0]]
        - X-axis: keypoint[kp_index_for_wobj_x_axis[0]] → keypoint[kp_index_for_wobj_x_axis[1]] direction
        - Z-axis: positive direction aligns with base Z-axis (upward)
        - Y-axis: follows right-hand rule (Z × X)
        
        Args:
            session_result_dir (str): Path to session result directory (e.g., 'result/20231119_143022')
                                     If None, uses the most recent session
            kp_index_for_wobj_x_axis (list): [start_index, end_index] for X-axis definition
            
        Returns:
            dict: Coordinate system information or None if failed
        """
        self.get_logger().info(">>> Building wobj Coordinate System")
        
        try:
            # Determine session result directory
            if session_result_dir is None:
                # Use current session result directory if available
                if hasattr(self, 'session_result_dir') and self.session_result_dir:
                    session_result_dir = self.session_result_dir
                    self.get_logger().info(f"Using current session: {os.path.basename(session_result_dir)}")
                else:
                    # Find most recent session in result directory
                    if not os.path.exists(self.result_dir):
                        self.get_logger().error(f"Result directory not found: {self.result_dir}")
                        return None
                    
                    sessions = [d for d in os.listdir(self.result_dir) 
                               if os.path.isdir(os.path.join(self.result_dir, d))]
                    
                    if not sessions:
                        self.get_logger().error("No session directories found in result directory")
                        return None
                    
                    # Sort by name (assuming timestamp-based naming)
                    sessions.sort(reverse=True)
                    session_result_dir = os.path.join(self.result_dir, sessions[0])
                    self.get_logger().info(f"Using most recent session: {os.path.basename(session_result_dir)}")
            
            # Verify session result directory exists
            if not os.path.exists(session_result_dir):
                self.get_logger().error(f"Session result directory not found: {session_result_dir}")
                return None
            
            # Load 3D positioning result from session result directory
            positioning_result_file = os.path.join(session_result_dir, '3d_positioning_result.json')
            
            if not os.path.exists(positioning_result_file):
                self.get_logger().error(f"3D positioning result file not found: {positioning_result_file}")
                return None
            
            self.get_logger().info(f"📖 Loading 3D positioning results from: {positioning_result_file}")
            
            with open(positioning_result_file, 'r') as f:
                positioning_data = json.load(f)
            
            points_3d = positioning_data.get('points_3d', [])
            
            if len(points_3d) == 0:
                self.get_logger().error("No 3D points found in positioning results")
                return None
            
            self.get_logger().info(f"✓ Loaded {len(points_3d)} 3D points")
            
            # Get keypoint indices for X-axis
            kp_start_idx = kp_index_for_wobj_x_axis[0]
            kp_end_idx = kp_index_for_wobj_x_axis[1]
            
            if len(points_3d) < max(kp_start_idx, kp_end_idx) + 1:
                self.get_logger().error(
                    f"Insufficient keypoints for wobj coordinate system "
                    f"(need at least {max(kp_start_idx, kp_end_idx) + 1}, got {len(points_3d)})"
                )
                return None
            
            # Extract 3D coordinates of the specified keypoints
            kp_start = np.array(points_3d[kp_start_idx])
            kp_end = np.array(points_3d[kp_end_idx])
            
            self.get_logger().info(f"✓ Using keypoints for wobj coordinate system:")
            self.get_logger().info(f"  KP{kp_start_idx}: ({kp_start[0]:.6f}, {kp_start[1]:.6f}, {kp_start[2]:.6f})")
            self.get_logger().info(f"  KP{kp_end_idx}: ({kp_end[0]:.6f}, {kp_end[1]:.6f}, {kp_end[2]:.6f})")
            
            # Origin of the coordinate system is at the start keypoint
            origin = kp_start.copy()
            self.get_logger().info(f"  Origin defined at KP{kp_start_idx}: ({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f})")
            
            # X-axis: from start keypoint to end keypoint
            x_vec_raw = kp_end - kp_start
            x_axis_length = np.linalg.norm(x_vec_raw)
            x_vec = x_vec_raw / x_axis_length
            
            self.get_logger().info(f"  X-axis defined as from KP{kp_start_idx}→KP{kp_end_idx}:")
            self.get_logger().info(f"  X = [{x_vec[0]:.6f}, {x_vec[1]:.6f}, {x_vec[2]:.6f}]")
            
            # Z-axis: should align with base Z-axis (positive upward)
            base_z = np.array([0.0, 0.0, 1.0])
            
            # Make Z-axis orthogonal to X-axis using Gram-Schmidt process
            # Project base_z onto plane perpendicular to X-axis
            z_vec_raw = base_z - np.dot(base_z, x_vec) * x_vec
            z_vec_norm = np.linalg.norm(z_vec_raw)
            
            if z_vec_norm < 1e-6:
                self.get_logger().warn("Warning: X-axis is nearly parallel to base Z-axis, using alternative Z-axis")
                # Use alternative: pick perpendicular vector
                if abs(x_vec[2]) < 0.9:
                    z_vec_raw = np.cross(np.array([1.0, 0.0, 0.0]), x_vec)
                else:
                    z_vec_raw = np.cross(np.array([0.0, 1.0, 0.0]), x_vec)
                z_vec_norm = np.linalg.norm(z_vec_raw)
            
            z_vec = z_vec_raw / z_vec_norm
            
            # Check alignment with base Z
            z_dot_base_z = np.dot(z_vec, base_z)
            self.get_logger().info(f"  Z-axis defined align with base Z:")
            self.get_logger().info(f"  Z = [{z_vec[0]:.6f}, {z_vec[1]:.6f}, {z_vec[2]:.6f}]")
            
            # Y-axis: right-hand rule (Z × X)
            y_vec = np.cross(z_vec, x_vec)
            y_vec = y_vec / np.linalg.norm(y_vec)
            
            self.get_logger().info(f"  Y-axis defined follow from right-hand rule (Y = Z × X)")
            self.get_logger().info(f"  Y = [{y_vec[0]:.6f}, {y_vec[1]:.6f}, {y_vec[2]:.6f}]")
            
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
                    "z": float(origin[2])
                },
                "axes": {
                    "x_axis": {
                        "vector": [float(x_vec[0]), float(x_vec[1]), float(x_vec[2])],
                        "source": f"KP{kp_start_idx}→KP{kp_end_idx}",
                        "length": float(x_axis_length)
                    },
                    "y_axis": {
                        "vector": [float(y_vec[0]), float(y_vec[1]), float(y_vec[2])],
                        "source": "right_hand_rule_Z_cross_X"
                    },
                    "z_axis": {
                        "vector": [float(z_vec[0]), float(z_vec[1]), float(z_vec[2])],
                        "source": "aligned_with_base_z",
                        "alignment_with_base_z": float(z_dot_base_z)
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
                "keypoints_used": [
                    {"index": kp_start_idx, "coordinates": [float(kp_start[0]), float(kp_start[1]), float(kp_start[2])]},
                    {"index": kp_end_idx, "coordinates": [float(kp_end[0]), float(kp_end[1]), float(kp_end[2])]}
                ],
                "kp_index_for_wobj_x_axis": kp_index_for_wobj_x_axis,
                "timestamp": datetime.now().isoformat(),
                "method": f"kp{kp_start_idx}_kp{kp_end_idx}_based_coordinate_system",
                "source_file": positioning_result_file
            }
            
            # Save coordinate system to session result directory
            coord_system_path = os.path.join(session_result_dir, 'wobj_frame_building_result.json')
            with open(coord_system_path, 'w') as f:
                json.dump(coord_system, f, indent=2)
            
            self.get_logger().info(f"💾 Wobj coordinate system saved to: {coord_system_path}")
            self.get_logger().info(f"🎯 Wobj coordinate system established successfully!")
            
            # Save wobj coordinate system to robot_status
            if self.robot_status_client:
                try:
                    # Save origin
                    if self.robot_status_client.set_status(self.operation_name, 'wobj_origin', [float(origin[0]), float(origin[1]), float(origin[2])]):
                        self.get_logger().info(f"✓ wobj_origin saved to robot_status")
                    
                    # Save x_axis
                    if self.robot_status_client.set_status(self.operation_name, 'wobj_x', [float(x_vec[0]), float(x_vec[1]), float(x_vec[2])]):
                        self.get_logger().info(f"✓ wobj_x saved to robot_status")
                    
                    # Save y_axis
                    if self.robot_status_client.set_status(self.operation_name, 'wobj_y', [float(y_vec[0]), float(y_vec[1]), float(y_vec[2])]):
                        self.get_logger().info(f"✓ wobj_y saved to robot_status")
                    
                    # Save z_axis
                    if self.robot_status_client.set_status(self.operation_name, 'wobj_z', [float(z_vec[0]), float(z_vec[1]), float(z_vec[2])]):
                        self.get_logger().info(f"✓ wobj_z saved to robot_status")
                    
                    self.get_logger().info(f"✓ Wobj coordinate system saved to robot_status (namespace: {self.operation_name})")
                except Exception as e:
                    self.get_logger().warning(f"Error saving wobj coordinate system to robot_status: {e}")
            
            # Validate wobj frame building results
            self.get_logger().info(">>> Validating wobj frame building results...")
            validation_success = self.validate_wobj_frame_building_results(
                session_result_dir, 
                coord_system, 
                verbose=self.verbose
            )
            
            if validation_success:
                self.get_logger().info("✓ Validation completed successfully!")
            else:
                self.get_logger().warn("⚠ Validation completed with warnings")
            
            return coord_system
            
        except Exception as e:
            self.get_logger().error(f"Error building wobj coordinate system: {e}")
            import traceback
            traceback.print_exc()
            return None

    def validate_wobj_frame_building_results(self, session_result_dir, coord_system, verbose=True):
        """
        Validate workpiece coordinate system by drawing it on captured images.
        
        Draws the coordinate system axes on each captured image by:
        1. Projecting the 3D origin and axis endpoints to 2D image coordinates
        2. Drawing arrows for X (red), Y (green), Z (blue) axes on the images
        
        Args:
            session_result_dir (str): Path to session result directory
            coord_system (dict): Dictionary containing coordinate system information
                                (output from perform_wobj_frame_building)
            verbose (bool): If True, saves validation images to disk
            
        Returns:
            bool: True if validation successful, False otherwise
        """

        self.get_logger().info("Drawing Wobj Coordinate System on Images")
        
        try:
            if coord_system is None:
                self.get_logger().error("Error: coord_system is None")
                return False
            
            # Extract origin and axes from coord_system
            origin_3d = np.array([
                coord_system['origin']['x'],
                coord_system['origin']['y'],
                coord_system['origin']['z']
            ])
            
            x_axis = np.array(coord_system['axes']['x_axis']['vector'])
            y_axis = np.array(coord_system['axes']['y_axis']['vector'])
            z_axis = np.array(coord_system['axes']['z_axis']['vector'])
                        
            # Define arrow length in 3D space (meters)
            arrow_length = 0.10  # 5cm arrows
            
            # Calculate 3D endpoints of axes
            x_end_3d = origin_3d + x_axis * arrow_length
            y_end_3d = origin_3d + y_axis * arrow_length
            z_end_3d = origin_3d + z_axis * arrow_length
            
            # Get session_dir from session_result_dir
            session_name = os.path.basename(session_result_dir)
            session_dir = os.path.join(self.data_dir, session_name)
            
            if not os.path.exists(session_dir):
                self.get_logger().error(f"Session data directory not found: {session_dir}")
                return False
            
            # Find all test images and their pose files
            test_img_dir = Path(session_dir)
            image_files = sorted(test_img_dir.glob("*.jpg"))
            
            if len(image_files) == 0:
                self.get_logger().error(f"No images found in {session_dir}")
                return False
            
            self.get_logger().info(f"📖 Found {len(image_files)} images for validation")
            
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
                    self.get_logger().warn(f"⚠ Failed to load image: {img_file}")
                    ax.axis('off')
                    continue
                
                # Convert BGR to RGB for matplotlib
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
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
                
                # Prepare 3D points for projection: origin and axis endpoints
                points_3d_to_project = np.array([
                    origin_3d,
                    x_end_3d,
                    y_end_3d,
                    z_end_3d
                ], dtype=np.float32)
                
                # Project all points using cv2.projectPoints
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
                
                self.get_logger().info(f"  ✓ Drawn coordinate system on {img_file.name}")
            
            # Hide unused subplots
            for idx in range(num_images, len(axes)):
                axes[idx].axis('off')
            
            # Add overall title
            fig.suptitle('Workpiece Coordinate System Validation\nRed: X-axis | Green: Y-axis | Blue: Z-axis',
                        fontsize=14, fontweight='bold', y=0.98)
            
            # Add coordinate system information as text
            kp_indices = coord_system.get('kp_index_for_wobj_x_axis', [0, 1])
            info_text = (
                f"Coordinate System Properties:\n"
                f"Origin: ({origin_3d[0]:.4f}, {origin_3d[1]:.4f}, {origin_3d[2]:.4f}) m\n"
                f"X-axis: KP{kp_indices[0]}→KP{kp_indices[1]}, length: {coord_system['axes']['x_axis']['length']:.4f}m\n"
                f"Z alignment with base: {coord_system['axes']['z_axis']['alignment_with_base_z']:.4f}\n"
                f"Arrow length: {arrow_length:.3f}m"
            )
            
            fig.text(0.02, 0.02, info_text, fontsize=9,
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                    verticalalignment='bottom')
            
            # Save figure to session result directory (if verbose)
            if verbose:
                output_path = os.path.join(session_result_dir, 'wobj_coordinate_system_validation.jpg')
                plt.tight_layout()
                plt.savefig(output_path, dpi=150, bbox_inches='tight')
                self.get_logger().info(f"💾 Wobj frame visualization result saved to: {output_path}")
            
            # Close plot to free memory
            plt.close(fig)
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error validating coordinate system: {e}")
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
    
    # Initialize ROS2
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
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
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
                
                # # Perform workpiece frame building
                # print(">>> Building Workpiece Coordinate System...")
                
                # coord_system = ur_locate.perform_wobj_frame_building()
                
                # if coord_system:
                #     print("\n" + "="*60)
                #     print("✓ Wobj Coordinate System built successfully!")
                #     origin = coord_system['origin']
                #     pose = coord_system['pose_representation']
                #     print(f"  Origin: ({origin['x']:.4f}, {origin['y']:.4f}, {origin['z']:.4f}) m")
                #     print(f"  Pose: x={pose['x']:.4f}, y={pose['y']:.4f}, z={pose['z']:.4f}")
                #     print(f"        rx={pose['rx']:.4f}, ry={pose['ry']:.4f}, rz={pose['rz']:.4f}")
                #     print("="*60 + "\n")
                # else:
                #     print("\n✗ Wobj Coordinate System building failed!")
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
                
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_locate.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()


if __name__ == "__main__":
    main()