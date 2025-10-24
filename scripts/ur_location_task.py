#!/usr/bin/env python3

"""
UR Location Task Script

This script performs location estimation tasks for UR15 robot, including:
1. Capture images and robot poses at various positions
2. Estimate handle (keypoint) positions using optical flow and triangulation
3. Move robot to estimated positions
4. Perform precise triangulation estimation from multiple viewpoints

Features:
- Initial coarse estimation using two viewpoints
- TCP normalization to align tool orientation
- Precise triangulation using 5 viewpoints (x±5cm, y±5cm, z-5cm)
- Multi-view 3D reconstruction for improved accuracy

Start position: -0.6656, -1.772, -1.250, -1.522, 1.635, -0.6822

Directories:
- Data: ../temp/ur15_location_task_data/ (coarse estimation)
- Data: ../temp/ur15_precise_location_data/ (precise estimation)
- Results: ../temp/ur15_location_task_result/ (coarse results)
- Results: ../temp/ur15_precise_location_result/ (precise results)
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import json
import os
from datetime import datetime
from ur15_robot_arm.ur15 import UR15Robot
from cv_bridge import CvBridge
import cv2
import time
import threading
import math
import traceback

# Import the FFPPWebAPIKeypointTracker
import sys
sys.path.append(os.path.join(os.path.dirname(__file__), 'ThirdParty', 'robot_vision'))
from core.ffpp_webapi_keypoint_tracker import FFPPWebAPIKeypointTracker

class URLocationTask(Node):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, api_url="http://10.172.151.12:8001"):
        super().__init__('ur_location_task')
        
        # UR15 Robot connection for reading actual pose and joints
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.ur_robot = None
        
        # FlowFormer++ API tracker
        self.api_url = api_url
        self.tracker = None
        self._initialize_tracker()
        
        # Current camera image
        self.latest_image = None
        self.cv_bridge = CvBridge()
        
        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribe to camera image
        self.image_subscriber = self.create_subscription(
            Image,
            '/ur15_camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('UR Location Task Node started')
        
        # Connect to UR robot for reading actual data
        self._connect_to_robot()
    
    def _initialize_tracker(self):
        """Initialize the FlowFormer++ Web API tracker"""
        try:
            self.get_logger().info(f'Initializing FlowFormer++ API tracker at {self.api_url}...')
            self.tracker = FFPPWebAPIKeypointTracker(
                service_url=self.api_url,
                timeout=60,
                image_format="jpg",
                jpeg_quality=95
            )
            self.get_logger().info('✓ FlowFormer++ API tracker initialized successfully')
        except Exception as e:
            self.get_logger().error(f'Failed to initialize tracker: {e}')
            self.tracker = None
    
    def _connect_to_robot(self):
        """Connect to UR robot for reading actual pose and joint data"""
        try:
            self.get_logger().info(f'Connecting to UR robot at {self.robot_ip}:{self.robot_port}...')
            self.ur_robot = UR15Robot(self.robot_ip, self.robot_port)
            result = self.ur_robot.open()
            if result == 0:
                self.get_logger().info('Successfully connected to UR robot for data reading')
            else:
                self.get_logger().warn('Failed to connect to UR robot. Pose saving will not work.')
                self.ur_robot = None
        except Exception as e:
            self.get_logger().error(f'Error connecting to UR robot: {e}')
            self.ur_robot = None
    
    def _rotvec_to_matrix(self, rx, ry, rz):
        """Convert rotation vector to rotation matrix"""

        angle = math.sqrt(rx**2 + ry**2 + rz**2)
        if angle < 1e-10:
            return np.eye(3)
        
        # Normalize axis
        kx, ky, kz = rx/angle, ry/angle, rz/angle
        
        # Rodrigues' rotation formula
        c = math.cos(angle)
        s = math.sin(angle)
        v = 1 - c
        
        R = np.array([
            [kx*kx*v + c,    kx*ky*v - kz*s, kx*kz*v + ky*s],
            [ky*kx*v + kz*s, ky*ky*v + c,    ky*kz*v - kx*s],
            [kz*kx*v - ky*s, kz*ky*v + kx*s, kz*kz*v + c]
        ])
        return R
    
    def _matrix_to_rotvec(self, R):
        """
        Convert rotation matrix to rotation vector (axis-angle representation)
        
        Args:
            R: 3x3 rotation matrix
            
        Returns:
            (rx, ry, rz): rotation vector components
        """
        # Calculate rotation angle
        trace = np.trace(R)
        angle = math.acos(np.clip((trace - 1) / 2, -1.0, 1.0))
        
        # Handle special cases
        if angle < 1e-10:
            # No rotation
            return 0.0, 0.0, 0.0
        elif abs(angle - math.pi) < 1e-6:
            # 180-degree rotation - need special handling
            # Find the axis from the diagonal elements
            if R[0, 0] >= R[1, 1] and R[0, 0] >= R[2, 2]:
                kx = math.sqrt((R[0, 0] + 1) / 2)
                ky = R[0, 1] / (2 * kx)
                kz = R[0, 2] / (2 * kx)
            elif R[1, 1] >= R[2, 2]:
                ky = math.sqrt((R[1, 1] + 1) / 2)
                kx = R[0, 1] / (2 * ky)
                kz = R[1, 2] / (2 * ky)
            else:
                kz = math.sqrt((R[2, 2] + 1) / 2)
                kx = R[0, 2] / (2 * kz)
                ky = R[1, 2] / (2 * kz)
        else:
            # General case
            kx = (R[2, 1] - R[1, 2]) / (2 * math.sin(angle))
            ky = (R[0, 2] - R[2, 0]) / (2 * math.sin(angle))
            kz = (R[1, 0] - R[0, 1]) / (2 * math.sin(angle))
        
        # Return rotation vector
        return angle * kx, angle * ky, angle * kz
    
    def _pose_to_matrix(self, pose):
        """Convert pose [X,Y,Z,Rx,Ry,Rz] to 4x4 homogeneous transformation matrix"""
        T = np.eye(4)
        T[0:3, 0:3] = self._rotvec_to_matrix(pose[3], pose[4], pose[5])
        T[0:3, 3] = [pose[0], pose[1], pose[2]]
        return T
    
    def image_callback(self, msg):
        """
        Update current camera image
        
        Args:
            msg: Image message from camera
        """
        self.latest_image = msg
    
    def capture_image_and_pose(self, save_dir="../temp/ur15_location_task_data", 
                              img_filename="test_img.jpg", 
                              pose_filename="test_pose.json",
                              metadata=None):
        """
        Capture current camera image and robot pose
        
        Args:
            save_dir: Directory to save the files
            img_filename: Filename for the image (default: "test_img.jpg")
            pose_filename: Filename for the pose JSON (default: "test_pose.json")
            metadata: Optional dict with extra metadata to include in pose JSON
            
        Returns:
            bool: True if successful, False otherwise
        """
        self.get_logger().info('>>> Capturing image and pose...')
        
        # Create save directory if it doesn't exist
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            self.get_logger().info(f'Created save directory: {save_dir}')
        
        if self.ur_robot is None:
            self.get_logger().error('UR robot not connected. Cannot capture pose.')
            return False
        
        try:
            # Read actual joint positions
            joint_positions = self.ur_robot.get_actual_joint_positions()
            if joint_positions is None:
                self.get_logger().error('Failed to read joint positions')
                return False
            
            # Read actual TCP pose
            tcp_pose = self.ur_robot.get_actual_tcp_pose()
            if tcp_pose is None:
                self.get_logger().error('Failed to read TCP pose')
                return False
            
            # Calculate end2base transformation matrix
            end2base = self._pose_to_matrix(tcp_pose)
            
            # Prepare data structure
            data = {
                "joint_angles": list(joint_positions),
                "end_xyzrpy": {
                    "x": tcp_pose[0],
                    "y": tcp_pose[1],
                    "z": tcp_pose[2],
                    "rx": tcp_pose[3],
                    "ry": tcp_pose[4],
                    "rz": tcp_pose[5]
                },
                "end2base": end2base.tolist(),
                "timestamp": datetime.now().isoformat()
            }
            
            # Add optional metadata
            if metadata is not None:
                data.update(metadata)
            
            # Save to JSON file
            json_filename = os.path.join(save_dir, pose_filename)
            with open(json_filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            self.get_logger().info(f'✓ Saved pose to: {json_filename}')
            self.get_logger().info(f'  Joint angles: {[f"{j:.4f}" for j in joint_positions]}')
            self.get_logger().info(f'  TCP pose: {[f"{p:.4f}" for p in tcp_pose]}')
            
            # Save current camera image if available
            if self.latest_image is not None:
                try:
                    # Convert ROS Image message to OpenCV format
                    cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
                    
                    # Save image
                    image_filename = os.path.join(save_dir, img_filename)
                    cv2.imwrite(image_filename, cv_image)
                    self.get_logger().info(f'✓ Saved image to: {image_filename}')
                except Exception as e:
                    self.get_logger().error(f'Error saving camera image: {e}')
                    return False
            else:
                self.get_logger().warn('No camera image available to save')
                return False
            
            self.get_logger().info('<<< Capture completed successfully!')
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error capturing image and pose: {e}')
            return False
    
    def cleanup(self):
        """Clean up robot connection and tracker"""
        if self.ur_robot is not None:
            self.ur_robot.close()
        if self.tracker is not None:
            # Clean up any remaining references
            try:
                self.tracker.remove_reference_image()
            except:
                pass
    
    def estimate_handles_xy_coordinates(self, 
                                        ref_img,
                                        ref_pose,
                                        ref_keypoints,
                                        test_img,
                                        test_pose,
                                        result_dir="../temp/ur15_location_task_result",
                                        enable_retry=True,
                                        retry_shift=0.01,
                                        max_attempts=2):
        """
        Estimate handles xy coordinates (keypoints 3 and 4) using FlowFormer++ API tracker.
        """
        try:
            # Check if tracker is available
            if self.tracker is None:
                self.get_logger().error('FlowFormer++ tracker not available')
                return None
                        
            # Fixed parameters
            camera_params_dir = "../temp/ur15_cam_calibration_result/ur15_camera_parameters"
            
            # Retry tracking
            attempt_count = 0
            retry_log = {
                "retry_enabled": enable_retry,
                "max_attempts": max_attempts,
                "retry_shift_m": retry_shift,
                "attempts": []
            }
            
            # ==============================================================
            # One-time setup: Load reference data and set reference image
            # (These don't change across retry attempts)
            # ==============================================================
            
            # Setup file paths
            ref_img_path = ref_img
            ref_keypoints_path = ref_keypoints
            ref_pose_path = ref_pose
            test_img_path = test_img
            test_pose_path = test_pose
            
            # Validate input files exist
            required_files = {
                "Reference image": ref_img_path,
                "Reference keypoints": ref_keypoints_path,
                "Reference pose": ref_pose_path,
                "Test image": test_img_path,
                "Test pose": test_pose_path
            }
            
            for name, path in required_files.items():
                if not os.path.exists(path):
                    self.get_logger().error(f"{name} not found: {path}")
                    return None
            
            self.get_logger().info("✓ All required input files found")
            
            # Load reference keypoints
            self.get_logger().info("\n📖 Loading reference keypoints...")
            with open(ref_keypoints_path, 'r') as f:
                ref_keypoints_data = json.load(f)
            
            ref_keypoints_list = []
            for kp in ref_keypoints_data.get('keypoints', []):
                ref_keypoints_list.append({
                    'x': float(kp['x']),
                    'y': float(kp['y'])
                })
            
            self.get_logger().info(f"✓ Loaded {len(ref_keypoints_list)} reference keypoints")
            
            # Load camera parameters
            self.get_logger().info("\n📖 Loading camera parameters...")
            eye_in_hand_path = os.path.join(camera_params_dir, "ur15_cam_eye_in_hand_result.json")
            if not os.path.exists(eye_in_hand_path):
                self.get_logger().error(f"Camera parameters not found: {eye_in_hand_path}")
                return None
            
            with open(eye_in_hand_path, 'r') as f:
                eye_in_hand_data = json.load(f)
            
            camera_matrix = np.array(eye_in_hand_data['camera_matrix'])
            dist_coeffs = np.array(eye_in_hand_data['distortion_coefficients'])
            cam2end_matrix = np.array(eye_in_hand_data['cam2end_matrix'])
            
            self.get_logger().info("✓ Loaded camera parameters")
            
            # Load reference robot pose
            self.get_logger().info("\n📖 Loading reference robot pose...")
            with open(ref_pose_path, 'r') as f:
                ref_pose_data = json.load(f)
            ref_end2base = np.array(ref_pose_data['end2base'])
            self.get_logger().info("✓ Loaded reference pose")
            
            # Set reference image using tracker (only once, outside the retry loop)
            self.get_logger().info("\n📤 Setting reference image with keypoints...")
            
            # Load reference image
            ref_img_bgr = cv2.imread(ref_img_path)
            if ref_img_bgr is None:
                self.get_logger().error(f"Failed to load reference image: {ref_img_path}")
                return None
            
            # Convert BGR to RGB for tracker
            ref_img_rgb = cv2.cvtColor(ref_img_bgr, cv2.COLOR_BGR2RGB)
            
            # Set reference image using tracker
            set_result = self.tracker.set_reference_image(
                image=ref_img_rgb,
                keypoints=ref_keypoints_list,
                image_name='ur_location_task_reference'
            )
            
            if not set_result.get('success', False):
                self.get_logger().error(f"Failed to set reference image: {set_result.get('error', 'Unknown error')}")
                return None
            
            self.get_logger().info("✓ Reference image set successfully")
            
            # ==============================================================
            # Main retry loop - only test image tracking happens here
            # ==============================================================
            while attempt_count < max_attempts:
                attempt_count += 1
                attempt_info = {
                    "attempt_number": attempt_count,
                    "timestamp": datetime.now().isoformat()
                }
                
                self.get_logger().info("=" * 60)
                self.get_logger().info(f"Attempt {attempt_count}/{max_attempts}: Starting handles xy coordinate estimation...")
                self.get_logger().info("=" * 60)
                
                # Create result directory if it doesn't exist
                if not os.path.exists(result_dir):
                    os.makedirs(result_dir)
                    self.get_logger().info(f'Created result directory: {result_dir}')
                
                # Load test robot pose (may change in retry after micro-movement)
                self.get_logger().info("\n📖 Loading test robot pose...")
                with open(test_pose_path, 'r') as f:
                    test_pose_data = json.load(f)
                test_end2base = np.array(test_pose_data['end2base'])
                self.get_logger().info("✓ Loaded test pose")
                
                # Track keypoints in test image
                self.get_logger().info("\n📤 Tracking keypoints in test image...")
                
                # Load test image
                test_img_bgr = cv2.imread(test_img_path)
                if test_img_bgr is None:
                    self.get_logger().error(f"Failed to load test image: {test_img_path}")
                    attempt_info["status"] = "failed"
                    attempt_info["failure_reason"] = "Failed to load test image"
                    retry_log["attempts"].append(attempt_info)
                    self._save_retry_log(result_dir, retry_log)
                    return None
                
                # Convert BGR to RGB for tracker
                test_img_rgb = cv2.cvtColor(test_img_bgr, cv2.COLOR_BGR2RGB)
                
                # Track keypoints using tracker
                tracking_result = self.tracker.track_keypoints(
                    target_image=test_img_rgb,
                    reference_name='ur_location_task_reference',
                    bidirectional=True,
                    return_flow=False
                )
                
                if not tracking_result.get('success', False):
                    self.get_logger().error(f"Failed to track keypoints: {tracking_result.get('error', 'Unknown error')}")
                    attempt_info["status"] = "failed"
                    attempt_info["failure_reason"] = "Tracker call failed: track_keypoints"
                    retry_log["attempts"].append(attempt_info)
                    self._save_retry_log(result_dir, retry_log)
                    return None
                
                tracked_keypoints = tracking_result.get('tracked_keypoints', [])
                
                self.get_logger().info(f"✓ Tracked {len(tracked_keypoints)} keypoints")
                
                # Check bidirectional validation results
                validation_passed = False
                bidirectional_stats = tracking_result.get('bidirectional_stats')
                if bidirectional_stats:
                    # Extract forward-backward errors from tracked_keypoints' consistency_distance
                    forward_backward_errors = []
                    for kp in tracked_keypoints:
                        if 'consistency_distance' in kp:
                            forward_backward_errors.append(kp['consistency_distance'])
                    
                    if forward_backward_errors:
                        avg_error = np.mean(forward_backward_errors)
                        max_error = np.max(forward_backward_errors)
                        
                        self.get_logger().info(f"\n🔄 Bidirectional validation results:")
                        self.get_logger().info(f"  Average error: {avg_error:.3f} pixels")
                        self.get_logger().info(f"  Max error: {max_error:.3f} pixels")
                        
                        # Record validation metrics
                        attempt_info["avg_fb_error"] = float(avg_error)
                        attempt_info["max_fb_error"] = float(max_error)
                        
                        # Check if validation passed (average error < 3 pixel)
                        if avg_error >= 3.0:
                            self.get_logger().error(f"❌ Bidirectional validation FAILED (avg error >= 3.0 pixel)")
                            attempt_info["status"] = "failed"
                            attempt_info["failure_reason"] = f"avg_fb_error >= 3.0 ({avg_error:.3f})"
                        else:
                            self.get_logger().info(f"✅ Bidirectional validation PASSED (avg error < 3.0 pixel)")

                            # --------------------------------------------------
                            # Critical keypoint validation: Only validate KP3 and KP4
                            # (indices 2 and 3) as these are the handle keypoints used
                            # for position estimation.
                            # --------------------------------------------------
                            per_point_threshold = 1.5
                            critical_indices = [2, 3]  # KP3 and KP4
                            self.get_logger().info(f"  Critical keypoint FB errors (KP3 & KP4, threshold {per_point_threshold:.2f}px):")
                            failing = []
                            
                            # Check all keypoints for logging, but only fail on critical ones
                            for idx, e in enumerate(forward_backward_errors):
                                is_critical = idx in critical_indices
                                flag = "OK" if e <= per_point_threshold else "FAIL"
                                marker = "🎯" if is_critical else "  "
                                self.get_logger().info(f"  {marker} KP{idx+1}: {e:.3f} px [{flag}]")
                                
                                # Only add to failing list if it's a critical keypoint
                                if is_critical and e > per_point_threshold:
                                    failing.append({"keypoint_index": idx+1, "error_px": float(e)})

                            if failing:
                                self.get_logger().error("❌ Critical keypoint FB validation failed. The following handle keypoints exceed 1.0 px:")
                                for f in failing:
                                    self.get_logger().error(f"   🎯 KP{f['keypoint_index']}: {f['error_px']:.3f} px")
                                attempt_info["status"] = "failed"
                                attempt_info["failure_reason"] = f"critical_kp_fb_validation_failed ({len(failing)} handle keypoints)"
                                attempt_info["failing_keypoints"] = failing
                                
                                # Save report
                                try:
                                    quality_dir = os.path.join("../temp", "ur15_location_task_result")
                                    os.makedirs(quality_dir, exist_ok=True)
                                    quality_json_path = os.path.join(quality_dir, "bidirectional_quality.json")
                                    quality_payload = {
                                        "timestamp": datetime.now().isoformat(),
                                        "attempt": attempt_count,
                                        "forward_backward_errors": [float(x) for x in forward_backward_errors],
                                        "avg_error": float(avg_error),
                                        "max_error": float(max_error),
                                        "threshold_per_point": per_point_threshold,
                                        "failing_keypoints": failing,
                                        "pass": False
                                    }
                                    with open(quality_json_path, 'w') as qf:
                                        json.dump(quality_payload, qf, indent=2, ensure_ascii=False)
                                    self.get_logger().info(f"  Saved bidirectional quality report: {quality_json_path}")
                                    
                                    # Save visualization
                                    vis_img = self._visualize_bidirectional_errors(
                                        test_img_path, tracked_keypoints, forward_backward_errors,
                                        critical_indices, per_point_threshold
                                    )
                                    if vis_img is not None:
                                        vis_path = os.path.join(quality_dir, "bidirectional_quality.jpg")
                                        cv2.imwrite(vis_path, vis_img)
                                        self.get_logger().info(f"  Saved bidirectional visualization: {vis_path}")
                                except Exception as e:
                                    self.get_logger().warn(f"Failed to save bidirectional quality report (on failure): {e}")
                            else:
                                # All points passed
                                validation_passed = True
                                try:
                                    quality_dir = os.path.join("../temp", "ur15_location_task_result")
                                    os.makedirs(quality_dir, exist_ok=True)
                                    quality_json_path = os.path.join(quality_dir, "bidirectional_quality.json")
                                    quality_payload = {
                                        "timestamp": datetime.now().isoformat(),
                                        "attempt": attempt_count,
                                        "forward_backward_errors": [float(x) for x in forward_backward_errors],
                                        "avg_error": float(avg_error),
                                        "max_error": float(max_error),
                                        "threshold_per_point": per_point_threshold,
                                        "failing_keypoints": [],
                                        "pass": True
                                    }
                                    with open(quality_json_path, 'w') as qf:
                                        json.dump(quality_payload, qf, indent=2, ensure_ascii=False)
                                    self.get_logger().info(f"  Saved bidirectional quality report: {quality_json_path}")
                                    
                                    # Save visualization
                                    vis_img = self._visualize_bidirectional_errors(
                                        test_img_path, tracked_keypoints, forward_backward_errors,
                                        critical_indices, per_point_threshold
                                    )
                                    if vis_img is not None:
                                        vis_path = os.path.join(quality_dir, "bidirectional_quality.jpg")
                                        cv2.imwrite(vis_path, vis_img)
                                        self.get_logger().info(f"  Saved bidirectional visualization: {vis_path}")
                                except Exception as e:
                                    self.get_logger().warn(f"Failed to save bidirectional quality report: {e}")
                                self.get_logger().info("✅ All keypoints passed per-point FB validation")
                    else:
                        self.get_logger().warn("⚠️ No forward-backward error data available")
                        attempt_info["status"] = "failed"
                        attempt_info["failure_reason"] = "no_fb_error_data"
                else:
                    self.get_logger().warn("⚠️ No bidirectional statistics returned by API")
                    attempt_info["status"] = "failed"
                    attempt_info["failure_reason"] = "no_bidirectional_stats"
                
                # If validation failed and retry is enabled, attempt micro-movement and retry
                if not validation_passed:
                    retry_log["attempts"].append(attempt_info)
                    
                    if enable_retry and attempt_count < max_attempts:
                        self.get_logger().warn(f"🔁 Attempt {attempt_count} failed. Initiating retry with micro-movement...")
                        
                        # Perform micro-movement
                        if not self._perform_micro_movement_retry(retry_shift, os.path.dirname(test_img)):
                            self.get_logger().error("❌ Micro-movement retry failed. Aborting.")
                            self._save_retry_log(result_dir, retry_log)
                            return None
                        
                        # Continue to next iteration
                        continue
                    else:
                        self.get_logger().error(f"❌ All {max_attempts} attempts exhausted or retry disabled. Giving up.")
                        self._save_retry_log(result_dir, retry_log)
                        return None
                
                # Validation passed, continue with 3D estimation
                attempt_info["validation_status"] = "passed"
                
                # Validate we have at least 4 keypoints
                if len(ref_keypoints_list) < 4 or len(tracked_keypoints) < 4:
                    self.get_logger().error(f"Need at least 4 keypoints, got {len(ref_keypoints_list)} ref and {len(tracked_keypoints)} tracked")
                    attempt_info["status"] = "failed"
                    attempt_info["failure_reason"] = "insufficient_keypoints"
                    retry_log["attempts"].append(attempt_info)
                    self._save_retry_log(result_dir, retry_log)
                    return None
                
                # Extract keypoints 3 and 4 (indices 2 and 3)
                ref_kp3 = ref_keypoints_list[2]
                ref_kp4 = ref_keypoints_list[3]
                test_kp3 = tracked_keypoints[2]
                test_kp4 = tracked_keypoints[3]
                
                self.get_logger().info(f"\n🔍 Keypoint coordinates:")
                self.get_logger().info(f"  Reference KP3: ({ref_kp3['x']:.2f}, {ref_kp3['y']:.2f})")
                self.get_logger().info(f"  Reference KP4: ({ref_kp4['x']:.2f}, {ref_kp4['y']:.2f})")
                self.get_logger().info(f"  Test KP3: ({test_kp3['x']:.2f}, {test_kp3['y']:.2f})")
                self.get_logger().info(f"  Test KP4: ({test_kp4['x']:.2f}, {test_kp4['y']:.2f})")
                
                # Estimate 3D positions (assuming z=0 plane in base frame)
                plane_z = 0.0
                
                self.get_logger().info(f"\n📐 Estimating 3D positions (plane z={plane_z})...")
                
                # Estimate from reference image
                ref_kp3_base = self._estimate_keypoint_3d_position(
                    ref_kp3['x'], ref_kp3['y'], camera_matrix, dist_coeffs,
                    cam2end_matrix, ref_end2base, plane_z
                )
                ref_kp4_base = self._estimate_keypoint_3d_position(
                    ref_kp4['x'], ref_kp4['y'], camera_matrix, dist_coeffs,
                    cam2end_matrix, ref_end2base, plane_z
                )
                
                # Estimate from test image
                test_kp3_base = self._estimate_keypoint_3d_position(
                    test_kp3['x'], test_kp3['y'], camera_matrix, dist_coeffs,
                    cam2end_matrix, test_end2base, plane_z
                )
                test_kp4_base = self._estimate_keypoint_3d_position(
                    test_kp4['x'], test_kp4['y'], camera_matrix, dist_coeffs,
                    cam2end_matrix, test_end2base, plane_z
                )
                
                if None in [ref_kp3_base, ref_kp4_base, test_kp3_base, test_kp4_base]:
                    self.get_logger().error("Failed to estimate 3D positions")
                    attempt_info["status"] = "failed"
                    attempt_info["failure_reason"] = "3d_estimation_failed"
                    retry_log["attempts"].append(attempt_info)
                    self._save_retry_log(result_dir, retry_log)
                    return None
                
                self.get_logger().info(f"\n✅ 3D Position estimation completed:")
                self.get_logger().info(f"  Reference KP3: ({ref_kp3_base[0]:.4f}, {ref_kp3_base[1]:.4f}, {ref_kp3_base[2]:.4f})")
                self.get_logger().info(f"  Reference KP4: ({ref_kp4_base[0]:.4f}, {ref_kp4_base[1]:.4f}, {ref_kp4_base[2]:.4f})")
                self.get_logger().info(f"  Test KP3: ({test_kp3_base[0]:.4f}, {test_kp3_base[1]:.4f}, {test_kp3_base[2]:.4f})")
                self.get_logger().info(f"  Test KP4: ({test_kp4_base[0]:.4f}, {test_kp4_base[1]:.4f}, {test_kp4_base[2]:.4f})")

                # ------------------------------------------------------------
                # Reprojection quality check (only test image points are used
                # as the final estimation reference). If either keypoint error
                # > 1.0 px => treat as unreliable and abort.
                # ------------------------------------------------------------
                reprojection_passed = False
                try:
                    reproj_output_dir = os.path.join("../temp", "ur15_location_task_result")
                    if not os.path.exists(reproj_output_dir):
                        os.makedirs(reproj_output_dir)

                    def _reproject_point(base_point, pixel_uv, end2base_mat, cam2end_mat, cam_K):
                        # base -> camera
                        cam2base = end2base_mat @ cam2end_mat
                        base2cam = np.linalg.inv(cam2base)
                        P_base_h = np.array([base_point[0], base_point[1], base_point[2], 1.0])
                        P_cam = base2cam @ P_base_h
                        Xc, Yc, Zc = P_cam[:3]
                        if Zc <= 1e-9:
                            return None, None, None
                        u_proj = cam_K[0,0] * (Xc / Zc) + cam_K[0,2]
                        v_proj = cam_K[1,1] * (Yc / Zc) + cam_K[1,2]
                        du = u_proj - pixel_uv[0]
                        dv = v_proj - pixel_uv[1]
                        err = float(np.sqrt(du*du + dv*dv))
                        return (float(u_proj), float(v_proj), err)

                    # Reproject test keypoints (since we are using test image estimation)
                    (u3_proj, v3_proj, err3) = _reproject_point(
                        test_kp3_base,
                        (test_kp3['x'], test_kp3['y']),
                        test_end2base,
                        cam2end_matrix,
                        camera_matrix
                    )
                    (u4_proj, v4_proj, err4) = _reproject_point(
                        test_kp4_base,
                        (test_kp4['x'], test_kp4['y']),
                        test_end2base,
                        cam2end_matrix,
                        camera_matrix
                    )

                    reproj_data = {
                        "timestamp": datetime.now().isoformat(),
                        "attempt": attempt_count,
                        "threshold_px": 1.0,
                        "keypoints": {
                            "test_kp3": {
                                "observed_pixel": {"u": float(test_kp3['x']), "v": float(test_kp3['y'])},
                                "reprojected_pixel": {"u": u3_proj, "v": v3_proj},
                                "reprojection_error_px": err3
                            },
                            "test_kp4": {
                                "observed_pixel": {"u": float(test_kp4['x']), "v": float(test_kp4['y'])},
                                "reprojected_pixel": {"u": u4_proj, "v": v4_proj},
                                "reprojection_error_px": err4
                            }
                        }
                    }

                    max_err = max(err3, err4)
                    reproj_data["max_error_px"] = max_err
                    reproj_data["pass"] = bool(max_err <= 1.0)
                    
                    attempt_info["reprojection_max_error"] = float(max_err)

                    reproj_file = os.path.join(reproj_output_dir, "reprojection_quality.json")
                    with open(reproj_file, 'w') as rf:
                        json.dump(reproj_data, rf, indent=2, ensure_ascii=False)
                    
                    # Save visualization
                    vis_img = self._visualize_reprojection_errors(
                        test_img_path, reproj_data
                    )
                    if vis_img is not None:
                        vis_path = os.path.join(reproj_output_dir, "reprojection_quality.jpg")
                        cv2.imwrite(vis_path, vis_img)
                        self.get_logger().info(f"Saved reprojection visualization: {vis_path}")

                    if not reproj_data["pass"]:
                        self.get_logger().error(f"❌ Reprojection quality failed. Max error = {max_err:.3f} px (>1.0)")
                        self.get_logger().error(f"Details saved: {reproj_file}")
                        attempt_info["status"] = "failed"
                        attempt_info["failure_reason"] = f"reprojection_error_exceeded ({max_err:.3f}px)"
                        retry_log["attempts"].append(attempt_info)
                        
                        # Retry if enabled
                        if enable_retry and attempt_count < max_attempts:
                            self.get_logger().warn(f"🔁 Attempt {attempt_count} failed on reprojection. Retrying with micro-movement...")
                            if not self._perform_micro_movement_retry(retry_shift, os.path.dirname(test_img)):
                                self.get_logger().error("❌ Micro-movement retry failed. Aborting.")
                                self._save_retry_log(result_dir, retry_log)
                                return None
                            continue
                        else:
                            self._save_retry_log(result_dir, retry_log)
                            return None
                    else:
                        self.get_logger().info(f"🔎 Reprojection quality OK. Max error = {max_err:.3f} px ≤ 1.0")
                        self.get_logger().info(f"Saved reprojection report: {reproj_file}")
                        reprojection_passed = True

                except Exception as e:
                    self.get_logger().error(f"Error during reprojection validation: {e}")
                    attempt_info["status"] = "failed"
                    attempt_info["failure_reason"] = f"reprojection_exception: {str(e)}"
                    retry_log["attempts"].append(attempt_info)
                    self._save_retry_log(result_dir, retry_log)
                    return None
                
                # All validations passed - success!
                if reprojection_passed:
                    attempt_info["status"] = "success"
                    retry_log["attempts"].append(attempt_info)
                    retry_log["final_status"] = "success"
                    retry_log["successful_attempt"] = attempt_count
                    self._save_retry_log(result_dir, retry_log)
                    
                    self.get_logger().info(f"\n✅ Attempt {attempt_count} succeeded after all validations!")
                    # Break out of retry loop
                    break
            
            # Prepare result dictionary (after breaking out of loop successfully)
            result = {
                "success": True,
                "timestamp": datetime.now().isoformat(),
                "attempts": attempt_count,
                "retry_enabled": enable_retry,
                "plane_z_in_base": plane_z,
                "reference_image": {
                    "keypoint_3": {
                        "pixel_coordinates": {"u": ref_kp3['x'], "v": ref_kp3['y']},
                        "base_coordinates_3d": {"x": ref_kp3_base[0], "y": ref_kp3_base[1], "z": ref_kp3_base[2]},
                        "base_coordinates_2d": {"x": ref_kp3_base[0], "y": ref_kp3_base[1]}
                    },
                    "keypoint_4": {
                        "pixel_coordinates": {"u": ref_kp4['x'], "v": ref_kp4['y']},
                        "base_coordinates_3d": {"x": ref_kp4_base[0], "y": ref_kp4_base[1], "z": ref_kp4_base[2]},
                        "base_coordinates_2d": {"x": ref_kp4_base[0], "y": ref_kp4_base[1]}
                    }
                },
                "test_image": {
                    "keypoint_3": {
                        "pixel_coordinates": {"u": test_kp3['x'], "v": test_kp3['y']},
                        "base_coordinates_3d": {"x": test_kp3_base[0], "y": test_kp3_base[1], "z": test_kp3_base[2]},
                        "base_coordinates_2d": {"x": test_kp3_base[0], "y": test_kp3_base[1]}
                    },
                    "keypoint_4": {
                        "pixel_coordinates": {"u": test_kp4['x'], "v": test_kp4['y']},
                        "base_coordinates_3d": {"x": test_kp4_base[0], "y": test_kp4_base[1], "z": test_kp4_base[2]},
                        "base_coordinates_2d": {"x": test_kp4_base[0], "y": test_kp4_base[1]}
                    }
                },
                "camera_parameters": {
                    "camera_matrix": camera_matrix.tolist(),
                    "distortion_coefficients": dist_coeffs.tolist()
                }
            }
            
            # Save estimation result
            output_path = os.path.join(result_dir, "handles_xy_estimation_result.json")
            with open(output_path, 'w') as f:
                json.dump(result, f, indent=2, ensure_ascii=False)
            
            self.get_logger().info(f"\n💾 Saved estimation result to: {output_path}")
            
            # Clean up: remove reference image from tracker
            try:
                cleanup_result = self.tracker.remove_reference_image('ur_location_task_reference')
                if cleanup_result.get('success', False):
                    self.get_logger().info("✓ Reference image cleaned up successfully")
            except Exception as e:
                self.get_logger().warn(f"Failed to cleanup reference image: {e}")
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error estimating handles xy coordinates: {str(e)}")
            
            self.get_logger().error(traceback.format_exc())
            # Clean up: remove reference image from tracker even on error
            try:
                self.tracker.remove_reference_image('ur_location_task_reference')
            except:
                pass
            return None
    
    def _save_retry_log(self, result_dir, retry_log):
        """Save retry attempt log to JSON file"""
        try:
            log_path = os.path.join(result_dir, "retry_log.json")
            with open(log_path, 'w') as f:
                json.dump(retry_log, f, indent=2, ensure_ascii=False)
            self.get_logger().info(f"📄 Saved retry log to: {log_path}")
        except Exception as e:
            self.get_logger().warn(f"Failed to save retry log: {e}")
    
    def _perform_micro_movement_retry(self, shift_distance, test_data_dir):
        """
        Perform micro-movement and recapture test image.
        """
        try:
            if self.ur_robot is None:
                self.get_logger().error("Robot not connected. Cannot perform micro-movement.")
                return False
            
            # Get current TCP pose
            current_pose = self.ur_robot.get_actual_tcp_pose()
            if current_pose is None:
                self.get_logger().error("Failed to get current TCP pose for micro-movement.")
                return False
            
            # Create target pose with small X offset
            retry_pose = current_pose.copy()
            retry_pose[0] += shift_distance # x-positive direction
            
            self.get_logger().info(f"  Current pose: {[f'{p:.4f}' for p in current_pose]}")
            self.get_logger().info(f"  Moving by ΔX = {shift_distance:.4f} m...")
            self.get_logger().info(f"  Target pose:  {[f'{p:.4f}' for p in retry_pose]}")
            
            # Execute movement
            move_result = self.ur_robot.movel(retry_pose, a=0.3, v=0.1)
            if move_result != 0:
                self.get_logger().error(f"Micro-movement failed with result code: {move_result}")
                return False
            
            self.get_logger().info("  ✓ Micro-movement completed")
            
            # Wait for stabilization
            time.sleep(0.5)
            
            # Recapture test image and pose
            self.get_logger().info("  📸 Recapturing test image and pose...")
            if not self.capture_image_and_pose(save_dir=test_data_dir):
                self.get_logger().error("Failed to recapture image/pose after micro-movement.")
                return False
            
            self.get_logger().info("  ✓ Recapture completed. Ready for retry.")
            return True
            
        except Exception as e:
            self.get_logger().error(f"Exception during micro-movement retry: {e}")
            self.get_logger().error(traceback.format_exc())
            return False
    
    def _estimate_keypoint_3d_position(self, pixel_u: float, pixel_v: float, 
                                       camera_matrix: np.ndarray, 
                                       dist_coeffs: np.ndarray,
                                       cam2end_matrix: np.ndarray,
                                       end2base_matrix: np.ndarray,
                                       plane_z: float = 0.0):
        """
        Estimate 3D position of a keypoint in base frame.
        """
        try:
            # Step 1: Undistort the pixel coordinates and get normalized image coordinates
            # Note: When P=None, cv2.undistortPoints returns normalized coordinates directly
            pixel_point = np.array([[[pixel_u, pixel_v]]], dtype=np.float32)
            undistorted_point = cv2.undistortPoints(pixel_point, camera_matrix, dist_coeffs, P=None)
            x_norm, y_norm = undistorted_point[0, 0]
            
            # Ray direction in camera frame (normalized)
            ray_cam = np.array([x_norm, y_norm, 1.0])
            ray_cam = ray_cam / np.linalg.norm(ray_cam)
            
            # Step 3: Transform ray to base frame
            # cam2base = end2base @ cam2end
            cam2base_matrix = end2base_matrix @ cam2end_matrix
            
            # Ray direction in base frame
            ray_base = cam2base_matrix[:3, :3] @ ray_cam
            
            # Camera origin in base frame
            camera_origin_base = cam2base_matrix[:3, 3]
            
            # Step 4: Intersect ray with plane z = plane_z
            # Ray equation: P = camera_origin + t * ray_direction
            # Plane equation: z = plane_z
            # Solve for t: camera_origin_z + t * ray_direction_z = plane_z
            
            if abs(ray_base[2]) < 1e-6:
                self.get_logger().warn("Ray is parallel to the plane, cannot compute intersection")
                return None, None, None
            
            t = (plane_z - camera_origin_base[2]) / ray_base[2]
            
            if t < 0:
                self.get_logger().warn(f"Intersection point is behind camera (t={t})")
            
            # Compute intersection point
            point_base = camera_origin_base + t * ray_base
            
            return float(point_base[0]), float(point_base[1]), float(point_base[2])
            
        except Exception as e:
            self.get_logger().error(f"Error in 3D position estimation: {str(e)}")
            return None, None, None
    
    def move_to_xy_position(self, target_x, target_y, a=0.5, v=0.2):
        """
        Move robot TCP to target XY position in base frame, keeping current Z and orientation.
        """
        try:
            if self.ur_robot is None:
                self.get_logger().error('UR robot not connected. Cannot move.')
                return False
            
            # Get current TCP pose
            current_tcp_pose = self.ur_robot.get_actual_tcp_pose()
            if current_tcp_pose is None:
                self.get_logger().error('Failed to get current TCP pose')
                return False
            
            self.get_logger().info(f'Current TCP pose: {[f"{p:.4f}" for p in current_tcp_pose]}')
            
            # Create target pose: move to target XY, keep current Z and orientation
            target_pose = [
                target_x,             # X: target x
                target_y,             # Y: target y
                current_tcp_pose[2],  # Z: keep current z
                current_tcp_pose[3],  # Rx: keep current orientation
                current_tcp_pose[4],  # Ry: keep current orientation
                current_tcp_pose[5]   # Rz: keep current orientation
            ]
            
            self.get_logger().info(f'Target pose: {[f"{p:.4f}" for p in target_pose]}')
            self.get_logger().info(f'Executing movel command (a={a}, v={v})...')
            
            # Move to target pose using movel (linear movement)
            move_result = self.ur_robot.movel(target_pose, a=a, v=v)
            
            if move_result == 0:
                self.get_logger().info('✅ Successfully moved to target position!')
                return True
            else:
                self.get_logger().error(f'❌ Failed to move to target position (result: {move_result})')
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error moving to XY position: {str(e)}")
            self.get_logger().error(traceback.format_exc())
            return False

    def precise_triangulation_estimation(self, 
                                         ref_img="../temp/ur15_precise_location_data/ref_img.jpg",
                                         ref_pose="../temp/ur15_precise_location_data/ref_pose.json",
                                         ref_keypoints="../temp/ur15_precise_location_data/ref_keypoints.json",
                                         data_dir="../temp/ur15_precise_location_data",
                                         result_dir="../temp/ur15_precise_location_result"):
        """
        Perform precise triangulation estimation by capturing images from 5 viewpoints.
        """
        try:
            self.get_logger().info("=" * 80)
            self.get_logger().info("🎯 Starting Precise Triangulation Estimation")
            self.get_logger().info("=" * 80)
            
            # Create directories
            os.makedirs(data_dir, exist_ok=True)
            os.makedirs(result_dir, exist_ok=True)
            self.get_logger().info(f"Data directory: {data_dir}")
            self.get_logger().info(f"Result directory: {result_dir}")
            
            # Check if robot is connected
            if self.ur_robot is None:
                self.get_logger().error('UR robot not connected. Cannot perform precise estimation.')
                return None
            
            # Get current TCP pose as base position
            base_pose = self.ur_robot.get_actual_tcp_pose()
            if base_pose is None:
                self.get_logger().error('Failed to get current TCP pose')
                return None
            
            self.get_logger().info(f"Base position: {[f'{p:.4f}' for p in base_pose]}")
            
            # Define 5 capture positions relative to base
            # Format: (description, [dx, dy, dz, drx, dry, drz])
            capture_positions = [
                ("x+5cm", [0.05, 0, 0, 0, 0, 0]),
                ("z+2cm", [0, 0, 0.02, 0, 0, 0]),
                ("y+5cm", [0, 0.05, 0, 0, 0, 0]),
                ("y-5cm", [0, -0.05, 0, 0, 0, 0]),
                ("z-5cm", [0, 0, -0.05, 0, 0, 0])
            ]
            
            # Check tracker availability and setup
            if self.tracker is None:
                self.get_logger().error('FlowFormer++ tracker not available')
                return None
                
            camera_params_dir = "../temp/ur15_cam_calibration_result/ur15_camera_parameters"
            
            # Load reference keypoints
            self.get_logger().info("\n📖 Loading reference keypoints...")
            with open(ref_keypoints, 'r') as f:
                ref_keypoints_data = json.load(f)
            
            # Convert keypoints to list format expected by API
            # Check if it's the new format (with 'keypoints' array) or old format (dict)
            if 'keypoints' in ref_keypoints_data:
                # New format: {"keypoints": [...]}
                ref_keypoints_list = []
                for kp in ref_keypoints_data['keypoints']:
                    ref_keypoints_list.append({
                        'x': kp['x'],
                        'y': kp['y'],
                        'name': kp.get('name', f"Point {kp.get('id', len(ref_keypoints_list)+1)}")
                    })
            else:
                # Old format: {"keypoint_1": {...}, "keypoint_2": {...}}
                ref_keypoints_list = []
                for kp_name in sorted(ref_keypoints_data.keys()):
                    kp = ref_keypoints_data[kp_name]
                    ref_keypoints_list.append({
                        'x': kp['x'],
                        'y': kp['y'],
                        'name': kp_name
                    })
            
            self.get_logger().info(f"✓ Loaded {len(ref_keypoints_list)} reference keypoints")
            
            # Set reference image using tracker (before capture loop)
            try:
                self.get_logger().info("Setting reference image with keypoints...")
                
                # Load reference image
                ref_img_bgr = cv2.imread(ref_img)
                if ref_img_bgr is None:
                    self.get_logger().error(f"Failed to load reference image: {ref_img}")
                    return None
                
                # Convert BGR to RGB for tracker
                ref_img_rgb = cv2.cvtColor(ref_img_bgr, cv2.COLOR_BGR2RGB)
                
                # Set reference image using tracker
                set_result = self.tracker.set_reference_image(
                    image=ref_img_rgb,
                    keypoints=ref_keypoints_list,
                    image_name='ur_precise_location_reference'
                )
                
                if not set_result.get('success', False):
                    self.get_logger().error(f"Failed to set reference image: {set_result.get('error', 'Unknown error')}")
                    return None
                
                self.get_logger().info("✓ Reference image set successfully")
                
            except Exception as e:
                self.get_logger().error(f"Error setting reference image: {e}")
                self.get_logger().error(traceback.format_exc())
                return None
            
            # Capture images, poses and track keypoints (integrated loop)
            triangulation_data = []
            
            for idx, (desc, offset) in enumerate(capture_positions):
                self.get_logger().info(f"\n📸 Position {idx}: {desc}")
                
                # Calculate target pose
                target_pose = [
                    base_pose[0] + offset[0],
                    base_pose[1] + offset[1],
                    base_pose[2] + offset[2],
                    base_pose[3] + offset[3],
                    base_pose[4] + offset[4],
                    base_pose[5] + offset[5]
                ]
                
                # Move to target position
                self.get_logger().info(f"  Moving to {desc}: {[f'{p:.4f}' for p in target_pose]}")
                result = self.ur_robot.movel(target_pose, a=0.5, v=0.2)
                
                if result != 0:
                    self.get_logger().error(f"  Failed to move to position {idx} ({desc})")
                    # Try to return to base position
                    self.ur_robot.movel(base_pose, a=0.5, v=0.2)
                    return None
                
                # Wait for robot to settle
                time.sleep(2.0)
                
                # Capture image and pose
                metadata = {
                    "description": desc,
                    "offset": offset
                }
                
                capture_success = self.capture_image_and_pose(
                    save_dir=data_dir,
                    img_filename=f"{idx}.jpg",
                    pose_filename=f"{idx}.json",
                    metadata=metadata
                )
                
                if not capture_success:
                    self.get_logger().error(f"  Failed to capture data at position {idx} ({desc})")
                    # Try to return to base position
                    self.ur_robot.movel(base_pose, a=0.5, v=0.2)
                    return None
                
                # Wait for image stabilization
                time.sleep(1.0)
                
                # Immediately track keypoints in the captured image
                self.get_logger().info(f"  Tracking keypoints in image {idx}...")
                
                try:
                    # Read the just-captured image
                    image_path = os.path.join(data_dir, f"{idx}.jpg")
                    img_bgr = cv2.imread(image_path)
                    if img_bgr is None:
                        self.get_logger().error(f"  Failed to load captured image: {image_path}")
                        self.get_logger().warn(f"  Skipping this viewpoint...")
                        continue
                    
                    # Convert BGR to RGB for tracker
                    img_rgb = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2RGB)
                    
                    # Track keypoints using tracker
                    tracking_result = self.tracker.track_keypoints(
                        target_image=img_rgb,
                        reference_name='ur_precise_location_reference',
                        bidirectional=False,  # No need for bidirectional check here
                        return_flow=False
                    )
                    
                    if not tracking_result.get('success', False):
                        self.get_logger().error(f"  Failed to track keypoints in image {idx}: {tracking_result.get('error', 'Unknown error')}")
                        self.get_logger().warn(f"  Skipping this viewpoint...")
                        continue
                    
                    tracked_keypoints = tracking_result.get('tracked_keypoints', [])
                    
                    if len(tracked_keypoints) < 4:
                        self.get_logger().error(f"  Not enough keypoints tracked in image {idx}")
                        self.get_logger().warn(f"  Skipping this viewpoint...")
                        continue
                    
                    # Extract KP3 and KP4 (indices 2 and 3)
                    kp3 = tracked_keypoints[2]
                    kp4 = tracked_keypoints[3]
                    
                    # Store successful tracking result
                    pose_path = os.path.join(data_dir, f"{idx}.json")
                    triangulation_data.append({
                        "index": idx,
                        "description": desc,
                        "pose_path": pose_path,
                        "kp3": kp3,
                        "kp4": kp4
                    })
                    
                    self.get_logger().info(f"  ✓ Tracked: KP3=({kp3['x']:.1f}, {kp3['y']:.1f}), KP4=({kp4['x']:.1f}, {kp4['y']:.1f})")
                    
                except Exception as e:
                    self.get_logger().error(f"  Error tracking keypoints in image {idx}: {e}")
                    self.get_logger().warn(f"  Skipping this viewpoint...")
                    continue
            
            # Return to base position
            self.get_logger().info("\n↩️  Returning to base position...")
            self.ur_robot.movel(base_pose, a=0.5, v=0.2)
            time.sleep(1.0)
            
            if len(triangulation_data) < 2:
                self.get_logger().error("Not enough valid views for triangulation (need at least 2)")
                return None
            
            self.get_logger().info(f"\n✅ Successfully tracked keypoints in {len(triangulation_data)} images")
            
            # Perform triangulation for midpoint between KP3 and KP4
            self.get_logger().info("\n🔺 Triangulating midpoint between KP3 and KP4...")
            
            # Load camera parameters from calibration result
            calib_result_path = os.path.join(camera_params_dir, "ur15_cam_calibration_result.json")
            with open(calib_result_path, 'r') as f:
                calib_data = json.load(f)
            
            cam_matrix = np.array(calib_data['camera_matrix'])
            fx, fy = cam_matrix[0, 0], cam_matrix[1, 1]
            cx, cy = cam_matrix[0, 2], cam_matrix[1, 2]
            
            self.get_logger().info(f"✓ Loaded camera intrinsics: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
            
            # Load camera to end-effector transformation
            eye_in_hand_path = os.path.join(camera_params_dir, "ur15_cam_eye_in_hand_result.json")
            with open(eye_in_hand_path, 'r') as f:
                eye_in_hand_data = json.load(f)
            
            cam2end_matrix = np.array(eye_in_hand_data['cam2end_matrix'])
            self.get_logger().info(f"✓ Loaded camera extrinsics (cam2end transformation)")
            
            # Collect rays for triangulation
            rays = []
            origins = []
            
            for tri_data in triangulation_data:
                # Load pose
                with open(tri_data["pose_path"], 'r') as f:
                    pose_data = json.load(f)
                
                end2base_matrix = np.array(pose_data['end2base'])
                
                # Calculate camera to base transformation
                cam2base_matrix = end2base_matrix @ cam2end_matrix
                
                # Calculate midpoint pixel coordinates
                kp3 = tri_data["kp3"]
                kp4 = tri_data["kp4"]
                mid_u = (kp3['x'] + kp4['x']) / 2.0
                mid_v = (kp3['y'] + kp4['y']) / 2.0
                
                # Convert to normalized image coordinates
                x_norm = (mid_u - cx) / fx
                y_norm = (mid_v - cy) / fy
                
                # Ray in camera frame (normalized)
                ray_cam = np.array([x_norm, y_norm, 1.0])
                ray_cam = ray_cam / np.linalg.norm(ray_cam)
                
                # Transform to base frame
                ray_base = cam2base_matrix[:3, :3] @ ray_cam
                origin_base = cam2base_matrix[:3, 3]
                
                rays.append(ray_base)
                origins.append(origin_base)
            
            # Triangulate using least squares
            # For each ray: P = origin + t * direction
            # Find point P that minimizes distance to all rays
            
            A = []
            b = []
            
            for ray, origin in zip(rays, origins):
                # For each ray, we want to minimize |P - (origin + t*ray)|^2
                # This gives us: (I - ray*ray^T) * P = (I - ray*ray^T) * origin
                I = np.eye(3)
                ray_outer = np.outer(ray, ray)
                A.append(I - ray_outer)
                b.append((I - ray_outer) @ origin)
            
            A = np.vstack(A)
            b = np.hstack(b)
            
            # Solve least squares
            P_estimated, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
            
            self.get_logger().info(f"\n✅ Triangulation complete!")
            self.get_logger().info(f"Estimated 3D position: ({P_estimated[0]:.6f}, {P_estimated[1]:.6f}, {P_estimated[2]:.6f})")
            
            # Save result
            result = {
                "timestamp": datetime.now().isoformat(),
                "method": "multi_view_triangulation",
                "num_views": len(triangulation_data),
                "estimated_position": {
                    "x": float(P_estimated[0]),
                    "y": float(P_estimated[1]),
                    "z": float(P_estimated[2])
                },
                "views": triangulation_data,
                "reference_files": {
                    "ref_img": ref_img,
                    "ref_pose": ref_pose,
                    "ref_keypoints": ref_keypoints
                }
            }
            
            result_path = os.path.join(result_dir, "precise_triangulation_result.json")
            with open(result_path, 'w') as f:
                json.dump(result, f, indent=2)
            
            self.get_logger().info(f"✓ Saved result to: {result_path}")
            
            # Perform reprojection validation using the estimated 3D position
            self.get_logger().info("\n🔍 Performing reprojection validation...")
            self._perform_reprojection_validation(
                estimated_3d_position=P_estimated,
                triangulation_data=triangulation_data,
                cam_matrix=cam_matrix,
                cam2end_matrix=cam2end_matrix,
                data_dir=data_dir,
                result_dir=result_dir
            )
            
            self.get_logger().info("=" * 80)
            self.get_logger().info("✅ Precise Triangulation Estimation Complete")
            self.get_logger().info("=" * 80)
            
            # Clean up: remove reference image from tracker
            try:
                cleanup_result = self.tracker.remove_reference_image('ur_precise_location_reference')
                if cleanup_result.get('success', False):
                    self.get_logger().info("✓ Reference image cleaned up successfully")
            except Exception as e:
                self.get_logger().warn(f"Failed to cleanup reference image: {e}")
            
            return result
            
        except Exception as e:
            self.get_logger().error(f"Error in precise triangulation estimation: {e}")
            self.get_logger().error(traceback.format_exc())
            # Clean up: remove reference image from tracker even on error
            try:
                self.tracker.remove_reference_image('ur_precise_location_reference')
            except:
                pass
            return None

    def tcp_normalize(self, x_axis=[1, 0, 0], y_axis=[0, -1, 0], z_axis=[0, 0, -1], a=0.5, v=0.2):
        """
        Normalize TCP orientation according to specified axis alignment.
        """
        try:
            if self.ur_robot is None:
                self.get_logger().error('UR robot not connected. Cannot normalize TCP.')
                return False
            
            # Get current TCP pose
            current_pose = self.ur_robot.get_actual_tcp_pose()
            if current_pose is None:
                self.get_logger().error("Failed to get current TCP pose")
                return False
            
            # Extract position (keep the same)
            x, y, z = current_pose[0], current_pose[1], current_pose[2]
            
            # Build rotation matrix from the desired axes
            # The input axes define how TCP axes should align with base axes
            x_vec = np.array(x_axis, dtype=float)
            y_vec = np.array(y_axis, dtype=float)
            z_vec = np.array(z_axis, dtype=float)
            
            # Normalize the vectors
            x_vec = x_vec / np.linalg.norm(x_vec)
            y_vec = y_vec / np.linalg.norm(y_vec)
            z_vec = z_vec / np.linalg.norm(z_vec)
            
            # Check orthogonality
            dot_xy = np.dot(x_vec, y_vec)
            dot_xz = np.dot(x_vec, z_vec)
            dot_yz = np.dot(y_vec, z_vec)
            
            if abs(dot_xy) > 0.01 or abs(dot_xz) > 0.01 or abs(dot_yz) > 0.01:
                self.get_logger().warn(f'Input axes are not orthogonal! dot(x,y)={dot_xy:.4f}, dot(x,z)={dot_xz:.4f}, dot(y,z)={dot_yz:.4f}')
                # Orthogonalize using Gram-Schmidt process
                y_vec = y_vec - np.dot(y_vec, x_vec) * x_vec
                y_vec = y_vec / np.linalg.norm(y_vec)
                z_vec = np.cross(x_vec, y_vec)
                z_vec = z_vec / np.linalg.norm(z_vec)
                self.get_logger().info('Axes orthogonalized using Gram-Schmidt process')
            
            # Construct rotation matrix (columns are the axis vectors)
            R_target = np.column_stack([x_vec, y_vec, z_vec])
            
            # Verify it's a valid rotation matrix (det should be 1)
            det = np.linalg.det(R_target)
            if abs(det - 1.0) > 0.01:
                self.get_logger().error(f'Invalid rotation matrix! det={det:.4f}. Check if axes form a right-handed coordinate system.')
                return False
            
            # Convert rotation matrix to axis-angle representation (rotation vector)
            rx, ry, rz = self._matrix_to_rotvec(R_target)
            
            # Create target pose
            target_pose = [x, y, z, rx, ry, rz]
            
            self.get_logger().info(f"Current TCP pose: {[f'{p:.4f}' for p in current_pose]}")
            self.get_logger().info(f"Desired TCP axes - X: {x_axis}, Y: {y_axis}, Z: {z_axis}")
            self.get_logger().info(f"Target normalized pose: {[f'{p:.4f}' for p in target_pose]}")
            self.get_logger().info(f"Normalizing TCP orientation (a={a}, v={v})...")
            
            # Move to target pose using movel (linear movement in base frame)
            result = self.ur_robot.movel(target_pose, a=a, v=v)
            
            if result == 0:
                self.get_logger().info("✅ TCP normalized successfully")
                return True
            else:
                self.get_logger().error(f"❌ Failed to normalize TCP (result: {result})")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error normalizing TCP: {str(e)}")
            self.get_logger().error(traceback.format_exc())
            return False

    def tool_orientation_correction(self, z_rotation_deg=30, a=0.5, v=0.2):
        """
        Correct tool orientation by rotating around its Z-axis.
        """
        try:
            if self.ur_robot is None:
                self.get_logger().error('UR robot not connected. Cannot correct tool orientation.')
                return False
            
            # Get current TCP pose
            current_pose = self.ur_robot.get_actual_tcp_pose()
            if current_pose is None:
                self.get_logger().error("Failed to get current TCP pose")
                return False
            
            # Extract position and orientation
            x, y, z = current_pose[0], current_pose[1], current_pose[2]
            rx, ry, rz = current_pose[3], current_pose[4], current_pose[5]
            
            # Get current rotation matrix
            R_current = self._rotvec_to_matrix(rx, ry, rz)
            
            # Create rotation matrix for Z-axis rotation (in tool frame)
            # Rotation around Z-axis
            angle_rad = math.radians(z_rotation_deg)
            Rz_tool = np.array([
                [math.cos(angle_rad), -math.sin(angle_rad), 0],
                [math.sin(angle_rad),  math.cos(angle_rad), 0],
                [0,                    0,                   1]
            ])
            
            # Apply rotation: R_new = R_current * Rz_tool
            # (rotate in tool frame means post-multiply)
            R_new = np.matmul(R_current, Rz_tool)
            
            # Convert back to rotation vector
            rx_new, ry_new, rz_new = self._matrix_to_rotvec(R_new)
            
            # Create target pose
            target_pose = [x, y, z, rx_new, ry_new, rz_new]
            
            self.get_logger().info(f"Current TCP pose: {[f'{p:.4f}' for p in current_pose]}")
            self.get_logger().info(f"Rotating tool {z_rotation_deg}° around Z-axis...")
            self.get_logger().info(f"Target pose: {[f'{p:.4f}' for p in target_pose]}")
            
            # Move to target pose using movel
            result = self.ur_robot.movel(target_pose, a=a, v=v)
            
            if result == 0:
                self.get_logger().info(f"✅ Tool orientation corrected successfully (rotated {z_rotation_deg}° around Z-axis)")
                return True
            else:
                self.get_logger().error(f"❌ Failed to correct tool orientation (result: {result})")
                return False
                
        except Exception as e:
            self.get_logger().error(f"Error correcting tool orientation: {str(e)}")
            self.get_logger().error(traceback.format_exc())
            return False

    def _visualize_bidirectional_errors(self, test_img_path, tracked_keypoints, 
                                       forward_backward_errors, critical_indices, threshold):
        """
        Visualize bidirectional tracking errors on the test image.
        
        Args:
            test_img_path: Path to test image
            tracked_keypoints: List of tracked keypoint dicts with 'x', 'y'
            forward_backward_errors: List of FB errors for each keypoint
            critical_indices: List of critical keypoint indices (0-based)
            threshold: Error threshold in pixels
            
        Returns:
            Annotated image (BGR) or None if failed
        """
        try:
            # Load test image
            img = cv2.imread(test_img_path)
            if img is None:
                self.get_logger().error(f"Failed to load image: {test_img_path}")
                return None
            
            # Create overlay
            overlay = img.copy()
            
            # Draw keypoints with color coding
            for idx, (kp, error) in enumerate(zip(tracked_keypoints, forward_backward_errors)):
                x, y = int(kp['x']), int(kp['y'])
                is_critical = idx in critical_indices
                
                # Color: Green if pass, Red if fail
                if error <= threshold:
                    color = (0, 255, 0)  # Green - PASS
                else:
                    color = (0, 0, 255)  # Red - FAIL
                
                # Draw circle (larger for critical keypoints)
                radius = 8 if is_critical else 5
                thickness = 3 if is_critical else 2
                cv2.circle(overlay, (x, y), radius, color, thickness)
                
                # Draw marker for critical keypoints
                if is_critical:
                    cv2.drawMarker(overlay, (x, y), color, cv2.MARKER_CROSS, 15, 2)
                
                # Draw label with keypoint number and error
                label = f"KP{idx+1}"
                if is_critical:
                    label += " 🎯"
                label += f": {error:.2f}px"
                
                # Background for text
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.5
                font_thickness = 1
                (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)
                
                text_x = x + 10
                text_y = y - 10
                if text_y < text_h + 5:
                    text_y = y + 20
                
                cv2.rectangle(overlay, 
                             (text_x - 2, text_y - text_h - 2),
                             (text_x + text_w + 2, text_y + baseline + 2),
                             (0, 0, 0), -1)
                cv2.putText(overlay, label, (text_x, text_y), font, font_scale, (255, 255, 255), font_thickness)
            
            # Add legend
            legend_y = 30
            avg_error = np.mean(forward_backward_errors)
            max_error = np.max(forward_backward_errors)
            
            cv2.putText(overlay, "Bidirectional Validation", (10, legend_y), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(overlay, f"Avg Error: {avg_error:.3f}px", (10, legend_y + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(overlay, f"Max Error: {max_error:.3f}px", (10, legend_y + 45),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(overlay, f"Threshold: {threshold:.1f}px", (10, legend_y + 65),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Status indicator
            status_text = "PASS" if avg_error < 2.0 else "FAIL"
            status_color = (0, 255, 0) if avg_error < 2.0 else (0, 0, 255)
            cv2.putText(overlay, f"Status: {status_text}", (10, legend_y + 90),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
            
            return overlay
            
        except Exception as e:
            self.get_logger().error(f"Error creating bidirectional visualization: {e}")
            return None

    def _visualize_reprojection_errors(self, test_img_path, reproj_data):
        """
        Visualize reprojection errors on the test image.
        
        Args:
            test_img_path: Path to test image
            reproj_data: Reprojection data dictionary from validation
            
        Returns:
            Annotated image (BGR) or None if failed
        """
        try:
            # Load test image
            img = cv2.imread(test_img_path)
            if img is None:
                self.get_logger().error(f"Failed to load image: {test_img_path}")
                return None
            
            overlay = img.copy()
            threshold = reproj_data.get('threshold_px', 1.0)
            
            # Draw keypoints
            for kp_name, kp_data in reproj_data['keypoints'].items():
                observed = kp_data['observed_pixel']
                reprojected = kp_data['reprojected_pixel']
                error = kp_data['reprojection_error_px']
                
                obs_x, obs_y = int(observed['u']), int(observed['v'])
                repr_x, repr_y = int(reprojected['u']), int(reprojected['v'])
                
                # Color based on error
                color = (0, 255, 0) if error <= threshold else (0, 0, 255)
                
                # Draw observed point (solid circle)
                cv2.circle(overlay, (obs_x, obs_y), 8, color, 2)
                cv2.drawMarker(overlay, (obs_x, obs_y), color, cv2.MARKER_CROSS, 15, 2)
                
                # Draw reprojected point (cross)
                cv2.circle(overlay, (repr_x, repr_y), 5, (255, 0, 255), -1)
                
                # Draw line connecting observed and reprojected
                cv2.line(overlay, (obs_x, obs_y), (repr_x, repr_y), (255, 255, 0), 2)
                
                # Label
                label = kp_name.replace('test_', '').upper()
                label += f": {error:.2f}px"
                
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.5
                font_thickness = 1
                (text_w, text_h), baseline = cv2.getTextSize(label, font, font_scale, font_thickness)
                
                text_x = obs_x + 15
                text_y = obs_y - 10
                if text_y < text_h + 5:
                    text_y = obs_y + 25
                
                cv2.rectangle(overlay,
                             (text_x - 2, text_y - text_h - 2),
                             (text_x + text_w + 2, text_y + baseline + 2),
                             (0, 0, 0), -1)
                cv2.putText(overlay, label, (text_x, text_y), font, font_scale, (255, 255, 255), font_thickness)
            
            # Add legend
            legend_y = 30
            max_error = reproj_data.get('max_error_px', 0)
            status = "PASS" if reproj_data.get('pass', False) else "FAIL"
            status_color = (0, 255, 0) if reproj_data.get('pass', False) else (0, 0, 255)
            
            cv2.putText(overlay, "Reprojection Validation", (10, legend_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(overlay, f"Max Error: {max_error:.3f}px", (10, legend_y + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(overlay, f"Threshold: {threshold:.1f}px", (10, legend_y + 45),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.putText(overlay, f"Status: {status}", (10, legend_y + 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, status_color, 2)
            
            # Legend for markers
            legend_x = img.shape[1] - 250
            cv2.putText(overlay, "Legend:", (legend_x, legend_y),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            cv2.circle(overlay, (legend_x + 10, legend_y + 20), 5, (0, 255, 0), 2)
            cv2.putText(overlay, "Observed", (legend_x + 20, legend_y + 25),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            cv2.circle(overlay, (legend_x + 10, legend_y + 40), 5, (255, 0, 255), -1)
            cv2.putText(overlay, "Reprojected", (legend_x + 20, legend_y + 45),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
            
            return overlay
            
        except Exception as e:
            self.get_logger().error(f"Error creating reprojection visualization: {e}")
            return None
    
    def _perform_reprojection_validation(self, estimated_3d_position, triangulation_data, 
                                        cam_matrix, cam2end_matrix, data_dir, result_dir):
        """
        Perform reprojection validation of the estimated 3D position on all captured images.
        
        Args:
            estimated_3d_position: Estimated 3D position in base frame [x, y, z]
            triangulation_data: Data from triangulation including pose information
            cam_matrix: Camera intrinsic matrix
            cam2end_matrix: Camera to end-effector transformation matrix
            data_dir: Directory containing captured images and poses
            result_dir: Directory to save reprojection validation results
        """
        try:
            self.get_logger().info("🔍 Starting reprojection validation...")
            
            # Create validation results
            validation_results = {
                "timestamp": datetime.now().isoformat(),
                "estimated_3d_position": {
                    "x": float(estimated_3d_position[0]),
                    "y": float(estimated_3d_position[1]),
                    "z": float(estimated_3d_position[2])
                },
                "reprojections": []
            }
            
            fx, fy = cam_matrix[0, 0], cam_matrix[1, 1]
            cx, cy = cam_matrix[0, 2], cam_matrix[1, 2]
            
            # Process each captured image
            for idx, tri_data in enumerate(triangulation_data):
                image_path = os.path.join(data_dir, f"{idx}.jpg")
                pose_path = tri_data["pose_path"]
                
                self.get_logger().info(f"  Processing image {idx}: {tri_data['description']}")
                
                # Load image
                img = cv2.imread(image_path)
                if img is None:
                    self.get_logger().error(f"Failed to load image: {image_path}")
                    continue
                
                # Load pose data
                with open(pose_path, 'r') as f:
                    pose_data = json.load(f)
                
                end2base_matrix = np.array(pose_data['end2base'])
                
                # Calculate camera to base transformation
                cam2base_matrix = end2base_matrix @ cam2end_matrix
                
                # Transform 3D position from base frame to camera frame
                P_base_homogeneous = np.array([
                    estimated_3d_position[0],
                    estimated_3d_position[1], 
                    estimated_3d_position[2],
                    1.0
                ])
                
                # base -> camera transformation
                base2cam_matrix = np.linalg.inv(cam2base_matrix)
                P_cam = base2cam_matrix @ P_base_homogeneous
                
                Xc, Yc, Zc = P_cam[:3]
                
                if Zc <= 1e-9:
                    self.get_logger().warn(f"Point is behind camera in view {idx}, skipping")
                    continue
                
                # Project to image coordinates
                u_proj = fx * (Xc / Zc) + cx
                v_proj = fy * (Yc / Zc) + cy
                
                # Calculate midpoint of observed keypoints for comparison
                kp3 = tri_data["kp3"]
                kp4 = tri_data["kp4"]
                u_observed = (kp3['x'] + kp4['x']) / 2.0
                v_observed = (kp3['y'] + kp4['y']) / 2.0
                
                # Calculate reprojection error
                error_u = u_proj - u_observed
                error_v = v_proj - v_observed
                error_pixel = np.sqrt(error_u**2 + error_v**2)
                
                # Store reprojection result
                reprojection_result = {
                    "view_index": idx,
                    "description": tri_data['description'],
                    "observed_midpoint": {"u": float(u_observed), "v": float(v_observed)},
                    "reprojected_point": {"u": float(u_proj), "v": float(v_proj)},
                    "error_pixels": float(error_pixel),
                    "error_components": {"u": float(error_u), "v": float(error_v)}
                }
                
                validation_results["reprojections"].append(reprojection_result)
                
                # Create visualization
                img_vis = img.copy()
                
                # Draw observed midpoint (green circle)
                cv2.circle(img_vis, (int(u_observed), int(v_observed)), 8, (0, 255, 0), 2)
                cv2.putText(img_vis, "Observed", (int(u_observed) + 10, int(v_observed) - 10),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
                
                # Draw reprojected point (magenta filled circle)
                cv2.circle(img_vis, (int(u_proj), int(v_proj)), 6, (255, 0, 255), -1)
                cv2.putText(img_vis, "Reprojected", (int(u_proj) + 10, int(v_proj) + 20),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 255), 2)
                
                # Draw connection line
                cv2.line(img_vis, (int(u_observed), int(v_observed)), 
                        (int(u_proj), int(v_proj)), (255, 255, 0), 1)
                
                # Draw individual keypoints (KP3 and KP4) for reference
                cv2.circle(img_vis, (int(kp3['x']), int(kp3['y'])), 4, (0, 255, 255), 2)
                cv2.circle(img_vis, (int(kp4['x']), int(kp4['y'])), 4, (0, 255, 255), 2)
                cv2.putText(img_vis, "KP3", (int(kp3['x']) + 5, int(kp3['y']) - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                cv2.putText(img_vis, "KP4", (int(kp4['x']) + 5, int(kp4['y']) - 5),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 255), 1)
                
                # Add info text
                info_y = 30
                cv2.putText(img_vis, f"View {idx}: {tri_data['description']}", (10, info_y),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(img_vis, f"Reprojection Error: {error_pixel:.2f} px", (10, info_y + 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)
                cv2.putText(img_vis, f"3D Position: ({estimated_3d_position[0]:.4f}, {estimated_3d_position[1]:.4f}, {estimated_3d_position[2]:.4f})",
                           (10, info_y + 60), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
                
                # Save visualization
                vis_filename = f"reprojection_validation_{idx}.jpg"
                vis_path = os.path.join(result_dir, vis_filename)
                cv2.imwrite(vis_path, img_vis)
                
                self.get_logger().info(f"    Error: {error_pixel:.2f} px, Saved: {vis_filename}")
            
            # Calculate overall statistics
            if validation_results["reprojections"]:
                errors = [r["error_pixels"] for r in validation_results["reprojections"]]
                validation_results["statistics"] = {
                    "mean_error_pixels": float(np.mean(errors)),
                    "max_error_pixels": float(np.max(errors)),
                    "min_error_pixels": float(np.min(errors)),
                    "std_error_pixels": float(np.std(errors)),
                    "num_views": len(errors)
                }
                
                self.get_logger().info(f"\n📊 Reprojection validation statistics:")
                self.get_logger().info(f"  Mean error: {validation_results['statistics']['mean_error_pixels']:.3f} px")
                self.get_logger().info(f"  Max error:  {validation_results['statistics']['max_error_pixels']:.3f} px")
                self.get_logger().info(f"  Min error:  {validation_results['statistics']['min_error_pixels']:.3f} px")
                self.get_logger().info(f"  Std error:  {validation_results['statistics']['std_error_pixels']:.3f} px")
            
            # Save validation results
            validation_path = os.path.join(result_dir, "reprojection_validation_result.json")
            with open(validation_path, 'w') as f:
                json.dump(validation_results, f, indent=2)
            
            self.get_logger().info(f"✅ Reprojection validation completed. Results saved to: {validation_path}")
            
        except Exception as e:
            self.get_logger().error(f"Error in reprojection validation: {e}")
            self.get_logger().error(traceback.format_exc())
    
    def _generate_location_log_file(self, precise_result):
        """
        Generate location log file after successful correction.
        
        Args:
            precise_result: Result from precise triangulation estimation
        """
        try:
            if self.ur_robot is None:
                self.get_logger().error('UR robot not connected. Cannot capture current state.')
                return False
            
            # Get current TCP pose
            current_tcp_pose = self.ur_robot.get_actual_tcp_pose()
            if current_tcp_pose is None:
                self.get_logger().error('Failed to get current TCP pose for location log')
                return False
            
            # Get current joint positions
            current_joint_positions = self.ur_robot.get_actual_joint_positions()
            if current_joint_positions is None:
                self.get_logger().error('Failed to get current joint positions for location log')
                return False
            
            # Create location log data structure
            location_log = {
                "timestamp": datetime.now().isoformat(),
                "status": "correct_success_completed",
                "current_robot_state": {
                    "tcp_pose": {
                        "x": float(current_tcp_pose[0]),
                        "y": float(current_tcp_pose[1]),
                        "z": float(current_tcp_pose[2]),
                        "rx": float(current_tcp_pose[3]),
                        "ry": float(current_tcp_pose[4]),
                        "rz": float(current_tcp_pose[5])
                    },
                    "joint_positions": {
                        "joint_1": float(current_joint_positions[0]),
                        "joint_2": float(current_joint_positions[1]),
                        "joint_3": float(current_joint_positions[2]),
                        "joint_4": float(current_joint_positions[3]),
                        "joint_5": float(current_joint_positions[4]),
                        "joint_6": float(current_joint_positions[5])
                    },
                    "joint_positions_degrees": {
                        "joint_1": float(np.degrees(current_joint_positions[0])),
                        "joint_2": float(np.degrees(current_joint_positions[1])),
                        "joint_3": float(np.degrees(current_joint_positions[2])),
                        "joint_4": float(np.degrees(current_joint_positions[3])),
                        "joint_5": float(np.degrees(current_joint_positions[4])),
                        "joint_6": float(np.degrees(current_joint_positions[5]))
                    }
                },
                "estimated_position": {
                    "x": float(precise_result['estimated_position']['x']),
                    "y": float(precise_result['estimated_position']['y']),
                    "z": float(precise_result['estimated_position']['z'])
                },
                "estimation_metadata": {
                    "method": precise_result.get('method', 'unknown'),
                    "num_views": precise_result.get('num_views', 0),
                    "estimation_timestamp": precise_result.get('timestamp', 'unknown')
                }
            }
            
            # Save to result directory
            result_dir = "../temp/ur15_precise_location_result"
            os.makedirs(result_dir, exist_ok=True)
            
            log_filename = f"location_log_file.json"
            log_path = os.path.join(result_dir, log_filename)
            
            with open(log_path, 'w') as f:
                json.dump(location_log, f, indent=2, ensure_ascii=False)
            
            self.get_logger().info(f"✅ Location log file saved: {log_path}")
            self.get_logger().info(f"📊 Current TCP position: ({current_tcp_pose[0]:.4f}, {current_tcp_pose[1]:.4f}, {current_tcp_pose[2]:.4f})")
            self.get_logger().info(f"🎯 Estimated position: ({precise_result['estimated_position']['x']:.4f}, {precise_result['estimated_position']['y']:.4f}, {precise_result['estimated_position']['z']:.4f})")
            
            return True
            
        except Exception as e:
            self.get_logger().error(f"Error generating location log file: {e}")
            self.get_logger().error(traceback.format_exc())
            return False


def main(args=None):
    """
    Main function to initialize and run the location task node
    """
    rclpy.init(args=args)
    
    # You can customize these parameters
    robot_ip = "192.168.1.15"  # UR robot IP address
    robot_port = 30002          # UR robot port
    api_url = "http://10.172.151.12:8001"  # FlowFormer++ API URL
    
    try:
        task_node = URLocationTask(
            robot_ip=robot_ip,
            robot_port=robot_port,
            api_url=api_url
        )
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(task_node)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        task_node.get_logger().info('Node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        # Example: Call the capture function
        success = task_node.capture_image_and_pose()
        
        if success:
            task_node.get_logger().info('Capture completed successfully!')
        else:
            task_node.get_logger().error('Capture failed!')
        
        # Example: Uncomment to run handles estimation
        result = task_node.estimate_handles_xy_coordinates(
            ref_img="../temp/ur15_location_task_data/ref_img.jpg",
            ref_pose="../temp/ur15_location_task_data/ref_pose.json",
            ref_keypoints="../temp/ur15_location_task_data/ref_keypoints.json",
            test_img="../temp/ur15_location_task_data/test_img.jpg",
            test_pose="../temp/ur15_location_task_data/test_pose.json"
        )
        if result:
            task_node.get_logger().info('Estimation completed successfully!')
            
            # Move robot to the midpoint between keypoint 3 and keypoint 4
            task_node.get_logger().info('Moving robot to midpoint between KP3 and KP4...')
            
            # Get estimated positions from test_image (the actual estimation result)
            test_kp3 = result['test_image']['keypoint_3']['base_coordinates_2d']
            test_kp4 = result['test_image']['keypoint_4']['base_coordinates_2d']
            
            # Calculate midpoint
            midpoint_x = (test_kp3['x'] + test_kp4['x']) / 2
            midpoint_y = (test_kp3['y'] + test_kp4['y']) / 2
            
            task_node.get_logger().info(f'Estimated KP3 position: ({test_kp3["x"]:.4f}, {test_kp3["y"]:.4f})')
            task_node.get_logger().info(f'Estimated KP4 position: ({test_kp4["x"]:.4f}, {test_kp4["y"]:.4f})')
            task_node.get_logger().info(f'Midpoint: ({midpoint_x:.4f}, {midpoint_y:.4f})')
            
            # Move to midpoint using the new method
            move_success = task_node.move_to_xy_position(midpoint_x, midpoint_y, a=0.2, v=0.2)
            time.sleep(5)  # Wait a moment

            # If move was successful, normalize TCP orientation
            if move_success:
                task_node.get_logger().info('Move successful! Normalizing TCP orientation...')
                # TCP x-axis+ aligns with base y-axis- (same direction)
                # TCP y-axis+ aligns with base x-axis- (opposite direction)
                # TCP z-axis+ aligns with base z-axis- (opposite direction, pointing downward)
                normalize_success = task_node.tcp_normalize(
                    x_axis=[0, -1, 0], 
                    y_axis=[-1, 0, 0], 
                    z_axis=[0, 0, -1],
                    a=0.2, 
                    v=0.2
                )
                time.sleep(2)
                if normalize_success:
                    task_node.get_logger().info('✅ TCP normalization completed!')
                    
                    # Move to a closer position for finer estimation
                    task_node.get_logger().info('\n🔍 Moving closer for precise estimation...')
                    move_result = task_node.ur_robot.move_tcp([0, 0.1, 0.25, 0, 0, 0], a=0.5, v=0.2)
                    time.sleep(2)
                    
                    if move_result == 0:
                        task_node.get_logger().info('✅ Moved to closer position!')
                        
                        # Perform precise triangulation estimation
                        task_node.get_logger().info('\n🎯 Starting precise triangulation estimation...')
                        precise_result = task_node.precise_triangulation_estimation(
                            ref_img="../temp/ur15_precise_location_data/ref_img.jpg",
                            ref_pose="../temp/ur15_precise_location_data/ref_pose.json",
                            ref_keypoints="../temp/ur15_precise_location_data/ref_keypoints.json",
                            data_dir="../temp/ur15_precise_location_data",
                            result_dir="../temp/ur15_precise_location_result"
                        )
                        
                        if precise_result:
                            task_node.get_logger().info('✅ Precise triangulation estimation completed!')
                            task_node.get_logger().info(f"Precise estimated position: "
                                                       f"({precise_result['estimated_position']['x']:.6f}, "
                                                       f"{precise_result['estimated_position']['y']:.6f}, "
                                                       f"{precise_result['estimated_position']['z']:.6f})")
                            
                            # Move to the precise estimated position (keep Z unchanged)
                            task_node.get_logger().info('\n📍 Moving to precise estimated position...')
                            precise_x = precise_result['estimated_position']['x']
                            precise_y = precise_result['estimated_position']['y']
                            
                            precise_move_success = task_node.move_to_xy_position(
                                precise_x, 
                                precise_y, 
                                a=0.2, 
                                v=0.1
                            )
                            time.sleep(2)
                            
                            if precise_move_success:
                                task_node.get_logger().info('✅ Successfully moved to precise estimated position!')
                                
                                # Perform tool orientation correction (rotate 30 degrees around Z-axis)
                                task_node.get_logger().info('Performing tool orientation correction...')
                                correction_success = task_node.tool_orientation_correction(
                                    z_rotation_deg=30,
                                    a=0.5,
                                    v=0.2
                                )
                                time.sleep(2)
                                
                                if correction_success:
                                    task_node.get_logger().info('✅ Tool orientation correction completed!')
                                    
                                    # Generate location log file after successful correction
                                    task_node.get_logger().info('📝 Generating location log file...')
                                    task_node._generate_location_log_file(precise_result)
                                    
                                    # # Perform Z-axis movement to estimated 3D position Z
                                    # task_node.get_logger().info('\n📏 Moving to estimated Z position...')
                                    
                                    # # Get current TCP pose
                                    # current_tcp_pose = task_node.ur_robot.get_actual_tcp_pose()
                                    # if current_tcp_pose is not None:
                                    #     current_z = current_tcp_pose[2]  # Adjust for tool length if needed
                                    #     estimated_z = precise_result['estimated_position']['z']
                                    #     target_z = estimated_z+0.2265-0.015
                                        
                                    #     task_node.get_logger().info(f'Current TCP Z: {current_z:.6f} m')
                                    #     task_node.get_logger().info(f'Target Z (estimated): {target_z:.6f} m')
                                    #     task_node.get_logger().info(f'Z displacement: {target_z - current_z:.6f} m')
                                        
                                    #     # Create target pose with only Z changed
                                    #     target_pose_z = [
                                    #         current_tcp_pose[0],  # X: keep current
                                    #         current_tcp_pose[1],  # Y: keep current
                                    #         target_z,             # Z: interpolated value
                                    #         current_tcp_pose[3],  # Rx: keep current orientation
                                    #         current_tcp_pose[4],  # Ry: keep current orientation
                                    #         current_tcp_pose[5]   # Rz: keep current orientation
                                    #     ]
                                        
                                    #     task_node.get_logger().info(f'Moving in Z direction to: {[f"{p:.4f}" for p in target_pose_z]}')
                                    #     z_move_result = task_node.ur_robot.movel(target_pose_z, a=0.2, v=0.05)
                                    #     time.sleep(2)
                                        
                                    #     if z_move_result == 0:
                                    #         task_node.get_logger().info('✅ Z-axis movement completed successfully!')
                                            
                                    #         # Verify final position
                                    #         final_tcp_pose = task_node.ur_robot.get_actual_tcp_pose()
                                    #         if final_tcp_pose is not None:
                                    #             task_node.get_logger().info(f'Final TCP pose: {[f"{p:.4f}" for p in final_tcp_pose]}')
                                    #             task_node.get_logger().info(f'Final Z position: {final_tcp_pose[2]:.6f} m')
                                    #             task_node.get_logger().info(f'Distance to estimated position: '
                                    #                                       f'X={abs(final_tcp_pose[0] - precise_x):.6f} m, '
                                    #                                       f'Y={abs(final_tcp_pose[1] - precise_y):.6f} m, '
                                    #                                       f'Z={abs(final_tcp_pose[2] - estimated_z):.6f} m')
                                    #     else:
                                    #         task_node.get_logger().warn(f'⚠️ Z-axis movement failed with result code: {z_move_result}')
                                    # else:
                                    #     task_node.get_logger().warn('⚠️ Failed to get current TCP pose for Z-axis movement.')
                                else:
                                    task_node.get_logger().warn('⚠️ Tool orientation correction failed.')
                            else:
                                task_node.get_logger().warn('⚠️ Failed to move to precise estimated position.')
                        else:
                            task_node.get_logger().warn('⚠️ Precise triangulation estimation failed.')
                    else:
                        task_node.get_logger().warn('⚠️ Failed to move to closer position.')
                else:
                    task_node.get_logger().warn('⚠️ TCP normalization failed, but position is correct.')

            

        # Keep node alive for a moment to ensure all logging is displayed
        time.sleep(1)
        
    except KeyboardInterrupt:
        print('\nCtrl+C pressed. Shutting down...')
    finally:
        # Clean up
        if 'task_node' in locals():
            task_node.cleanup()
            task_node.destroy_node()
        
        if 'executor' in locals():
            executor.shutdown()
        
        rclpy.shutdown()


if __name__ == '__main__':
    main()
