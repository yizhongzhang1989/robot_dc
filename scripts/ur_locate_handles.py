#!/usr/bin/env python3
"""
Handle Location Tracking Script
"""

import os
import sys
import json
import base64
import time
import logging
import requests
import cv2
import numpy as np
from typing import Dict, List, Optional, Tuple
import traceback

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(levelname)s - %(message)s'
)
logger = logging.getLogger(__name__)


class HandleLocationTracker:
    """Handle location tracker using FlowFormer++ API."""
    
    def __init__(self, api_url: str = "http://msraig-ubuntu-2:8001",
                 data_dir: str = None,
                 result_dir: str = None,
                 camera_params_dir: str = None):
        """Initialize the handle location tracker.
        
        Args:
            api_url: Base URL for the FlowFormer++ API
            data_dir: Path to handles location data directory (default: robot_dc/temp/ur15_handles_location_data)
            result_dir: Path to handles location result directory (default: robot_dc/temp/ur15_handles_location_result)
            camera_params_dir: Path to camera parameters directory (default: robot_dc/temp/ur15_cam_calibration_result/ur15_camera_parameters)
        """
        self.api_url = api_url.rstrip('/')
        self.session = requests.Session()
        self.session.timeout = 60  # 60 second timeout
        
        # Setup data paths with defaults
        project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        temp_dir = os.path.join(project_root, "temp")
        
        # Use provided paths or default paths
        if data_dir is None:
            self.data_dir = os.path.join(temp_dir, "ur15_handles_location_data")
        else:
            self.data_dir = os.path.abspath(data_dir)
        
        if result_dir is None:
            self.result_dir = os.path.join(temp_dir, "ur15_handles_location_result")
        else:
            self.result_dir = os.path.abspath(result_dir)
        
        # Setup file paths
        self.ref_img_path = os.path.join(self.data_dir, "ref_img.jpg")
        self.ref_keypoints_path = os.path.join(self.data_dir, "ref_keypoints.json")
        self.ref_pose_path = os.path.join(self.data_dir, "ref_pose.json")
        self.test_pose_path = os.path.join(self.data_dir, "test_pose.json")
        
        # Camera parameters paths
        if camera_params_dir is None:
            self.camera_params_dir = os.path.join(temp_dir, "ur15_cam_calibration_result", "ur15_camera_parameters")
        else:
            self.camera_params_dir = os.path.abspath(camera_params_dir)
        
        self.calibration_result_path = os.path.join(self.camera_params_dir, "calibration_result.json")
        self.eye_in_hand_result_path = os.path.join(self.camera_params_dir, "eye_in_hand_result.json")
        
        # Create result directory if it doesn't exist
        if not os.path.exists(self.result_dir):
            os.makedirs(self.result_dir)
            logger.info(f"Created result directory: {self.result_dir}")
        
        logger.info(f"Initialized HandleLocationTracker")
        logger.info(f"API URL: {self.api_url}")
        logger.info(f"Data directory: {self.data_dir}")
        logger.info(f"Result directory: {self.result_dir}")
        logger.info(f"Camera parameters directory: {self.camera_params_dir}")
    
    def check_api_status(self) -> bool:
        """Check if the API is ready.
        
        Returns:
            bool: True if API is ready, False otherwise
        """
        try:
            response = self.session.get(f"{self.api_url}/status")
            if response.status_code == 200:
                data = response.json()
                logger.info(f"API Status: {data}")
                return data.get('status') == 'ready'
            else:
                logger.error(f"Status check failed with code: {response.status_code}")
                return False
        except Exception as e:
            logger.error(f"Failed to check API status: {str(e)}")
            return False
    
    def load_reference_keypoints(self) -> List[Dict]:
        """Load reference keypoints from JSON file.
        
        Returns:
            List[Dict]: List of keypoint dictionaries with x, y coordinates
        """
        try:
            if not os.path.exists(self.ref_keypoints_path):
                raise FileNotFoundError(f"Reference keypoints file not found: {self.ref_keypoints_path}")
            
            with open(self.ref_keypoints_path, 'r') as f:
                data = json.load(f)
            
            # Extract keypoints in the format expected by API (simple x, y format)
            keypoints = []
            for kp in data.get('keypoints', []):
                keypoints.append({
                    'x': float(kp['x']),
                    'y': float(kp['y'])
                })
            
            logger.info(f"Loaded {len(keypoints)} reference keypoints")
            for i, kp in enumerate(keypoints, 1):
                logger.info(f"  Keypoint {i}: ({kp['x']:.2f}, {kp['y']:.2f})")
            
            return keypoints
            
        except Exception as e:
            logger.error(f"Failed to load reference keypoints: {str(e)}")
            raise
    
    def encode_image_to_base64(self, image_path: str) -> str:
        """Encode image file to base64 string.
        
        Args:
            image_path: Path to the image file
            
        Returns:
            str: Base64 encoded image string
        """
        with open(image_path, 'rb') as f:
            image_data = f.read()
            return base64.b64encode(image_data).decode('utf-8')
    
    def set_reference_image(self, keypoints: List[Dict]) -> bool:
        """Set reference image with keypoints using the API.
        
        Args:
            keypoints: List of keypoint dictionaries with x, y coordinates
            
        Returns:
            bool: True if successful, False otherwise
        """
        try:
            if not os.path.exists(self.ref_img_path):
                raise FileNotFoundError(f"Reference image not found: {self.ref_img_path}")
            
            logger.info("Encoding reference image to base64...")
            image_base64 = self.encode_image_to_base64(self.ref_img_path)
            
            # Prepare JSON request data
            data = {
                'image_base64': image_base64,
                'keypoints': keypoints,
                'image_name': 'handle_reference'
            }
            
            logger.info(f"Sending reference image with {len(keypoints)} keypoints...")
            response = self.session.post(
                f"{self.api_url}/set_reference_image",
                json=data
            )
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success', False):
                    logger.info(f"✅ Reference image set successfully")
                    if result.get('result'):
                        logger.info(f"   Message: {result['result'].get('message', 'N/A')}")
                    return True
                else:
                    logger.error(f"API returned success=False: {result.get('message', 'Unknown error')}")
                    return False
            else:
                logger.error(f"Failed to set reference image. Status: {response.status_code}")
                logger.error(f"Response: {response.text}")
                return False
                
        except Exception as e:
            logger.error(f"Error setting reference image: {str(e)}")
            logger.error(traceback.format_exc())
            return False
    
    def track_keypoints_in_image(self, target_image_path: str, 
                                  reference_name: str = 'handle_reference',
                                  bidirectional: bool = True) -> Optional[Dict]:
        """Track keypoints in a target image using the API.
        
        Args:
            target_image_path: Path to the target image
            reference_name: Name of the reference image to use
            bidirectional: Whether to enable bidirectional validation
            
        Returns:
            Dict: Tracking result or None if failed
        """
        try:
            if not os.path.exists(target_image_path):
                raise FileNotFoundError(f"Target image not found: {target_image_path}")
            
            logger.info(f"Encoding target image: {os.path.basename(target_image_path)}")
            image_base64 = self.encode_image_to_base64(target_image_path)
            
            # Prepare JSON request data
            data = {
                'image_base64': image_base64,
                'reference_name': reference_name,
                'bidirectional': bidirectional,
                'return_flow': False
            }
            
            logger.info(f"Tracking keypoints in {os.path.basename(target_image_path)}...")
            response = self.session.post(
                f"{self.api_url}/track_keypoints",
                json=data
            )
            
            if response.status_code == 200:
                result = response.json()
                if result.get('success', False):
                    tracking_data = result.get('result', {})
                    tracked_kps = tracking_data.get('tracked_keypoints', [])
                    logger.info(f"✅ Tracked {len(tracked_kps)} keypoints")
                    logger.info(f"   Processing time: {tracking_data.get('total_processing_time', 0):.3f}s")
                    if 'flow_magnitude' in tracking_data:
                        logger.info(f"   Flow magnitude: {tracking_data.get('flow_magnitude', 0):.2f}")
                    
                    # Check for bidirectional stats and log detailed info
                    if 'bidirectional_stats' in tracking_data:
                        logger.info(f"   ✅ Bidirectional stats available")
                    else:
                        logger.warning(f"   ⚠️ No bidirectional_stats in response")
                        logger.warning(f"   Available keys: {list(tracking_data.keys())}")
                        logger.warning(f"   Request had bidirectional={bidirectional}")
                    
                    return tracking_data
                else:
                    logger.error(f"Tracking failed: {result.get('message', 'Unknown error')}")
                    if result.get('error'):
                        logger.error(f"Error details: {result['error']}")
                    return None
            else:
                logger.error(f"API request failed. Status: {response.status_code}")
                logger.error(f"Response: {response.text}")
                return None
                
        except Exception as e:
            logger.error(f"Error tracking keypoints in {target_image_path}: {str(e)}")
            logger.error(traceback.format_exc())
            return None
    
    def save_tracking_result(self, image_name: str, tracking_data: Dict, 
                            original_keypoints: List[Dict]) -> bool:
        """Save tracking result to JSON file.
        
        Args:
            image_name: Name of the target image (without extension)
            tracking_data: Tracking result data from API
            original_keypoints: Original reference keypoints for comparison
            
        Returns:
            bool: True if saved successfully, False otherwise
        """
        try:
            output_path = os.path.join(self.result_dir, f"{image_name}_tracking_result.json")
            
            # Get tracked keypoints (API returns as list of {"x": value, "y": value} objects)
            tracked_kps_raw = tracking_data.get('tracked_keypoints', [])
            
            # Extract forward-backward errors for individual keypoints
            forward_backward_errors = []
            for kp in tracked_kps_raw:
                if isinstance(kp, dict) and 'consistency_distance' in kp:
                    forward_backward_errors.append(float(kp['consistency_distance']))
                else:
                    forward_backward_errors.append(None)  # No error data available
            
            # Format tracked keypoints with labels and forward-backward errors
            tracked_keypoints = []
            for i, kp in enumerate(tracked_kps_raw, 1):
                if isinstance(kp, dict) and 'x' in kp and 'y' in kp:
                    keypoint_data = {
                        'id': i,
                        'name': f'Handle Point {i}',
                        'x': float(kp['x']),
                        'y': float(kp['y']),
                        'coordinates_type': 'image_pixels'
                    }
                    
                    # Add forward-backward error if available
                    if 'consistency_distance' in kp:
                        keypoint_data['forward_backward_error'] = float(kp['consistency_distance'])
                    
                    tracked_keypoints.append(keypoint_data)
            
            # Extract bidirectional validation results if available
            bidirectional_validation = None
            if 'bidirectional_stats' in tracking_data:
                bidirectional_stats = tracking_data['bidirectional_stats']
                
                # API uses different field names: mean/max/min_consistency_distance instead of forward_backward_error
                mean_error = bidirectional_stats.get('mean_consistency_distance', 0)
                max_error = bidirectional_stats.get('max_consistency_distance', 0)
                min_error = bidirectional_stats.get('min_consistency_distance', 0)
                consistent_kps = bidirectional_stats.get('consistent_keypoints', 0)
                total_kps = bidirectional_stats.get('total_keypoints', 0)
                
                bidirectional_validation = {
                    "enabled": True,
                    "mean_consistency_distance": float(mean_error),
                    "max_consistency_distance": float(max_error),
                    "min_consistency_distance": float(min_error),
                    "validation_passed": mean_error < 1.0,
                    "statistics": {
                        "total_keypoints": int(total_kps),
                        "consistent_keypoints": int(consistent_kps),
                        "consistency_rate": float(consistent_kps / total_kps) if total_kps > 0 else 0.0,
                        "error_threshold": 1.0
                    }
                }
                logger.info(f"   ✅ Bidirectional validation: mean={mean_error:.3f}px, consistent={consistent_kps}/{total_kps}")
            
            # Log individual keypoint forward-backward errors
            if forward_backward_errors:
                valid_errors = [e for e in forward_backward_errors if e is not None]
                if valid_errors:
                    logger.info(f"   📊 Individual keypoint FB errors:")
                    for i, error in enumerate(forward_backward_errors, 1):
                        if error is not None:
                            status = "✅" if error < 2.0 else "❌"
                            logger.info(f"      KP{i}: {error:.3f} px {status}")
                        else:
                            logger.info(f"      KP{i}: N/A (no FB data)")
            
            # Format the result
            result = {
                "image_file": f"{image_name}.jpg",
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S'),
                "tracking_result": {
                    "success": True,
                    "tracked_keypoints": tracked_keypoints,
                    "keypoints_count": len(tracked_keypoints),
                    "processing_time": tracking_data.get('total_processing_time', 0),
                    "flow_magnitude": tracking_data.get('flow_magnitude', 0),
                    "reference_used": tracking_data.get('reference_used', 'handle_reference'),
                    "forward_backward_errors": forward_backward_errors
                },
                "reference_keypoints": [
                    {
                        'id': i,
                        'name': f'Handle Point {i}',
                        'x': kp['x'],
                        'y': kp['y']
                    }
                    for i, kp in enumerate(original_keypoints, 1)
                ],
                "displacement": self._calculate_displacement(original_keypoints, tracked_keypoints),
                "bidirectional_validation": bidirectional_validation
            }
            
            with open(output_path, 'w') as f:
                json.dump(result, f, indent=2, ensure_ascii=False)
            
            logger.info(f"💾 Saved tracking result to: {output_path}")
            return True
            
        except Exception as e:
            logger.error(f"Failed to save tracking result for {image_name}: {str(e)}")
            logger.error(traceback.format_exc())
            return False
    
    def _calculate_displacement(self, ref_kps: List[Dict], tracked_kps: List[Dict]) -> List[Dict]:
        """Calculate displacement between reference and tracked keypoints.
        
        Args:
            ref_kps: Reference keypoints
            tracked_kps: Tracked keypoints
            
        Returns:
            List[Dict]: List of displacement information
        """
        displacements = []
        for i, (ref, tracked) in enumerate(zip(ref_kps, tracked_kps), 1):
            dx = tracked['x'] - ref['x']
            dy = tracked['y'] - ref['y']
            distance = np.sqrt(dx**2 + dy**2)
            
            displacements.append({
                'keypoint_id': i,
                'dx': float(dx),
                'dy': float(dy),
                'distance': float(distance)
            })
        
        return displacements
    
    def find_calibration_file(self, base_name: str, possible_prefixes: List[str]) -> Optional[str]:
        """Find calibration file with various naming conventions.
        
        Args:
            base_name: Base file name (e.g., 'calibration_result.json')
            possible_prefixes: List of possible prefixes (e.g., ['ur15_cam_', 'ur15_', 'ur_cam_', 'ur_'])
            
        Returns:
            str: Path to the found file, or None if not found
        """
        # First try without prefix
        test_path = os.path.join(self.camera_params_dir, base_name)
        if os.path.exists(test_path):
            return test_path
        
        # Try with each prefix
        for prefix in possible_prefixes:
            test_path = os.path.join(self.camera_params_dir, f"{prefix}{base_name}")
            if os.path.exists(test_path):
                logger.info(f"Found calibration file: {prefix}{base_name}")
                return test_path
        
        return None
    
    def load_camera_parameters(self) -> Tuple[np.ndarray, np.ndarray, np.ndarray]:
        """Load camera intrinsic parameters and hand-eye calibration matrix.
        
        Returns:
            Tuple[np.ndarray, np.ndarray, np.ndarray]: camera_matrix, dist_coeffs, cam2end_matrix
        """
        try:
            # Try to find eye-in-hand calibration result file with various naming conventions
            possible_prefixes = ['ur15_cam_', 'ur15_', 'ur_cam_', 'ur_', 'duco_cam_', 'duco_']
            eye_in_hand_path = self.find_calibration_file('eye_in_hand_result.json', possible_prefixes)
            
            if eye_in_hand_path is None:
                raise FileNotFoundError(
                    f"Eye-in-hand calibration result not found in {self.camera_params_dir}. "
                    f"Looking for: eye_in_hand_result.json with prefixes like ur15_cam_, ur15_, ur_cam_, ur_, etc."
                )
            
            # Load eye-in-hand calibration result
            with open(eye_in_hand_path, 'r') as f:
                eye_in_hand_data = json.load(f)
            
            camera_matrix = np.array(eye_in_hand_data['camera_matrix'])
            dist_coeffs = np.array(eye_in_hand_data['distortion_coefficients'])
            cam2end_matrix = np.array(eye_in_hand_data['cam2end_matrix'])
            
            logger.info("✅ Loaded camera parameters")
            logger.info(f"   File: {os.path.basename(eye_in_hand_path)}")
            logger.info(f"   Camera matrix shape: {camera_matrix.shape}")
            logger.info(f"   Cam2End matrix shape: {cam2end_matrix.shape}")
            
            return camera_matrix, dist_coeffs, cam2end_matrix
            
        except Exception as e:
            logger.error(f"Failed to load camera parameters: {str(e)}")
            raise
    
    def load_robot_pose(self, pose_file_path: str) -> np.ndarray:
        """Load robot end-effector pose (end2base matrix).
        
        Args:
            pose_file_path: Path to the pose JSON file
            
        Returns:
            np.ndarray: 4x4 end2base transformation matrix
        """
        try:
            with open(pose_file_path, 'r') as f:
                pose_data = json.load(f)
            
            end2base_matrix = np.array(pose_data['end2base'])
            
            logger.info(f"✅ Loaded robot pose from {os.path.basename(pose_file_path)}")
            logger.info(f"   End position: ({pose_data['end_xyzrpy']['x']:.4f}, "
                       f"{pose_data['end_xyzrpy']['y']:.4f}, {pose_data['end_xyzrpy']['z']:.4f})")
            
            return end2base_matrix
            
        except Exception as e:
            logger.error(f"Failed to load robot pose from {pose_file_path}: {str(e)}")
            raise
    
    def estimate_keypoint_3d_position(self, pixel_u: float, pixel_v: float, 
                                     camera_matrix: np.ndarray, 
                                     dist_coeffs: np.ndarray,
                                     cam2end_matrix: np.ndarray,
                                     end2base_matrix: np.ndarray,
                                     plane_z: float = 0.0) -> Tuple[float, float, float]:
        """Estimate 3D position of a keypoint in base frame.
        
        Assumes the keypoint lies on a plane parallel to the base XOY plane at height plane_z.
        
        Args:
            pixel_u: Pixel u coordinate (x in image)
            pixel_v: Pixel v coordinate (y in image)
            camera_matrix: Camera intrinsic matrix (3x3)
            dist_coeffs: Distortion coefficients
            cam2end_matrix: Camera to end-effector transformation (4x4)
            end2base_matrix: End-effector to base transformation (4x4)
            plane_z: Z coordinate of the plane in base frame (default: 0.0)
            
        Returns:
            Tuple[float, float, float]: (x, y, z) position in base frame
        """
        # Step 1: Undistort the pixel coordinates
        pixel_point = np.array([[[pixel_u, pixel_v]]], dtype=np.float32)
        undistorted_point = cv2.undistortPoints(pixel_point, camera_matrix, dist_coeffs, P=camera_matrix)
        u_undist, v_undist = undistorted_point[0, 0]
        
        # Step 2: Convert to normalized image coordinates
        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        
        x_norm = (u_undist - cx) / fx
        y_norm = (v_undist - cy) / fy
        
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
            logger.warning("Ray is parallel to the plane, cannot compute intersection")
            return None, None, None
        
        t = (plane_z - camera_origin_base[2]) / ray_base[2]
        
        if t < 0:
            logger.warning(f"Intersection point is behind camera (t={t})")
        
        # Compute intersection point
        point_base = camera_origin_base + t * ray_base
        
        return float(point_base[0]), float(point_base[1]), float(point_base[2])
    
    def estimate_handles_location(self, ref_tracking_result: Dict, test_tracking_result: Dict) -> Dict:
        """Estimate 3D locations of keypoints 3 and 4 in base frame.
        
        Args:
            ref_tracking_result: Tracking result for reference image (original keypoints)
            test_tracking_result: Tracking result for test image
            
        Returns:
            Dict: Estimation results with 3D coordinates
        """
        try:
            logger.info("\n🔍 Estimating handle locations in base frame...")
            
            # Load camera parameters
            camera_matrix, dist_coeffs, cam2end_matrix = self.load_camera_parameters()
            
            # Load robot poses
            ref_end2base = self.load_robot_pose(self.ref_pose_path)
            test_end2base = self.load_robot_pose(self.test_pose_path)
            
            # Extract keypoints 3 and 4 from both reference and test images
            ref_keypoints = ref_tracking_result['tracking_result']['tracked_keypoints']
            test_keypoints = test_tracking_result['tracking_result']['tracked_keypoints']
            
            if len(ref_keypoints) < 4 or len(test_keypoints) < 4:
                raise ValueError("Need at least 4 keypoints for estimation")
            
            # Keypoint 3 and 4 (indices 2 and 3)
            ref_kp3 = ref_keypoints[2]  # id=3
            ref_kp4 = ref_keypoints[3]  # id=4
            test_kp3 = test_keypoints[2]
            test_kp4 = test_keypoints[3]
            
            logger.info(f"\nReference image keypoints:")
            logger.info(f"  KP3: ({ref_kp3['x']:.2f}, {ref_kp3['y']:.2f})")
            logger.info(f"  KP4: ({ref_kp4['x']:.2f}, {ref_kp4['y']:.2f})")
            
            logger.info(f"\nTest image keypoints:")
            logger.info(f"  KP3: ({test_kp3['x']:.2f}, {test_kp3['y']:.2f})")
            logger.info(f"  KP4: ({test_kp4['x']:.2f}, {test_kp4['y']:.2f})")
            
            # Estimate 3D positions (assuming z=0 plane in base frame)
            plane_z = 0.0
            
            # Estimate from reference image
            ref_kp3_base = self.estimate_keypoint_3d_position(
                ref_kp3['x'], ref_kp3['y'], camera_matrix, dist_coeffs,
                cam2end_matrix, ref_end2base, plane_z
            )
            ref_kp4_base = self.estimate_keypoint_3d_position(
                ref_kp4['x'], ref_kp4['y'], camera_matrix, dist_coeffs,
                cam2end_matrix, ref_end2base, plane_z
            )
            
            # Estimate from test image
            test_kp3_base = self.estimate_keypoint_3d_position(
                test_kp3['x'], test_kp3['y'], camera_matrix, dist_coeffs,
                cam2end_matrix, test_end2base, plane_z
            )
            test_kp4_base = self.estimate_keypoint_3d_position(
                test_kp4['x'], test_kp4['y'], camera_matrix, dist_coeffs,
                cam2end_matrix, test_end2base, plane_z
            )
            
            logger.info(f"\n✅ 3D Position estimation completed:")
            logger.info(f"\nReference image (base frame):")
            logger.info(f"  KP3: ({ref_kp3_base[0]:.4f}, {ref_kp3_base[1]:.4f}, {ref_kp3_base[2]:.4f})")
            logger.info(f"  KP4: ({ref_kp4_base[0]:.4f}, {ref_kp4_base[1]:.4f}, {ref_kp4_base[2]:.4f})")
            
            logger.info(f"\nTest image (base frame):")
            logger.info(f"  KP3: ({test_kp3_base[0]:.4f}, {test_kp3_base[1]:.4f}, {test_kp3_base[2]:.4f})")
            logger.info(f"  KP4: ({test_kp4_base[0]:.4f}, {test_kp4_base[1]:.4f}, {test_kp4_base[2]:.4f})")
            
            # Calculate average position
            avg_kp3_x = (ref_kp3_base[0] + test_kp3_base[0]) / 2
            avg_kp3_y = (ref_kp3_base[1] + test_kp3_base[1]) / 2
            avg_kp4_x = (ref_kp4_base[0] + test_kp4_base[0]) / 2
            avg_kp4_y = (ref_kp4_base[1] + test_kp4_base[1]) / 2
            
            logger.info(f"\n📊 Average position (2D in base frame):")
            logger.info(f"  KP3: ({avg_kp3_x:.4f}, {avg_kp3_y:.4f})")
            logger.info(f"  KP4: ({avg_kp4_x:.4f}, {avg_kp4_y:.4f})")
            
            # Prepare result dictionary
            result = {
                "success": True,
                "timestamp": time.strftime('%Y-%m-%d %H:%M:%S'),
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
                "average_position": {
                    "keypoint_3": {"x": avg_kp3_x, "y": avg_kp3_y},
                    "keypoint_4": {"x": avg_kp4_x, "y": avg_kp4_y}
                },
                "camera_parameters": {
                    "camera_matrix": camera_matrix.tolist(),
                    "distortion_coefficients": dist_coeffs.tolist()
                }
            }
            
            # Save estimation result
            output_path = os.path.join(self.result_dir, "handles_location_estimation_result.json")
            with open(output_path, 'w') as f:
                json.dump(result, f, indent=2, ensure_ascii=False)
            
            logger.info(f"\n💾 Saved estimation result to: {output_path}")
            
            return result
            
        except Exception as e:
            logger.error(f"Failed to estimate handle locations: {str(e)}")
            logger.error(traceback.format_exc())
            return None
    
    def draw_keypoints_comparison(self, image_path: str, tracking_data: Dict,
                                  original_keypoints: List[Dict], output_name: str) -> bool:
        """Draw comparison of reference and tracked keypoints on image.
        
        Args:
            image_path: Path to the target image
            tracking_data: Tracking result data
            original_keypoints: Original reference keypoints
            output_name: Name for the output image (without extension)
            
        Returns:
            bool: True if visualization saved successfully, False otherwise
        """
        try:
            # Load the image
            if not os.path.exists(image_path):
                logger.error(f"Image file not found: {image_path}")
                return False
            
            image = cv2.imread(image_path)
            if image is None:
                logger.error(f"Failed to load image: {image_path}")
                return False
            
            # Get tracked keypoints
            tracked_kps_raw = tracking_data.get('tracked_keypoints', [])
            if not tracked_kps_raw:
                logger.warning("No tracked keypoints found for visualization")
                return False
            
            # Define color for tracked keypoints (green)
            tracked_color = (0, 255, 0)  # Green
            
            # Draw tracked keypoints in green
            for i, kp in enumerate(tracked_kps_raw, 1):
                if isinstance(kp, dict) and 'x' in kp and 'y' in kp:
                    x, y = int(kp['x']), int(kp['y'])
                    
                    # Draw keypoint circle
                    cv2.circle(image, (x, y), 8, tracked_color, -1)
                    cv2.circle(image, (x, y), 10, (255, 255, 255), 2)
                    
                    # Draw keypoint label
                    cv2.putText(image, f"P{i}", (x + 15, y - 10), 
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, tracked_color, 2)
            
            # Add summary information
            processing_time = tracking_data.get('total_processing_time', 0)
            
            summary_text = f"Tracked: {len(tracked_kps_raw)} keypoints"
            time_text = f"Processing Time: {processing_time:.3f}s"
            
            # Draw summary background
            cv2.rectangle(image, (10, 10), (300, 55), (0, 0, 0), -1)
            cv2.putText(image, summary_text, (20, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            cv2.putText(image, time_text, (20, 48), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1)
            
            # Save visualization result
            output_path = os.path.join(self.result_dir, f"{output_name}_tracking_visualization.jpg")
            success = cv2.imwrite(output_path, image)
            
            if success:
                logger.info(f"🎨 Saved visualization: {output_path}")
                return True
            else:
                logger.error(f"Failed to save visualization: {output_path}")
                return False
                
        except Exception as e:
            logger.error(f"Error creating visualization: {str(e)}")
            logger.error(traceback.format_exc())
            return False
    
    def process_target_images(self, image_names: List[str] = None) -> Dict[str, Dict]:
        """Process target images for keypoint tracking.
        
        Args:
            image_names: List of image names (without extension). If None, auto-detect
            
        Returns:
            Dict[str, Dict]: Dictionary mapping image names to result dict with success status and tracking data
        """
        if image_names is None:
            # Auto-detect images in the data directory
            image_names = []
            for filename in os.listdir(self.data_dir):
                if filename.endswith('.jpg') and filename not in ['ref_img.jpg']:
                    image_names.append(filename[:-4])  # Remove .jpg extension
        
        if not image_names:
            logger.warning("No target images found to process")
            return {}
        
        results = {}
        
        logger.info(f"\n🚀 Starting keypoint tracking for {len(image_names)} images")
        logger.info("="*60)
        
        for image_name in image_names:
            logger.info(f"\n📸 Processing image: {image_name}.jpg")
            logger.info("-"*40)
            
            target_path = os.path.join(self.data_dir, f"{image_name}.jpg")
            
            if not os.path.exists(target_path):
                logger.warning(f"Image file not found: {target_path}")
                results[image_name] = {'success': False, 'tracking_data': None}
                continue
            
            # Track keypoints with bidirectional validation enabled
            tracking_data = self.track_keypoints_in_image(target_path, bidirectional=True)
            
            if tracking_data:
                # Load original keypoints for comparison
                original_kps = self.load_reference_keypoints()
                
                # Log bidirectional validation results if available
                if 'bidirectional_stats' in tracking_data:
                    bidirectional_stats = tracking_data['bidirectional_stats']
                    mean_error = bidirectional_stats.get('mean_consistency_distance', 0)
                    max_error = bidirectional_stats.get('max_consistency_distance', 0)
                    consistent_kps = bidirectional_stats.get('consistent_keypoints', 0)
                    total_kps = bidirectional_stats.get('total_keypoints', 0)
                    logger.info(f"🔄 Bidirectional validation:")
                    logger.info(f"   Mean consistency distance: {mean_error:.3f} pixels")
                    logger.info(f"   Max consistency distance: {max_error:.3f} pixels")
                    logger.info(f"   Consistent keypoints: {consistent_kps}/{total_kps}")
                    logger.info(f"   Status: {'✅ PASSED' if mean_error < 1.0 else '⚠️ WARNING'}")
                
                # Save result
                save_success = self.save_tracking_result(image_name, tracking_data, original_kps)
                
                # Create visualization
                viz_success = self.draw_keypoints_comparison(
                    target_path, tracking_data, original_kps, image_name
                )
                
                success = save_success and viz_success
                results[image_name] = {'success': success, 'tracking_data': tracking_data}
                
                if success:
                    logger.info(f"✅ Successfully processed {image_name}.jpg")
                else:
                    logger.warning(f"⚠️ Partially failed for {image_name}.jpg")
            else:
                results[image_name] = {'success': False, 'tracking_data': None}
                logger.error(f"❌ Failed to track keypoints in {image_name}.jpg")
        
        return results
    
    def run_tracking_pipeline(self) -> bool:
        """Run the complete handle location tracking pipeline.
        
        Returns:
            bool: True if pipeline completed successfully, False otherwise
        """
        try:
            logger.info("🎯 Handle Location Tracking Pipeline")
            logger.info("="*60)
            logger.info(f"Using API: {self.api_url}")
            logger.info("="*60)
            
            # Step 1: Check API status
            logger.info("\n📡 Step 1: Checking API status...")
            if not self.check_api_status():
                logger.error("❌ API is not ready. Please ensure the service is running.")
                logger.info("You can check the API status at: " + self.api_url + "/docs")
                return False
            logger.info("✅ API is ready")
            
            # Step 2: Load reference keypoints
            logger.info("\n📋 Step 2: Loading reference keypoints...")
            keypoints = self.load_reference_keypoints()
            logger.info(f"✅ Loaded {len(keypoints)} keypoints")
            
            # Step 3: Set reference image
            logger.info("\n🖼️  Step 3: Setting reference image...")
            if not self.set_reference_image(keypoints):
                logger.error("❌ Failed to set reference image")
                return False
            logger.info("✅ Reference image set successfully")
            
            # Step 4: Process target images (with bidirectional validation)
            logger.info("\n🔍 Step 4: Processing target images with bidirectional validation...")
            results = self.process_target_images()
            
            # Step 5: Estimate 3D positions of keypoints 3 and 4
            logger.info("\n📐 Step 5: Estimating 3D positions...")
            estimation_result = None
            if 'test_img' in results and results['test_img']['success']:
                try:
                    # Get tracking data with bidirectional validation
                    test_tracking_data = results['test_img']['tracking_data']
                    
                    # Check bidirectional validation on test image
                    skip_estimation = False
                    if 'bidirectional_stats' in test_tracking_data:
                        bidirectional_stats = test_tracking_data['bidirectional_stats']
                        mean_error = bidirectional_stats.get('mean_consistency_distance', 0)
                        if mean_error >= 2.0:
                            logger.error(f"❌ Test image bidirectional consistency distance = {mean_error:.3f} pixels (>= 2.0)")
                            logger.error("Keypoint tracking is too unreliable, skipping 3D coordinate estimation")
                            skip_estimation = True
                        elif mean_error >= 1.0:
                            logger.warning(f"⚠️ Test image bidirectional consistency distance = {mean_error:.3f} pixels (>= 1.0)")
                            logger.warning("Keypoint tracking may be unreliable")
                    
                    if not skip_estimation:
                        # Load the reference and test tracking results
                        test_result_path = os.path.join(self.result_dir, "test_img_tracking_result.json")
                        
                        if os.path.exists(test_result_path):
                            with open(test_result_path, 'r') as f:
                                test_result = json.load(f)
                            
                            # Use original reference keypoints (not tracked)
                            original_keypoints = self.load_reference_keypoints()
                            
                            ref_result = {
                                "tracking_result": {
                                    "tracked_keypoints": [
                                        {
                                            "id": i,
                                            "x": kp['x'],
                                            "y": kp['y']
                                        }
                                        for i, kp in enumerate(original_keypoints, 1)
                                    ]
                                }
                            }
                            
                            estimation_result = self.estimate_handles_location(ref_result, test_result)
                            
                            if estimation_result:
                                logger.info("✅ 3D position estimation completed successfully")
                            else:
                                logger.warning("⚠️ 3D position estimation failed")
                        else:
                            logger.warning("Test tracking result not found, skipping 3D estimation")
                    else:
                        logger.info("🚫 Skipping 3D estimation due to high bidirectional consistency error")
                    
                except Exception as e:
                    logger.error(f"Error during 3D estimation: {str(e)}")
                    logger.error(traceback.format_exc())
            else:
                logger.info("Skipping 3D estimation (test_img not processed successfully)")
            
            # Step 6: Summary
            logger.info("\n" + "="*60)
            logger.info("📊 FINAL SUMMARY")
            logger.info("="*60)
            
            if not results:
                logger.warning("No images were processed")
                return False
            
            successful = sum(1 for result in results.values() if result['success'])
            total = len(results)
            
            logger.info(f"Total images processed: {total}")
            logger.info(f"Successful: {successful}")
            logger.info(f"Failed: {total - successful}")
            logger.info(f"Success rate: {(successful/total*100):.1f}%")
            
            logger.info("\nDetailed results:")
            for image_name, result in results.items():
                status = "✅ Success" if result['success'] else "❌ Failed"
                logger.info(f"  {image_name}.jpg: {status}")
                
                # Show bidirectional validation results if available
                if result['success'] and result['tracking_data']:
                    tracking_data = result['tracking_data']
                    if 'bidirectional_stats' in tracking_data:
                        bidirectional_stats = tracking_data['bidirectional_stats']
                        mean_error = bidirectional_stats.get('mean_consistency_distance', 0)
                        logger.info(f"    Bidirectional consistency: {mean_error:.3f} pixels")
            
            if successful > 0:
                logger.info(f"\n📁 Results saved to: {self.result_dir}")
                logger.info("Files created:")
                for image_name, result in results.items():
                    if result['success']:
                        logger.info(f"  - {image_name}_tracking_result.json")
                        logger.info(f"  - {image_name}_tracking_visualization.jpg")
                
                if estimation_result:
                    logger.info(f"  - handles_location_estimation_result.json")
                    logger.info(f"\n🎯 Estimated handle positions (base frame 2D):")
                    avg_kp3 = estimation_result['average_position']['keypoint_3']
                    avg_kp4 = estimation_result['average_position']['keypoint_4']
                    logger.info(f"  Keypoint 3: x={avg_kp3['x']:.4f}m, y={avg_kp3['y']:.4f}m")
                    logger.info(f"  Keypoint 4: x={avg_kp4['x']:.4f}m, y={avg_kp4['y']:.4f}m")
            
            pipeline_success = successful == total
            if pipeline_success:
                logger.info("\n🎉 Handle location tracking pipeline completed successfully!")
            else:
                logger.warning("\n⚠️ Pipeline completed with some failures.")
            
            return pipeline_success
            
        except Exception as e:
            logger.error(f"Pipeline failed with error: {str(e)}")
            logger.error(traceback.format_exc())
            return False


def main():
    """Main function to run the handle location tracking."""
    import argparse
    
    logger.info("🎯 Handle Location Tracking - FlowFormer++ API")
    logger.info("="*70)
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(
        description='Handle Location Tracking using FlowFormer++ API',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Use default paths and API URL
  python3 duco_locate_handles.py
  
  # Specify custom API URL
  python3 duco_locate_handles.py --api-url http://localhost:8001
  
  # Specify custom data and result directories
  python3 duco_locate_handles.py --data-dir /path/to/data --result-dir /path/to/results
  
  # Specify custom camera parameters directory
  python3 duco_locate_handles.py --camera-params /path/to/camera_params
  
  # Specify all parameters
  python3 duco_locate_handles.py --api-url http://server:8001 --data-dir /path/to/data --result-dir /path/to/results --camera-params /path/to/camera_params
        """
    )
    
    # Get default paths
    project_root = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    default_data_dir = os.path.join(project_root, "temp", "ur15_handles_location_data")
    default_result_dir = os.path.join(project_root, "temp", "ur15_handles_location_result")
    default_camera_params_dir = os.path.join(project_root, "temp", "ur15_cam_calibration_result", "ur15_camera_parameters")
    
    parser.add_argument(
        '--api-url',
        type=str,
        default="http://msraig-ubuntu-2:8001",
        help='FlowFormer++ API base URL (default: http://msraig-ubuntu-2:8001)'
    )
    
    parser.add_argument(
        '--data-dir',
        type=str,
        default=None,
        help=f'Path to handles location data directory (default: {default_data_dir})'
    )
    
    parser.add_argument(
        '--result-dir',
        type=str,
        default=None,
        help=f'Path to handles location result directory (default: {default_result_dir})'
    )
    
    parser.add_argument(
        '--camera-params',
        type=str,
        default=None,
        help=f'Path to camera parameters directory (default: {default_camera_params_dir})'
    )
    
    args = parser.parse_args()
    
    # Display configuration
    logger.info(f"Configuration:")
    logger.info(f"  API URL: {args.api_url}")
    logger.info(f"  Data directory: {args.data_dir if args.data_dir else default_data_dir}")
    logger.info(f"  Result directory: {args.result_dir if args.result_dir else default_result_dir}")
    logger.info(f"  Camera parameters directory: {args.camera_params if args.camera_params else default_camera_params_dir}")
    logger.info("="*70)
    
    # Initialize tracker
    tracker = HandleLocationTracker(
        api_url=args.api_url,
        data_dir=args.data_dir,
        result_dir=args.result_dir,
        camera_params_dir=args.camera_params
    )
    
    # Run the tracking pipeline
    success = tracker.run_tracking_pipeline()
    
    if success:
        logger.info("\n✅ Task completed successfully!")
        sys.exit(0)
    else:
        logger.error("\n❌ Task failed!")
        sys.exit(1)


if __name__ == "__main__":
    main()
