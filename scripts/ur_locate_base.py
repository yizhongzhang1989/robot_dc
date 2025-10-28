from ur15_robot_arm.ur15 import UR15Robot
import time
import socket
import math
import numpy as np
import json
import os
import sys
import cv2
from datetime import datetime
import threading
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d import Axes3D
import requests

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Import the FFPPWebAPIKeypointTracker
sys.path.append(os.path.join(os.path.dirname(__file__), 'ThirdParty', 'robot_vision'))
from core.ffpp_webapi_keypoint_tracker import FFPPWebAPIKeypointTracker


class URLocateBase(Node):
    def __init__(self, api_url="http://10.172.151.12:8001", robot_ip="192.168.1.15", robot_port=30002):
        """
        Initialize URLocateBase class for UR robot location test tasks
        
        Args:
            api_url (str): URL for the FlowFormer++ Web API service
            robot_ip (str): IP address of the UR15 robot
            robot_port (int): Port number of the UR15 robot
        """
        # Initialize ROS node
        super().__init__('ur_locate_test')
        
        # Get the script directory for relative paths
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # API URL for FlowFormer++ service
        self.api_url = api_url

        # Lift platform web service base URL
        self.lift_web_base = "http://192.168.1.3:8090"
        
        # Robot connection parameters
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.robot = None
        
        # Camera-related attributes
        self.cv_bridge = CvBridge()
        self.latest_image = None
        
        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribe to camera topic
        self.image_subscription = self.create_subscription(
            Image,
            '/ur15_camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Camera parameters paths
        self.intrinsic_params_path = os.path.join(
            self.script_dir, '..', 'temp', 'ur15_cam_calibration_result',
            'ur15_camera_parameters', 'ur15_cam_calibration_result.json'
        )
        self.extrinsic_params_path = os.path.join(
            self.script_dir, '..', 'temp', 'ur15_cam_calibration_result',
            'ur15_camera_parameters', 'ur15_cam_eye_in_hand_result.json'
        )
        
        # Camera parameters storage
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.cam2end_matrix = None
        self.target2base_matrix = None
        
        # Keypoint tracker
        self.tracker = None
        
        # Predefined collect position joint angles (radians)
        self.collect_start_position = [-4.666847888623373, -0.8728431028178711, 1.7595298925982874, -3.2923394642271937, -1.8568695227252405, 0.11747467517852783]
        
        # Data collection movement offsets (in base coordinate system, unit: meters)
        # Format: {movement_name: [delta_x, delta_y, delta_z, delta_rx, delta_ry, delta_rz]}
        self.movements = {
            "movement1": [0.03, 0, 0, 0, 0, 0],      # X_positive
            "movement2": [-0.03, 0, 0, 0, 0, 0],     # X_negative
            "movement3": [0, 0.03, 0, 0, 0, 0],      # Y_positive
            "movement4": [0, -0.03, 0, 0, 0, 0],     # Y_negative
            "movement5": [0, 0, -0.03, 0, 0, 0]      # Z_negative
        }
        
        # Data directory path (for storing collected data)
        self.data_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_test_data')
        
        # Reference data paths
        self.ref_img_path = os.path.join(self.data_dir, 'ref_img.jpg')
        self.ref_keypoints_path = os.path.join(self.data_dir, 'ref_keypoints.json')
        self.ref_pose_path = os.path.join(self.data_dir, 'ref_pose.json')
        
        # Result file names
        self.TRACKING_RESULT_FILENAME = "log_tracking_result.json"
        self.ESTIMATION_RESULT_FILENAME = "log_estimation_result.json"
        self.COORD_SYSTEM_RESULT_FILENAME = "log_local_coordinate_system_result.json"
        
        # Result directory path
        self.result_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_test_result')
        
        # Local coordinate system configuration
        # Defines which keypoints to use for building the local X-axis
        # Format: [start_keypoint_index, end_keypoint_index]
        # X-axis direction: from keypoint[start_index] to keypoint[end_index]
        self.local_x_kp_index = [0, 1]
        
        # Initialize the keypoint tracker
        self._initialize_tracker()
        
        # Initialize the robot connection
        self._initialize_robot()
    
    def image_callback(self, msg):
        """
        Callback function for camera image subscription
        
        Args:
            msg: ROS Image message from camera
        """
        self.latest_image = msg
    
    def _initialize_tracker(self):
        """Initialize the FlowFormer++ Web API tracker"""
        try:
            print(f'Initializing FlowFormer++ API tracker at {self.api_url}...')
            self.tracker = FFPPWebAPIKeypointTracker(
                service_url=self.api_url,
                timeout=60,
                image_format="jpg",
                jpeg_quality=95
            )
            print('✓ FlowFormer++ API tracker initialized successfully')
        except Exception as e:
            print(f'Failed to initialize tracker: {e}')
            self.tracker = None
    
    def _initialize_robot(self):
        """Initialize UR15 robot instance and establish connection"""
        try:
            print(f'Initializing UR15 robot at {self.robot_ip}:{self.robot_port}...')
            self.robot = UR15Robot(ip=self.robot_ip, port=self.robot_port)
            
            # Attempt to connect
            res = self.robot.open()
            if res == 0:
                print('✓ UR15 robot connected successfully')
            else:
                print(f'✗ Failed to connect to UR15 robot (error code: {res})')
                self.robot = None
        except Exception as e:
            print(f'Failed to initialize robot: {e}')
            self.robot = None
    
    def _load_ref_data(self):
        """
        Load reference data (image, keypoints, and pose) for tracking
        
        Returns:
            dict: Dictionary containing loaded reference data with keys:
                  - 'success': bool indicating if loading was successful
                  - 'ref_img': numpy array of reference image (BGR format)
                  - 'ref_keypoints': list of keypoint dictionaries with 'x' and 'y' keys
                  - 'ref_pose': dictionary containing pose data
                  - 'error': error message if loading failed
        """
        result = {
            'success': False,
            'ref_img': None,
            'ref_keypoints': None,
            'ref_pose': None,
            'error': None
        }
        
        try:
            # Check if all reference files exist
            if not os.path.exists(self.ref_img_path):
                result['error'] = f"Reference image not found: {self.ref_img_path}"
                return result
            
            if not os.path.exists(self.ref_keypoints_path):
                result['error'] = f"Reference keypoints not found: {self.ref_keypoints_path}"
                return result
            
            if not os.path.exists(self.ref_pose_path):
                result['error'] = f"Reference pose not found: {self.ref_pose_path}"
                return result
            
            # Load reference image
            ref_img = cv2.imread(self.ref_img_path)
            if ref_img is None:
                result['error'] = f"Failed to load reference image: {self.ref_img_path}"
                return result
            
            # Load reference keypoints
            with open(self.ref_keypoints_path, 'r') as f:
                ref_keypoints_data = json.load(f)
            
            ref_keypoints = []
            for kp in ref_keypoints_data.get('keypoints', []):
                ref_keypoints.append({
                    'x': float(kp['x']),
                    'y': float(kp['y'])
                })
            
            if len(ref_keypoints) == 0:
                result['error'] = "No keypoints found in reference keypoints file"
                return result
            
            # Load reference pose
            with open(self.ref_pose_path, 'r') as f:
                ref_pose = json.load(f)
            
            # Populate result
            result['success'] = True
            result['ref_img'] = ref_img
            result['ref_keypoints'] = ref_keypoints
            result['ref_pose'] = ref_pose
            
            print(f"✓ Loaded reference data:")
            print(f"  - Image: {self.ref_img_path} (shape: {ref_img.shape})")
            print(f"  - Keypoints: {len(ref_keypoints)} points")
            print(f"  - Pose: {self.ref_pose_path}")
            
            return result
            
        except json.JSONDecodeError as e:
            result['error'] = f"Invalid JSON format: {e}"
            return result
        except Exception as e:
            result['error'] = f"Error loading reference data: {e}"
            return result
    
    def load_camera_parameters(self):
        """
        Load UR camera intrinsic and extrinsic parameters from calibration result files
        
        Returns:
            bool: True if parameters loaded successfully, False otherwise
        """
        try:
            # Load intrinsic parameters
            if not os.path.exists(self.intrinsic_params_path):
                print(f"Intrinsic parameters file not found: {self.intrinsic_params_path}")
                return False
            
            with open(self.intrinsic_params_path, 'r') as f:
                intrinsic_data = json.load(f)
            
            if not intrinsic_data.get('success', False):
                print("Intrinsic calibration was not successful")
                return False
            
            self.camera_matrix = np.array(intrinsic_data['camera_matrix'])
            self.distortion_coefficients = np.array(intrinsic_data['distortion_coefficients'])
            
            print(f"Intrinsic parameters loaded successfully")
            
            # Load extrinsic parameters
            if not os.path.exists(self.extrinsic_params_path):
                print(f"Extrinsic parameters file not found: {self.extrinsic_params_path}")
                return False
            
            with open(self.extrinsic_params_path, 'r') as f:
                extrinsic_data = json.load(f)
            
            if not extrinsic_data.get('success', False):
                print("Extrinsic calibration was not successful")
                return False
            
            self.cam2end_matrix = np.array(extrinsic_data['cam2end_matrix'])
            self.target2base_matrix = np.array(extrinsic_data['target2base_matrix'])
            
            print(f"Extrinsic parameters loaded successfully")
            
            return True
            
        except FileNotFoundError as e:
            print(f"Error: Camera parameter file not found - {e}")
            return False
        except json.JSONDecodeError as e:
            print(f"Error: Invalid JSON format in parameter file - {e}")
            return False
        except Exception as e:
            print(f"Error loading camera parameters: {e}")
            return False
    
    def movej_to_collect_position(self, robot=None):
        """
        Move robot to predefined collect position using joint movement
        
        Args:
            robot: Optional UR15Robot instance (if None, uses self.robot)
        """
        if robot is None:
            robot = self.robot
        
        if robot is None:
            print("Error: Robot not initialized")
            return False
        
        try:
            print("\n" + "="*60)
            print("Move to Collect Position")
            print("="*60)
                
            # Move robot using movej function (joint movement)
            # Parameters: joint_positions, acceleration, velocity
            move_result = robot.movej(self.collect_start_position, a=0.5, v=0.5)
            time.sleep(0.5)  # Wait for movement to complete
            
            if move_result == 0:
                print("Movement to collect position completed successfully!")
                return True
            else:
                print(f"Movement to collect position failed! (result: {move_result})")
                return False
                
        except Exception as e:
            print(f"Error controlling robot during movement to collect position: {e}")
            return False
    
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
    
    def _pose_to_matrix(self, pose):
        """Convert pose [X,Y,Z,Rx,Ry,Rz] to 4x4 homogeneous transformation matrix"""
        T = np.eye(4)
        T[0:3, 0:3] = self._rotvec_to_matrix(pose[3], pose[4], pose[5])
        T[0:3, 3] = [pose[0], pose[1], pose[2]]
        return T
    
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
    
    def capture_image_and_pose(self, save_dir, img_filename, pose_filename, metadata=None, robot=None):
        """
        Capture current camera image and robot pose
        
        Args:
            save_dir: Directory to save the files
            img_filename: Filename for the image
            pose_filename: Filename for the pose JSON
            metadata: Optional dict with extra metadata to include in pose JSON
            robot: Optional UR15Robot instance (if None, uses self.robot)
            
        Returns:
            bool: True if successful, False otherwise
        """
        if robot is None:
            robot = self.robot
        
        if robot is None:
            print("Error: Robot not initialized")
            return False
        
        print(f'>>> Capturing image and pose to {save_dir}...')
        
        # Create save directory if it doesn't exist
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            print(f'Created save directory: {save_dir}')
        
        try:
            # Read actual joint positions
            joint_positions = robot.get_actual_joint_positions()
            if joint_positions is None:
                print('Failed to read joint positions')
                return False
            
            # Read actual TCP pose
            tcp_pose = robot.get_actual_tcp_pose()
            if tcp_pose is None:
                print('Failed to read TCP pose')
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
            
            print(f'✓ Saved pose to: {json_filename}')
            print(f'  Joint angles: {[f"{j:.4f}" for j in joint_positions]}')
            print(f'  TCP pose: {[f"{p:.4f}" for p in tcp_pose]}')
            
            # Save current camera image if available
            if self.latest_image is not None:
                try:
                    # Convert ROS Image message to OpenCV format
                    cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
                    
                    # Save image
                    image_filename = os.path.join(save_dir, img_filename)
                    cv2.imwrite(image_filename, cv_image)
                    print(f'✓ Saved image to: {image_filename}')
                    print(f'  Image size: {cv_image.shape}')
                except Exception as e:
                    print(f'Error saving camera image: {e}')
                    return False
            else:
                print('No camera image available to save')
                return False
            
            print('<<< Capture completed successfully!')
            return True
            
        except Exception as e:
            print(f'Error capturing image and pose: {e}')
            return False
    
    def auto_collect_data(self, save_dir=None, robot=None):
        """
        Automatically collect data by moving robot in 5 positions relative to current TCP pose
        First moves to predefined collect position, then moves along base frame: +X, -X, +Y, -Y, -Z (3cm each) using movel()
        
        Args:
            save_dir: Directory to save collected data (default: temp/ur_test_data)
            robot: Optional UR15Robot instance (if None, uses self.robot)
            
        Returns:
            bool: True if all data collection successful, False otherwise
        """
        if robot is None:
            robot = self.robot
        
        if robot is None:
            print("Error: Robot not initialized")
            return False
        
        if save_dir is None:
            save_dir = self.data_dir
        
        print("\n" + "="*60)
        print("Starting Auto Data Collection")
        print("="*60)
        
        try:
            if not self.movej_to_collect_position(robot):
                print("✗ Failed to move to collect position, aborting data collection")
                return False
            
            print("✓ Robot is at collect position, ready to collect data\n")
            
            # Get current TCP pose as reference
            current_tcp_pose = robot.get_actual_tcp_pose()
            if current_tcp_pose is None:
                print("Failed to get current TCP pose")
                return False
            
            print(f"Current TCP pose: {[f'{p:.4f}' for p in current_tcp_pose]}")
            
            # Use movements from class variable
            # Convert to list format for processing
            movements = []
            for idx, (movement_name, offset) in enumerate(self.movements.items()):
                movements.append({
                    "name": movement_name,
                    "offset": offset,
                    "index": idx
                })
            
            # Create save directory
            if not os.path.exists(self.data_dir):
                os.makedirs(self.data_dir)
                print(f"Created save directory: {self.data_dir}")
            
            # Wait for camera image to be available
            print("Waiting for camera image...")
            max_wait_time = 10.0  # Maximum wait time in seconds
            start_time = time.time()
            
            while self.latest_image is None and (time.time() - start_time) < max_wait_time:
                rclpy.spin_once(self, timeout_sec=0.1)
            
            if self.latest_image is None:
                print("Error: No camera image received within timeout. Make sure camera is running.")
                return False
            
            print("✓ Camera image available, starting data collection...")
            
            # Initialize tracking results
            tracking_results = {
                'reference_info': {
                    'ref_img_path': self.ref_img_path,
                    'ref_keypoints_path': self.ref_keypoints_path,
                    'ref_pose_path': self.ref_pose_path,
                },
                'tracking_results': [],
                'timestamp': datetime.now().isoformat()
            }
            
            # Load and set reference data if available
            tracking_enabled = False
            ref_data = self._load_ref_data()
            
            if ref_data['success'] and self.tracker is not None:
                print("📖 Setting up reference image for tracking...")
                try:
                    # Convert BGR to RGB for tracker
                    ref_img_rgb = cv2.cvtColor(ref_data['ref_img'], cv2.COLOR_BGR2RGB)
                    
                    # Set reference image using tracker
                    print("📤 Setting reference image with keypoints...")
                    set_result = self.tracker.set_reference_image(
                        image=ref_img_rgb,
                        keypoints=ref_data['ref_keypoints'],
                        image_name='ur_test_reference'
                    )
                    
                    if set_result.get('success', False):
                        print("✓ Reference image set successfully - tracking enabled")
                        print(f"  Number of keypoints: {len(ref_data['ref_keypoints'])}")
                        tracking_enabled = True
                        tracking_results['reference_info']['num_keypoints'] = len(ref_data['ref_keypoints'])
                    else:
                        print(f"✗ Failed to set reference image: {set_result.get('error', 'Unknown error')}")
                        
                except Exception as e:
                    print(f"✗ Error setting up reference image: {e}")
            else:
                if not ref_data['success']:
                    print(f"📝 Reference data not available: {ref_data.get('error', 'Unknown error')}")
                else:
                    print("📝 Tracker not available - tracking disabled")
            
            success_count = 0
            
            for movement in movements:
                try:
                    print(f"\n--- Movement {movement['index']}: {movement['name']} ---")
                    
                    # Calculate target pose by adding offset to current pose
                    target_pose = current_tcp_pose.copy()
                    for i in range(6):
                        target_pose[i] += movement['offset'][i]
                    
                    print(f"Moving to: {[f'{p:.4f}' for p in target_pose]}")
                    
                    # Move robot to target position
                    move_result = robot.movel(target_pose, a=0.1, v=0.05)
                    
                    if move_result != 0:
                        print(f"Movement failed for {movement['name']} (result: {move_result})")
                        continue
                    
                    # Wait for robot to settle
                    time.sleep(0.5)
                    
                    # Capture image and pose
                    img_filename = f"{movement['index']}.jpg"
                    pose_filename = f"{movement['index']}.json"
                    
                    metadata = {
                        "movement_type": movement['name'],
                        "movement_offset": movement['offset'],
                        "reference_tcp_pose": current_tcp_pose,
                        "movement_index": movement['index']
                    }
                    
                    if self.capture_image_and_pose(self.data_dir, img_filename, 
                                                 pose_filename, metadata, robot=robot):
                        print(f"✓ Successfully collected data for {movement['name']}")
                        success_count += 1
                        time.sleep(0.2)  
                        # Perform keypoint tracking immediately after capturing image
                        if tracking_enabled:
                            print(f"🔍 Tracking keypoints for {movement['name']}...")
                            track_success = self._track_single_image(
                                os.path.join(self.data_dir, img_filename),
                                os.path.join(self.data_dir, pose_filename),
                                movement['index'],
                                movement['name']
                            )
                            
                            if track_success:
                                tracking_results['tracking_results'].append(track_success)
                            
                    else:
                        print(f"✗ Failed to capture data for {movement['name']}")
                    
                except Exception as e:
                    print(f"Error during {movement['name']} movement: {e}")
                    continue
            
            # Return to original position
            print(f"\n--- Returning to original position ---")
            return_result = robot.movel(current_tcp_pose, a=0.1, v=0.05)
            
            time.sleep(0.5)  # Wait for movement to complete

            if return_result == 0:
                print("✓ Successfully returned to original position")
            else:
                print(f"✗ Failed to return to original position (result: {return_result})")
            
            print(f"\nData collection completed: {success_count}/{len(movements)} successful")
            
            # Save immediate tracking results if any were collected
            if tracking_enabled and tracking_results['tracking_results']:
                # Add metadata to results
                tracking_results['metadata'] = {
                    'timestamp': datetime.now().isoformat(),
                    'total_images': success_count,
                    'tracked_images': len(tracking_results['tracking_results']),
                    'reference_image': self.ref_img_path,
                    'keypoints_per_image': 4,
                    'tracking_mode': 'immediate'
                }
                
                # Save results to temp/ur_test_result directory
                result_dir = self.result_dir
                os.makedirs(result_dir, exist_ok=True)
                
                tracking_path = os.path.join(result_dir, self.TRACKING_RESULT_FILENAME)
                
                with open(tracking_path, 'w') as f:
                    json.dump(tracking_results, f, indent=2)
                    
                print(f"\n💾 Immediate tracking results saved to: {tracking_path}")
                print(f"✓ Keypoint tracking completed for {len(tracking_results['tracking_results'])}/5 images")
            
            print("="*60)
            
            return success_count == len(movements)
            
        except Exception as e:
            print(f"Error during auto data collection: {e}")
            return False
    
    def _track_single_image(self, image_path, pose_path, movement_index, movement_name):
        """
        Track keypoints for a single image immediately after capture
        
        Args:
            image_path: Path to the captured image
            pose_path: Path to the pose data file
            movement_index: Index of the movement (0-4)
            movement_name: Name of the movement (e.g., "X_positive")
            
        Returns:
            dict: Tracking result for this image or None if failed
        """
        try:
            # Load pose data
            with open(pose_path, 'r') as f:
                pose_data = json.load(f)
            
            # Load and prepare image for tracking
            test_img_bgr = cv2.imread(image_path)
            if test_img_bgr is None:
                print(f"  ✗ Failed to load image: {image_path}")
                return None
            
            # Convert BGR to RGB for tracker
            test_img_rgb = cv2.cvtColor(test_img_bgr, cv2.COLOR_BGR2RGB)
            
            # Perform tracking using the keypoint tracker
            track_results = self.tracker.track_keypoints(
                target_image=test_img_rgb,
                reference_name='ur_test_reference',
                bidirectional=True,
                return_flow=False
            )
            
            if track_results and track_results.get('success', False):
                tracked_keypoints = track_results.get('tracked_keypoints', [])
                result = {
                    'image_filename': os.path.basename(image_path),
                    'movement_name': movement_name,
                    'movement_index': movement_index,
                    'pose_data': pose_data,
                    'tracked_keypoints': tracked_keypoints,
                    'bidirectional_stats': track_results.get('bidirectional_stats'),
                    'timestamp': pose_data.get('timestamp', ''),
                    'success': True
                }
                
                print(f"  ✓ Tracked {len(tracked_keypoints)} keypoints for {movement_name}")
                return result
            else:
                error_msg = track_results.get('error', 'Unknown error') if track_results else 'No tracking result'
                print(f"  ✗ Tracking failed for {movement_name}: {error_msg}")
                return None
                
        except Exception as e:
            print(f"  ✗ Error tracking image {movement_name}: {e}")
            return None
    
    def estimate_3d_position(self, tracking_result_path=None):
        """
        Estimate 3D coordinates of keypoints using triangulation from multiple viewpoints
        
        Args:
            tracking_result_path: Path to tracking result JSON file
            
        Returns:
            bool: True if estimation successful, False otherwise
        """
        print("\n" + "="*60)
        print("Starting 3D Keypoint Estimation")
        print("="*60)
        
        try:
            # Define paths
            if tracking_result_path is None:
                result_dir = self.result_dir
                tracking_result_path = os.path.join(result_dir, self.TRACKING_RESULT_FILENAME)
            else:
                result_dir = os.path.dirname(tracking_result_path)
            
            estimation_result_path = os.path.join(result_dir, self.ESTIMATION_RESULT_FILENAME)
            
            # Check if tracking result exists
            if not os.path.exists(tracking_result_path):
                print(f"Tracking result file not found: {tracking_result_path}")
                return False
            
            # Load tracking results
            print("📖 Loading tracking results...")
            with open(tracking_result_path, 'r') as f:
                tracking_data = json.load(f)
            
            tracking_results = tracking_data.get('tracking_results', [])
            if not tracking_results:
                print("No tracking results found")
                return False
            
            print(f"✓ Loaded tracking data for {len(tracking_results)} images")
            
            # Check if camera parameters are loaded
            if self.camera_matrix is None or self.cam2end_matrix is None:
                print("Camera parameters not loaded. Loading now...")
                if not self.load_camera_parameters():
                    print("Failed to load camera parameters")
                    return False
            
            # Extract camera parameters
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            cam2end_matrix = self.cam2end_matrix
            
            print(f"Camera parameters: fx={fx:.2f}, fy={fy:.2f}, cx={cx:.2f}, cy={cy:.2f}")
            
            # Find valid tracking results (need at least 2 images for triangulation)
            valid_results = []
            for result in tracking_results:
                if result.get('success', False) and 'tracked_keypoints' in result:
                    keypoints = result['tracked_keypoints']
                    valid_results.append(result)
                    print(f"  ✓ {result.get('movement_name', 'unknown')}: {len(keypoints)} keypoints tracked")
            
            if len(valid_results) < 2:
                print(f"Insufficient valid images for triangulation (need ≥2, got {len(valid_results)})")
                return False
            
            print(f"✓ Found {len(valid_results)} valid images for triangulation")
            
            # Determine the number of keypoints (use the minimum across all images)
            num_keypoints = min(len(result['tracked_keypoints']) for result in valid_results)
            print(f"Number of keypoints to triangulate: {num_keypoints}")
            
            if num_keypoints == 0:
                print("No keypoints found in tracking results")
                return False
            
            estimated_3d_points = []
            
            # Triangulate each keypoint across all images
            for kp_idx in range(num_keypoints):
                print(f"\n--- Triangulating keypoint {kp_idx} ---")
                
                rays = []
                origins = []
                image_info = []
                
                # Collect observations of this keypoint from all images
                for result in valid_results:
                    keypoints = result['tracked_keypoints']
                    
                    # Skip if this image doesn't have this keypoint
                    if kp_idx >= len(keypoints):
                        print(f"  ⚠ {result.get('movement_name', 'unknown')}: keypoint {kp_idx} not available")
                        continue
                    
                    kp = keypoints[kp_idx]
                    pixel_u = kp['x']
                    pixel_v = kp['y']
                    
                    # Get robot pose data
                    pose_data = result['pose_data']
                    end2base_matrix = np.array(pose_data['end2base'])
                    
                    # Calculate camera to base transformation
                    cam2base_matrix = end2base_matrix @ cam2end_matrix
                    
                    # Undistort and convert to normalized image coordinates
                    # This is crucial for accurate 3D estimation
                    if self.distortion_coefficients is not None:
                        # undistortPoints without P parameter returns normalized coordinates directly
                        pixel_point = np.array([[pixel_u, pixel_v]], dtype=np.float32).reshape(-1, 1, 2)
                        normalized_points = cv2.undistortPoints(
                            pixel_point,
                            self.camera_matrix,
                            self.distortion_coefficients
                        )
                        x_norm, y_norm = normalized_points.reshape(-1, 2)[0]
                    else:
                        # No distortion correction, manually normalize
                        x_norm = (pixel_u - cx) / fx
                        y_norm = (pixel_v - cy) / fy
                    
                    # Ray in camera frame (normalized)
                    ray_cam = np.array([x_norm, y_norm, 1.0])
                    ray_cam = ray_cam / np.linalg.norm(ray_cam)
                    
                    # Transform to base frame
                    ray_base = cam2base_matrix[:3, :3] @ ray_cam
                    origin_base = cam2base_matrix[:3, 3]
                    
                    rays.append(ray_base)
                    origins.append(origin_base)
                    
                    movement_name = result.get('movement_name', 'unknown')
                    image_info.append(movement_name)
                    print(f"  {movement_name}: pixel=({pixel_u:.1f}, {pixel_v:.1f})")
                
                # Check if we have enough views for this keypoint
                if len(rays) < 2:
                    print(f"  ✗ Insufficient views for keypoint {kp_idx} (need ≥2, got {len(rays)})")
                    estimated_3d_points.append(None)
                    continue
                
                print(f"  Using {len(rays)} images: {', '.join(image_info)}")
                
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
                
                estimated_3d_points.append({
                    'keypoint_index': kp_idx,
                    'x': float(P_estimated[0]),
                    'y': float(P_estimated[1]), 
                    'z': float(P_estimated[2]),
                    'num_views': len(rays),
                    'residual_norm': float(np.linalg.norm(residuals)) if len(residuals) > 0 else 0.0
                })
                
                print(f"  ✓ 3D position: ({P_estimated[0]:.6f}, {P_estimated[1]:.6f}, {P_estimated[2]:.6f})")
                print(f"  Used {len(rays)} views, residual: {np.linalg.norm(residuals):.6f}")
            
            # Prepare final results
            estimation_results = {
                'metadata': {
                    'timestamp': datetime.now().isoformat(),
                    'tracking_result_source': tracking_result_path,
                    'num_valid_images': len(valid_results),
                    'num_keypoints': num_keypoints,
                    'camera_parameters': {
                        'fx': fx, 'fy': fy, 'cx': cx, 'cy': cy
                    },
                    'distortion_correction': True if self.distortion_coefficients is not None else False,
                    'triangulation_method': 'ray_casting_least_squares'
                },
                'estimated_keypoints': [kp for kp in estimated_3d_points if kp is not None],
                'valid_images_used': [result.get('image_filename', 'unknown') for result in valid_results]
            }
            
            # Save results
            os.makedirs(result_dir, exist_ok=True)
            with open(estimation_result_path, 'w') as f:
                json.dump(estimation_results, f, indent=2)
            
            print(f"\n💾 3D estimation results saved to: {estimation_result_path}")
            print(f"✅ Successfully estimated 3D coordinates for {len([kp for kp in estimated_3d_points if kp is not None])}/{num_keypoints} keypoints")
            print("="*60)
            
            return True
            
        except Exception as e:
            print(f"Error during 3D estimation: {e}")
            import traceback
            traceback.print_exc()
            return False

    def build_local_coordinate_system(self, estimation_result_path=None):
        """
        Build a coordinate system based on keypoints
        
        The coordinate system is defined as follows:
        - Origin: at keypoint[local_x_kp_index[0]]
        - X-axis: keypoint[local_x_kp_index[0]] → keypoint[local_x_kp_index[1]] direction
        - Z-axis: positive direction aligns with base Z-axis (upward)
        - Y-axis: follows right-hand rule (Z × X)
        
        Args:
            estimation_result_path: Path to 3D estimation result JSON file
            
        Returns:
            dict: Coordinate system information including transformation matrix,
                  or None if failed
        """
        print("\n" + "="*60)
        print("Building Keypoint Coordinate System")
        print("="*60)
        
        try:
            # Define default path if not provided
            if estimation_result_path is None:
                result_dir = self.result_dir
                estimation_result_path = os.path.join(result_dir, self.ESTIMATION_RESULT_FILENAME)
            
            # Check if estimation result file exists
            if not os.path.exists(estimation_result_path):
                print(f"Estimation result file not found: {estimation_result_path}")
                return None
            
            # Load 3D estimation results
            print("📖 Loading 3D estimation results...")
            with open(estimation_result_path, 'r') as f:
                estimation_data = json.load(f)
            
            estimated_keypoints = estimation_data.get('estimated_keypoints', [])
            
            # Get keypoint indices for X-axis
            kp_start_idx = self.local_x_kp_index[0]
            kp_end_idx = self.local_x_kp_index[1]
            
            if len(estimated_keypoints) < max(kp_start_idx, kp_end_idx) + 1:
                print(f"Insufficient keypoints for coordinate system (need at least {max(kp_start_idx, kp_end_idx) + 1}, got {len(estimated_keypoints)})")
                return None
            
            # Extract 3D coordinates of the specified keypoints
            kp_start = None
            kp_end = None
            
            for kp in estimated_keypoints:
                if kp.get('keypoint_index') == kp_start_idx:
                    kp_start = np.array([kp['x'], kp['y'], kp['z']])
                elif kp.get('keypoint_index') == kp_end_idx:
                    kp_end = np.array([kp['x'], kp['y'], kp['z']])
            
            if kp_start is None or kp_end is None:
                print(f"Error: Could not find keypoint{kp_start_idx} or keypoint{kp_end_idx} in estimation results")
                return None
            
            print(f"✓ Loaded keypoints:")
            print(f"  KP{kp_start_idx}: ({kp_start[0]:.6f}, {kp_start[1]:.6f}, {kp_start[2]:.6f})")
            print(f"  KP{kp_end_idx}: ({kp_end[0]:.6f}, {kp_end[1]:.6f}, {kp_end[2]:.6f})")
            
            # Origin of the coordinate system is at the start keypoint
            origin = kp_start.copy()
            print(f"✓ Origin at KP{kp_start_idx}: ({origin[0]:.6f}, {origin[1]:.6f}, {origin[2]:.6f})")
            
            # X-axis: from start keypoint to end keypoint
            x_vec_raw = kp_end - kp_start
            x_axis_length = np.linalg.norm(x_vec_raw)
            x_vec = x_vec_raw / x_axis_length
            
            print(f"\n📐 Building coordinate system:")
            print(f"  X-axis from KP{kp_start_idx}→KP{kp_end_idx}: length = {x_axis_length:.6f}m")
            print(f"  X = [{x_vec[0]:.6f}, {x_vec[1]:.6f}, {x_vec[2]:.6f}]")
            
            # Z-axis: should align with base Z-axis (positive upward)
            base_z = np.array([0.0, 0.0, 1.0])
            
            # Make Z-axis orthogonal to X-axis using Gram-Schmidt process
            # Project base_z onto plane perpendicular to X-axis
            z_vec_raw = base_z - np.dot(base_z, x_vec) * x_vec
            z_vec_norm = np.linalg.norm(z_vec_raw)
            
            if z_vec_norm < 1e-6:
                print("Warning: X-axis is nearly parallel to base Z-axis, using alternative Z-axis")
                # Use alternative: pick perpendicular vector
                if abs(x_vec[2]) < 0.9:
                    z_vec_raw = np.cross(np.array([1.0, 0.0, 0.0]), x_vec)
                else:
                    z_vec_raw = np.cross(np.array([0.0, 1.0, 0.0]), x_vec)
                z_vec_norm = np.linalg.norm(z_vec_raw)
            
            z_vec = z_vec_raw / z_vec_norm
            
            # Check alignment with base Z
            z_dot_base_z = np.dot(z_vec, base_z)
            print(f"  Z-axis alignment with base Z: {z_dot_base_z:.6f}")
            print(f"  Z = [{z_vec[0]:.6f}, {z_vec[1]:.6f}, {z_vec[2]:.6f}]")
            
            # Y-axis: right-hand rule (Z × X)
            y_vec = np.cross(z_vec, x_vec)
            y_vec = y_vec / np.linalg.norm(y_vec)
            
            print(f"  Y-axis from right-hand rule (Z × X)")
            print(f"  Y = [{y_vec[0]:.6f}, {y_vec[1]:.6f}, {y_vec[2]:.6f}]")
            
            # Verify orthogonality
            dot_xy = np.dot(x_vec, y_vec)
            dot_xz = np.dot(x_vec, z_vec)
            dot_yz = np.dot(y_vec, z_vec)
            
            print(f"\n🔍 Orthogonality check:")
            print(f"  X·Y = {dot_xy:.8f} (should be ~0)")
            print(f"  X·Z = {dot_xz:.8f} (should be ~0)")
            print(f"  Y·Z = {dot_yz:.8f} (should be ~0)")
            
            # Build rotation matrix (columns are the axis vectors)
            R = np.column_stack([x_vec, y_vec, z_vec])
            
            # Verify it's a valid rotation matrix
            det_R = np.linalg.det(R)
            print(f"  det(R) = {det_R:.8f} (should be ~1)")
            
            if abs(det_R - 1.0) > 0.01:
                print(f"⚠ Warning: Rotation matrix determinant is not 1, may indicate numerical issues")
            
            # Build 4x4 transformation matrix
            T = np.eye(4)
            T[:3, :3] = R  # Rotation part
            T[:3, 3] = origin  # Translation part
            
            # Convert rotation matrix to axis-angle representation for robot use
            rx, ry, rz = self._matrix_to_rotvec(R)
            
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
                "local_x_kp_index": self.local_x_kp_index,
                "timestamp": datetime.now().isoformat(),
                "method": f"kp{kp_start_idx}_kp{kp_end_idx}_based_coordinate_system"
            }
            
            # Save coordinate system to file
            result_dir = self.result_dir
            os.makedirs(result_dir, exist_ok=True)
            
            coord_system_path = os.path.join(result_dir, self.COORD_SYSTEM_RESULT_FILENAME)
            with open(coord_system_path, 'w') as f:
                json.dump(coord_system, f, indent=2)
            
            print(f"\n💾 Coordinate system saved to: {coord_system_path}")
            print(f"🎯 Coordinate system established successfully!")
            print(f"   Origin at KP{kp_start_idx}: ({origin[0]:.4f}, {origin[1]:.4f}, {origin[2]:.4f})")
            print(f"   X-axis: KP{kp_start_idx}→KP{kp_end_idx}")
            print(f"   Z-axis: aligned with base Z-axis")
            print(f"   Y-axis: right-hand rule (Z × X)")
            print(f"   Pose: x={origin[0]:.4f}, y={origin[1]:.4f}, z={origin[2]:.4f}, rx={rx:.4f}, ry={ry:.4f}, rz={rz:.4f}")
            
            print("="*60)
            return coord_system
            
        except Exception as e:
            print(f"Error building coordinate system: {e}")
            import traceback
            traceback.print_exc()
            return None
    
    def validate_local_coordinate_system(self, coord_system, tracking_result_path=None):
        """
        Visualize and validate the local coordinate system by drawing it on actual images (0-4.jpg)
        
        Draws the coordinate system axes on each captured image by:
        1. Projecting the 3D origin and axis endpoints to 2D image coordinates
        2. Drawing arrows for X (red), Y (green), Z (blue) axes on the images
        
        Args:
            coord_system: Dictionary containing coordinate system information
                         (output from build_local_coordinate_system)
            tracking_result_path: Optional path to tracking result JSON (to get pose data)
        
        Returns:
            bool: True if visualization created successfully, False otherwise
        """
        print("\n" + "="*60)
        print("Validating Local Coordinate System on Images")
        print("="*60)
        
        try:
            if coord_system is None:
                print("Error: coord_system is None")
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
            
            print(f"✓ Origin: ({origin_3d[0]:.4f}, {origin_3d[1]:.4f}, {origin_3d[2]:.4f})")
            print(f"✓ X-axis: [{x_axis[0]:.4f}, {x_axis[1]:.4f}, {x_axis[2]:.4f}]")
            print(f"✓ Y-axis: [{y_axis[0]:.4f}, {y_axis[1]:.4f}, {y_axis[2]:.4f}]")
            print(f"✓ Z-axis: [{z_axis[0]:.4f}, {z_axis[1]:.4f}, {z_axis[2]:.4f}]")
            
            # Define arrow length in 3D space (meters)
            arrow_length = 0.05  # 5cm arrows
            
            # Calculate 3D endpoints of axes
            x_end_3d = origin_3d + x_axis * arrow_length
            y_end_3d = origin_3d + y_axis * arrow_length
            z_end_3d = origin_3d + z_axis * arrow_length
            
            # Load tracking results to get pose data for each image
            if tracking_result_path is None:
                result_dir = self.result_dir
                tracking_result_path = os.path.join(result_dir, self.TRACKING_RESULT_FILENAME)
            else:
                result_dir = os.path.dirname(tracking_result_path)
            
            if not os.path.exists(tracking_result_path):
                print(f"Tracking result file not found: {tracking_result_path}")
                return False
            
            print("📖 Loading tracking results for pose data...")
            with open(tracking_result_path, 'r') as f:
                tracking_data = json.load(f)
            
            tracking_results = tracking_data.get('tracking_results', [])
            if not tracking_results:
                print("No tracking results found")
                return False
            
            # Check if camera parameters are loaded
            if self.camera_matrix is None or self.cam2end_matrix is None:
                print("Camera parameters not loaded. Loading now...")
                if not self.load_camera_parameters():
                    print("Failed to load camera parameters")
                    return False
            
            # Get camera parameters
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            cam2end_matrix = self.cam2end_matrix
            
            # Data directory
            data_dir = self.data_dir
            
            # Create visualization for each image
            num_images = len(tracking_results)
            cols = min(3, num_images)
            rows = (num_images + cols - 1) // cols
            
            fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 5*rows))
            if num_images == 1:
                axes = np.array([axes])
            axes = axes.flatten() if num_images > 1 else axes
            
            for idx, result in enumerate(tracking_results):
                if idx >= len(axes):
                    break
                
                ax = axes[idx]
                
                # Load image
                image_filename = result.get('image_filename', '')
                image_path = os.path.join(data_dir, image_filename)
                
                if not os.path.exists(image_path):
                    print(f"⚠ Image not found: {image_path}")
                    ax.axis('off')
                    continue
                
                img = cv2.imread(image_path)
                if img is None:
                    print(f"⚠ Failed to load image: {image_path}")
                    ax.axis('off')
                    continue
                
                # Convert BGR to RGB for matplotlib
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                # Get robot pose for this image
                pose_data = result.get('pose_data', {})
                end2base_matrix = np.array(pose_data.get('end2base', np.eye(4)))
                
                # Calculate camera to base transformation
                cam2base_matrix = end2base_matrix @ cam2end_matrix
                base2cam_matrix = np.linalg.inv(cam2base_matrix)
                
                # Project 3D points to 2D image coordinates
                def project_3d_to_2d(point_3d_base):
                    """Project a 3D point in base frame to 2D image coordinates"""
                    point_3d_homog = np.array([point_3d_base[0], point_3d_base[1], point_3d_base[2], 1.0])
                    point_3d_cam = base2cam_matrix @ point_3d_homog
                    
                    x_cam = point_3d_cam[0]
                    y_cam = point_3d_cam[1]
                    z_cam = point_3d_cam[2]
                    
                    if z_cam <= 0:
                        return None  # Point is behind camera
                    
                    # Apply distortion and projection
                    if self.distortion_coefficients is not None:
                        # Normalize coordinates
                        x_norm = x_cam / z_cam
                        y_norm = y_cam / z_cam
                        
                        # Apply distortion
                        k1, k2, p1, p2, k3 = self.distortion_coefficients.flatten()[:5]
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
                
                movement_name = result.get('movement_name', 'unknown')
                ax.set_title(f'{movement_name}\n{image_filename}', fontsize=10)
                ax.axis('off')
                
                print(f"  ✓ Drawn coordinate system on {image_filename}")
            
            # Hide unused subplots
            for idx in range(num_images, len(axes)):
                axes[idx].axis('off')
            
            # Add overall title
            fig.suptitle('Local Coordinate System Validation\nRed: X-axis | Green: Y-axis | Blue: Z-axis',
                        fontsize=14, fontweight='bold', y=0.98)
            
            # Add coordinate system information as text
            info_text = (
                f"Coordinate System Properties:\n"
                f"Origin: ({origin_3d[0]:.4f}, {origin_3d[1]:.4f}, {origin_3d[2]:.4f}) m\n"
                f"X-axis length: {coord_system['axes']['x_axis']['length']:.4f}m (KP0→KP1)\n"
                f"Z alignment with base: {coord_system['axes']['z_axis']['alignment_with_base_z']:.4f}\n"
                f"Arrow length: {arrow_length:.3f}m"
            )
            
            fig.text(0.02, 0.02, info_text, fontsize=9,
                    bbox=dict(boxstyle='round', facecolor='wheat', alpha=0.8),
                    verticalalignment='bottom')
            
            # Save figure
            output_path = os.path.join(result_dir, 'local_coordinate_system_validate_result.jpg')
            plt.tight_layout()
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"\n💾 Validation visualization saved to: {output_path}")
            
            # Close plot to free memory
            plt.close(fig)
            
            print("✅ Coordinate system validation completed successfully!")
            print("="*60)
            
            return True
            
        except Exception as e:
            print(f"Error validating coordinate system: {e}")
            import traceback
            traceback.print_exc()
            return False
    
    def validate_keypoints_3d_estimate_result(self, tracking_result_path=None, estimation_result_path=None):
        """
        Validate 3D keypoint estimation by comparing tracked keypoints with reprojected 3D estimates
        
        Creates a visualization showing:
        - Red circles: Original tracked keypoints
        - Green circles: Reprojected 3D estimated keypoints
        
        Args:
            tracking_result_path: Path to tracking result JSON file
            estimation_result_path: Path to 3D estimation result JSON file
            
        Returns:
            bool: True if validation visualization created successfully, False otherwise
        """
        print("\n" + "="*60)
        print("Validating 3D Keypoint Estimation Results")
        print("="*60)
        
        try:
            # Define paths
            if tracking_result_path is None:
                result_dir = self.result_dir
                tracking_result_path = os.path.join(result_dir, self.TRACKING_RESULT_FILENAME)
            else:
                result_dir = os.path.dirname(tracking_result_path)
            
            if estimation_result_path is None:
                estimation_result_path = os.path.join(result_dir, self.ESTIMATION_RESULT_FILENAME)
            
            # Check if files exist
            if not os.path.exists(tracking_result_path):
                print(f"Tracking result file not found: {tracking_result_path}")
                return False
            
            if not os.path.exists(estimation_result_path):
                print(f"Estimation result file not found: {estimation_result_path}")
                return False
            
            # Load tracking results
            print("📖 Loading tracking results...")
            with open(tracking_result_path, 'r') as f:
                tracking_data = json.load(f)
            
            tracking_results = tracking_data.get('tracking_results', [])
            if not tracking_results:
                print("No tracking results found")
                return False
            
            # Load estimation results
            print("📖 Loading 3D estimation results...")
            with open(estimation_result_path, 'r') as f:
                estimation_data = json.load(f)
            
            estimated_keypoints = estimation_data.get('estimated_keypoints', [])
            if not estimated_keypoints:
                print("No estimated keypoints found")
                return False
            
            print(f"✓ Loaded {len(tracking_results)} tracking results")
            print(f"✓ Loaded {len(estimated_keypoints)} estimated keypoints")
            
            # Check if camera parameters are loaded
            if self.camera_matrix is None or self.cam2end_matrix is None:
                print("Camera parameters not loaded. Loading now...")
                if not self.load_camera_parameters():
                    print("Failed to load camera parameters")
                    return False
            
            # Extract camera parameters
            fx = self.camera_matrix[0, 0]
            fy = self.camera_matrix[1, 1]
            cx = self.camera_matrix[0, 2]
            cy = self.camera_matrix[1, 2]
            cam2end_matrix = self.cam2end_matrix
            
            # Load images and create visualization for each tracking result
            data_dir = self.data_dir
            
            # Create a figure with subplots for each image
            num_images = len(tracking_results)
            cols = min(3, num_images)
            rows = (num_images + cols - 1) // cols
            
            fig, axes = plt.subplots(rows, cols, figsize=(6*cols, 5*rows))
            if num_images == 1:
                axes = np.array([axes])
            axes = axes.flatten() if num_images > 1 else axes
            
            # Process each tracking result
            reprojection_errors = []
            
            for idx, result in enumerate(tracking_results):
                if idx >= len(axes):
                    break
                
                ax = axes[idx]
                
                # Load image
                image_filename = result.get('image_filename', '')
                image_path = os.path.join(data_dir, image_filename)
                
                if not os.path.exists(image_path):
                    print(f"⚠ Image not found: {image_path}")
                    ax.axis('off')
                    continue
                
                img = cv2.imread(image_path)
                if img is None:
                    print(f"⚠ Failed to load image: {image_path}")
                    ax.axis('off')
                    continue
                
                # Convert BGR to RGB for matplotlib
                img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                
                # Display image
                ax.imshow(img_rgb)
                
                # Get tracked keypoints (red circles)
                tracked_keypoints = result.get('tracked_keypoints', [])
                
                # Get robot pose for this image
                pose_data = result.get('pose_data', {})
                end2base_matrix = np.array(pose_data.get('end2base', np.eye(4)))
                
                # Calculate camera to base transformation
                cam2base_matrix = end2base_matrix @ cam2end_matrix
                
                # Reproject 3D estimated keypoints (green circles)
                for est_kp in estimated_keypoints:
                    kp_idx = est_kp.get('keypoint_index', -1)
                    
                    # Get 3D position in base frame
                    point_3d_base = np.array([est_kp['x'], est_kp['y'], est_kp['z'], 1.0])
                    
                    # Transform to camera frame
                    base2cam_matrix = np.linalg.inv(cam2base_matrix)
                    point_3d_cam = base2cam_matrix @ point_3d_base
                    
                    # Project to image plane
                    x_cam = point_3d_cam[0]
                    y_cam = point_3d_cam[1]
                    z_cam = point_3d_cam[2]
                    
                    if z_cam <= 0:
                        # Point is behind camera
                        continue
                    
                    # Apply distortion and projection
                    if self.distortion_coefficients is not None:
                        # Normalize coordinates
                        x_norm = x_cam / z_cam
                        y_norm = y_cam / z_cam
                        
                        # Apply distortion
                        k1, k2, p1, p2, k3 = self.distortion_coefficients.flatten()[:5]
                        r2 = x_norm**2 + y_norm**2
                        
                        radial = 1 + k1*r2 + k2*r2**2 + k3*r2**3
                        x_distorted = x_norm * radial + 2*p1*x_norm*y_norm + p2*(r2 + 2*x_norm**2)
                        y_distorted = y_norm * radial + p1*(r2 + 2*y_norm**2) + 2*p2*x_norm*y_norm
                        
                        # Convert to pixel coordinates
                        u_reproj = fx * x_distorted + cx
                        v_reproj = fy * y_distorted + cy
                    else:
                        # No distortion
                        u_reproj = fx * (x_cam / z_cam) + cx
                        v_reproj = fy * (y_cam / z_cam) + cy
                    
                    # Plot reprojected point (green circle - larger)
                    ax.plot(u_reproj, v_reproj, 'go', markersize=8, 
                           markerfacecolor='none', markeredgewidth=2, 
                           label='Reprojected' if kp_idx == 0 else '')
                    
                    # Add keypoint index label
                    ax.text(u_reproj + 5, v_reproj - 5, f'{kp_idx}', 
                           color='green', fontsize=8, fontweight='bold')
                    
                    # Calculate reprojection error if tracked point exists
                    if kp_idx < len(tracked_keypoints):
                        tracked_kp = tracked_keypoints[kp_idx]
                        u_track = tracked_kp['x']
                        v_track = tracked_kp['y']
                        
                        error = np.sqrt((u_reproj - u_track)**2 + (v_reproj - v_track)**2)
                        reprojection_errors.append({
                            'image': image_filename,
                            'keypoint_index': kp_idx,
                            'error_pixels': float(error)
                        })
                
                # Plot tracked keypoints (red circles - smaller)
                for kp_idx, tracked_kp in enumerate(tracked_keypoints):
                    u_track = tracked_kp['x']
                    v_track = tracked_kp['y']
                    
                    ax.plot(u_track, v_track, 'ro', markersize=6,
                           markerfacecolor='none', markeredgewidth=2,
                           label='Tracked' if kp_idx == 0 else '')
                    
                    # Add keypoint index label
                    ax.text(u_track + 5, v_track + 5, f'{kp_idx}', 
                           color='red', fontsize=8, fontweight='bold')
                
                # Set title and legend
                movement_name = result.get('movement_name', 'unknown')
                ax.set_title(f'{movement_name}\n{image_filename}', fontsize=10)
                ax.axis('off')
                
                # Add legend only to first subplot
                if idx == 0:
                    ax.legend(loc='upper right', fontsize=8)
            
            # Hide unused subplots
            for idx in range(num_images, len(axes)):
                axes[idx].axis('off')
            
            # Add overall title
            fig.suptitle('3D Keypoint Estimation Validation\nRed: Tracked | Green: Reprojected from 3D Estimate',
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
                
                print(f"\n📊 Reprojection Error Statistics:")
                print(f"  Mean error: {mean_error:.2f} pixels")
                print(f"  Std error: {std_error:.2f} pixels")
                print(f"  Min error: {min_error:.2f} pixels")
                print(f"  Max error: {max_error:.2f} pixels")
                print(f"  Total samples: {len(errors)}")
            
            # Save figure
            output_path = os.path.join(result_dir, 'keypoints_3d_estimate_validate_result.jpg')
            plt.tight_layout()
            plt.savefig(output_path, dpi=150, bbox_inches='tight')
            print(f"\n💾 Validation visualization saved to: {output_path}")
            
            # Close plot to free memory
            plt.close(fig)
            
            # Save detailed reprojection errors to JSON
            if reprojection_errors:
                error_log_path = os.path.join(result_dir, 'reprojection_errors.json')
                error_data = {
                    'metadata': {
                        'timestamp': datetime.now().isoformat(),
                        'num_images': len(tracking_results),
                        'num_keypoints': len(estimated_keypoints),
                        'total_samples': len(reprojection_errors)
                    },
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
                
                print(f"💾 Reprojection error log saved to: {error_log_path}")
            
            print("✅ 3D estimation validation completed successfully!")
            print("="*60)
            
            return True
            
        except Exception as e:
            print(f"Error validating 3D estimation: {e}")
            import traceback
            traceback.print_exc()
            return False
    

    def lift_platform_to_base(self):
        """
        Move the lift platform downward (pulse relay).
        
        Sends a POST request to the lift web service to trigger downward motion.
        """
        print("\n⬇️  Lift Platform Down")
        print("   Sending DOWN command to lift platform...")
        
        url = f"{self.lift_web_base}/api/cmd"
        payload = {"command": "down", "target": "platform"}
        headers = {"Content-Type": "application/json"}
        
        try:
            response = requests.post(url, json=payload, headers=headers, timeout=5)
            if response.ok:
                print("✅ Lift platform DOWN command sent successfully")
                return response.json()
            else:
                print(f"❌ Lift platform DOWN command failed: HTTP {response.status_code}")
                return {"success": False, "status_code": response.status_code}
        except requests.exceptions.ConnectionError:
            print("❌ Cannot connect to lift platform web service")
            return {"success": False, "error": "Connection failed"}
        except requests.exceptions.Timeout:
            print("❌ Timeout sending lift platform DOWN command")
            return {"success": False, "error": "Timeout"}
        except Exception as e:
            print(f"❌ Error sending lift platform DOWN command: {e}")
            return {"success": False, "error": str(e)}

    def pushrod_to_base(self):
        """
        Move pushrod to 'base' position (home/retracted position).
        
        Sends goto_point command with point='base'.
        """
        print("\n🏠 Pushrod Go to Base")
        print("   Moving pushrod to base position...")
        
        url = f"{self.lift_web_base}/api/cmd"
        payload = {"command": "goto_point", "target": "pushrod", "point": "base"}
        headers = {"Content-Type": "application/json"}
        
        try:
            response = requests.post(url, json=payload, headers=headers, timeout=10)
            if response.ok:
                print("✅ Pushrod 'base' command sent successfully")
                return response.json()
            else:
                print(f"❌ Pushrod goto 'base' failed: HTTP {response.status_code}")
                return {"success": False, "status_code": response.status_code}
        except requests.exceptions.ConnectionError:
            print("❌ Cannot connect to pushrod web service")
            return {"success": False, "error": "Connection failed"}
        except requests.exceptions.Timeout:
            print("❌ Timeout sending pushrod goto command")
            return {"success": False, "error": "Timeout"}
        except Exception as e:
            print(f"❌ Error sending pushrod goto command: {e}")
            return {"success": False, "error": str(e)}

    def lift_platform_coarse_adjust(self, target_height=900.0):
        """
        Platform coarse adjustment - Move lift platform to a specific height.
        
        Args:
            target_height: Target height in millimeters (default: 900.0mm)
        """
        print(f"\n🎯 Platform Coarse Adjustment: {target_height}mm")
        
        url = f"{self.lift_web_base}/api/cmd"
        payload = {
            "command": "goto_height",
            "target": "platform",
            "target_height": target_height
        }
        headers = {"Content-Type": "application/json"}
        
        try:
            response = requests.post(url, json=payload, headers=headers, timeout=5)
            if response.ok:
                print(f"✅ Platform coarse adjustment command sent successfully: {target_height}mm")
                return response.json()
            else:
                print(f"❌ Platform coarse adjustment command failed: HTTP {response.status_code}")
                return {"success": False, "status_code": response.status_code}
        except requests.exceptions.ConnectionError:
            print("❌ Cannot connect to lift platform web service")
            return {"success": False, "error": "Connection failed"}
        except requests.exceptions.Timeout:
            print("❌ Platform coarse adjustment command timeout")
            return {"success": False, "error": "Timeout"}
        except Exception as e:
            print(f"❌ Platform coarse adjustment command error: {e}")
            return {"success": False, "error": str(e)}


    def pushrod_fine_adjust(self, target_height=900.0):
        """
        Pushrod fine adjustment - Precise height control using pushrod.
        
        Args:
            target_height: Target height in millimeters (default: 900.0mm)
        """
        print(f"\n🔧 Pushrod Fine Adjustment: {target_height}mm")
        
        url = f"{self.lift_web_base}/api/cmd"
        payload = {
            "command": "goto_height",
            "target": "pushrod",
            "target_height": target_height
        }
        headers = {"Content-Type": "application/json"}
        
        try:
            response = requests.post(url, json=payload, headers=headers, timeout=5)
            if response.ok:
                print(f"✅ Pushrod fine adjustment command sent successfully: {target_height}mm")
                return response.json()
            else:
                print(f"❌ Pushrod fine adjustment command failed: HTTP {response.status_code}")
                return {"success": False, "status_code": response.status_code}
        except requests.exceptions.ConnectionError:
            print("❌ Cannot connect to lift platform web service")
            return {"success": False, "error": "Connection failed"}
        except requests.exceptions.Timeout:
            print("❌ Pushrod fine adjustment command timeout")
            return {"success": False, "error": "Timeout"}
        except Exception as e:
            print(f"❌ Pushrod fine adjustment command error: {e}")
            return {"success": False, "error": str(e)}

def main():
    """
    Main function for testing URLocateBase class
    """
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Initialize URLocateBase instance (robot connection is handled internally)
        ur_test = URLocateBase()
        
        # Check if robot was initialized successfully
        if ur_test.robot is None or not ur_test.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_test)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print('Node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        # Load camera parameters
        print("Loading camera parameters...")
        if not ur_test.load_camera_parameters():
            print("Failed to load camera parameters!")
            return
        
        try:
            # Perform auto data collection (includes moving to collect position)
            if ur_test.auto_collect_data():
                print("\n✅ Data collection completed successfully!")
                
                # Perform 3D keypoint estimation after data collection
                if ur_test.estimate_3d_position():
                    print("✅ 3D estimation completed successfully!")
                    
                    # Validate 3D estimation with reprojection
                    print("\n" + "="*60)
                    print("Validating 3D Estimation with Reprojection...")
                    print("="*60)
                    if ur_test.validate_keypoints_3d_estimate_result():
                        print("✅ 3D estimation validation completed!")
                    else:
                        print("⚠ 3D estimation validation failed!")
                    
                    # Build keypoint coordinate system
                    coord_system = ur_test.build_local_coordinate_system()
                    if coord_system:
                        print("✅ Coordinate system built successfully!")
                        
                        # Validate and visualize coordinate system
                        print("\n" + "="*60)
                        print("Validating Coordinate System...")
                        print("="*60)
                        if ur_test.validate_local_coordinate_system(coord_system):
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
            if ur_test.robot is not None:
                try:
                    ur_test.robot.close()
                    print("Robot disconnected successfully")
                except Exception as e:
                    print(f"Error disconnecting robot: {e}")
                
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_test.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()


if __name__ == "__main__":
    main()
