#!/usr/bin/env python3
"""
URPositioning - Simplified robot camera capture system
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
from scipy.spatial.transform import Rotation as R

# Robot control imports
from ur15_robot_arm.ur15 import UR15Robot

# ROS2 imports
import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

# Redis client for robot status
from robot_status_redis import RobotStatusClient
from common.workspace_utils import get_workspace_root, get_scripts_directory, get_temp_directory

# Positioning client - add ThirdParty/robot_vision to sys.path using common package
Positioning3DWebAPIClient = None
try:
    scripts_dir = get_scripts_directory()
    if scripts_dir is None:
        scripts_dir = os.path.dirname(os.path.abspath(__file__))
    
    robot_vision_path = os.path.join(scripts_dir, 'ThirdParty', 'robot_vision')
    if robot_vision_path not in sys.path:
        sys.path.insert(0, robot_vision_path)
    
    from core.positioning_3d_webapi import Positioning3DWebAPIClient
    print("✓ Positioning3DWebAPIClient imported successfully")
except ImportError as e:
    Positioning3DWebAPIClient = None
    print(f"Warning: Positioning3DWebAPIClient not available: {e}")

class URPositioning(Node):
    def __init__(self, robot_ip=None, robot_port=None, camera_topic=None, operation_name="test_operation"):
        """
        Initialize URPositioning class for basic robot camera capture
        
        Args:
            robot_ip (str): IP address of the UR15 robot. If None, loads from config file
            robot_port (int): Port number of the UR15 robot. If None, loads from config file
            camera_topic (str): ROS topic name for camera images. If None, loads from config file
            operation_name (str): Name of the operation for data directory
        """
        # Initialize ROS node
        super().__init__('ur_positioning')
        # Update ROS node name for URPositioning
        self.get_logger().info("URPositioning node initialized")
        
        # =========================== Configurable Parameters ===========================
        # Get the script directory for relative paths
        try:
            from common.workspace_utils import get_scripts_directory
            self.script_dir = get_scripts_directory() if get_scripts_directory() else os.path.dirname(os.path.abspath(__file__))
        except ImportError:
            self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Load config once for all parameters
        config_params = self._load_config_parameters()
        
        # IP address and port for UR15 robot - load from config if not provided
        self.robot_ip = robot_ip if robot_ip is not None else config_params['robot_ip']
        self.robot_port = robot_port if robot_port is not None else config_params['robot_port']
        
        # Camera topic name - load from config if not provided
        self.camera_topic = camera_topic if camera_topic is not None else config_params['camera_topic']
        
        # Operation name for data organization
        self.operation_name = operation_name
        
        # ============================= Instance variables ==============================
        # robot arm instance
        self.robot = None
        # Camera-related attributes
        self.cv_bridge = CvBridge()
        self.latest_image = None
        # Redis client for robot status and camera parameters
        self.robot_status_client = None
        # Positioning client for 3D positioning
        self.positioning_client = None

        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        # =============================== Other variables ==============================      
        # Default fallback values if config file not found
        self.collect_start_position = None
        
        # Collection movement offsets (in tcp coordinate system, unit: meters)
        self.movements = {
            "movement1": [0, 0, 0, 0, 0, 0],         # No offset
            "movement2": [0.01, 0, 0, 0, 0, 0],      # X+1cm
            "movement3": [-0.01, 0, 0, 0, 0, 0],     # X-1cm
            "movement4": [0, 0.01, 0, 0, 0, 0],      # Y+1cm
            "movement5": [0, -0.01, 0, 0, 0, 0]      # Y-1cm
        }
        
        self.template_points = [
            {"name": "Server_Top_Left_Corner",  "x": -0.094, "y": 0, "z": 0.017},
            {"name": "Server_Top_Right_Corner", "x": 0.186, "y": 0, "z": 0.017},
            {"name": "Server_Bottom_Left_Corner", "x": -0.094, "y": 0, "z": 0.006},
            {"name": "Server_Bottom_Right_Corner", "x": 0.186, "y": 0, "z": 0.006}
        ]

        # ============================ Initialization =================================
        # Setup paths for data directory
        self._setup_paths(self.operation_name)
        # Initialize the robot connection
        self._initialize_robot()
        # Initialize Redis client
        self._init_robot_status_client()
        # Initialize positioning client
        self._init_positioning_client()

        # Subscribe to camera topic
        self.image_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
    
    def _load_config_parameters(self):
        """
        Load robot IP, port, and camera topic from robot_config.yaml
        
        Returns:
            dict: Dictionary with robot_ip, robot_port, and camera_topic
        """
        # Default values
        defaults = {
            'robot_ip': '192.168.1.15',
            'robot_port': 30002,
            'camera_topic': '/ur15_camera/image_raw'
        }
        
        try:
            import yaml
            
            # Get workspace root
            workspace_root = get_workspace_root()
            if workspace_root is None:
                workspace_root = os.path.abspath(os.path.join(self.script_dir, '..'))
            
            # Path to config file
            config_path = os.path.join(workspace_root, 'config', 'robot_config.yaml')
            
            if not os.path.exists(config_path):
                self.get_logger().warning(f"Config file not found: {config_path}")
                self.get_logger().warning(f"Using default values: IP={defaults['robot_ip']}, Port={defaults['robot_port']}, Topic={defaults['camera_topic']}")
                return defaults
            
            # Load config file
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            ur15_config = config.get('ur15', {})
            
            # Get robot IP from ur15.robot.ip
            robot_ip = ur15_config.get('robot', {}).get('ip', defaults['robot_ip'])
            
            # Get robot port from ur15.robot.ports.control
            robot_port = ur15_config.get('robot', {}).get('ports', {}).get('control', defaults['robot_port'])
            
            # Get camera topic from ur15.camera.topic
            camera_topic = ur15_config.get('camera', {}).get('topic', defaults['camera_topic'])
            
            self.get_logger().info(f"✓ Loaded config: IP={robot_ip}, Port={robot_port}, Topic={camera_topic}")
            
            return {
                'robot_ip': robot_ip,
                'robot_port': robot_port,
                'camera_topic': camera_topic
            }
            
        except Exception as e:
            self.get_logger().warning(f"Error loading config: {e}")
            self.get_logger().warning(f"Using default values: IP={defaults['robot_ip']}, Port={defaults['robot_port']}, Topic={defaults['camera_topic']}")
            return defaults
    
    def image_callback(self, msg):
        """
        Callback function for camera image subscription
        
        Args:
            msg: ROS Image message from camera
        """
        self.latest_image = msg
    
    def _setup_paths(self, operation_name):
        """
        Setup data directory paths using common workspace utilities
        
        Args:
            operation_name (str): Name of the operation for data directory
        """
        try:            
            # Get workspace root (robot_dc directory)
            workspace_root = get_workspace_root()
            if workspace_root is None:
                raise RuntimeError("Could not determine workspace root directory")
            
            # Define directory paths
            self.workspace_dir = workspace_root
            self.scripts_dir = get_scripts_directory() or os.path.join(workspace_root, 'scripts')
            self.dataset_dir = os.path.join(workspace_root, 'dataset')
            self.temp_dir = get_temp_directory()
            self.operation_data_dir = os.path.join(self.dataset_dir, operation_name, 'test')
            
            # Create necessary directories
            os.makedirs(self.operation_data_dir, exist_ok=True)
            
        except ImportError as e:
            # Fallback to relative path if common package not available
            self.get_logger().warning(f"Could not import workspace_utils: {e}")
            self.get_logger().warning("Falling back to relative paths")
            
            self.script_dir = os.path.dirname(os.path.abspath(__file__))
            self.workspace_dir = os.path.abspath(os.path.join(self.script_dir, '..'))
            self.scripts_dir = self.script_dir
            self.dataset_dir = os.path.join(self.workspace_dir, 'dataset')
            self.temp_dir = os.path.join(self.workspace_dir, 'temp')
            self.operation_data_dir = os.path.join(self.dataset_dir, operation_name, 'test')
            
            os.makedirs(self.operation_data_dir, exist_ok=True)
    
    def _initialize_robot(self):
        """Initialize UR15 robot instance and establish connection"""
        try:
            print(f'>>> Initializing UR15 robot at {self.robot_ip}:{self.robot_port}...')
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
    
    def _init_robot_status_client(self):
        """Initialize the RobotStatusClient for Redis connection"""
        try:
            self.robot_status_client = RobotStatusClient(self, timeout_sec=5.0)
            self.get_logger().info("✓ RobotStatusClient initialized successfully")
        except Exception as e:
            self.get_logger().warning(f"✗ Failed to initialize RobotStatusClient: {e}")
            self.get_logger().warning("  Continuing without Redis functionality")
            self.robot_status_client = None
    
    def _init_positioning_client(self, service_url="http://localhost:8004"):
        """Initialize the Positioning3DWebAPIClient"""
        if Positioning3DWebAPIClient is None:
            self.get_logger().warning("✗ Positioning3DWebAPIClient not available")
            self.positioning_client = None
            return
        
        try:
            self.positioning_client = Positioning3DWebAPIClient(service_url=service_url)
            health = self.positioning_client.check_health()
            if health.get('success'):
                self.get_logger().info(f"✓ Positioning service connected: {service_url}")
            else:
                self.get_logger().warning(f"⚠️  Positioning service health check failed")
        except Exception as e:
            self.get_logger().warning(f"✗ Failed to initialize positioning client: {e}")
            self.positioning_client = None
    
    def _load_camera_parameters_from_service(self):
        """
        Load camera intrinsic and extrinsic parameters from Redis (ur15 namespace)
        
        Returns:
            dict: Dictionary containing camera parameters, or None if loading fails
        """
        try:
            camera_params = {
                'camera_matrix': None,
                'distortion_coefficients': None,
                'cam2end_matrix': None
            }
            
            if self.robot_status_client is None:
                print("✗ RobotStatusClient not initialized, cannot load camera parameters")
                return camera_params
            
            # Load camera intrinsic parameters from Redis (ur15 namespace)
            try:
                camera_matrix = self.robot_status_client.get_status('ur15', 'camera_matrix')
                if camera_matrix is not None:
                    camera_params['camera_matrix'] = camera_matrix
                    print("✓ Loaded camera_matrix from Redis (ur15 namespace)")
                else:
                    print("✗ camera_matrix not found in Redis (ur15 namespace)")
            except Exception as e:
                print(f"✗ Failed to load camera_matrix from Redis: {e}")
            
            # Load distortion coefficients from Redis (ur15 namespace)
            try:
                distortion_coefficients = self.robot_status_client.get_status('ur15', 'distortion_coefficients')
                if distortion_coefficients is not None:
                    camera_params['distortion_coefficients'] = distortion_coefficients
                    print("✓ Loaded distortion_coefficients from Redis (ur15 namespace)")
                else:
                    print("✗ distortion_coefficients not found in Redis (ur15 namespace)")
            except Exception as e:
                print(f"✗ Failed to load distortion_coefficients from Redis: {e}")
            
            # Load cam2end matrix from Redis (ur15 namespace)
            try:
                cam2end_matrix = self.robot_status_client.get_status('ur15', 'cam2end_matrix')
                if cam2end_matrix is not None:
                    camera_params['cam2end_matrix'] = cam2end_matrix
                    print("✓ Loaded cam2end_matrix from Redis (ur15 namespace)")
                else:
                    print("✗ cam2end_matrix not found in Redis (ur15 namespace)")
            except Exception as e:
                print(f"✗ Failed to load cam2end_matrix from Redis: {e}")
            
            return camera_params
            
        except Exception as e:
            print(f"Error loading camera parameters from Redis: {e}")
            return {
                'camera_matrix': None,
                'distortion_coefficients': None,
                'cam2end_matrix': None
            }
    
    def _create_session_directory(self):
        """
        Create a new session directory with auto-incremented number
        
        Returns:
            str: Path to the created session directory
        """
        base_session_dir = self.operation_data_dir
        
        # Find the next available session number
        session_num = 1
        while True:
            session_dir = os.path.join(base_session_dir, f"session_{session_num:03d}")
            if not os.path.exists(session_dir):
                break
            session_num += 1
        
        # Create the session directory
        os.makedirs(session_dir, exist_ok=True)
        print(f"📁 Created new session directory: {session_dir}")
        
        return session_dir
    
    def _load_extrinsic_from_pose_json(self, pose_json_path):
        """
        Load extrinsic matrix (world2cam) from pose JSON file
        
        Args:
            pose_json_path: Path to the pose JSON file
            
        Returns:
            np.ndarray: Extrinsic matrix (world2cam/base2cam) for triangulation
        """
        try:
            with open(pose_json_path, 'r') as f:
                data = json.load(f)
            
            # Extract transformation matrices
            end2base = np.array(data['end2base'], dtype=np.float64)
            cam2end = np.array(data['cam2end_matrix'], dtype=np.float64)
            
            # Calculate cam2base (camera to world/base)
            cam2base = end2base @ cam2end
            
            # Triangulation expects world2cam (world to camera), so invert
            extrinsic = np.linalg.inv(cam2base)
            
            return extrinsic
            
        except Exception as e:
            print(f"Error loading extrinsic from {pose_json_path}: {e}")
            return None
    
    def get_image_and_pose_at_current_pose(self, save_dir=None):
        """
        Capture image and pose at current robot position, save with auto-incremented index
        
        Args:
            save_dir: Directory to save the files. If None, uses operation_data_dir
            
        Returns:
            bool: True if capture successful, False otherwise
        """
        if self.robot is None:
            print("Error: Robot not initialized")
            return False
        
        # Use temp_dir/operation_name_data if save_dir not specified
        if save_dir is None:
            save_dir = os.path.join(self.temp_dir, f'{self.operation_name}_data')
        
        # Create save directory if it doesn't exist
        if not os.path.exists(save_dir):
            os.makedirs(save_dir)
            print(f'Created save directory: {save_dir}')
        
        # Find the next available index by counting existing jpg files
        existing_jpgs = [f for f in os.listdir(save_dir) if f.endswith('.jpg')]
        next_index = len(existing_jpgs)
        
        img_filename = f'{next_index}.jpg'
        pose_filename = f'{next_index}.json'
        
        print(f'>>> Capturing image and pose at current position to {save_dir}...')
        print(f'    Using index: {next_index}')
        
        try:
            # Read actual joint positions
            joint_positions = self.robot.get_actual_joint_positions()
            if joint_positions is None:
                print('✗ Failed to read joint positions')
                return False
            
            # Read actual TCP pose
            tcp_pose = self.robot.get_actual_tcp_pose()
            if tcp_pose is None:
                print('✗ Failed to read TCP pose')
                return False
            
            # Calculate end2base transformation matrix
            end2base = np.eye(4)
            end2base[:3, :3] = R.from_rotvec(tcp_pose[3:6]).as_matrix()
            end2base[:3, 3] = tcp_pose[:3]
            
            # Load camera parameters from Redis
            camera_params = self._load_camera_parameters_from_service()
            
            # Prepare pose data structure
            pose_data = {
                "joint_angles": list(joint_positions),
                "end_xyzrpy": {
                    "x": float(tcp_pose[0]),
                    "y": float(tcp_pose[1]),
                    "z": float(tcp_pose[2]),
                    "rx": float(tcp_pose[3]),
                    "ry": float(tcp_pose[4]),
                    "rz": float(tcp_pose[5])
                },
                "end2base": end2base.tolist(),
                "camera_matrix": camera_params['camera_matrix'].tolist() if camera_params['camera_matrix'] is not None else None,
                "distortion_coefficients": camera_params['distortion_coefficients'].tolist() if camera_params['distortion_coefficients'] is not None else None,
                "cam2end_matrix": camera_params['cam2end_matrix'].tolist() if camera_params['cam2end_matrix'] is not None else None,
                "timestamp": datetime.now().isoformat()
            }
            
            # Save pose data
            pose_file = os.path.join(save_dir, pose_filename)
            with open(pose_file, 'w') as f:
                json.dump(pose_data, f, indent=2)
            print(f'✓ Saved pose to: {pose_file}')
            
            # Check if camera image is available
            if self.latest_image is None:
                print('✗ No camera image available')
                return False
            
            # Convert ROS Image to OpenCV format
            cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
            
            # Save image
            img_file = os.path.join(save_dir, img_filename)
            cv2.imwrite(img_file, cv_image)
            print(f'✓ Saved image to: {img_file}')
            
            print('✓ Successfully captured image and pose at current position')
            return True
            
        except Exception as e:
            print(f'✗ Error capturing image and pose: {e}')
            return False
    
    def auto_positioning(self):
        """
        Automatic 3D positioning workflow with multiple views
        
        Steps:
        1. Upload reference
        2. Initialize session
        3. Capture images from multiple poses and upload views
        4. Get positioning result (fitting mode)
        5. Terminate session
        
        Returns:
            dict: Positioning result or None if failed
        """
        if self.robot is None:
            print("Error: Robot not initialized")
            return None
        
        if self.positioning_client is None:
            print("Error: Positioning client not initialized")
            return None
        
        try:
            # Step 1: Upload reference
            print("\n>>> Step 1: Uploading reference data...")
            result = self.positioning_client.upload_references()
            if result.get('success'):
                print(f"  ✓ Reference uploaded: {result.get('references_loaded', 0)}/{result.get('references_found', 0)}")
            else:
                print(f"  ✗ Failed to upload reference: {result.get('error')}")
                return None
            
            # Step 2: Initialize session
            print("\n>>> Step 2: Initializing session...")
            session_result = self.positioning_client.init_session()
            if not session_result.get('success'):
                print(f"  ✗ Failed to initialize session: {session_result.get('error')}")
                return None
            
            session_id = session_result['session_id']
            print(f"  ✓ Session created: {session_id}")
            
            # Step 3: Capture and upload views
            print("\n>>> Step 3: Capturing images and uploading views...")
            
            # Create session directory for saving images
            save_dir = self._create_session_directory()
            print(f"  Saving images to: {save_dir}")
            
            # Load camera parameters once (already loaded in get_image_and_pose_at_current_pose)
            # We'll read them from the first saved JSON file
            camera_params = self._load_camera_parameters_from_service()
            intrinsic = camera_params['camera_matrix']
            distortion = camera_params['distortion_coefficients']
            
            if intrinsic is None or distortion is None:
                print("  ✗ Camera parameters not available")
                self.positioning_client.terminate_session(session_id)
                return None
            
            view_count = 0
            
            # View 1: Current position
            print(f"\n  View 1: Capturing at current position...")
            if not self.get_image_and_pose_at_current_pose(save_dir):
                print("  ✗ Failed to capture view 1")
                self.positioning_client.terminate_session(session_id)
                return None
            
            # Upload view 1
            img_path = os.path.join(save_dir, '0.jpg')
            pose_path = os.path.join(save_dir, '0.json')
            image = cv2.imread(img_path)
            extrinsic = self._load_extrinsic_from_pose_json(pose_path)
            
            if extrinsic is None:
                print("  ✗ Failed to load extrinsic matrix")
                self.positioning_client.terminate_session(session_id)
                return None
            
            result = self.positioning_client.upload_view(
                session_id=session_id,
                reference_name=self.operation_name,
                image=image,
                intrinsic=intrinsic,
                distortion=distortion,
                extrinsic=extrinsic
            )
            
            if result.get('success'):
                view_count += 1
                print(f"  ✓ View 1 uploaded, queue position: {result.get('queue_position', 'N/A')}")
            else:
                print(f"  ✗ Upload failed: {result.get('error')}")
                self.positioning_client.terminate_session(session_id)
                return None
            
            # View 2: Move +X, capture, return
            print(f"\n  View 2: Moving +X (10mm)...")
            if self.robot.move_tcp([0.01, 0, 0, 0, 0, 0], a=0.1, v=0.05) != 0:
                print("  ✗ Failed to move robot")
                self.positioning_client.terminate_session(session_id)
                return None
            
            time.sleep(1.0)
            
            if not self.get_image_and_pose_at_current_pose(save_dir):
                print("  ✗ Failed to capture view 2")
                self.positioning_client.terminate_session(session_id)
                return None
            
            # Upload view 2
            img_path = os.path.join(save_dir, '1.jpg')
            pose_path = os.path.join(save_dir, '1.json')
            image = cv2.imread(img_path)
            extrinsic = self._load_extrinsic_from_pose_json(pose_path)
            
            result = self.positioning_client.upload_view(
                session_id=session_id,
                reference_name=self.operation_name,
                image=image,
                intrinsic=intrinsic,
                distortion=distortion,
                extrinsic=extrinsic
            )
            
            if result.get('success'):
                view_count += 1
                print(f"  ✓ View 2 uploaded, queue position: {result.get('queue_position', 'N/A')}")
            else:
                print(f"  ✗ Upload failed: {result.get('error')}")
            
            # Return to original position
            print("  Returning to original position...")
            self.robot.move_tcp([-0.01, 0, 0, 0, 0, 0], a=0.1, v=0.05)
            time.sleep(1.0)
            
            # View 3: Move +Y, capture, return
            print(f"\n  View 3: Moving +Y (10mm)...")
            if self.robot.move_tcp([0, 0.01, 0, 0, 0, 0], a=0.1, v=0.05) != 0:
                print("  ✗ Failed to move robot")
                self.positioning_client.terminate_session(session_id)
                return None
            
            time.sleep(1.0)
            
            if not self.get_image_and_pose_at_current_pose(save_dir):
                print("  ✗ Failed to capture view 3")
                self.positioning_client.terminate_session(session_id)
                return None
            
            # Upload view 3
            img_path = os.path.join(save_dir, '2.jpg')
            pose_path = os.path.join(save_dir, '2.json')
            image = cv2.imread(img_path)
            extrinsic = self._load_extrinsic_from_pose_json(pose_path)
            
            result = self.positioning_client.upload_view(
                session_id=session_id,
                reference_name=self.operation_name,
                image=image,
                intrinsic=intrinsic,
                distortion=distortion,
                extrinsic=extrinsic
            )
            
            if result.get('success'):
                view_count += 1
                print(f"  ✓ View 3 uploaded, queue position: {result.get('queue_position', 'N/A')}")
            else:
                print(f"  ✗ Upload failed: {result.get('error')}")
            
            # Return to original position
            print("  Returning to original position...")
            self.robot.move_tcp([0, -0.01, 0, 0, 0, 0], a=0.1, v=0.05)
            time.sleep(1.0)
            
            print(f"\n  ✓ Total {view_count} views uploaded")
            
            # Step 4: Get positioning result
            print("\n>>> Step 4: Getting positioning result (fitting mode)...")
            print(f"  Using {len(self.template_points)} template points")
            
            result = self.positioning_client.get_result(
                session_id,
                template_points=self.template_points,
                timeout=30000
            )
            
            if not result.get('success'):
                print(f"  ✗ Failed to get result: {result.get('error')}")
                self.positioning_client.terminate_session(session_id)
                return None
            
            # Print results
            print("\n" + "="*80)
            print("POSITIONING RESULTS")
            print("="*80)
            
            if 'result' in result:
                triangulation_result = result['result']
                points_3d = triangulation_result.get('points_3d', [])
                mean_error = triangulation_result.get('mean_error', 0)
                processing_time = triangulation_result.get('processing_time', 0)
                views_data = result.get('views', [])
                
                print(f"Number of 3D points: {len(points_3d)}")
                print(f"Mean reprojection error: {mean_error:.3f} pixels")
                print(f"Processing time: {processing_time:.2f} seconds")
                print(f"Number of views: {len(views_data)}")
                
                # Display local2world transformation (server2base)
                if 'local2world' in triangulation_result:
                    local2world = triangulation_result['local2world']
                    print("\nServer-to-Base Transformation Matrix (local2world):")
                    for i, row in enumerate(local2world):
                        print(f"  [{row[0]:8.4f}  {row[1]:8.4f}  {row[2]:8.4f}  {row[3]:8.4f}]")
                    
                    # Extract translation (position of server origin in base frame)
                    tx, ty, tz = local2world[0][3], local2world[1][3], local2world[2][3]
                    print(f"\nServer position (local origin in base frame):")
                    print(f"  X: {tx:.4f} m")
                    print(f"  Y: {ty:.4f} m")
                    print(f"  Z: {tz:.4f} m")
                    
                    # Save server2base matrix to Redis
                    if self.robot_status_client is not None:
                        try:
                            server2base_matrix = np.array(local2world, dtype=np.float64)
                            self.robot_status_client.set_status('ur15', 'server2base_matrix', server2base_matrix)
                            print(f"  ✓ Saved server2base_matrix to Redis (ur15 namespace)")
                        except Exception as e:
                            print(f"  ✗ Failed to save server2base_matrix to Redis: {e}")
                    else:
                        print(f"  ⚠️  RobotStatusClient not available, cannot save to Redis")
                
                print("\n3D Points in World Coordinates:")
                for i, pt in enumerate(points_3d[:5]):  # Show first 5 points
                    pt_name = self.template_points[i]['name'] if i < len(self.template_points) else f"Point_{i}"
                    print(f"  {pt_name}: [{pt[0]:7.4f}, {pt[1]:7.4f}, {pt[2]:7.4f}]")
                
                if len(points_3d) > 5:
                    print(f"  ... and {len(points_3d) - 5} more points")
            
            print("="*80)
            
            # Step 5: Terminate session
            print("\n>>> Step 5: Terminating session...")
            term_result = self.positioning_client.terminate_session(session_id)
            if term_result.get('success'):
                print(f"  ✓ Session {session_id} terminated")
            else:
                print(f"  ⚠️  Failed to terminate session: {term_result.get('error')}")
            
            print("\n✓ Auto positioning completed successfully!")
            return result
            
        except Exception as e:
            print(f"\n✗ Error during auto positioning: {e}")
            import traceback
            traceback.print_exc()
            return None

def main():
    """
    Main function for testing URPositioning class
    """
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='URPositioning - Robot camera capture system')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot')
    parser.add_argument('--camera-topic', type=str, default='/ur15_camera/image_raw',
                       help='ROS topic name for camera images')
    parser.add_argument('--operation-name', type=str, default='test_operation',
                       help='Name of the operation for data directory')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    try:
        # Initialize URPositioning instance with command line arguments
        ur_positioning = URPositioning(
            robot_ip=args.robot_ip,
            robot_port=args.robot_port,
            camera_topic=args.camera_topic,
            operation_name=args.operation_name
        )
        
        # Check if robot was initialized successfully
        if ur_positioning.robot is None or not ur_positioning.robot.connected:
            print("✗ Robot initialization failed. Please check robot connection and try again.")
            return
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(ur_positioning)
        
        # Start executor in a separate thread
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        print('URPositioning node is running. Waiting for camera image...')
        
        # Wait a bit for the camera to start publishing
        time.sleep(2)
        
        try:
            # Auto positioning with multiple views
            result = ur_positioning.auto_positioning()
            if result:
                print("\n🎉 Auto positioning completed successfully!")
            else:
                print("\n❌ Failed to complete auto positioning!")
                
        except KeyboardInterrupt:
            print("\n🛑 Operation interrupted by user")
        except Exception as e:
            print(f"\n❌ Error during execution: {e}")
            import traceback
            traceback.print_exc()
        
        finally:
            # Always disconnect robot in finally block
            if ur_positioning.robot is not None:
                print("Disconnecting robot...")
                ur_positioning.robot.close()
                print("✓ Robot disconnected")
                
            # Shutdown executor
            executor.shutdown()
            # Destroy ROS node
            ur_positioning.destroy_node()
    
    finally:
        # Shutdown ROS2
        rclpy.shutdown()


if __name__ == "__main__":
    main()
