import os
import json
import numpy as np
import requests
from ur15_robot_arm.ur15 import UR15Robot
import time
import socket
import argparse
from robot_status_redis.client_utils import RobotStatusClient
from scipy.spatial.transform import Rotation as R


class UROperate:
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, operation_name="test_operation"):

        # =========================== Configurable Parameters ===========================
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.rs485_port = 54321
        self.operation_name = operation_name

        # ========================= Instance variables =========================
        self.robot = None
        self.rs485_socket = None
        self.robot_status_client = None
        
        # ========================= Other variables =========================
        # Initialize parameter storage
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.cam2end_matrix = None
        
        # Local coordinate system storage
        self.local_transformation_matrix = None
        self.local_origin = None
        
        # Wobj coordinate system storage
        self.wobj_transformation_matrix = None
        self.wobj_origin = None

        # Reference pose storage
        self.ref_joint_angles = None
        
        # Estimated keypoints storage
        self.estimated_keypoints = None
        
        # Task position information storage
        self.offset_in_local = None
        self.offset_in_local_type = None  # 'single' or 'multiple'

        # ========================= Initialization =========================
        self._setup_paths()
        self._initialize_robot()
        self._init_rs485_socket()
        self._initialize_robot_status_client()
        
        # Automatically load neccessary parameters
        self._load_camera_params_from_service()
        self._load_ref_joint_angles_from_json()
        self._load_3d_positioning_result_from_service()
        self._load_local_information_from_service()
        self._load_wobj_information_from_service()
        self._load_operation_config_from_service()
    
    # ================================== Private Helper Methods ==================================
    def _setup_paths(self):
        """Setup directory paths for script, dataset, data and results"""
        # Get the script directory for relative paths
        try:
            from common.workspace_utils import get_workspace_root, get_scripts_directory
            workspace_root = get_workspace_root()
            self.script_dir = get_scripts_directory() if get_scripts_directory() else os.path.dirname(os.path.abspath(__file__))
        except ImportError:
            self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Dataset directory path
        self.dataset_dir = os.path.join(self.script_dir, '..', 'dataset', self.operation_name)
    
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
    
    def _init_rs485_socket(self):
        """Initialize RS485 socket connection"""
        try:
            print(f'Initializing RS485 socket connection at {self.robot_ip}:{self.rs485_port}...')
            # Open RS485 socket connection
            self.rs485_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            rs485_start_res = self.rs485_socket.connect((self.robot_ip, self.rs485_port))
            print(f"✓ RS485 socket connection result: {rs485_start_res}")
        except Exception as e:
            print(f'✗ Failed to initialize RS485 socket: {e}')
            self.rs485_socket = None
    
    def _initialize_robot_status_client(self):
        """Initialize robot status client for getting camera parameters and other status"""
        try:
            print(f'Initializing robot status client...')
            self.robot_status_client = RobotStatusClient()
            print('✓ Robot status client initialized successfully')
        except ConnectionError as e:
            print(f'✗ Redis connection error: {e}')
            print('  Make sure Redis server is running: sudo systemctl start redis-server')
            self.robot_status_client = None
        except Exception as e:
            print(f'✗ Failed to initialize robot status client: {e}')
            import traceback
            traceback.print_exc()
            self.robot_status_client = None

    def _load_camera_params_from_service(self):
        """
        Load camera parameters (intrinsic and extrinsic) from robot status service
        """
        if self.robot_status_client is None:
            print("Robot status client is not initialized, skipping camera parameters loading")
            return False
        
        try:
            # Get camera matrix
            camera_matrix = self.robot_status_client.get_status('ur15', 'camera_matrix')
            if camera_matrix is None:
                print(f"Failed to get camera_matrix from robot status")
                return False
            
            # Get distortion coefficients
            distortion_coefficients = self.robot_status_client.get_status('ur15', 'distortion_coefficients')
            if distortion_coefficients is None:
                print(f"Failed to get distortion_coefficients from robot status")
                return False
            
            # Get cam2end matrix
            cam2end_matrix = self.robot_status_client.get_status('ur15', 'cam2end_matrix')
            if cam2end_matrix is None:
                print(f"Failed to get cam2end_matrix from robot status")
                return False
            
            self.camera_matrix = np.array(camera_matrix)
            self.distortion_coefficients = np.array(distortion_coefficients)
            self.cam2end_matrix = np.array(cam2end_matrix)
            
            if self.camera_matrix.size == 0 or self.distortion_coefficients.size == 0:
                print(f"Invalid camera intrinsic parameters")
                return False
            
            if self.cam2end_matrix.size == 0:
                print(f"Invalid camera extrinsic parameters")
                return False
            
            print(f"Camera parameters loaded successfully from robot status")
            return True
            
        except Exception as e:
            print(f"Error loading camera parameters: {e}")
            return False
    

    def _load_3d_positioning_result_from_service(self):
        """
        Load estimated keypoints coordinates from robot status service
        """
        if self.robot_status_client is None:
            print("Robot status client is not initialized, skipping 3D positioning result loading")
            return None
        
        try:
            # Get 3D points using operation_name and 'points_3d' parameter
            points_3d = self.robot_status_client.get_status(self.operation_name, 'points_3d')
            
            if points_3d is None:
                print(f"Failed to get points_3d from robot status for operation '{self.operation_name}'")
                return None
            
            self.estimated_keypoints = points_3d
            num_keypoints = len(self.estimated_keypoints) if isinstance(self.estimated_keypoints, list) else 0
            
            print(f"Estimated keypoints loaded successfully from robot status")
            print(f"Number of keypoints: {num_keypoints}")
            
            return self.estimated_keypoints
            
        except Exception as e:
            print(f"Error loading estimated keypoints: {e}")
            return None
    
    def _load_local_information_from_service(self):
        """
        Load local coordinate system from robot status service
        Returns: Dictionary with 'transformation_matrix' and 'origin' if successful, None otherwise
        """
        if self.robot_status_client is None:
            print("Robot status client is not initialized, skipping local information loading")
            return None
        
        try:
            # Get local_origin
            local_origin = self.robot_status_client.get_status(self.operation_name, 'local_origin')
            if local_origin is None:
                print(f"Failed to get local_origin from robot status for operation '{self.operation_name}'")
                return None
            
            # Get local_x axis
            local_x = self.robot_status_client.get_status(self.operation_name, 'local_x')
            if local_x is None:
                print(f"Failed to get local_x from robot status for operation '{self.operation_name}'")
                return None
            
            # Get local_y axis
            local_y = self.robot_status_client.get_status(self.operation_name, 'local_y')
            if local_y is None:
                print(f"Failed to get local_y from robot status for operation '{self.operation_name}'")
                return None
            
            # Get local_z axis
            local_z = self.robot_status_client.get_status(self.operation_name, 'local_z')
            if local_z is None:
                print(f"Failed to get local_z from robot status for operation '{self.operation_name}'")
                return None
            
            # Convert to numpy arrays
            origin = np.array(local_origin)
            x_axis = np.array(local_x)
            y_axis = np.array(local_y)
            z_axis = np.array(local_z)
            
            # Build transformation matrix from axes
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, 0] = x_axis  # X-axis as first column
            transformation_matrix[:3, 1] = y_axis  # Y-axis as second column
            transformation_matrix[:3, 2] = z_axis  # Z-axis as third column
            transformation_matrix[:3, 3] = origin  # Origin as translation
            
            self.local_transformation_matrix = transformation_matrix
            self.local_origin = {
                'x': float(origin[0]),
                'y': float(origin[1]),
                'z': float(origin[2])
            }
            
            print(f"Local coordinate system loaded successfully from robot status")
            
            return {
                'transformation_matrix': self.local_transformation_matrix,
                'origin': self.local_origin
            }
            
        except Exception as e:
            print(f"Error loading local coordinate system: {e}")
            return None

    def _load_wobj_information_from_service(self):
        """
        Load the crack local coordinate system from robot status service
        """
        if self.robot_status_client is None:
            print("Robot status client is not initialized, skipping wobj information loading")
            return None
        
        try:
            # Get wobj_origin
            wobj_origin = self.robot_status_client.get_status('wobj', 'wobj_origin')
            if wobj_origin is None:
                print(f"Failed to get wobj_origin from robot status for 'wobj'")
                return None
            
            # Get wobj_x axis
            wobj_x = self.robot_status_client.get_status('wobj', 'wobj_x')
            if wobj_x is None:
                print(f"Failed to get wobj_x from robot status for 'wobj'")
                return None
            
            # Get wobj_y axis
            wobj_y = self.robot_status_client.get_status('wobj', 'wobj_y')
            if wobj_y is None:
                print(f"Failed to get wobj_y from robot status for 'wobj'")
                return None
            
            # Get wobj_z axis
            wobj_z = self.robot_status_client.get_status('wobj', 'wobj_z')
            if wobj_z is None:
                print(f"Failed to get wobj_z from robot status for 'wobj'")
                return None
            
            # Convert to numpy arrays
            origin = np.array(wobj_origin)
            x_axis = np.array(wobj_x)
            y_axis = np.array(wobj_y)
            z_axis = np.array(wobj_z)
            
            # Build transformation matrix from axes
            transformation_matrix = np.eye(4)
            transformation_matrix[:3, 0] = x_axis  # X-axis as first column
            transformation_matrix[:3, 1] = y_axis  # Y-axis as second column
            transformation_matrix[:3, 2] = z_axis  # Z-axis as third column
            transformation_matrix[:3, 3] = origin  # Origin as translation
            
            self.wobj_transformation_matrix = transformation_matrix
            self.wobj_origin = origin
            
            print(f"Crack coordinate system loaded successfully from robot status")
            
            return True
            
        except Exception as e:
            print(f"Error loading crack coordinate system: {e}")
            return None
        
    def _load_ref_joint_angles_from_json(self):
        """
        Load reference joint angles from dataset/operation_name/ref_img_1_pose.json
        """
        ref_pose_file = os.path.join(self.dataset_dir, "ref_img_1_pose.json")
        
        try:
            with open(ref_pose_file, 'r') as f:
                data = json.load(f)
            
            if 'joint_angles' not in data:
                print(f"No 'joint_angles' field found in reference pose file")
                return None
            
            self.ref_joint_angles = data['joint_angles']
            
            print(f"Reference joint angles loaded successfully from {ref_pose_file}")
            print(f"Joint angles: {self.ref_joint_angles}")
            
            return self.ref_joint_angles
            
        except FileNotFoundError:
            print(f"Reference pose file not found: {ref_pose_file}")
            return None
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return None
        except Exception as e:
            print(f"Error loading reference joint angles: {e}")
            return None
    
    def _load_operation_config_from_service(self):
        """
        Load task position offset information from robot status service
        The offsets are represented in the local coordinate system
        Can load single offset or multiple steps (step1, step2)
        All offsets are stored as dictionaries with 'x', 'y', 'z' keys
        Returns: Dictionary with offset information if successful, None otherwise
        """
        if self.robot_status_client is None:
            print("Robot status client is not initialized, skipping operation config loading")
            return None
        
        try:
            # Get operation_config from robot status
            operation_config = self.robot_status_client.get_status('operation_config', self.operation_name)
            
            if operation_config is None:
                print(f"Failed to get operation_config.{self.operation_name} from robot status")
                return None
            
            self.offset_in_local = {}
            
            # Check if it's a multi-step operation
            if isinstance(operation_config, dict):
                # Check for step1/step2 pattern
                if 'step1' in operation_config or 'step2' in operation_config:
                    # Multi-step operation
                    self.offset_in_local_type = 'multiple'
                    for step_key in ['step1', 'step2']:
                        if step_key in operation_config:
                            step_data = operation_config[step_key]
                            if isinstance(step_data, list) and len(step_data) == 3:
                                self.offset_in_local[step_key] = {
                                    'x': step_data[0],
                                    'y': step_data[1],
                                    'z': step_data[2]
                                }
                    
                    if not self.offset_in_local:
                        print(f"No valid step data found in operation_config.{self.operation_name}")
                        return None
                    
                    print(f"Task position information loaded successfully (multi-step)")
                    for step_key, offset in self.offset_in_local.items():
                        print(f"  {step_key}: x={offset['x']:.6f}, y={offset['y']:.6f}, z={offset['z']:.6f}")
                else:
                    print(f"Unexpected format for operation_config.{self.operation_name}")
                    return None
            elif isinstance(operation_config, list) and len(operation_config) == 3:
                # Single offset operation
                self.offset_in_local_type = 'single'
                self.offset_in_local = {
                    'x': operation_config[0],
                    'y': operation_config[1],
                    'z': operation_config[2]
                }
                
                print(f"Task position information loaded successfully (single offset)")
                print(f"  offset: x={self.offset_in_local['x']:.6f}, "
                      f"y={self.offset_in_local['y']:.6f}, "
                      f"z={self.offset_in_local['z']:.6f}")
            else:
                print(f"Invalid format for operation_config.{self.operation_name}")
                return None
            
            return self.offset_in_local
            
        except Exception as e:
            print(f"Error loading task position information: {e}")
            return None

    # ============================= Robot Movement and Control Methods =============================
    def movej_to_reference_position(self):
        """
        Move robot to reference joint positions
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.ref_joint_angles is None:
            print("Reference joint angles not loaded")
            return -1
        
        print("Moving robot to reference joint positions...")
        print(f"Target joint angles: {self.ref_joint_angles}")
        
        res = self.robot.movej(self.ref_joint_angles, a=0.5, v=0.5)
        
        if res == 0:
            print("Robot moved to reference position successfully")
        else:
            print(f"Failed to move robot to reference position (error code: {res})")
        
        return res
    
    def movel_in_wobj_frame(self, offset):
        """
        Move robot based on "offset" (dx, dy, dz) in wobj coordinate system.   
        Args:
            offset: List/tuple [dx, dy, dz] in wobj coordinate system (meters)
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.wobj_transformation_matrix is None:
            print("Wobj coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Parse offset parameter - must be a 3D array
        if isinstance(offset, (list, tuple)) and len(offset) == 3:
            wobj_local_displacement = np.array(offset)
        else:
            print("Invalid offset parameter. Must be [dx, dy, dz] in wobj coordinate system")
            return -1
        
        # Extract rotation matrix from wobj coordinate system transformation matrix (3x3)
        wobj_rotation = self.wobj_transformation_matrix[:3, :3]
        
        # Transform displacement from wobj coordinate system to base coordinate system
        # displacement_base = R_wobj_to_base * displacement_wobj
        base_displacement = wobj_rotation @ wobj_local_displacement
        
        # Calculate target pose in base coordinate system
        target_pose = [
            current_pose[0] + base_displacement[0],  # x
            current_pose[1] + base_displacement[1],  # y
            current_pose[2] + base_displacement[2],  # z
            current_pose[3],                         # rx (keep current orientation)
            current_pose[4],                         # ry
            current_pose[5]                          # rz
        ]
        
        print(f"[INFO] Moving robot: {wobj_local_displacement} in wobj frame...")
        print(f"Wobj displacement (wobj frame): {wobj_local_displacement}")

        res = self.robot.movel(target_pose, a=0.1, v=0.1)
        time.sleep(0.5)

        if res == 0:
            print("[INFO] Successfully moved robot")
        else:
            print(f"[ERROR] Failed to move robot (error code: {res})")
        
        return res
    
    def movel_to_correct_tcp_pose(self, tcp_x_to_wobj=[1, 0, 0], tcp_y_to_wobj=[0, 0, -1], tcp_z_to_wobj=[0, 1, 0], angle_deg=31):
        """
        Correct tool TCP orientation based on wobj coordinate system.
        
        Args:
            tcp_x_to_wobj: List [x, y, z] indicating TCP X+ axis alignment to wobj axes
                          e.g., [1, 0, 0] means TCP X+ aligns with Wobj X+
                                [0, 1, 0] means TCP X+ aligns with Wobj Y+
                                [0, 0, -1] means TCP X+ aligns with Wobj Z- (default: [1, 0, 0])
            tcp_y_to_wobj: List [x, y, z] indicating TCP Y+ axis alignment to wobj axes (default: [0, 0, -1])
            tcp_z_to_wobj: List [x, y, z] indicating TCP Z+ axis alignment to wobj axes (default: [0, 1, 0])
            angle_deg: Additional rotation angle around TCP Z axis in degrees (default: 31)
        
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.wobj_transformation_matrix is None:
            print("Wobj coordinate system transformation matrix not loaded")
            return -1
        
        # Convert alignment vectors to numpy arrays
        tcp_x_align = np.array(tcp_x_to_wobj, dtype=float)
        tcp_y_align = np.array(tcp_y_to_wobj, dtype=float)
        tcp_z_align = np.array(tcp_z_to_wobj, dtype=float)
        
        # Validate alignment vectors
        # Check 1: Each vector should have length 1 or sqrt(2) or sqrt(3)
        tolerance = 1e-6
        valid_lengths = [1.0, np.sqrt(2), np.sqrt(3)]
        
        for vec, name in [(tcp_x_align, 'tcp_x_to_wobj'), 
                          (tcp_y_align, 'tcp_y_to_wobj'), 
                          (tcp_z_align, 'tcp_z_to_wobj')]:
            vec_length = np.linalg.norm(vec)
            if not any(abs(vec_length - valid_len) < tolerance for valid_len in valid_lengths):
                print(f"[ERROR] Invalid {name}: length is {vec_length:.6f}, expected 1.0, √2, or √3")
                print(f"        Values should be from {{-1, 0, 1}} only")
                return -1
        
        # Check 2: Vectors should be mutually orthogonal
        dot_xy = np.dot(tcp_x_align, tcp_y_align)
        dot_xz = np.dot(tcp_x_align, tcp_z_align)
        dot_yz = np.dot(tcp_y_align, tcp_z_align)
        
        if abs(dot_xy) > tolerance or abs(dot_xz) > tolerance or abs(dot_yz) > tolerance:
            print(f"[ERROR] Alignment vectors are not orthogonal:")
            print(f"        tcp_x · tcp_y = {dot_xy:.6f}")
            print(f"        tcp_x · tcp_z = {dot_xz:.6f}")
            print(f"        tcp_y · tcp_z = {dot_yz:.6f}")
            print(f"        All dot products should be 0")
            return -1
        
        # Check 3: Should form a right-handed coordinate system (cross product check)
        cross_product = np.cross(tcp_x_align, tcp_y_align)
        cross_product_normalized = cross_product / np.linalg.norm(cross_product)
        tcp_z_normalized = tcp_z_align / np.linalg.norm(tcp_z_align)
        
        if not np.allclose(cross_product_normalized, tcp_z_normalized, atol=tolerance):
            print(f"[ERROR] Vectors do not form a right-handed coordinate system")
            print(f"        tcp_x × tcp_y = {cross_product_normalized}")
            print(f"        tcp_z (normalized) = {tcp_z_normalized}")
            print(f"        For right-handed system: tcp_x × tcp_y should equal tcp_z")
            return -1
        
        print("[INFO] Alignment vectors validated successfully")
        print(f"       TCP X -> Wobj: {tcp_x_align}")
        print(f"       TCP Y -> Wobj: {tcp_y_align}")
        print(f"       TCP Z -> Wobj: {tcp_z_align}")
        
        # Get current TCP pose
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract rotation matrix from wobj coordinate system transformation matrix (3x3)
        wobj_rotation = self.wobj_transformation_matrix[:3, :3]
        
        # Wobj coordinate system axes in base coordinates
        wobj_x = wobj_rotation[:, 0]  # Wobj X+ direction
        wobj_y = wobj_rotation[:, 1]  # Wobj Y+ direction
        wobj_z = wobj_rotation[:, 2]  # Wobj Z+ direction
        
        # Construct target rotation matrix for tool based on alignment parameters
        # TCP X+ axis direction in base frame
        tool_x_direction = (tcp_x_align[0] * wobj_x + 
                           tcp_x_align[1] * wobj_y + 
                           tcp_x_align[2] * wobj_z)
        
        # TCP Y+ axis direction in base frame
        tool_y_direction = (tcp_y_align[0] * wobj_x + 
                           tcp_y_align[1] * wobj_y + 
                           tcp_y_align[2] * wobj_z)
        
        # TCP Z+ axis direction in base frame
        tool_z_direction = (tcp_z_align[0] * wobj_x + 
                           tcp_z_align[1] * wobj_y + 
                           tcp_z_align[2] * wobj_z)
        
        target_tool_rotation = np.column_stack([
            tool_x_direction,  # Tool X+ direction
            tool_y_direction,  # Tool Y+ direction
            tool_z_direction   # Tool Z+ direction
        ])
        
        # Convert rotation matrix to rotation vector (axis-angle representation)
        # UR uses rotation vector [rx, ry, rz] where the direction is the axis and the magnitude is the angle in radians
        rotation_obj = R.from_matrix(target_tool_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Step 1: Align with wobj coordinate system
        target_pose = [
            current_pose[0],      # x (keep current position)
            current_pose[1],      # y
            current_pose[2],      # z
            rotation_vector[0],   # rx
            rotation_vector[1],   # ry
            rotation_vector[2]    # rz
        ]
        
        print("\nStep 1: Aligning tool TCP...")
        print(f"Target pose: {target_pose}")
        
        res = self.robot.movel(target_pose, a=0.1, v=0.1)
        time.sleep(0.5)
        
        if res != 0:
            print(f"Failed to align tool TCP (error code: {res})")
            return res
        else:
            print("Aligned tool tcp pose with wobj coordinate system successfully")
        
        # Step 2: Rotate around TCP Z axis
        angle_rad = np.deg2rad(angle_deg)
        
        # The Z axis in tool corresponds to wobj_y in base coordinates
        rotation_axis = wobj_y / np.linalg.norm(wobj_y)  # Normalize (should already be normalized)
        additional_rotation = R.from_rotvec(angle_rad * rotation_axis)
        
        # Combine the rotations: first align, then rotate around Z
        combined_rotation = additional_rotation * rotation_obj
        combined_rotation_vector = combined_rotation.as_rotvec()
        
        final_pose = [
            current_pose[0],              # x (keep current position)
            current_pose[1],              # y
            current_pose[2],              # z
            combined_rotation_vector[0],  # rx
            combined_rotation_vector[1],  # ry
            combined_rotation_vector[2]   # rz
        ]
        
        print(f"\nStep 2: Rotating {angle_deg} degrees around TCP Z axis to correct tool orientation...")
                
        res = self.robot.movel(final_pose, a=0.1, v=0.1)
        time.sleep(0.5)

        if res == 0:
            print("Tool TCP orientation corrected successfully")
        else:
            print(f"Failed to rotate around TCP Z axis (error code: {res})")
        
        return res
    
    def calculate_start_position_in_base(self, step_key=None):
        """
        Calculate target position in base coordinate system from offset in local coordinate system
        Uses the local-to-base transformation matrix and the task position offset
        
        Args:
            step_key: For multi-step operations, specify 'step1' or 'step2'. 
                     For single offset operations, leave as None.
        
        Returns: numpy array [x, y, z] in base coordinate system if successful, None otherwise
        """
        if self.local_transformation_matrix is None:
            print("Local coordinate system transformation matrix not loaded")
            return None
        
        if self.offset_in_local is None:
            print("Task position offset not loaded")
            return None
        
        # Determine which offset to use
        if self.offset_in_local_type == 'multiple':
            # Multi-step operation
            if step_key is None:
                step_key = 'step1'  # Default to step1
            
            if step_key not in self.offset_in_local:
                print(f"Step '{step_key}' not found in task position offset")
                print(f"Available steps: {list(self.offset_in_local.keys())}")
                return None
            
            offset = self.offset_in_local[step_key]
        else:
            # Single offset operation
            offset = self.offset_in_local
        
        # Create homogeneous coordinates for the offset point in local coordinate system
        offset_local = np.array([
            offset['x'],
            offset['y'],
            offset['z'],
            1.0  # homogeneous coordinate
        ])
        
        # Transform to base coordinate system
        # local_transformation_matrix is the local-to-base transformation (4x4)
        position_base_homogeneous = np.dot(self.local_transformation_matrix, offset_local)
        
        # Extract x, y, z from homogeneous coordinates
        position_base = position_base_homogeneous[:3]
        
        step_info = f" ({step_key})" if self.offset_in_local_type == 'multiple' else ""
        print(f"Target position calculation{step_info}:")
        print(f"  Offset in local coordinate system: ({offset['x']:.6f}, "
              f"{offset['y']:.6f}, {offset['z']:.6f})")
        print(f"  Position in base coordinate system: ({position_base[0]:.6f}, "
              f"{position_base[1]:.6f}, {position_base[2]:.6f})")
        
        return position_base
    
    def movel_to_start_position(self, step_key=None, execution_order=[1, 3, 2]):
        """
        Move robot to start position using linear movement based on wobj coordinate system.
        This method moves in multiple steps according to execution_order to avoid collision.
        
        Args:
            step_key: For multi-step operations, specify 'step1' or 'step2'. 
                     For single offset operations, leave as None.
            execution_order: List of integers [1,2,3] representing movement sequence along wobj axes.
                            1=X axis, 2=Y axis, 3=Z axis
                            e.g., [1,2,3] means move along X first, then Y, then Z
                            e.g., [1,3,2] means move along X first, then Z, then Y (default)
        
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.wobj_transformation_matrix is None:
            print("Wobj coordinate system transformation matrix not loaded")
            return -1
        
        # Validate execution_order parameter
        if not isinstance(execution_order, list) or len(execution_order) != 3:
            print("[ERROR] execution_order must be a list of 3 integers")
            return -1
        
        if set(execution_order) != {1, 2, 3}:
            print("[ERROR] execution_order must contain exactly [1,2,3] in any order")
            print(f"       Received: {execution_order}")
            return -1
        
        # Get target position in base coordinate system
        target_position = self.calculate_start_position_in_base(step_key)
        if target_position is None:
            print("Failed to calculate target position")
            return -1
        
        # Get current TCP pose to preserve orientation
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract wobj coordinate system axes from transformation matrix
        wobj_rotation = self.wobj_transformation_matrix[:3, :3]
        # Normalize axes to ensure they are unit vectors for accurate projection
        wobj_axes = {
            1: wobj_rotation[:, 0] / np.linalg.norm(wobj_rotation[:, 0]),  # Wobj X+ direction (normalized)
            2: wobj_rotation[:, 1] / np.linalg.norm(wobj_rotation[:, 1]),  # Wobj Y+ direction (normalized)
            3: wobj_rotation[:, 2] / np.linalg.norm(wobj_rotation[:, 2])   # Wobj Z+ direction (normalized)
        }
        axis_names = {1: 'X', 2: 'Y', 3: 'Z'}

        # Calculate movement vector from current position to target
        current_position = np.array(current_pose[:3])
        target_position_array = np.array(target_position)
        movement_vector = target_position_array - current_position
        print(f"\nMovement vector in base coordinates: {movement_vector}")
        
        # Project movement vector onto wobj coordinate system axes
        movement_components = {
            1: np.dot(movement_vector, wobj_axes[1]),  # X component
            2: np.dot(movement_vector, wobj_axes[2]),  # Y component
            3: np.dot(movement_vector, wobj_axes[3])   # Z component
        }
        
        print(f"Movement in wobj frame: X={movement_components[1]:.6f}, Y={movement_components[2]:.6f}, Z={movement_components[3]:.6f}")
        print(f"Execution order: {[axis_names[i] for i in execution_order]}")

        # Execute movements according to execution_order
        accumulated_position = current_position.copy()
        
        for step_idx, axis_id in enumerate(execution_order, start=1):
            axis_name = axis_names[axis_id]
            axis_vector = wobj_axes[axis_id]
            movement_amount = movement_components[axis_id]
            
            # Calculate next position by adding current axis movement
            movement_delta = movement_amount * axis_vector
            accumulated_position = accumulated_position + movement_delta
            
            next_pose = [
                accumulated_position[0],  # x
                accumulated_position[1],  # y
                accumulated_position[2],  # z
                current_pose[3],          # rx (keep current orientation)
                current_pose[4],          # ry
                current_pose[5]           # rz
            ]
            
            print(f"\nStep {step_idx}: Moving along wobj {axis_name} direction...")
            print(f"  Movement amount: {movement_amount:.6f}m")
            print(f"  Target pose: {next_pose}")
            
            res = self.robot.movel(next_pose, a=0.1, v=0.1)
            time.sleep(0.5)

            if res != 0:
                print(f"[ERROR] Failed to move along wobj {axis_name} (error code: {res})")
                return res
            
            print(f"Step {step_idx} completed successfully")
        
        print("\n✓ Robot moved to start position successfully")
        return 0
    
    # ============================ Quick Changer Control Methods =============================
    def lock_quick_changer(self):
        """
        Lock the quick changer via RS485 command
        Returns: True if successful, False otherwise
        """
        if self.rs485_socket is None:
            print("RS485 socket is not initialized")
            return False
        
        try:
            lock_command = [0x53, 0x26, 0x01, 0x01, 0x01, 0x3A, 0xD4]
            print("Sending lock command to quick changer...")
            
            self.rs485_socket.sendall(bytes(lock_command))
            time.sleep(0.5)
            
            rs485_data = self.rs485_socket.recv(1024)
            print(f"✓ RS485 Received Data: {' '.join(f'0x{b:02x}' for b in rs485_data)}")
            time.sleep(0.5)
            
            print("Quick changer locked successfully")
            return True
            
        except Exception as e:
            print(f"✗ Failed to lock quick changer: {e}")
            return False
    
    def unlock_quick_changer(self):
        """
        Unlock the quick changer via RS485 command
        Returns: True if successful, False otherwise
        """
        if self.rs485_socket is None:
            print("RS485 socket is not initialized")
            return False
        
        try:
            unlock_command = [0x53, 0x26, 0x01, 0x01, 0x02, 0x7A, 0xD5]
            print("Sending unlock command to quick changer...")
            
            self.rs485_socket.sendall(bytes(unlock_command))
            time.sleep(0.5)
            
            rs485_data = self.rs485_socket.recv(1024)
            print(f"✓ RS485 Received Data: {' '.join(f'0x{b:02x}' for b in rs485_data)}")
            time.sleep(0.5)
            
            print("Quick changer unlocked successfully")
            return True
            
        except Exception as e:
            print(f"✗ Failed to unlock quick changer: {e}")
            return False
    

if __name__ == "__main__":
    # Initialize ROS2
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='UROperate - Robot operation and control system')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot (default: 192.168.1.15)')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot (default: 30002)')
    parser.add_argument('--operation-name', type=str, default='test_operation',
                       help='Name of the operation (default: test_operation)')
    
    args = parser.parse_args()
    
    # Create UROperate instance with command line arguments
    ur_operate = UROperate(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        operation_name=args.operation_name
    )
    
    