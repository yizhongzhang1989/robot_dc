import os
import json
import numpy as np
import requests
from ur15_robot_arm.ur15 import UR15Robot
import time
import socket
from robot_status.client_utils import RobotStatusClient


class UROperate:
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, rs485_port=54321):

        # =========================== Configurable Parameters ===========================
        # IP address and port for UR15 robot
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        
        # Port for RS485 connection of UR15
        self.rs485_port = rs485_port
        
        # Data directory path (for storing collected data)
        self.data_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_crack_data"
        )
        
        # Result directory path (for storing results)
        self.result_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_crack_result"
        )
        
        # File names for coordinate system data
        self.COORD_SYSTEM_RESULT_FILENAME = "log_local_coordinate_system_result.json"

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
        
        # Crack coordinate system storage
        self.crack_coord_origin = None
        self.crack_coord_transformation = None
        self.crack_coord_pose = None
        
        # Reference pose storage
        self.ref_joint_angles = None
        
        # Estimated keypoints storage
        self.estimated_keypoints = None
        
        # Task position information storage
        self.task_position_offset = None

        # ========================= Initialization =========================
        # Initialize the robot connection
        self._initialize_robot()
        
        # Initialize RS485 socket connection
        self._init_rs485_socket()
        
        # Initialize robot status client
        self._initialize_robot_status_client()
        
        # Automatically load all parameters
        self._load_camera_intrinsic()
        self._load_camera_extrinsic()
        self._load_estimated_kp_coordinates()
        self._load_local_coordinate_system()
        self._load_crack_local_coordinate_system()
        self._load_ref_joint_angles()
        self._load_task_position_information()
    
    def _load_camera_intrinsic(self):
        """
        Load camera intrinsic parameters from robot status service
        """
        try:
            status = self.robot_status_client.get_status()
            
            if status is None:
                print(f"Failed to get robot status")
                return False
            
            camera_params = status.get('camera_params', {})
            intrinsic = camera_params.get('intrinsic', {})
            
            if not intrinsic:
                print(f"No camera intrinsic parameters found in robot status")
                return False
            
            self.camera_matrix = np.array(intrinsic.get('camera_matrix', []))
            self.distortion_coefficients = np.array(intrinsic.get('distortion_coefficients', []))
            
            if self.camera_matrix.size == 0 or self.distortion_coefficients.size == 0:
                print(f"Invalid camera intrinsic parameters")
                return False
            
            print(f"Camera intrinsic parameters loaded successfully from robot status")
            return True
            
        except Exception as e:
            print(f"Error loading intrinsic parameters: {e}")
            return False
    
    def _load_camera_extrinsic(self):
        """
        Load camera extrinsic parameters from robot status service
        """
        try:
            status = self.robot_status_client.get_status()
            
            if status is None:
                print(f"Failed to get robot status")
                return False
            
            camera_params = status.get('camera_params', {})
            extrinsic = camera_params.get('extrinsic', {})
            
            if not extrinsic:
                print(f"No camera extrinsic parameters found in robot status")
                return False
            
            self.cam2end_matrix = np.array(extrinsic.get('cam2end_matrix', []))
            
            if self.cam2end_matrix.size == 0:
                print(f"Invalid camera extrinsic parameters")
                return False
            
            print(f"Camera extrinsic parameters loaded successfully from robot status")
            return True
            
        except Exception as e:
            print(f"Error loading extrinsic parameters: {e}")
            return False
    
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
        except Exception as e:
            print(f'✗ Failed to initialize robot status client: {e}')
            self.robot_status_client = None
    
    def _load_estimated_kp_coordinates(self):
        """
        Load estimated keypoints coordinates from log_estimation_result.json
        """
        estimation_file = os.path.join(
            self.result_dir,
            "log_estimation_result.json"
        )
        
        try:
            with open(estimation_file, 'r') as f:
                data = json.load(f)
            
            if 'estimated_keypoints' not in data:
                print(f"No 'estimated_keypoints' field found in estimation result file")
                return None
            
            self.estimated_keypoints = data['estimated_keypoints']
            num_keypoints = len(self.estimated_keypoints)
            
            print(f"Estimated keypoints loaded successfully")
            print(f"Number of keypoints: {num_keypoints}")
            
            return self.estimated_keypoints
            
        except FileNotFoundError:
            print(f"Estimation result file not found: {estimation_file}")
            return None
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return None
        except Exception as e:
            print(f"Error loading estimated keypoints: {e}")
            return None
    
    def _load_local_coordinate_system(self):
        """
        Load local coordinate system transformation matrix and origin from log_local_coordinate_system_result.json
        Returns: Dictionary with 'transformation_matrix' and 'origin' if successful, None otherwise
        """
        coordinate_system_file = os.path.join(
            self.result_dir,
            "log_local_coordinate_system_result.json"
        )
        
        try:
            with open(coordinate_system_file, 'r') as f:
                data = json.load(f)
            
            if 'transformation_matrix' not in data:
                print(f"No 'transformation_matrix' field found in coordinate system result file")
                return None
            
            if 'origin' not in data:
                print(f"No 'origin' field found in coordinate system result file")
                return None
            
            self.local_transformation_matrix = np.array(data['transformation_matrix'])
            self.local_origin = data['origin']
            
            print(f"Local coordinate system loaded successfully")
            print(f"Origin: ({self.local_origin['x']:.6f}, {self.local_origin['y']:.6f}, {self.local_origin['z']:.6f})")
            
            return {
                'transformation_matrix': self.local_transformation_matrix,
                'origin': self.local_origin
            }
            
        except FileNotFoundError:
            print(f"Coordinate system result file not found: {coordinate_system_file}")
            return None
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return None
        except Exception as e:
            print(f"Error loading local coordinate system: {e}")
            return None
    
    def _load_ref_joint_angles(self):
        """
        Load reference joint angles from ref_pose.json
        Returns: List of joint angles if successful, None otherwise
        """
        ref_pose_file = os.path.join(
            self.data_dir,
            "ref_pose.json"
        )
        
        try:
            with open(ref_pose_file, 'r') as f:
                data = json.load(f)
            
            if 'joint_angles' not in data:
                print(f"No 'joint_angles' field found in reference pose file")
                return None
            
            self.ref_joint_angles = data['joint_angles']
            
            print(f"Reference joint angles loaded successfully")
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
    
    def _load_task_position_information(self):
        """
        Load task position offset information from task_position_information.json
        The offsets are represented in the local coordinate system
        Returns: Dictionary with offset information if successful, None otherwise
        """
        task_position_file = os.path.join(
            self.data_dir,
            "task_position_information.json"
        )
        
        try:
            with open(task_position_file, 'r') as f:
                data = json.load(f)
            
            required_fields = ['x_offset_in_local', 'y_offset_in_local', 'z_offset_in_local']
            missing_fields = [field for field in required_fields if field not in data]
            
            if missing_fields:
                print(f"Missing fields in task position information file: {missing_fields}")
                return None
            
            self.task_position_offset = {
                'x': data['x_offset_in_local'],
                'y': data['y_offset_in_local'],
                'z': data['z_offset_in_local']
            }
            
            print(f"Task position information loaded successfully")
            print(f"Offset (in local coordinate system): x={self.task_position_offset['x']:.6f}, "
                  f"y={self.task_position_offset['y']:.6f}, "
                  f"z={self.task_position_offset['z']:.6f}")
            
            return self.task_position_offset
            
        except FileNotFoundError:
            print(f"Task position information file not found: {task_position_file}")
            return None
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return None
        except Exception as e:
            print(f"Error loading task position information: {e}")
            return None

    def _load_crack_local_coordinate_system(self, coord_system_file_path=None):
        """
        Load the local coordinate system established for crack detection/tracking
        """
        try:
            print("\n" + "="*50)
            print("Loading Crack Local Coordinate System")
            print("="*50)
            
            # Determine file path
            if coord_system_file_path is None:
                coord_system_file_path = os.path.join(self.result_dir, self.COORD_SYSTEM_RESULT_FILENAME)
            
            # Check if file exists
            if not os.path.exists(coord_system_file_path):
                error_msg = f"Coordinate system file not found: {coord_system_file_path}"
                print(f"✗ {error_msg}")
                return {
                    'success': False,
                    'error': error_msg,
                    'origin': None,
                    'transformation_matrix': None,
                    'pose_representation': None,
                    'axes': None
                }
            
            print(f"📖 Loading from: {coord_system_file_path}")
            
            # Load coordinate system data
            with open(coord_system_file_path, 'r') as f:
                coord_data = json.load(f)
            
            # Validate required fields
            required_fields = ['origin', 'transformation_matrix', 'pose_representation', 'axes']
            missing_fields = [field for field in required_fields if field not in coord_data]
            
            if missing_fields:
                error_msg = f"Missing required fields in coordinate system file: {missing_fields}"
                print(f"✗ {error_msg}")
                return {
                    'success': False,
                    'error': error_msg,
                    'origin': None,
                    'transformation_matrix': None,
                    'pose_representation': None,
                    'axes': None
                }
            
            # Extract origin coordinates
            origin = coord_data['origin']
            origin_point = np.array([origin['x'], origin['y'], origin['z']])
            
            # Extract transformation matrix
            transformation_matrix = np.array(coord_data['transformation_matrix'])
            
            # Validate transformation matrix shape
            if transformation_matrix.shape != (4, 4):
                error_msg = f"Invalid transformation matrix shape: {transformation_matrix.shape} (expected 4x4)"
                print(f"✗ {error_msg}")
                return {
                    'success': False,
                    'error': error_msg,
                    'origin': None,
                    'transformation_matrix': None,
                    'pose_representation': None,
                    'axes': None
                }
            
            # Extract pose representation
            pose_repr = coord_data['pose_representation']
            
            # Extract axes information
            axes_info = coord_data['axes']
            
            # ========== Store to instance variables for easy access ==========
            self.crack_coord_origin = origin_point.copy()
            self.crack_coord_transformation = transformation_matrix.copy()
            self.crack_coord_pose = pose_repr.copy()
            
            # Print loaded information
            print("✓ Successfully loaded coordinate system:")
            print(f"  Origin: ({origin_point[0]:.6f}, {origin_point[1]:.6f}, {origin_point[2]:.6f}) m")

            # Print axes information
            x_axis = axes_info['x_axis']['vector']
            y_axis = axes_info['y_axis']['vector']
            z_axis = axes_info['z_axis']['vector']
            
            print(f"  X-axis: [{x_axis[0]:.4f}, {x_axis[1]:.4f}, {x_axis[2]:.4f}] ({axes_info['x_axis']['source']})")
            print(f"  Y-axis: [{y_axis[0]:.4f}, {y_axis[1]:.4f}, {y_axis[2]:.4f}] ({axes_info['y_axis']['source']})")
            print(f"  Z-axis: [{z_axis[0]:.4f}, {z_axis[1]:.4f}, {z_axis[2]:.4f}] ({axes_info['z_axis']['source']})")
                        
            print("="*50)
            
            # Return successful result
            return {
                'success': True,
                'origin': origin,
                'origin_array': origin_point,
                'transformation_matrix': transformation_matrix,
                'pose_representation': pose_repr,
                'axes': axes_info,
                'full_data': coord_data,
                'file_path': coord_system_file_path,
                'error': None
            }
            
        except FileNotFoundError:
            error_msg = f"Coordinate system file not found: {coord_system_file_path}"
            print(f"✗ {error_msg}")
            return {
                'success': False,
                'error': error_msg,
                'origin': None,
                'transformation_matrix': None,
                'pose_representation': None,
                'axes': None
            }
        except json.JSONDecodeError as e:
            error_msg = f"Invalid JSON format in coordinate system file: {e}"
            print(f"✗ {error_msg}")
            return {
                'success': False,
                'error': error_msg,
                'origin': None,
                'transformation_matrix': None,
                'pose_representation': None,
                'axes': None
            }
        except KeyError as e:
            error_msg = f"Missing key in coordinate system file: {e}"
            print(f"✗ {error_msg}")
            return {
                'success': False,
                'error': error_msg,
                'origin': None,
                'transformation_matrix': None,
                'pose_representation': None,
                'axes': None
            }
        except Exception as e:
            error_msg = f"Error loading coordinate system: {e}"
            print(f"✗ {error_msg}")
            return {
                'success': False,
                'error': error_msg,
                'origin': None,
                'transformation_matrix': None,
                'pose_representation': None,
                'axes': None
            }


if __name__ == "__main__":
    # Example usage
    ur_operate = UROperate()
    
    # All parameters are automatically loaded during initialization
    # Access them directly from instance variables
    
    if ur_operate.camera_matrix is not None:
        print("\nCamera Matrix:")
        print(ur_operate.camera_matrix)
        print("\nDistortion Coefficients:")
        print(ur_operate.distortion_coefficients)
    
    if ur_operate.cam2end_matrix is not None:
        print("\nCamera to End-effector Matrix:")
        print(ur_operate.cam2end_matrix)
    
    if ur_operate.estimated_keypoints is not None:
        print("\nEstimated Keypoints:")
        for kp in ur_operate.estimated_keypoints:
            print(f"Keypoint {kp['keypoint_index']}: ({kp['x']:.6f}, {kp['y']:.6f}, {kp['z']:.6f})")
    
    if ur_operate.local_transformation_matrix is not None:
        print("\nLocal Coordinate System Transformation Matrix:")
        print(ur_operate.local_transformation_matrix)
    
    if ur_operate.ref_joint_angles is not None:
        print("\nReference Joint Angles (radians):")
        for i, angle in enumerate(ur_operate.ref_joint_angles):
            print(f"Joint {i}: {angle:.6f}")
    
    if ur_operate.task_position_offset is not None:
        print("\nTask Position Offset (in local coordinate system):")
        print(f"x: {ur_operate.task_position_offset.get('x', 0):.6f}")
        print(f"y: {ur_operate.task_position_offset.get('y', 0):.6f}")
        print(f"z: {ur_operate.task_position_offset.get('z', 0):.6f}")
