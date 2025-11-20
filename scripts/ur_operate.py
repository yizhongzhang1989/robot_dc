import os
import json
import numpy as np
import requests
from ur15_robot_arm.ur15 import UR15Robot
import time
import socket
import argparse
from robot_status.client_utils import RobotStatusClient


class UROperate:
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, operation_name="test_operation"):

        # =========================== Configurable Parameters ===========================
        # IP address and port for UR15 robot
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        # Port for RS485 connection of UR15 (fixed)
        self.rs485_port = 54321
        # Operation name for data organization
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
        
        # Crack coordinate system storage
        self.wobj_origin = None
        self.wobj_transformation_matrix = None

        # Reference pose storage
        self.ref_joint_angles = None
        
        # Estimated keypoints storage
        self.estimated_keypoints = None
        
        # Task position information storage
        self.task_position_offset = None

        # ========================= Initialization =========================
        self._setup_paths()
        self._initialize_robot()
        self._init_rs485_socket()
        self._initialize_robot_status_client()
        
        # Automatically load neccessary parameters
        self._load_camera_params_from_service()
        self._load_3d_positioning_result_from_servive()
        self._load_local_coordinate_system_from_service()
        self._load_wobj_coordinate_system_from_service()
        self._load_ref_joint_angles_from_json()

        self._load_task_position_information()
    
    def _setup_paths(self):
        """Setup directory paths for script, dataset, data and results"""
        # Get the script directory for relative paths
        self.script_dir = os.path.dirname(os.path.abspath(__file__))
        
        # Dataset directory path
        self.dataset_dir = os.path.join(self.script_dir, '..', 'dataset', self.operation_name)
        
        # Data directory path (for storing collected data)
        self.data_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_crack_data')
        
        # Result directory path (for storing results)
        self.result_dir = os.path.join(self.script_dir, '..', 'temp', 'ur_locate_crack_result')
    
    def _load_camera_params_from_service(self):
        """
        Load camera parameters (intrinsic and extrinsic) from robot status service
        """
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
    
    def _load_3d_positioning_result_from_servive(self):
        """
        Load estimated keypoints coordinates from robot status service
        """
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
    
    def _load_local_coordinate_system_from_service(self):
        """
        Load local coordinate system from robot status service
        Returns: Dictionary with 'transformation_matrix' and 'origin' if successful, None otherwise
        """
        try:
            # Get wobj_origin
            wobj_origin = self.robot_status_client.get_status(self.operation_name, 'wobj_origin')
            if wobj_origin is None:
                print(f"Failed to get wobj_origin from robot status for operation '{self.operation_name}'")
                return None
            
            # Get wobj_x axis
            wobj_x = self.robot_status_client.get_status(self.operation_name, 'wobj_x')
            if wobj_x is None:
                print(f"Failed to get wobj_x from robot status for operation '{self.operation_name}'")
                return None
            
            # Get wobj_y axis
            wobj_y = self.robot_status_client.get_status(self.operation_name, 'wobj_y')
            if wobj_y is None:
                print(f"Failed to get wobj_y from robot status for operation '{self.operation_name}'")
                return None
            
            # Get wobj_z axis
            wobj_z = self.robot_status_client.get_status(self.operation_name, 'wobj_z')
            if wobj_z is None:
                print(f"Failed to get wobj_z from robot status for operation '{self.operation_name}'")
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

    def _load_wobj_coordinate_system_from_service(self):
        """
        Load the crack local coordinate system from robot status service
        """
        try:
            # Get wobj_origin
            wobj_origin = self.robot_status_client.get_status('rack', 'wobj_origin')
            if wobj_origin is None:
                print(f"Failed to get wobj_origin from robot status for 'rack'")
                return None
            
            # Get wobj_x axis
            wobj_x = self.robot_status_client.get_status('rack', 'wobj_x')
            if wobj_x is None:
                print(f"Failed to get wobj_x from robot status for 'rack'")
                return None
            
            # Get wobj_y axis
            wobj_y = self.robot_status_client.get_status('rack', 'wobj_y')
            if wobj_y is None:
                print(f"Failed to get wobj_y from robot status for 'rack'")
                return None
            
            # Get wobj_z axis
            wobj_z = self.robot_status_client.get_status('rack', 'wobj_z')
            if wobj_z is None:
                print(f"Failed to get wobj_z from robot status for 'rack'")
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


if __name__ == "__main__":
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
    
