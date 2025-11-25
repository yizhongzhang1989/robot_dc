import os
import json
import numpy as np
import requests
from ur15_robot_arm.ur15 import UR15Robot
import time
import socket


class URExecuteBase:
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, rs485_port=54321):

        # =========================== Configurable Parameters ===========================
        # IP address and port for UR15 robot
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        
        # URL for Lift platform web service
        self.lift_platform_web_url = "http://192.168.1.3:8090"
        
        # Port for RS485 connection of UR15
        self.rs485_port = rs485_port

        # Define the path to camera parameters
        self.camera_params_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur15_cam_calibration_result",
            "ur15_camera_parameters"
        )
        
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
        Load camera intrinsic parameters from ur15_cam_calibration_result.json
        """
        intrinsic_file = os.path.join(
            self.camera_params_dir, 
            "ur15_cam_calibration_result.json"
        )
        
        try:
            with open(intrinsic_file, 'r') as f:
                data = json.load(f)
            
            if not data.get('success', False):
                print(f"Failed to load intrinsic parameters: calibration was not successful")
                return False
            
            self.camera_matrix = np.array(data['camera_matrix'])
            self.distortion_coefficients = np.array(data['distortion_coefficients'])
            
            print(f"Camera intrinsic parameters loaded successfully")
            print(f"RMS Error: {data.get('rms_error', None)}")
            return True
            
        except FileNotFoundError:
            print(f"Intrinsic parameter file not found: {intrinsic_file}")
            return False
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return False
        except Exception as e:
            print(f"Error loading intrinsic parameters: {e}")
            return False
    
    def _load_camera_extrinsic(self):
        """
        Load camera extrinsic parameters from ur15_cam_eye_in_hand_result.json
        """
        extrinsic_file = os.path.join(
            self.camera_params_dir, 
            "ur15_cam_eye_in_hand_result.json"
        )
        
        try:
            with open(extrinsic_file, 'r') as f:
                data = json.load(f)
            
            if not data.get('success', False):
                print(f"Failed to load extrinsic parameters: calibration was not successful")
                return False
            
            self.cam2end_matrix = np.array(data['cam2end_matrix'])
            
            print(f"Camera extrinsic parameters loaded successfully")
            print(f"RMS Error: {data.get('rms_error', None)}")
            return True
            
        except FileNotFoundError:
            print(f"Extrinsic parameter file not found: {extrinsic_file}")
            return False
        except json.JSONDecodeError as e:
            print(f"Error decoding JSON file: {e}")
            return False
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
    
# ============================= Robot Movement and Control Methods =============================
    def movej_to_zero_state(self):
        """
        Move robot to zero state position
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        zero_position = [0, -1.57, 0, -1.57, 0, 0]
        print("Moving robot to zero state position...")
        
        res = self.robot.movej(zero_position, a=0.5, v=0.5)
        time.sleep(0.5)
        
        if res == 0:
            print("Robot moved to zero state successfully")
        else:
            print(f"Failed to move robot to zero state (error code: {res})")
        
        return res
    
    def movej_to_reference_joint_positions(self):
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
        
        res = self.robot.movej(self.ref_joint_angles, a=0.5, v=0.3)
        
        if res == 0:
            print("Robot moved to reference position successfully")
        else:
            print(f"Failed to move robot to reference position (error code: {res})")
        
        return res
    
    def get_target_position(self):
        """
        Calculate target position in base coordinate system from offset in local coordinate system
        Uses the local-to-base transformation matrix and the task position offset
        Returns: numpy array [x, y, z] in base coordinate system if successful, None otherwise
        """
        if self.local_transformation_matrix is None:
            print("Local coordinate system transformation matrix not loaded")
            return None
        
        if self.task_position_offset is None:
            print("Task position offset not loaded")
            return None
        
        # Create homogeneous coordinates for the offset point in local coordinate system
        offset_local = np.array([
            self.task_position_offset['x'],
            self.task_position_offset['y'],
            self.task_position_offset['z'],
            1.0  # homogeneous coordinate
        ])
        
        # Transform to base coordinate system
        # local_transformation_matrix is the local-to-base transformation (4x4)
        position_base_homogeneous = np.dot(self.local_transformation_matrix, offset_local)
        
        # Extract x, y, z from homogeneous coordinates
        position_base = position_base_homogeneous[:3]
        
        print(f"Target position calculation:")
        print(f"  Offset in local coordinate system: ({self.task_position_offset['x']:.6f}, "
              f"{self.task_position_offset['y']:.6f}, {self.task_position_offset['z']:.6f})")
        print(f"  Position in base coordinate system: ({position_base[0]:.6f}, "
              f"{position_base[1]:.6f}, {position_base[2]:.6f})")
        
        return position_base

    def movel_in_crack_frame(self, offset):
        """
        Move robot based on "offset" (dx, dy, dz) in crack local coordinate system.   
        Args:
            offset: List/tuple [dx, dy, dz] in crack local coordinate system (meters)
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if not hasattr(self, 'crack_coord_transformation') or self.crack_coord_transformation is None:
            print("Crack local coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Parse offset parameter - must be a 3D array
        if isinstance(offset, (list, tuple)) and len(offset) == 3:
            crack_local_displacement = np.array(offset)
        else:
            print("Invalid offset parameter. Must be [dx, dy, dz] in crack local coordinate system")
            return -1
        
        # Extract rotation matrix from crack local coordinate system transformation matrix (3x3)
        crack_local_rotation = self.crack_coord_transformation[:3, :3]
        
        # Transform displacement from crack local coordinate system to base coordinate system
        # displacement_base = R_crack_local_to_base * displacement_crack_local
        base_displacement = crack_local_rotation @ crack_local_displacement
        
        # Calculate target pose in base coordinate system
        target_pose = [
            current_pose[0] + base_displacement[0],  # x
            current_pose[1] + base_displacement[1],  # y
            current_pose[2] + base_displacement[2],  # z
            current_pose[3],                         # rx (keep current orientation)
            current_pose[4],                         # ry
            current_pose[5]                          # rz
        ]
        
        print(f"\n[INFO] Moving robot: {crack_local_displacement} in crack local frame...")
        print(f"Crack local displacement (crack local frame): {crack_local_displacement}")
        print(f"Base displacement (base frame): {base_displacement}")
        
        res = self.robot.movel(target_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res == 0:
            print("[INFO] Successfully moved robot")
        else:
            print(f"[ERROR] Failed to move robot (error code: {res})")
        
        return res

# ============================ Quick Changer Control Methods =============================
    def ur_lock_quick_changer(self):
        """
        Lock the quick changer  via RS485 command
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
    
    def ur_unlock_quick_changer(self):
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





# ═══════════════════════════════════════════════════════════════════════
# CourierRobot - Lift Platform and Pushrod Control via HTTP API
# ═══════════════════════════════════════════════════════════════════════
class CourierRobot:
    """
    Courier robot controller for lift platform and pushrod via HTTP API
    Provides all control functions available in the web interface
    """
    
    def __init__(self, base_url="http://192.168.1.3:8090"):
        """
        Initialize courier robot controller
        
        Args:
            base_url: HTTP server base URL (default: http://192.168.1.3:8090)
        """
        self.base_url = base_url
        print(f"📡 CourierRobot initialized with base URL: {base_url}")
    
    def _send_command(self, target, command, **kwargs):
        """
        Internal method to send HTTP command to lift platform/pushrod
        
        Args:
            target: 'platform' or 'pushrod'
            command: command name
            **kwargs: additional parameters
        
        Returns:
            dict with 'success' boolean and optional data
        """
        try:
            url = f"{self.base_url}/api/cmd"
            payload = {'command': command, 'target': target, **kwargs}
            
            response = requests.post(url, json=payload, timeout=10)
            
            if response.status_code == 200:
                return {"success": True, "data": response.json()}
            else:
                return {
                    "success": False,
                    "error": f"HTTP {response.status_code}: {response.text}"
                }
        except requests.exceptions.Timeout:
            return {"success": False, "error": "Request timeout"}
        except requests.exceptions.ConnectionError:
            return {"success": False, "error": "Connection failed"}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def get_status(self):
        """Get current status of platform and pushrod"""
        try:
            url = f"{self.base_url}/api/status"
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                return {"success": True, "data": response.json()}
            return {"success": False, "error": f"HTTP {response.status_code}"}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    def get_sensor_data(self):
        """Get latest sensor data (height and force)"""
        try:
            url = f"{self.base_url}/api/latest"
            response = requests.get(url, timeout=2)
            if response.status_code == 200:
                return {"success": True, "data": response.json()}
            return {"success": False, "error": f"HTTP {response.status_code}"}
        except Exception as e:
            return {"success": False, "error": str(e)}
    
    # ==================== Platform Manual Control ====================
    def platform_up(self):
        """Platform manual up movement"""
        print("⬆️  [Platform] Manual UP")
        return self._send_command('platform', 'up')
    
    def platform_down(self):
        """Platform manual down movement"""
        print("⬇️  [Platform] Manual DOWN")
        return self._send_command('platform', 'down')
    
    def platform_stop(self):
        """Platform stop"""
        print("⏹️  [Platform] STOP")
        return self._send_command('platform', 'stop')
    
    # ==================== Platform Height Control ====================
    def platform_goto_height(self, target_height):
        """
        Platform goto specific height
        
        Args:
            target_height: Target height in mm
        """
        print(f"🎯 [Platform] Goto height: {target_height}mm")
        return self._send_command('platform', 'goto_height', target_height=target_height)
    
    # ==================== Platform Force Control ====================
    def platform_force_up(self, target_force):
        """
        Platform force-controlled up movement
        
        Args:
            target_force: Target force in Newtons
        """
        print(f"⚡⬆️  [Platform] Force UP to {target_force}N")
        return self._send_command('platform', 'force_up', target_force=target_force)
    
    def platform_force_down(self, target_force):
        """
        Platform force-controlled down movement
        
        Args:
            target_force: Target force in Newtons
        """
        print(f"⚡⬇️  [Platform] Force DOWN to {target_force}N")
        return self._send_command('platform', 'force_down', target_force=target_force)
    
    # ==================== Platform Hybrid Control ====================
    def platform_hybrid_control(self, target_height, target_force):
        """
        Platform hybrid control (height OR force, whichever reached first)
        
        Args:
            target_height: Target height in mm
            target_force: Target force in Newtons
        """
        print(f"🎯⚡ [Platform] Hybrid: {target_height}mm OR {target_force}N")
        return self._send_command('platform', 'hybrid_control', 
                                 target_height=target_height, 
                                 target_force=target_force)
    
    # ==================== Pushrod Manual Control ====================
    def pushrod_up(self):
        """Pushrod manual up movement"""
        print("⬆️  [Pushrod] Manual UP")
        return self._send_command('pushrod', 'up')
    
    def pushrod_down(self):
        """Pushrod manual down movement"""
        print("⬇️  [Pushrod] Manual DOWN")
        return self._send_command('pushrod', 'down')
    
    def pushrod_stop(self):
        """Pushrod stop"""
        print("⏹️  [Pushrod] STOP")
        return self._send_command('pushrod', 'stop')
    
    # ==================== Pushrod Height Control ====================
    def pushrod_goto_height(self, target_height, mode='absolute'):
        """
        Pushrod goto specific height
        
        Args:
            target_height: Target height in mm (absolute) or offset in mm (relative)
            mode: 'absolute' or 'relative'
        """
        print(f"🎯 [Pushrod] Goto height: {target_height}mm (mode: {mode})")
        return self._send_command('pushrod', 'goto_height', 
                                 target_height=target_height, 
                                 mode=mode)
    
    # ==================== Emergency Reset ====================
    def emergency_reset(self):
        """
        Emergency reset - stops all movements and clears all states
        """
        print("🚨 EMERGENCY RESET")
        return self._send_command('platform', 'reset')


if __name__ == "__main__":
    # Example usage
    ur_base = URExecuteBase()
    
    # All parameters are automatically loaded during initialization
    # Access them directly from instance variables
    
    if ur_base.camera_matrix is not None:
        print("\nCamera Matrix:")
        print(ur_base.camera_matrix)
        print("\nDistortion Coefficients:")
        print(ur_base.distortion_coefficients)
    
    if ur_base.cam2end_matrix is not None:
        print("\nCamera to End-effector Matrix:")
        print(ur_base.cam2end_matrix)
    
    if ur_base.estimated_keypoints is not None:
        print("\nEstimated Keypoints:")
        for kp in ur_base.estimated_keypoints:
            print(f"Keypoint {kp['keypoint_index']}: ({kp['x']:.6f}, {kp['y']:.6f}, {kp['z']:.6f})")
    
    if ur_base.local_transformation_matrix is not None:
        print("\nLocal Coordinate System Transformation Matrix:")
        print(ur_base.local_transformation_matrix)
    
    if ur_base.ref_joint_angles is not None:
        print("\nReference Joint Angles (radians):")
        for i, angle in enumerate(ur_base.ref_joint_angles):
            print(f"Joint {i}: {angle:.6f}")
    
    if ur_base.task_position_offset is not None:
        print("\nTask Position Offset (in local coordinate system):")
        print(f"x: {ur_base.task_position_offset.get('x', 0):.6f}")
        print(f"y: {ur_base.task_position_offset.get('y', 0):.6f}")
        print(f"z: {ur_base.task_position_offset.get('z', 0):.6f}")
    
    # Calculate target position in base coordinate system
    target_position = ur_base.get_target_position()
    if target_position is not None:
        print("\nTarget Position in Base Coordinate System:")
        print(target_position)