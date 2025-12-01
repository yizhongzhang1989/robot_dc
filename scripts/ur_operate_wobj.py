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
from generate_server_frame import GenerateServerFrame


class UROperateWobj:
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, server_index=14):

        # =========================== Configurable Parameters ===========================
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.rs485_port = 54321
        self.server_index = server_index

        # ========================= Instance variables =========================
        self.robot = None
        self.rs485_socket = None
        self.robot_status_client = None
        self.server_frame_generator = None
        
        # ========================= Other variables =========================
        # Initialize parameter storage
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.cam2end_matrix = None
        
        # Rack coordinate system storage
        self.rack_transformation_matrix_in_base = None
        self.rack_origin_in_base = None

        # ========================= Initialization =========================
        self._setup_paths()
        self._initialize_robot()
        self._init_rs485_socket()
        self._initialize_robot_status_client()
        self._initialize_server_frame_generator()
        
        # Automatically load neccessary parameters
        self._load_camera_params_from_service()
        self._load_rack2base_from_service()
    
    # ================================== Private Helper Methods ==================================
    def _setup_paths(self):
        """Setup directory paths for script, dataset, data and results"""
        # Get the script directory for relative paths
        try:
            from common.workspace_utils import get_scripts_directory
            self.script_dir = get_scripts_directory() if get_scripts_directory() else os.path.dirname(os.path.abspath(__file__))
        except ImportError:
            self.script_dir = os.path.dirname(os.path.abspath(__file__))
    
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

    def _initialize_server_frame_generator(self):
        """Initialize server frame generator for generating target server frames"""
        try:
            print(f'Initializing server frame generator...')
            self.server_frame_generator = GenerateServerFrame()
            print('✓ Server frame generator initialized successfully')
        except Exception as e:
            print(f'✗ Failed to initialize server frame generator: {e}')
            import traceback
            traceback.print_exc()
            self.server_frame_generator = None

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

    def _load_rack2base_from_service(self):
        """
        Load the rack to base coordinate system from robot status service
        """
        if self.robot_status_client is None:
            print("Robot status client is not initialized, skipping rack2base information loading")
            return None
        
        try:
            # Get rack2base_matrix
            rack2base_matrix = self.robot_status_client.get_status('ur15', 'rack2base_matrix')
            if rack2base_matrix is None:
                print(f"Failed to get rack2base_matrix from robot status")
                return None
            
            # Convert to numpy array
            transformation_matrix = np.array(rack2base_matrix)
            
            # Validate matrix shape (should be 4x4)
            if transformation_matrix.shape != (4, 4):
                print(f"Invalid rack2base_matrix shape: {transformation_matrix.shape}, expected (4, 4)")
                return None
            
            # Extract origin from the transformation matrix (last column, first 3 rows)
            origin = transformation_matrix[:3, 3]
            
            # Store the transformation matrix and origin
            self.rack_transformation_matrix_in_base = transformation_matrix
            self.rack_origin_in_base = origin
            
            print(f"Rack to base coordinate system loaded successfully from robot status")
            print(f"  Origin: x={origin[0]:.6f}, y={origin[1]:.6f}, z={origin[2]:.6f}")
            
            return True
            
        except Exception as e:
            print(f"Error loading rack to base coordinate system: {e}")
            return None

    def _load_operating_unit_id_from_service(self):
        """
        Load the operating unit ID from robot status service and set as server_index
        """
        if self.robot_status_client is None:
            print("Robot status client is not initialized, skipping operating unit ID loading")
            return False
        
        try:
            # Get rack_operating_unit_id
            operating_unit_id = self.robot_status_client.get_status('ur15', 'rack_operating_unit_id')
            if operating_unit_id is None:
                print(f"Failed to get rack_operating_unit_id from robot status")
                return False
            
            # Update server_index
            self.server_index = operating_unit_id
            
            print(f"Operating unit ID loaded successfully from robot status: {operating_unit_id}")
            
            return True
            
        except Exception as e:
            print(f"Error loading operating unit ID: {e}")
            return False

    # ============================= Robot Movement and Control Methods =============================
    def movel_in_rack_frame(self, offset_in_rack=[0, 0, 0]):
        """
        Move robot in rack coordinate system by specified offset.
        
        Args:
            offset_in_rack: List [x, y, z] offset in rack coordinate system (default: [0, 0, 0])
        
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.rack_transformation_matrix_in_base is None:
            print("Rack coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract rack coordinate system rotation
        rack_rotation = self.rack_transformation_matrix_in_base[:3, :3]
        rack_x = rack_rotation[:, 0] / np.linalg.norm(rack_rotation[:, 0])
        rack_y = rack_rotation[:, 1] / np.linalg.norm(rack_rotation[:, 1])
        rack_z = rack_rotation[:, 2] / np.linalg.norm(rack_rotation[:, 2])
        
        # Calculate movement vector in base frame
        offset_array = np.array(offset_in_rack)
        movement_vector = (offset_array[0] * rack_x + 
                          offset_array[1] * rack_y + 
                          offset_array[2] * rack_z)
        
        # Calculate target position
        current_position = np.array(current_pose[:3])
        target_position = current_position + movement_vector
        
        target_pose = [
            target_position[0],  # x
            target_position[1],  # y
            target_position[2],  # z
            current_pose[3],     # rx (keep current orientation)
            current_pose[4],     # ry
            current_pose[5]      # rz
        ]
        
        print(f"\n[Movement in Rack Frame]")
        print(f"  Offset: X={offset_array[0]:.6f}m, Y={offset_array[1]:.6f}m, Z={offset_array[2]:.6f}m")
        
        res = self.robot.movel(target_pose, a=0.1, v=0.1)
        if res != 0:
            print(f"Failed to move in rack frame with error code: {res}")
            return res
        
        time.sleep(0.5)
        print("[INFO] Successfully moved in rack frame")
        return 0

    def movel_to_correct_tcp_pose(self, tcp_x_to_rack=[1, 0, 0], tcp_y_to_rack=[0, 0, -1], tcp_z_to_rack=[0, 1, 0], angle_deg=0):
        """
        Correct tool TCP orientation based on rack coordinate system.
        
        Args:
            tcp_x_to_rack: List [x, y, z] indicating TCP X+ axis alignment to rack axes
                          e.g., [1, 0, 0] means TCP X+ aligns with Rack X+
                                [0, 1, 0] means TCP X+ aligns with Rack Y+
                                [0, 0, -1] means TCP X+ aligns with Rack Z- (default: [1, 0, 0])
            tcp_y_to_rack: List [x, y, z] indicating TCP Y+ axis alignment to rack axes (default: [0, 0, -1])
            tcp_z_to_rack: List [x, y, z] indicating TCP Z+ axis alignment to rack axes (default: [0, 1, 0])
            angle_deg: Additional rotation angle around TCP Z axis in degrees (default: 0)
        
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.rack_transformation_matrix_in_base is None:
            print("Rack coordinate system transformation matrix not loaded")
            return -1
        
        # Convert alignment vectors to numpy arrays
        tcp_x_align = np.array(tcp_x_to_rack, dtype=float)
        tcp_y_align = np.array(tcp_y_to_rack, dtype=float)
        tcp_z_align = np.array(tcp_z_to_rack, dtype=float)
        
        # Validate alignment vectors
        # Check 1: Each vector should have length 1 or sqrt(2) or sqrt(3)
        tolerance = 1e-6
        valid_lengths = [1.0, np.sqrt(2), np.sqrt(3)]
        
        for vec, name in [(tcp_x_align, 'tcp_x_to_rack'), 
                          (tcp_y_align, 'tcp_y_to_rack'), 
                          (tcp_z_align, 'tcp_z_to_rack')]:
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
        # print(f"       TCP X -> Rack: {tcp_x_align}")
        # print(f"       TCP Y -> Rack: {tcp_y_align}")
        # print(f"       TCP Z -> Rack: {tcp_z_align}")
        
        # Get current TCP pose
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract rotation matrix from rack coordinate system transformation matrix (3x3)
        rack_rotation = self.rack_transformation_matrix_in_base[:3, :3]
        
        # Rack coordinate system axes in base coordinates
        rack_x = rack_rotation[:, 0]  # Rack X+ direction
        rack_y = rack_rotation[:, 1]  # Rack Y+ direction
        rack_z = rack_rotation[:, 2]  # Rack Z+ direction
        
        # Construct target rotation matrix for tool based on alignment parameters
        # TCP X+ axis direction in base frame
        tool_x_direction = (tcp_x_align[0] * rack_x + 
                           tcp_x_align[1] * rack_y + 
                           tcp_x_align[2] * rack_z)
        
        # TCP Y+ axis direction in base frame
        tool_y_direction = (tcp_y_align[0] * rack_x + 
                           tcp_y_align[1] * rack_y + 
                           tcp_y_align[2] * rack_z)
        
        # TCP Z+ axis direction in base frame
        tool_z_direction = (tcp_z_align[0] * rack_x + 
                           tcp_z_align[1] * rack_y + 
                           tcp_z_align[2] * rack_z)
        
        target_tool_rotation = np.column_stack([
            tool_x_direction,  # Tool X+ direction
            tool_y_direction,  # Tool Y+ direction
            tool_z_direction   # Tool Z+ direction
        ])
        
        # Convert rotation matrix to rotation vector (axis-angle representation)
        # UR uses rotation vector [rx, ry, rz] where the direction is the axis and the magnitude is the angle in radians
        rotation_obj = R.from_matrix(target_tool_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Step 1: Align with rack coordinate system
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
            print("Aligned tool tcp pose with rack coordinate system successfully")
        
        # Step 2: Rotate around TCP Z axis
        angle_rad = np.deg2rad(angle_deg)
        
        # The Z axis in tool corresponds to the axis specified by tcp_z_to_rack
        # Calculate which base axis corresponds to TCP Z
        rotation_axis_in_base = (tcp_z_align[0] * rack_x + 
                                 tcp_z_align[1] * rack_y + 
                                 tcp_z_align[2] * rack_z)
        rotation_axis = rotation_axis_in_base / np.linalg.norm(rotation_axis_in_base)  # Normalize
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
    
    def movel_to_target_position(self, index=14, execution_order=[1, 3, 2], offset_in_rack=[0, 0, 0]):
        """
        Move robot to target server position using linear movement based on rack coordinate system.
        This method moves in multiple steps according to execution_order to avoid collision.
        
        Args:
            index: Server index (default: 14)
            execution_order: List of integers [1,2,3] representing movement sequence along rack axes.
                            1=X axis, 2=Y axis, 3=Z axis
                            e.g., [1,2,3] means move along X first, then Y, then Z
                            e.g., [1,3,2] means move along X first, then Z, then Y (default)
            offset_in_rack: List [x, y, z] offset in rack coordinate system to apply to target position (default: [0, 0, 0])
                           e.g., [0.1, 0, 0] adds 0.1m along rack X axis
        
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.server_frame_generator is None:
            print("Server frame generator is not initialized")
            return -1
        
        if self.rack_transformation_matrix_in_base is None:
            print("Rack coordinate system transformation matrix not loaded")
            return -1
        
        # Validate execution_order parameter
        if not isinstance(execution_order, list) or len(execution_order) != 3:
            print("[ERROR] execution_order must be a list of 3 integers")
            return -1
        
        if set(execution_order) != {1, 2, 3}:
            print("[ERROR] execution_order must contain exactly [1,2,3] in any order")
            print(f"       Received: {execution_order}")
            return -1
        
        # Step 1: Generate server frame in rack coordinate system
        print(f"\nStep 1: Generating server frame for index {index} in rack coordinate system...")
        server_frame_in_rack = self.server_frame_generator.generate_server_frame_in_rack(index)
        
        if server_frame_in_rack is None:
            print("Failed to generate server frame in rack")
            return -1
        
        # Extract server2rack transformation matrix
        server2rack = server_frame_in_rack['target_server_transformation_matrix_in_rack']
        print(f"✓ Server frame generated successfully")
        
        # Apply offset in rack coordinate system
        offset_array = np.array(offset_in_rack)
        if np.linalg.norm(offset_array) > 0:
            print(f"Applying offset in rack coordinate system: ({offset_array[0]:.6f}, {offset_array[1]:.6f}, {offset_array[2]:.6f})")
            # Add offset to the translation part of server2rack matrix
            server2rack[:3, 3] += offset_array
            print(f"✓ Offset applied to server position in rack frame")
        
        # Step 2: Reload rack2base transformation matrix from service (in case it has changed)
        print(f"\nStep 2: Reloading rack2base transformation matrix from service...")
        self._load_rack2base_from_service()
        
        if self.rack_transformation_matrix_in_base is None:
            print("Failed to reload rack2base transformation matrix")
            return -1
        
        rack2base = self.rack_transformation_matrix_in_base
        print(f"✓ rack2base transformation matrix reloaded successfully")
        
        # Step 3: Calculate server2base = rack2base @ server2rack
        print(f"\nStep 3: Calculating server2base transformation...")
        server2base = rack2base @ server2rack
        
        # Extract target position (xyz) from server2base
        target_position = server2base[:3, 3]
        print(f"Target position in base coordinate system: ({target_position[0]:.6f}, "
              f"{target_position[1]:.6f}, {target_position[2]:.6f})")
        
        # Get current TCP pose
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Use current pose orientation instead of rack orientation
        current_rotation_vector = current_pose[3:6]
        print(f"Using current TCP pose (rotation vector): ({current_rotation_vector[0]:.6f}, "
              f"{current_rotation_vector[1]:.6f}, {current_rotation_vector[2]:.6f})")
        
        # Step 4: Execute movement in steps according to execution_order
        print(f"\nStep 4: Moving robot to target position...")
        
        # Extract rack coordinate system axes from transformation matrix
        rack_rotation = rack2base[:3, :3]
        # Normalize axes to ensure they are unit vectors for accurate projection
        rack_axes = {
            1: rack_rotation[:, 0] / np.linalg.norm(rack_rotation[:, 0]),  # Rack X+ direction (normalized)
            2: rack_rotation[:, 1] / np.linalg.norm(rack_rotation[:, 1]),  # Rack Y+ direction (normalized)
            3: rack_rotation[:, 2] / np.linalg.norm(rack_rotation[:, 2])   # Rack Z+ direction (normalized)
        }
        axis_names = {1: 'X', 2: 'Y', 3: 'Z'}

        # Calculate movement vector from current position to target
        current_position = np.array(current_pose[:3])
        target_position_array = np.array(target_position)
        movement_vector = target_position_array - current_position
        print(f"Movement vector in base coordinates: {movement_vector}")
        
        # Project movement vector onto rack coordinate system axes
        movement_components = {
            1: np.dot(movement_vector, rack_axes[1]),  # X component
            2: np.dot(movement_vector, rack_axes[2]),  # Y component
            3: np.dot(movement_vector, rack_axes[3])   # Z component
        }
        
        print(f"Movement in rack frame: X={movement_components[1]:.6f}, Y={movement_components[2]:.6f}, Z={movement_components[3]:.6f}")
        print(f"Execution order: {[axis_names[i] for i in execution_order]}")

        # Execute movements according to execution_order
        accumulated_position = current_position.copy()
        
        for step_idx, axis_id in enumerate(execution_order, start=1):
            axis_name = axis_names[axis_id]
            axis_vector = rack_axes[axis_id]
            movement_amount = movement_components[axis_id]
            
            # Calculate next position by adding current axis movement
            movement_delta = movement_amount * axis_vector
            accumulated_position = accumulated_position + movement_delta
            
            # Use current pose (rotation vector) for orientation
            next_pose = [
                accumulated_position[0],        # x
                accumulated_position[1],        # y
                accumulated_position[2],        # z
                current_rotation_vector[0],     # rx (use current orientation)
                current_rotation_vector[1],     # ry
                current_rotation_vector[2]      # rz
            ]
            
            print(f"\nStep {step_idx}: Moving along rack {axis_name} direction...")
            print(f"  Movement amount: {movement_amount:.6f}m")
            print(f"  Target pose: {next_pose}")
            
            res = self.robot.movel(next_pose, a=0.1, v=0.1)
            time.sleep(0.5)

            if res != 0:
                print(f"[ERROR] Failed to move along rack {axis_name} (error code: {res})")
                return res
            
            print(f"Step {step_idx} completed successfully")
        
        print("\n✓ Robot moved to target server position successfully")
        return 0


if __name__ == "__main__":
    try:
        # Parse command line arguments
        parser = argparse.ArgumentParser(description='UR Robot Operation with Wobj')
        parser.add_argument('--robot_ip', type=str, default='192.168.1.15',
                            help='Robot IP address (default: 192.168.1.15)')
        parser.add_argument('--robot_port', type=int, default=30002,
                            help='Robot port (default: 30002)')
        parser.add_argument('--server_index', type=int, default=14,
                            help='Server index (default: 14)')
        
        args = parser.parse_args()
        
        # Create UROperateWobj instance
        ur_operate = UROperateWobj(
            robot_ip=args.robot_ip,
            robot_port=args.robot_port,
            server_index=args.server_index
        )
        print("UROperateWobj initialized successfully")
        
        # Step 1: Correct TCP pose
        print("\n" + "="*60)
        print("Step 1: Correcting TCP pose...")
        print("="*60)
        result = ur_operate.movel_to_correct_tcp_pose(
            tcp_x_to_rack=[1, 0, 0],
            tcp_y_to_rack=[0, 0, -1],
            tcp_z_to_rack=[0, 1, 0],
            angle_deg=0
        )
        
        if result != 0:
            print(f"\n✗ Failed to correct TCP pose (error code: {result})")
        else:
            print("\n✓ Successfully corrected TCP pose")
            
            # Step 2: Move to target server position
            print("\n" + "="*60)
            print("Step 2: Moving to target server position...")
            print("="*60)
            result = ur_operate.movel_to_target_position(
                index=args.server_index,
                execution_order=[1, 3, 2],
                offset_in_rack=[0, -0.60, 0]
            )
            
            if result == 0:
                print("\n✓ Successfully moved to target position")
            else:
                print(f"\n✗ Failed to move to target position (error code: {result})")
        
    except KeyboardInterrupt:
        print("\n\nProgram interrupted by user")
    except Exception as e:
        print(f"\n\nError occurred: {e}")
        import traceback
        traceback.print_exc()

