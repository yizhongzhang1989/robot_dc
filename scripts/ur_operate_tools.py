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


class UROperateTools:
    def __init__(self, robot_ip=None, robot_port=None):
        # ========================= Setup paths first =========================
        self._setup_paths()
        
        # Load config parameters if not provided
        config_params = self._load_ur15_config_from_file()
        
        # =========================== Configurable Parameters ===========================
        self.robot_ip = robot_ip if robot_ip is not None else config_params['robot_ip']
        self.robot_port = robot_port if robot_port is not None else config_params['robot_port']
        self.rs485_port = 54321
        
        # Movement parameters
        self.a_movej = 0.5  # Acceleration for joint movements (rad/s²)
        self.v_movej = 1.0  # Velocity for joint movements (rad/s)
        self.a_movel = 0.3  # Acceleration for joint movements (m/s²)
        self.v_movel = 0.2  # Velocity for joint movements (m/s)

        # ========================= Instance variables =========================
        self.robot = None
        self.rs485_socket = None
        self.robot_status_client = None
        
        # ========================= Other variables =========================     
        # Tool coordinate system storage
        self.tool_transformation_matrix = None
        self.tool_offset = None

        # ========================= Initialization =========================
        self._initialize_robot()
        self._init_rs485_socket()
        self._initialize_robot_status_client()
    
    # ================================== Private Helper Methods ==================================
    def _load_ur15_config_from_file(self):
        """Load UR15 robot IP and port from robot_config.yaml file"""
        # Default parameters
        default_params = {
            'robot_ip': '192.168.1.15',
            'robot_port': 30002
        }
        
        try:
            # Use common package to get config path
            from common.workspace_utils import get_config_directory
            config_dir = get_config_directory()
            
            if config_dir:
                config_path = os.path.join(config_dir, 'robot_config.yaml')
                
                if os.path.exists(config_path):
                    import yaml
                    with open(config_path, 'r') as f:
                        config = yaml.safe_load(f)
                        if 'ur15' in config and 'robot' in config['ur15']:
                            ur15_config = config['ur15']['robot']
                            default_params['robot_ip'] = ur15_config.get('ip', default_params['robot_ip'])
                            if 'ports' in ur15_config and 'control' in ur15_config['ports']:
                                default_params['robot_port'] = ur15_config['ports']['control']
        except Exception as e:
            print(f"Warning: Could not load UR15 config from file, using defaults: {e}")
        
        return default_params

    def _load_robot_parameters_from_config(self):
        """Load robot parameters from config file (deprecated, use _load_ur15_config_from_file)"""
        return self._load_ur15_config_from_file()
    
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
    
    def rs485_unlock(self):
        """
        Send unlock command via RS485 communication
        Sends predefined unlock command bytes to the RS485 device
        """
        if not self.rs485_socket:
            print("✗ RS485 socket not connected, cannot execute rs485_unlock")
            return False
        
        try:
            time.sleep(1)  # Small delay before sending command
            unlock_command = [0x53, 0x26, 0x01, 0x01, 0x02, 0x7A, 0xD5]
            print("Sending RS485 unlock command...")
            self.rs485_socket.sendall(bytes(unlock_command))
            time.sleep(2)  # Wait for command to take effect
            print("✓ RS485 unlock command sent successfully")
            return True
            
        except Exception as e:
            print(f"✗ Exception during rs485_unlock: {e}")
            return False
    
    def rs485_lock(self):
        """
        Send lock command via RS485 communication
        Sends predefined lock command bytes to the RS485 device
        """
        if not self.rs485_socket:
            print("✗ RS485 socket not connected, cannot execute rs485_lock")
            return False
        
        try:
            time.sleep(1)  # Small delay before sending command
            lock_command = [0x53, 0x26, 0x01, 0x01, 0x01, 0x3A, 0xD4]
            print("Sending RS485 lock command...")
            self.rs485_socket.sendall(bytes(lock_command))
            time.sleep(2)  # Wait for command to take effect
            print("✓ RS485 lock command sent successfully")
            return True
            
        except Exception as e:
            print(f"✗ Exception during rs485_lock: {e}")
            return False

# ================================== Tool Operation Methods ==================================
    def movej_from_task_to_tools(self):
        """
        Move robot joints through predefined waypoints from task position to tools area
        Uses 7 predefined joint positions for safe trajectory planning
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_from_task_to_tools")
            return False
        
        # Define the 7 waypoints in degrees (convert to radians for robot)
        waypoints_degrees = [
            [85.71, -45.96, 117.38, -63.29, 13.73, 183.24],   # Waypoint 1 (from task position)
            [76.47, -45.96, 140.84, -63.29, 13.73, 183.24],   # Waypoint 2
            [76.47, -122.06, 140.84, -63.29, 13.73, 183.24],  # Waypoint 3
            [76.47, -98.15, 18.49, -63.29, 13.73, 183.24],    # Waypoint 4
            [194.04, -98.15, 18.49, -63.29, 13.73, 183.24],   # Waypoint 5
            [194.04, -93.88, 79.31, -63.29, 13.73, 183.24],   # Waypoint 6
            [194.04, -93.88, 76.55, -71.93, -88.84, 0]   # Waypoint 7 (final tools position)
        ]
        
        # Convert degrees to radians
        waypoints_radians = []
        for waypoint in waypoints_degrees:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_radians.append(waypoint_rad)
        
        print("Starting movej_from_task_to_tools trajectory...")
        
        try:
            for i, waypoint in enumerate(waypoints_radians, 1):
                print(f"Moving to waypoint {i}/7...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                # Add small delay between movements for stability
                time.sleep(0.5)
            
            print("✓ Successfully completed movej_from_task_to_tools trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_from_task_to_tools: {e}")
            return False
    
    def movej_from_tool_to_task(self):
        """
        Move robot joints through predefined waypoints from tools area to task position
        Uses 7 predefined joint positions in reverse order for safe trajectory planning
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_task")
            return False
        
        # Define the 7 waypoints in degrees (reverse order from tools to task)
        waypoints_degrees = [
            [194.04, -93.88, 76.55, -71.93, -88.84, 0],  # Waypoint 1 (from tools position)
            [194.04, -93.88, 79.31, -63.29, 13.73, 183.24],   # Waypoint 2
            [194.04, -98.15, 18.49, -63.29, 13.73, 183.24],   # Waypoint 3
            [76.47, -98.15, 18.49, -63.29, 13.73, 183.24],    # Waypoint 4
            [76.47, -122.06, 140.84, -63.29, 13.73, 183.24],  # Waypoint 5
            [76.47, -45.96, 140.84, -63.29, 13.73, 183.24],   # Waypoint 6
            [85.71, -45.96, 117.38, -63.29, 13.73, 183.24]    # Waypoint 7 (final task position)
        ]
        
        # Convert degrees to radians
        waypoints_radians = []
        for waypoint in waypoints_degrees:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_radians.append(waypoint_rad)
        
        print("Starting movel_from_tool_to_task trajectory...")
        
        try:
            for i, waypoint in enumerate(waypoints_radians, 1):
                print(f"Moving to waypoint {i}/7...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                # Add small delay between movements for stability
                time.sleep(0.5)
            
            print("✓ Successfully completed movel_from_tool_to_task trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_task: {e}")
            return False
    
    def movel_from_tool_to_get_tool_pushpull(self):
        """
        Move robot using linear movements to get tool_pushpull from tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) with movel for precise tool pickup
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_get_tool_pushpull")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians)
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.49623, 0.50446, 0.47265, 3.041, -0.842, 0.001],  # Waypoint 1 (approach tool_pushpull)
            [0.49623, 0.50446, 0.39298, 3.041, -0.842, 0.001],  # Waypoint 2 (align with tool_pushpull)
            [0.49623, 0.63619, 0.39298, 3.041, -0.842, 0.001],  # Waypoint 3 (engage tool_pushpull)
            [0.49623, 0.63619, 0.65947, 3.041, -0.842, 0.001]   # Waypoint 4 (secure tool_pushpull)
        ]
        
        print("Starting movel_from_tool_to_get_tool_pushpull trajectory with movel...")
        
        try:
            # Execute first two waypoints
            print("Executing first phase: approach and align...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in getting tool_pushpull...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute RS485 lock command between phases
            lock_result = self.rs485_lock()
            if not lock_result:
                print("✗ Failed to execute RS485 lock command")
                return False
            
            # Execute last two waypoints
            print("Executing second phase: engage and secure tool...")
            for i, waypoint in enumerate(waypoints_cartesian[2:], 3):
                print(f"Moving to waypoint {i}/4 in getting tool_pushpull...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_get_tool_pushpull trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_get_tool_pushpull: {e}")
            return False
    
    def movel_from_tool_to_return_tool_pushpull(self):
        """
        Move robot using linear movements to return tool_pushpull to tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) in reverse order with movel for precise tool placement
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_return_tool_pushpull")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians) - reverse order of get_tool_pushpull
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.49623, 0.63619, 0.65947, 3.041, -0.842, 0.001],   # Waypoint 1 (from elevated position)
            [0.49623, 0.63619, 0.39298, 3.041, -0.842, 0.001],   # Waypoint 2 (approach release)
            [0.49623, 0.50446, 0.39298, 3.041, -0.842, 0.001],   # Waypoint 3 (release position)
            [0.49623, 0.50446, 0.47265, 3.041, -0.842, 0.001]    # Waypoint 4 (final position)
        ]
        
        print("Starting movel_from_tool_to_return_tool_pushpull trajectory with movel...")
        
        try:
            # Execute first two waypoints (return approach)
            print("Executing first phase: return approach...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in returning tool_pushpull...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute third waypoint (release position)
            print("Executing second phase: moving to release position...")
            waypoint = waypoints_cartesian[2]
            print(f"Moving to waypoint 3/4 in returning tool_pushpull...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 3, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 3")
            time.sleep(0.8)
            
            # Execute RS485 unlock command between step 3 and 4
            unlock_result = self.rs485_unlock()
            if not unlock_result:
                print("✗ Failed to execute RS485 unlock command")
                return False
            
            # Execute final waypoint
            print("Executing final phase: moving to final position...")
            waypoint = waypoints_cartesian[3]
            print(f"Moving to waypoint 4/4 using movel...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 4, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 4")
            time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_return_tool_pushpull trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_return_tool_pushpull: {e}")
            return False
    
    def movel_from_tool_to_get_tool_rotate(self):
        """
        Move robot using linear movements to get tool_rotate from tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) with movel for precise tool pickup
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_get_tool_rotate")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians)
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.64116, 0.47126, 0.47795, 1.532, -2.748, 0.009],  # Waypoint 1 (approach tool_rotate)
            [0.64116, 0.47126, 0.38944, 1.532, -2.748, 0.009],  # Waypoint 2 (align with tool_rotate)
            [0.75134, 0.46889, 0.39128, 1.448, -2.777, 0.005],  # Waypoint 3 (engage tool_rotate)
            [0.75134, 0.46889, 0.71440, 1.448, -2.777, 0.005]   # Waypoint 4 (secure tool_rotate)
        ]
        
        print("Starting movel_from_tool_to_get_tool_rotate trajectory with movel...")
        
        try:
            # Execute first two waypoints
            print("Executing first phase: approach and align...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in getting tool_rotate...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute RS485 lock command between phases
            lock_result = self.rs485_lock()
            if not lock_result:
                print("✗ Failed to execute RS485 lock command")
                return False
            
            # Execute last two waypoints
            print("Executing second phase: engage and secure tool...")
            for i, waypoint in enumerate(waypoints_cartesian[2:], 3):
                print(f"Moving to waypoint {i}/4 in getting tool_rotate...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_get_tool_rotate trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_get_tool_rotate: {e}")
            return False
    
    def movel_from_tool_to_return_tool_rotate(self):
        """
        Move robot using linear movements to return tool_rotate to tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) in reverse order with movel for precise tool placement
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_return_tool_rotate")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians) - reverse order of get_tool_rotate
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.75134, 0.46889, 0.71440, 1.448, -2.777, 0.005],   # Waypoint 1 (from elevated position)
            [0.75134, 0.46889, 0.39128, 1.448, -2.777, 0.005],   # Waypoint 2 (approach release)
            [0.64116, 0.47126, 0.38944, 1.532, -2.748, 0.009],   # Waypoint 3 (release position)
            [0.64116, 0.47126, 0.47795, 1.532, -2.748, 0.009]    # Waypoint 4 (final position)
        ]
        
        print("Starting movel_from_tool_to_return_tool_rotate trajectory with movel...")
        
        try:
            # Execute first two waypoints (return approach)
            print("Executing first phase: return approach...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in returning tool_rotate...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute third waypoint (release position)
            print("Executing second phase: moving to release position...")
            waypoint = waypoints_cartesian[2]
            print(f"Moving to waypoint 3/4 in returning tool_rotate...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 3, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 3")
            time.sleep(0.8)
            
            # Execute RS485 unlock command between step 3 and 4
            unlock_result = self.rs485_unlock()
            if not unlock_result:
                print("✗ Failed to execute RS485 unlock command")
                return False
            
            # Execute final waypoint
            print("Executing final phase: moving to final position...")
            waypoint = waypoints_cartesian[3]
            print(f"Moving to waypoint 4/4 using movel...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 4, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 4")
            time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_return_tool_rotate trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_return_tool_rotate: {e}")
            return False
    
    def movel_from_tool_to_get_tool_extract(self):
        """
        Move robot using linear movements to get tool_extract from tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) with movel for precise tool pickup
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_get_tool_extract")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians)
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.64134, 0.38184, 0.50183, 1.466, -2.779, 0.011],  # Waypoint 1 (approach tool_extract)
            [0.64134, 0.38184, 0.39193, 1.466, -2.779, 0.011],  # Waypoint 2 (align with tool_extract)
            [0.77048, 0.38184, 0.39193, 1.466, -2.779, 0.011],  # Waypoint 3 (engage tool_extract)
            [0.77048, 0.38184, 0.65495, 1.466, -2.779, 0.011]   # Waypoint 4 (secure tool_extract)
        ]
        
        print("Starting movel_from_tool_to_get_tool_extract trajectory with movel...")
        
        try:
            # Execute first two waypoints
            print("Executing first phase: approach and align...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in getting tool_extract...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute RS485 lock command between phases
            lock_result = self.rs485_lock()
            if not lock_result:
                print("✗ Failed to execute RS485 lock command")
                return False
            
            # Execute last two waypoints
            print("Executing second phase: engage and secure tool...")
            for i, waypoint in enumerate(waypoints_cartesian[2:], 3):
                print(f"Moving to waypoint {i}/4 in getting tool_extract...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_get_tool_extract trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_get_tool_extract: {e}")
            return False
    
    def movel_from_tool_to_return_tool_extract(self):
        """
        Move robot using linear movements to return tool_extract to tool storage area
        Uses 4 predefined Cartesian positions (xyz + rotation vector) in reverse order with movel for precise tool placement
        Position format: [x, y, z, rx, ry, rz] in meters and radians
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movel_from_tool_to_return_tool_extract")
            return False
        
        # Define the 4 waypoints in Cartesian space (meters + radians) - reverse order of get_tool_extract
        # Format: [x, y, z, rx, ry, rz]
        waypoints_cartesian = [
            [0.77048, 0.38184, 0.65495, 1.466, -2.779, 0.011],   # Waypoint 1 (from elevated position)
            [0.77048, 0.38184, 0.39193, 1.466, -2.779, 0.011],   # Waypoint 2 (approach release)
            [0.64134, 0.38184, 0.39193, 1.466, -2.779, 0.011],   # Waypoint 3 (release position)
            [0.64134, 0.38184, 0.50183, 1.466, -2.779, 0.011]    # Waypoint 4 (final position)
        ]
        
        print("Starting movel_from_tool_to_return_tool_extract trajectory with movel...")
        
        try:
            # Execute first two waypoints (return approach)
            print("Executing first phase: return approach...")
            for i, waypoint in enumerate(waypoints_cartesian[:2], 1):
                print(f"Moving to waypoint {i}/2 in returning tool_extract...")
                
                # Execute linear movement
                result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute third waypoint (release position)
            print("Executing second phase: moving to release position...")
            waypoint = waypoints_cartesian[2]
            print(f"Moving to waypoint 3/4 in returning tool_extract...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 3, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 3")
            time.sleep(0.8)
            
            # Execute RS485 unlock command between step 3 and 4
            unlock_result = self.rs485_unlock()
            if not unlock_result:
                print("✗ Failed to execute RS485 unlock command")
                return False
            
            # Execute final waypoint
            print("Executing final phase: moving to final position...")
            waypoint = waypoints_cartesian[3]
            print(f"Moving to waypoint 4/4 using movel...")
            
            result = self.robot.movel(waypoint, a=self.a_movel, v=self.v_movel)
            
            if result != 0:
                print(f"✗ Failed to move to waypoint 4, error code: {result}")
                return False
            
            print(f"✓ Reached waypoint 4")
            time.sleep(0.8)
            
            print("✓ Successfully completed movel_from_tool_to_return_tool_extract trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movel_from_tool_to_return_tool_extract: {e}")
            return False
    
    def get_tool_from_task_position(self, tool_name):
        """
        Complete workflow to get a tool from task position
        Executes: task->tools->get_tool->task
        
        Args:
            tool_name (str): Name of the tool ('tool_pushpull', 'tool_rotate', or 'tool_extract')
        """
        print(f"Starting get_tool_from_task_position workflow for {tool_name}...")
        
        # Validate tool name
        valid_tools = ['tool_pushpull', 'tool_rotate', 'tool_extract']
        if tool_name not in valid_tools:
            print(f"✗ Invalid tool name '{tool_name}'. Valid tools: {valid_tools}")
            return False
        
        try:
            # Step 1: Move from task position to tools area
            print("Step 1: Moving from task position to tools area...")
            if not self.movej_from_task_to_tools():
                print("✗ Failed to move from task to tools area")
                return False
            
            # Step 2: Get the specified tool
            print(f"Step 2: Getting {tool_name}...")
            get_method_name = f"movel_from_tool_to_get_{tool_name}"
            if hasattr(self, get_method_name):
                get_method = getattr(self, get_method_name)
                if not get_method():
                    print(f"✗ Failed to get {tool_name}")
                    return False
            else:
                print(f"✗ Method {get_method_name} not found")
                return False
            
            # Step 3: Return to task position with tool
            print("Step 3: Returning to task position with tool...")
            if not self.movej_from_tool_to_task():
                print("✗ Failed to return to task position")
                return False
            
            print(f"✓ Successfully completed get_tool_from_task_position workflow for {tool_name}")
            return True
            
        except Exception as e:
            print(f"✗ Exception during get_tool_from_task_position: {e}")
            return False
    
    def return_tool_from_task_position(self, tool_name):
        """
        Complete workflow to return a tool from task position
        Executes: task->tools->return_tool->task
        
        Args:
            tool_name (str): Name of the tool ('tool_pushpull', 'tool_rotate', or 'tool_extract')
        """
        print(f"Starting return_tool_from_task_position workflow for {tool_name}...")
        
        # Validate tool name
        valid_tools = ['tool_pushpull', 'tool_rotate', 'tool_extract']
        if tool_name not in valid_tools:
            print(f"✗ Invalid tool name '{tool_name}'. Valid tools: {valid_tools}")
            return False
        
        try:
            # Step 1: Move from task position to tools area
            print("Step 1: Moving from task position to tools area...")
            if not self.movej_from_task_to_tools():
                print("✗ Failed to move from task to tools area")
                return False
            
            # Step 2: Return the specified tool
            print(f"Step 2: Returning {tool_name}...")
            return_method_name = f"movel_from_tool_to_return_{tool_name}"
            if hasattr(self, return_method_name):
                return_method = getattr(self, return_method_name)
                if not return_method():
                    print(f"✗ Failed to return {tool_name}")
                    return False
            else:
                print(f"✗ Method {return_method_name} not found")
                return False
            
            # Step 3: Return to task position without tool
            print("Step 3: Returning to task position without tool...")
            if not self.movej_from_tool_to_task():
                print("✗ Failed to return to task position")
                return False
            
            print(f"✓ Successfully completed return_tool_from_task_position workflow for {tool_name}")
            return True
            
        except Exception as e:
            print(f"✗ Exception during return_tool_from_task_position: {e}")
            return False

    def return_tool1_get_tool2_from_task(self, tool1_name, tool2_name):
        """
        Complete workflow to return one tool and get another tool from task position
        Executes: task->tools->return_tool1->get_tool2->task
        
        Args:
            tool1_name (str): Name of the tool to return ('tool_pushpull', 'tool_rotate', or 'tool_extract')
            tool2_name (str): Name of the tool to get ('tool_pushpull', 'tool_rotate', or 'tool_extract')
        """
        print(f"Starting return_tool1_get_tool2_from_task workflow: returning {tool1_name}, getting {tool2_name}...")
        
        # Validate tool names
        valid_tools = ['tool_pushpull', 'tool_rotate', 'tool_extract']
        if tool1_name not in valid_tools:
            print(f"✗ Invalid tool1 name '{tool1_name}'. Valid tools: {valid_tools}")
            return False
        if tool2_name not in valid_tools:
            print(f"✗ Invalid tool2 name '{tool2_name}'. Valid tools: {valid_tools}")
            return False
        
        try:
            # Step 1: Move from task position to tools area
            print("Step 1: Moving from task position to tools area...")
            if not self.movej_from_task_to_tools():
                print("✗ Failed to move from task to tools area")
                return False
            
            # Step 2: Return tool1
            print(f"Step 2: Returning {tool1_name}...")
            return_method_name = f"movel_from_tool_to_return_{tool1_name}"
            if hasattr(self, return_method_name):
                return_method = getattr(self, return_method_name)
                if not return_method():
                    print(f"✗ Failed to return {tool1_name}")
                    return False
            else:
                print(f"✗ Method {return_method_name} not found")
                return False
            
            # Step 3: Get tool2
            print(f"Step 3: Getting {tool2_name}...")
            get_method_name = f"movel_from_tool_to_get_{tool2_name}"
            if hasattr(self, get_method_name):
                get_method = getattr(self, get_method_name)
                if not get_method():
                    print(f"✗ Failed to get {tool2_name}")
                    return False
            else:
                print(f"✗ Method {get_method_name} not found")
                return False
            
            # Step 4: Return to task position with tool2
            print("Step 4: Returning to task position with tool2...")
            if not self.movej_from_tool_to_task():
                print("✗ Failed to return to task position")
                return False
            
            print(f"✓ Successfully completed return_tool1_get_tool2_from_task workflow: returned {tool1_name}, got {tool2_name}")
            return True
            
        except Exception as e:
            print(f"✗ Exception during return_tool1_get_tool2_from_task: {e}")
            return False

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='UR15 Robot Tool Operation')
    parser.add_argument('--robot_ip', type=str, default=None, help='Robot IP address')
    parser.add_argument('--robot_port', type=int, default=None, help='Robot port number')
    
    args = parser.parse_args()
    
    # Initialize UROperateTools
    ur_tools = UROperateTools(robot_ip=args.robot_ip, robot_port=args.robot_port)

    print("UROperateTools initialized successfully")
    print("Ready for tool operations...")
    ur_tools.movej_from_tool_to_task()