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
        config_params = self._load_robot_parameters_from_config()
        
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
    def _load_robot_parameters_from_config(self):
        """Load robot parameters from config file"""
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
                        if 'robot' in config:
                            robot_config = config['robot']
                            default_params['robot_ip'] = robot_config.get('ip', default_params['robot_ip'])
                            default_params['robot_port'] = robot_config.get('port', default_params['robot_port'])
        except Exception as e:
            print(f"Warning: Could not load config file, using defaults: {e}")
        
        return default_params
    
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
            [194.04, -93.88, 76.55, -71.93, -88.84, 180.68]   # Waypoint 7 (final tools position)
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
                
                print(f"✓ Reached waypoint {i}")
                
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
            print("✗ Robot not connected, cannot execute movej_from_tool_to_task")
            return False
        
        # Define the 7 waypoints in degrees (reverse order from tools to task)
        waypoints_degrees = [
            [194.04, -93.88, 76.55, -71.93, -88.84, 180.68],  # Waypoint 1 (from tools position)
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
        
        print("Starting movej_from_tool_to_task trajectory...")
        
        try:
            for i, waypoint in enumerate(waypoints_radians, 1):
                print(f"Moving to waypoint {i}/7...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.5)
            
            print("✓ Successfully completed movej_from_tool_to_task trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_from_tool_to_task: {e}")
            return False
    
    def movej_from_tool_to_get_tool1(self):
        """
        Move robot using joint movements to get tool1 from tool storage area
        Uses 4 predefined joint positions with movej for precise tool pickup
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_from_tool_to_get_tool1")
            return False
        
        # Define the 4 waypoints in degrees for tool1 pickup sequence
        waypoints_degrees = [
            [210.91, -81.94, 110.19, -118.80, -89.19, 331.43],  # Waypoint 1 (approach tool1)
            [210.91, -77.77, 115.46, -128.24, -89.17, 331.45],  # Waypoint 2 (align with tool1)
            [219.33, -70.01, 104.34, -124.95, -89.18, 339.90],  # Waypoint 3 (engage tool1)
            [219.37, -78.10, 83.67, -96.18, -89.25, 339.80]     # Waypoint 4 (secure tool1)
        ]
        
        # Convert degrees to radians
        waypoints_radians = []
        for waypoint in waypoints_degrees:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_radians.append(waypoint_rad)
        
        print("Starting movej_from_tool_to_get_tool1 trajectory...")
        
        try:
            # Execute first two waypoints
            print("Executing first phase: approach and align...")
            for i, waypoint in enumerate(waypoints_radians[:2], 1):
                print(f"Moving to waypoint {i}/2 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
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
            for i, waypoint in enumerate(waypoints_radians[2:], 3):
                print(f"Moving to waypoint {i}/4 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movej_from_tool_to_get_tool1 trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_from_tool_to_get_tool1: {e}")
            return False
    
    def movej_from_tool_to_return_tool1(self):
        """
        Move robot using joint movements to return tool1 to tool storage area
        Uses 4 predefined positions in reverse order with movel for precise tool placement
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_from_tool_to_return_tool1")
            return False
        
        # Define the 4 waypoints in reverse order for tool1 return sequence
        waypoints_raw = [
            [219.37, -78.10, 83.67, -96.18, -89.25, 339.80],     # Waypoint 1 (from elevated position)
            [219.33, -70.01, 104.34, -124.95, -89.18, 339.90],  # Waypoint 2 (approach insertion)
            [210.91, -77.77, 115.46, -128.24, -89.17, 331.45],  # Waypoint 3 (insertion position)
            [210.91, -81.94, 110.19, -118.80, -89.19, 331.43]   # Waypoint 4 (final storage position)
        ]
        
        # Convert units: xyz from mm to m, rotation from degrees to radians
        waypoints_converted = []
        for waypoint in waypoints_raw:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_converted.append(waypoint_rad)
        
        print("Starting movej_from_tool_to_return_tool1 trajectory...")
        
        try:
            # Execute first two waypoints (return approach)
            print("Executing first phase: return approach...")
            for i, waypoint in enumerate(waypoints_converted[:2], 1):
                print(f"Moving to waypoint {i}/2 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute RS485 unlock command between phases
            unlock_result = self.rs485_unlock()
            if not unlock_result:
                print("✗ Failed to execute RS485 unlock command")
                return False
            
            # Execute last two waypoints (tool release and final position)
            print("Executing second phase: tool release and final positioning...")
            for i, waypoint in enumerate(waypoints_converted[2:], 3):
                print(f"Moving to waypoint {i}/4 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movej_from_tool_to_return_tool1 trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_from_tool_to_return_tool1: {e}")
            return False
    
    def movej_from_tool_to_get_tool2(self):
        """
        Move robot using joint movements to get tool2 from tool storage area
        Uses 4 predefined positions with movel for precise tool pickup
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_from_tool_to_get_tool2")
            return False
        
        # Define the 4 waypoints with xyz in mm and rotation in degrees for tool2 pickup sequence
        waypoints_raw = [
            [203.56, -74.67, 100.07, -115.64, -89.93, 54.85],  # Waypoint 1 (approach tool2)
            [203.53, -70.65, 105.63, -125.27, -89.90, 54.88],  # Waypoint 2 (align with tool2)
            [200.52, -63.71, 94.21, -119.93, -89.81, 55.05],   # Waypoint 3 (engage tool2)
            [200.52, -70.53, 67.11, -86.01, -89.88, 54.89]     # Waypoint 4 (secure tool2)
        ]
        
        # Convert units: xyz from mm to m, rotation from degrees to radians
        waypoints_converted = []
        for waypoint in waypoints_raw:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_converted.append(waypoint_rad)
        
        print("Starting movej_from_tool_to_get_tool2 trajectory...")
        
        try:
            # Execute first two waypoints
            print("Executing first phase: approach and align...")
            for i, waypoint in enumerate(waypoints_converted[:2], 1):
                print(f"Moving to waypoint {i}/2 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
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
            for i, waypoint in enumerate(waypoints_converted[2:], 3):
                print(f"Moving to waypoint {i}/4 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movej_from_tool_to_get_tool2 trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_from_tool_to_get_tool2: {e}")
            return False
    
    def movej_from_tool_to_return_tool2(self):
        """
        Move robot using joint movements to return tool2 to tool storage area
        Uses 4 predefined positions in reverse order with movel for precise tool placement
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_from_tool_to_return_tool2")
            return False
        
        # Define the 4 waypoints in reverse order for tool2 return sequence
        waypoints_raw = [
            [200.52, -70.53, 67.11, -86.01, -89.88, 54.89],     # Waypoint 1 (from elevated position)
            [200.52, -63.71, 94.21, -119.93, -89.81, 55.05],   # Waypoint 2 (approach insertion)
            [203.53, -70.65, 105.63, -125.27, -89.90, 54.88],  # Waypoint 3 (insertion position)
            [203.56, -74.67, 100.07, -115.64, -89.93, 54.85]   # Waypoint 4 (final storage position)
        ]
        
        # Convert units: xyz from mm to m, rotation from degrees to radians
        waypoints_converted = []
        for waypoint in waypoints_raw:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_converted.append(waypoint_rad)
        
        print("Starting movej_from_tool_to_return_tool2 trajectory...")
        
        try:
            # Execute first two waypoints (return approach)
            print("Executing first phase: return approach...")
            for i, waypoint in enumerate(waypoints_converted[:2], 1):
                print(f"Moving to waypoint {i}/2 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute RS485 unlock command between phases
            unlock_result = self.rs485_unlock()
            if not unlock_result:
                print("✗ Failed to execute RS485 unlock command")
                return False
            
            # Execute last two waypoints (tool release and final position)
            print("Executing second phase: tool release and final positioning...")
            for i, waypoint in enumerate(waypoints_converted[2:], 3):
                print(f"Moving to waypoint {i}/4 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movej_from_tool_to_return_tool2 trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_from_tool_to_return_tool2: {e}")
            return False
    
    def movej_from_tool_to_get_tool3(self):
        """
        Move robot using joint movements to get tool3 from tool storage area
        Uses 4 predefined positions with movel for precise tool pickup
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_from_tool_to_get_tool3")
            return False
        
        # Define the 4 waypoints with xyz in mm and rotation in degrees for tool3 pickup sequence
        waypoints_raw = [
            [197.17, -79.71, 103.71, -114.08, -90.04, 51.10],  # Waypoint 1 (approach tool3)
            [197.14, -74.58, 111.04, -126.54, -90.01, 51.14],  # Waypoint 2 (align with tool3)
            [194.58, -65.79, 97.71, -122.01, -89.92, 48.58],   # Waypoint 3 (engage tool3)
            [194.60, -73.07, 77.41, -94.44, -89.98, 48.46]     # Waypoint 4 (secure tool3)
        ]
        
        # Convert units: xyz from mm to m, rotation from degrees to radians
        waypoints_converted = []
        for waypoint in waypoints_raw:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_converted.append(waypoint_rad)
        
        print("Starting movej_from_tool_to_get_tool3 trajectory...")
        
        try:
            # Execute first two waypoints
            print("Executing first phase: approach and align...")
            for i, waypoint in enumerate(waypoints_converted[:2], 1):
                print(f"Moving to waypoint {i}/2 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
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
            for i, waypoint in enumerate(waypoints_converted[2:], 3):
                print(f"Moving to waypoint {i}/4 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movej_from_tool_to_get_tool3 trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_from_tool_to_get_tool3: {e}")
            return False
    
    def movej_from_tool_to_return_tool3(self):
        """
        Move robot using joint movements to return tool3 to tool storage area
        Uses 4 predefined positions in reverse order with movel for precise tool placement
        """
        if not self.robot:
            print("✗ Robot not connected, cannot execute movej_from_tool_to_return_tool3")
            return False
        
        # Define the 4 waypoints in reverse order for tool3 return sequence
        waypoints_raw = [
            [194.60, -73.07, 77.41, -94.44, -89.98, 48.46],     # Waypoint 1 (from elevated position)
            [194.58, -65.79, 97.71, -122.01, -89.92, 48.58],   # Waypoint 2 (approach insertion)
            [197.14, -74.58, 111.04, -126.54, -90.01, 51.14],  # Waypoint 3 (insertion position)
            [197.17, -79.71, 103.71, -114.08, -90.04, 51.10]   # Waypoint 4 (final storage position)
        ]
        
        # Convert units: xyz from mm to m, rotation from degrees to radians
        waypoints_converted = []
        for waypoint in waypoints_raw:
            waypoint_rad = [np.deg2rad(angle) for angle in waypoint]
            waypoints_converted.append(waypoint_rad)
        
        print("Starting movej_from_tool_to_return_tool3 trajectory...")
        
        try:
            # Execute first two waypoints (return approach)
            print("Executing first phase: return approach...")
            for i, waypoint in enumerate(waypoints_converted[:2], 1):
                print(f"Moving to waypoint {i}/2 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Completed first phase")
            
            # Execute RS485 unlock command between phases
            unlock_result = self.rs485_unlock()
            if not unlock_result:
                print("✗ Failed to execute RS485 unlock command")
                return False
            
            # Execute last two waypoints (tool release and final position)
            print("Executing second phase: tool release and final positioning...")
            for i, waypoint in enumerate(waypoints_converted[2:], 3):
                print(f"Moving to waypoint {i}/4 using movej...")
                
                # Execute joint movement
                result = self.robot.movej(waypoint, a=self.a_movej, v=self.v_movej)
                
                if result != 0:
                    print(f"✗ Failed to move to waypoint {i}, error code: {result}")
                    return False
                
                print(f"✓ Reached waypoint {i}")
                
                # Add small delay between movements for stability
                time.sleep(0.8)
            
            print("✓ Successfully completed movej_from_tool_to_return_tool3 trajectory")
            return True
            
        except Exception as e:
            print(f"✗ Exception during movej_from_tool_to_return_tool3: {e}")
            return False
    
    def get_tool_from_task(self, tool_name):
        """
        Complete workflow to get a tool from task position
        Executes: task->tools->get_tool->task
        
        Args:
            tool_name (str): Name of the tool ('tool1', 'tool2', or 'tool3')
        """
        print(f"Starting get_tool_from_task workflow for {tool_name}...")
        
        # Validate tool name
        valid_tools = ['tool1', 'tool2', 'tool3']
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
            get_method_name = f"movej_from_tool_to_get_{tool_name}"
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
            
            print(f"✓ Successfully completed get_tool_from_task workflow for {tool_name}")
            return True
            
        except Exception as e:
            print(f"✗ Exception during get_tool_from_task: {e}")
            return False
    
    def return_tool_from_task(self, tool_name):
        """
        Complete workflow to return a tool from task position
        Executes: task->tools->return_tool->task
        
        Args:
            tool_name (str): Name of the tool ('tool1', 'tool2', or 'tool3')
        """
        print(f"Starting return_tool_from_task workflow for {tool_name}...")
        
        # Validate tool name
        valid_tools = ['tool1', 'tool2', 'tool3']
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
            return_method_name = f"movej_from_tool_to_return_{tool_name}"
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
            
            print(f"✓ Successfully completed return_tool_from_task workflow for {tool_name}")
            return True
            
        except Exception as e:
            print(f"✗ Exception during return_tool_from_task: {e}")
            return False

if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='UR15 Robot Tool Operation')
    parser.add_argument('--robot_ip', type=str, default=None, help='Robot IP address')
    parser.add_argument('--robot_port', type=int, default=None, help='Robot port number')
    
    args = parser.parse_args()
    
    # Initialize UROperateTools
    ur_tools = UROperateTools(robot_ip=args.robot_ip, robot_port=args.robot_port)
    ur_tools.get_tool_from_task('tool1')

    print("UROperateTools initialized successfully")
    print("Ready for tool operations...")