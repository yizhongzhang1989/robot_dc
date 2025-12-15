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
        # Load config parameters if not provided
        config_params = self._load_robot_parameters_from_config()
        
        # =========================== Configurable Parameters ===========================
        self.robot_ip = robot_ip if robot_ip is not None else config_params['robot_ip']
        self.robot_port = robot_port if robot_port is not None else config_params['robot_port']
        self.rs485_port = 54321

        # ========================= Instance variables =========================
        self.robot = None
        self.rs485_socket = None
        self.robot_status_client = None
        
        # ========================= Other variables =========================     
        # Tool coordinate system storage
        self.tool_transformation_matrix = None
        self.tool_offset = None

        # ========================= Initialization =========================
        self._setup_paths()
        self._initialize_robot()
        self._init_rs485_socket()
        self._initialize_robot_status_client()
    
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