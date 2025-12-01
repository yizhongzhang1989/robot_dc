#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR15 Web Node

This node provides a web interface for UR15 robot camera calibration validation.
It displays real-time camera feed, robot joint positions, TCP pose, and allows:
- Loading intrinsic and extrinsic calibration parameters
- Visualizing base coordinate origin projection on camera image
- Controlling robot freedrive mode
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image, JointState
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import numpy as np
import threading
import socket
import json
import time
import sys
import os
import subprocess
import signal
import atexit
from flask import Flask, render_template_string, Response
from pathlib import Path
from ament_index_python.packages import get_package_share_directory
from common.workspace_utils import get_scripts_directory
from ur15_web import draw_utils
# Use robot_status_redis for better performance in Flask threads (no ROS2 service dependency)
from robot_status_redis import get_from_status, set_to_status

# Add scripts directory to path for camera calibration toolkit
scripts_dir = get_scripts_directory()
if scripts_dir and scripts_dir not in sys.path:
    sys.path.insert(0, scripts_dir)

from generate_server_frame import GenerateServerFrame

# Import camera calibration toolkit
try:
    from ThirdParty.camera_calibration_toolkit.core.calibration_patterns import create_pattern_from_json
    CALIBRATION_TOOLKIT_AVAILABLE = True
except ImportError as e:
    print(f"Warning: Camera calibration toolkit not available: {e}")
    CALIBRATION_TOOLKIT_AVAILABLE = False

# Add ur15_robot_arm to Python path
try:
    # Try to find the package in the workspace
    ur15_pkg_path = os.path.join(
        os.path.dirname(get_package_share_directory('ur15_web')),
        'ur15_robot_arm'
    )
    if os.path.exists(ur15_pkg_path):
        sys.path.insert(0, ur15_pkg_path)
    from ur15_robot_arm.ur15 import UR15Robot
except Exception as e:
    print(f"Warning: Could not import UR15Robot: {e}")
    UR15Robot = None


class UR15WebNode(Node):
    """ROS2 node for UR15 web interface with camera calibration validation."""
    
    def __init__(self):
        super().__init__('ur15_web_node')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/ur15_camera/image_raw')
        self.declare_parameter('web_port', 8030)
        self.declare_parameter('ur15_ip', '192.168.1.15')
        self.declare_parameter('ur15_port', 30002)
        self.declare_parameter('dataset_dir', '/tmp/dataset')
        self.declare_parameter('calib_data_dir', '/tmp/ur15_cam_calibration_data')
        self.declare_parameter('calib_result_dir', '/tmp/ur15_cam_calibration_result')
        self.declare_parameter('chessboard_config', '/tmp/ur15_cam_calibration_data/chessboard_config.json')
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        web_port = self.get_parameter('web_port').value
        self.ur15_ip = self.get_parameter('ur15_ip').value
        self.ur15_port = self.get_parameter('ur15_port').value
        self.dataset_dir = self.get_parameter('dataset_dir').value
        self.calibration_data_dir = self.get_parameter('calib_data_dir').value
        self.calibration_result_dir = self.get_parameter('calib_result_dir').value
        self.chessboard_config = self.get_parameter('chessboard_config').value
        
        # Use only the specified port, clear it if occupied
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                s.bind(('localhost', web_port))
                self.web_port = web_port
                self.get_logger().info(f"Web interface will run on port {web_port}")
        except OSError as e:
            self.get_logger().warning(f"Port {web_port} is occupied, attempting to clear it...")
            success = False
            
            try:
                # First try to find and kill processes using the port
                import subprocess
                result = subprocess.run(['lsof', '-ti', f':{web_port}'], 
                                      capture_output=True, text=True, timeout=5)
                
                if result.returncode == 0 and result.stdout.strip():
                    pids = result.stdout.strip().split('\n')
                    for pid in pids:
                        if pid.isdigit():
                            self.get_logger().info(f"Killing process {pid} using port {web_port}")
                            try:
                                subprocess.run(['kill', '-9', pid], timeout=5, check=True)
                            except subprocess.CalledProcessError:
                                pass  # Process might already be dead
                    
                    # Wait for processes to be killed
                    time.sleep(2)
                
                # Try multiple times with SO_REUSEADDR
                for attempt in range(3):
                    try:
                        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                            s.bind(('localhost', web_port))
                            self.web_port = web_port
                            self.get_logger().info(f"Successfully bound to port {web_port} after {attempt + 1} attempts")
                            success = True
                            break
                    except OSError:
                        if attempt < 2:
                            self.get_logger().info(f"Port {web_port} still occupied, waiting... (attempt {attempt + 1}/3)")
                            time.sleep(2)
                
                if not success:
                    # Final attempt with different bind options
                    try:
                        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                            s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
                            s.bind(('0.0.0.0', web_port))  # Bind to all interfaces as last resort
                            self.web_port = web_port
                            self.get_logger().warning(f"Bound to port {web_port} using SO_REUSEPORT on all interfaces")
                            success = True
                    except OSError:
                        pass
                
                if not success:
                    raise RuntimeError(f"Unable to bind to port {web_port} after multiple attempts. "
                                     f"Port may be in TIME_WAIT state or reserved by system. "
                                     f"Please wait a few minutes or restart the system. Original error: {e}")
                                     
            except subprocess.TimeoutExpired:
                raise RuntimeError(f"Timeout while trying to clear port {web_port}")
            except FileNotFoundError:
                # lsof not available, try with SO_REUSEADDR anyway
                self.get_logger().warning("lsof command not found, trying SO_REUSEADDR")
                try:
                    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                        s.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
                        s.bind(('localhost', web_port))
                        self.web_port = web_port
                        self.get_logger().info(f"Successfully bound to port {web_port} using SO_REUSEADDR")
                except OSError:
                    raise RuntimeError(f"Port {web_port} is occupied and 'lsof' command not available. Please free up the port manually.")
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Current image storage
        self.current_image = None
        self.image_lock = threading.Lock()
        
        # Camera calibration parameters
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.cam2end_matrix = None
        self.target2base_matrix = None
        self.calibration_lock = threading.Lock()
        
        # Generate 3D models for visualization
        self.ur15_base_model = draw_utils.generate_ur15_base_curve(ray_length=3)
        self.gb200rack_model = draw_utils.generate_gb200rack_curve()
        self.gb200server_model = draw_utils.generate_gb200server_wire()
        
        # Robot state data
        self.joint_positions = []
        self.tcp_pose = None
        self.robot_data_lock = threading.Lock()
        self.last_joint_update = None
        self.last_tcp_update = None
        
        # UR15 robot control
        self.ur15_robot = None
        self.freedrive_active = False
        self.validation_active = False
        self.corner_detection_enabled = False
        self.corner_detection_params = {}
        self.draw_rack_enabled = False
        self.draw_keypoints_enabled = False
        self.ur15_lock = threading.Lock()
        self._init_ur15_connection()
        
        # Child process management
        self.child_processes = []
        self.process_lock = threading.Lock()
        self._cleanup_done = False
        
        # Web log message queue
        self.web_log_messages = []
        self.web_log_lock = threading.Lock()
        
        # Create status service client
        from robot_status_redis.client_utils import RobotStatusClient
        # auto_spin=False because this node is already spinning via launch file
        self.status_client = RobotStatusClient(self)
        self.get_logger().info("Status service client created")
        
        # Create Dashboard client for freedrive control (doesn't interrupt External Control)
        from ur_dashboard_msgs.srv import RawRequest
        self.dashboard_client = self.create_client(RawRequest, '/dashboard_client/raw_request')
        self.get_logger().info("Dashboard client created for freedrive control")
        
        # Initialize GenerateServerFrame for server coordinate calculations
        import sys
        scripts_dir = get_scripts_directory()
        if scripts_dir and scripts_dir not in sys.path:
            sys.path.insert(0, scripts_dir)
        try:
            from generate_server_frame import GenerateServerFrame
            self.server_frame_generator = GenerateServerFrame()
            self.get_logger().info("Server frame generator initialized")
        except Exception as e:
            self.get_logger().warning(f"Failed to initialize server frame generator: {e}")
            self.server_frame_generator = None
        
        # Register cleanup handlers
        atexit.register(self.cleanup)
        signal.signal(signal.SIGINT, self._signal_handler)
        signal.signal(signal.SIGTERM, self._signal_handler)
        
        # Create QoS profile for camera subscription
        qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE
        )
        
        # Subscribe to camera topic
        self.camera_subscription = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            qos_profile
        )
        
        self.get_logger().info(f"Subscribed to camera topic: {self.camera_topic}")
        
        # Subscribe to joint states topic
        self.joint_states_subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_callback,
            10
        )
        
        self.get_logger().info("Subscribed to /joint_states topic")
        
        # Subscribe to TCP pose topic
        self.tcp_pose_subscription = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.tcp_pose_callback,
            10
        )
        
        self.get_logger().info("Subscribed to /tcp_pose_broadcaster/pose topic")
        
        # Setup Flask app
        web_dir = os.path.join(get_package_share_directory('ur15_web'), 'web')
        self.app = Flask(__name__, static_folder=web_dir, static_url_path='/static')
        self.flask_running = True
        self.setup_flask_routes()
        
        # Start Flask server in separate thread
        self.flask_thread = threading.Thread(target=self._run_flask_server, daemon=True)
        self.flask_thread.start()
        
        self.get_logger().info(f"Web interface running at http://0.0.0.0:{self.web_port}")
        
        # Create a timer to load calibration parameters after async cache is populated
        self._load_calib_retry_count = 0
        self._load_calib_timer = self.create_timer(0.5, self._load_calibration_from_status)
    
    def _load_calibration_from_status(self):
        """Load calibration parameters from robot_status service."""
        try:
            # Parameters to load
            params_to_load = [
                ('camera_matrix', 'camera_matrix'),
                ('distortion_coefficients', 'distortion_coefficients'),
                ('cam2end_matrix', 'cam2end_matrix'),
                ('target2base_matrix', 'target2base_matrix')
            ]
            
            loaded_count = 0
            pending_count = 0
            for param_name, attr_name in params_to_load:
                try:
                    value = self.status_client.get_status('ur15', param_name, timeout_sec=2.0)
                except Exception as e:
                    self.get_logger().debug(f"Failed to get {param_name}: {e}")
                    value = None
                
                if value is not None:
                    try:
                        with self.calibration_lock:
                            existing_value = getattr(self, attr_name, None)
                            # Only update if not already set
                            if existing_value is None:
                                setattr(self, attr_name, value)
                                loaded_count += 1
                                self.get_logger().info(f"Loaded {param_name} from robot_status (shape: {value.shape})")
                    except Exception as e:
                        self.get_logger().warning(f"Failed to parse {param_name}: {e}")
                else:
                    pending_count += 1
            
            # If we loaded something or nothing is pending after retries, stop the timer
            if loaded_count > 0:
                self.get_logger().info(f"Successfully loaded {loaded_count}/4 calibration parameters from robot_status")
                self.push_web_log(f"Loaded {loaded_count}/4 calibration parameters from robot_status", 'success')
                self._load_calib_timer.cancel()
            elif self._load_calib_retry_count >= 10:  # Stop after 5 seconds (10 * 0.5s)
                if pending_count == len(params_to_load):
                    self.get_logger().info("No calibration parameters found in robot_status")
                self._load_calib_timer.cancel()
            else:
                self._load_calib_retry_count += 1
                
        except Exception as e:
            self.get_logger().warning(f"Error loading calibration from robot_status: {e}")
            self._load_calib_timer.cancel()
    
    def _signal_handler(self, signum, frame):
        """Handle termination signals."""
        self.get_logger().info(f"Received signal {signum}, cleaning up...")
        self.cleanup()
        sys.exit(0)
    
    def __del__(self):
        """Destructor to clean up resources."""
        self.cleanup()
    
    def cleanup(self):
        """Clean up all resources including child processes."""
        # Avoid duplicate cleanup
        if self._cleanup_done:
            return
        self._cleanup_done = True
        
        self.get_logger().info("Cleaning up resources...")
        
        # Stop Flask server
        self.flask_running = False
        if hasattr(self, 'flask_thread') and self.flask_thread and self.flask_thread.is_alive():
            self.get_logger().info("Stopping Flask server...")
            # Flask is in daemon thread, just mark it as stopped
            # The thread will exit when the main process exits
        
        # Terminate all child processes
        with self.process_lock:
            for process in self.child_processes:
                if process.poll() is None:  # Process is still running
                    try:
                        self.get_logger().info(f"Terminating child process with PID: {process.pid}")
                        process.terminate()
                        # Wait for process to terminate (with timeout)
                        try:
                            process.wait(timeout=5)
                            self.get_logger().info(f"Process {process.pid} terminated successfully")
                        except subprocess.TimeoutExpired:
                            self.get_logger().warning(f"Process {process.pid} did not terminate, killing it")
                            process.kill()
                            process.wait()
                    except Exception as e:
                        self.get_logger().error(f"Error terminating process {process.pid}: {e}")
            
            self.child_processes.clear()
        
        # Exit freedrive mode if active
        if self.freedrive_active and self.ur15_robot:
            try:
                with self.ur15_lock:
                    self.ur15_robot.end_freedrive_mode()
                    self.get_logger().info("Exited freedrive mode during cleanup")
                    self.freedrive_active = False
            except Exception as e:
                self.get_logger().error(f"Error exiting freedrive mode: {e}")
        
        # Disconnect from robot
        if self.ur15_robot is not None:
            try:
                self.ur15_robot.close()
                self.get_logger().info("Disconnected from UR15 robot")
            except Exception as e:
                self.get_logger().error(f"Error disconnecting from robot: {e}")
    
    def _simplify_path(self, path):
        """Simplify path by replacing home directory with ~."""
        if not path:
            return path
        
        # Replace home directory with ~
        home_dir = os.path.expanduser('~')
        if path.startswith(home_dir):
            return path.replace(home_dir, '~', 1)
        
        return path
    
    def _init_ur15_connection(self):
        """Initialize connection to UR15 robot for freedrive control."""
        if UR15Robot is None:
            self.get_logger().warning("UR15Robot class not available, freedrive control disabled")
            return
            
        try:
            self.ur15_robot = UR15Robot(self.ur15_ip, self.ur15_port)
            result = self.ur15_robot.open()
            
            if result == 0:
                self.get_logger().info(f"Connected to UR15 robot at {self.ur15_ip}:{self.ur15_port} for freedrive control")
            else:
                self.get_logger().warning(f"Failed to connect to UR15 robot for freedrive control")
                self.ur15_robot = None
        except Exception as e:
            self.get_logger().error(f"Error initializing UR15 connection: {e}")
            self.ur15_robot = None
    
    def destroy_node(self):
        """Override destroy_node to clean up UR15 connection."""
        if self.ur15_robot:
            try:
                self.ur15_robot.close()
            except Exception as e:
                self.get_logger().error(f"Error closing UR15 connection: {e}")
        super().destroy_node()
    
    def _rotvec_to_matrix(self, rx, ry, rz):
        """Convert rotation vector to rotation matrix using Rodrigues' formula."""
        import math
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
        """Convert pose [X,Y,Z,Rx,Ry,Rz] to 4x4 homogeneous transformation matrix."""
        T = np.eye(4)
        T[0:3, 0:3] = self._rotvec_to_matrix(pose[3], pose[4], pose[5])
        T[0:3, 3] = [pose[0], pose[1], pose[2]]
        return T
    
    def _get_next_file_number(self):
        """Find the next available file number by checking existing files in data_dir."""
        if not os.path.exists(self.dataset_dir):
            os.makedirs(self.dataset_dir, exist_ok=True)
            return 0
        
        existing_files = [f for f in os.listdir(self.dataset_dir) if f.endswith('.json')]
        if not existing_files:
            return 0
        
        # Extract numbers from filenames like "0.json", "1.json", etc.
        numbers = []
        for f in existing_files:
            try:
                num = int(f.replace('.json', ''))
                numbers.append(num)
            except ValueError:
                continue
        
        if not numbers:
            return 0
        
        return max(numbers) + 1
    
    def _get_next_file_number_in_dir(self, directory):
        """Find the next available file number by checking existing files in specified directory."""
        if not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
            return 0
        
        existing_files = [f for f in os.listdir(directory) if f.endswith('.json')]
        if not existing_files:
            return 0
        
        # Extract numbers from filenames like "0.json", "1.json", etc.
        numbers = []
        for f in existing_files:
            try:
                num = int(f.replace('.json', ''))
                numbers.append(num)
            except ValueError:
                continue
        
        if not numbers:
            return 0
        
        return max(numbers) + 1
    
    def _get_next_capture_file_number(self, directory):
        """Find the next available capture file number by checking existing ref_img_* files."""
        if not os.path.exists(directory):
            os.makedirs(directory, exist_ok=True)
            return 1  # Start from 1 for capture files
        
        # Look for existing ref_img_* files
        import glob
        existing_img_files = glob.glob(os.path.join(directory, 'ref_img_*.jpg'))
        existing_json_files = glob.glob(os.path.join(directory, 'ref_img_*.json'))
        existing_pose_files = glob.glob(os.path.join(directory, 'ref_img_*_pose.json'))
        
        all_files = existing_img_files + existing_json_files + existing_pose_files
        
        if not all_files:
            return 1  # Start from 1 for capture files
        
        # Extract numbers from filenames like "ref_img_1.jpg", "ref_img_1.json", "ref_img_1_pose.json", etc.
        numbers = []
        for f in all_files:
            basename = os.path.basename(f)
            # Extract number from patterns like ref_img_N.jpg, ref_img_N.json, ref_img_N_pose.json
            if basename.startswith('ref_img_'):
                try:
                    if '_pose.json' in basename:
                        # Extract number from ref_img_N_pose.json
                        num_part = basename.replace('ref_img_', '').replace('_pose.json', '')
                        if num_part.isdigit():
                            numbers.append(int(num_part))
                    else:
                        # Extract number from ref_img_N.ext (jpg or json)
                        num_part = basename.replace('ref_img_', '').split('.')[0]
                        if num_part.isdigit():
                            numbers.append(int(num_part))
                except:
                    continue
        
        if not numbers:
            return 1  # Start from 1 for capture files
        
        return max(numbers) + 1
        
    def _run_flask_server(self):
        """Run Flask server with proper error handling."""
        try:
            # Disable Flask access logs
            import logging
            log = logging.getLogger('werkzeug')
            log.setLevel(logging.ERROR)
            
            self.app.run(
                host='0.0.0.0', 
                port=self.web_port, 
                debug=False, 
                use_reloader=False,
                threaded=True
            )
        except Exception as e:
            self.get_logger().error(f"Flask server error: {e}")
    
    def image_callback(self, msg):
        """Callback function for receiving camera images."""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            with self.image_lock:
                self.current_image = cv_image.copy()
            
            if not hasattr(self, '_first_image_logged'):
                self.get_logger().info(f"First image received! Size: {cv_image.shape}")
                self._first_image_logged = True
                
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
    
    def joint_states_callback(self, msg):
        """Callback function for receiving joint states."""
        try:
            with self.robot_data_lock:
                # The joint order in /joint_states is:
                # [shoulder_lift(J2), elbow(J3), wrist_1(J4), wrist_2(J5), wrist_3(J6), shoulder_pan(J1)]
                # We need to rearrange to standard order: [J1, J2, J3, J4, J5, J6]
                if len(msg.position) >= 6:
                    positions_radians = list(msg.position)
                    j1 = positions_radians[5]  # shoulder_pan (J1)
                    j2 = positions_radians[0]  # shoulder_lift (J2)
                    j3 = positions_radians[1]  # elbow (J3)
                    j4 = positions_radians[2]  # wrist_1 (J4)
                    j5 = positions_radians[3]  # wrist_2 (J5)
                    j6 = positions_radians[4]  # wrist_3 (J6)
                    
                    # Convert to degrees in correct order
                    self.joint_positions = [
                        j1 * 180.0 / np.pi,
                        j2 * 180.0 / np.pi,
                        j3 * 180.0 / np.pi,
                        j4 * 180.0 / np.pi,
                        j5 * 180.0 / np.pi,
                        j6 * 180.0 / np.pi
                    ]
                else:
                    self.joint_positions = [pos * 180.0 / np.pi for pos in msg.position]
                
                self.last_joint_update = time.time()
            
            if not hasattr(self, '_first_joint_logged'):
                self.get_logger().info(f"First joint state received! Joint names: {msg.name}")
                self._first_joint_logged = True
                
        except Exception as e:
            self.get_logger().error(f"Error processing joint states: {e}")
    
    def tcp_pose_callback(self, msg):
        """Callback function for receiving TCP pose."""
        try:
            with self.robot_data_lock:
                pose = msg.pose
                self.tcp_pose = {
                    'x': pose.position.x * 1000.0,  # Convert to mm
                    'y': pose.position.y * 1000.0,
                    'z': pose.position.z * 1000.0,
                    'qx': pose.orientation.x,
                    'qy': pose.orientation.y,
                    'qz': pose.orientation.z,
                    'qw': pose.orientation.w
                }
                self.last_tcp_update = time.time()
            
            if not hasattr(self, '_first_tcp_logged'):
                self.get_logger().info(f"First TCP pose received!")
                self._first_tcp_logged = True
                
        except Exception as e:
            self.get_logger().error(f"Error processing TCP pose: {e}")
    
    def push_web_log(self, message, log_type='info'):
        """Push a message to the web log queue."""
        import time
        with self.web_log_lock:
            timestamp = time.strftime('%H:%M:%S')
            log_entry = {
                'timestamp': timestamp,
                'message': message,
                'type': log_type
            }
            self.web_log_messages.append(log_entry)
            # Keep only the last 100 messages to prevent memory issues
            if len(self.web_log_messages) > 100:
                self.web_log_messages = self.web_log_messages[-100:]
    
    def setup_flask_routes(self):
        """Setup Flask web routes."""
        
        from flask import render_template
        
        @self.app.route('/')
        def index():
            """Main web interface."""
            try:
                web_dir = os.path.join(get_package_share_directory('ur15_web'), 'web')
                html_file_path = os.path.join(web_dir, 'index.html')
                with open(html_file_path, 'r', encoding='utf-8') as f:
                    html_content = f.read()
                
                # Replace template variables
                html_content = html_content.replace('{{ camera_topic }}', self.camera_topic)
                html_content = html_content.replace('{{ data_dir }}', self._simplify_path(self.dataset_dir))
                html_content = html_content.replace('{{ calibration_data_dir }}', self._simplify_path(self.calibration_data_dir))
                html_content = html_content.replace('{{ chessboard_config }}', self._simplify_path(self.chessboard_config))
                
                return html_content
            except Exception as e:
                self.get_logger().error(f"Error loading HTML template: {e}")
                return f"<html><body><h1>Error loading web interface: {str(e)}</h1></body></html>"
        
        @self.app.route('/video_feed')
        def video_feed():
            """Video streaming route."""
            return Response(self.generate_frames(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/js/<filename>')
        def serve_js_file(filename):
            """Serve JavaScript files."""
            from flask import send_from_directory
            try:
                web_dir = os.path.join(get_package_share_directory('ur15_web'), 'web')
                js_dir = os.path.join(web_dir, 'js')
                return send_from_directory(js_dir, filename, mimetype='application/javascript')
            except Exception as e:
                self.get_logger().error(f"Error serving JS file {filename}: {e}")
                return f"Error: {str(e)}", 404
        
        @self.app.route('/get_status')
        def get_status():
            """Get current status."""
            from flask import jsonify
            has_image = self.current_image is not None
            
            # Debug log for camera status (disabled - not critical)
            # if hasattr(self, '_debug_counter'):
            #     self._debug_counter += 1
            # else:
            #     self._debug_counter = 1
            # 
            # if self._debug_counter % 20 == 0:  # Log every 10 seconds (20 * 500ms)
            #     self.get_logger().info(f"Camera status check: has_image={has_image}, current_image type: {type(self.current_image)}")
            
            with self.calibration_lock:
                has_intrinsic = self.camera_matrix is not None and self.distortion_coefficients is not None
                has_extrinsic = self.cam2end_matrix is not None and self.target2base_matrix is not None
            
            with self.robot_data_lock:
                joint_positions = list(self.joint_positions) if self.joint_positions else []
                tcp_pose = dict(self.tcp_pose) if self.tcp_pose else None
                last_joint_update = self.last_joint_update
                last_tcp_update = self.last_tcp_update
            
            with self.ur15_lock:
                freedrive_active = self.freedrive_active
                robot_connected = self.ur15_robot is not None
            
            # Check if data is recent (within last 2 seconds)
            current_time = time.time()
            joint_data_valid = last_joint_update and (current_time - last_joint_update) < 2.0
            tcp_data_valid = last_tcp_update and (current_time - last_tcp_update) < 2.0
            
            # Read board type from chessboard config
            board_type = None
            board_type_display = 'Unloaded'
            board_type_loaded = False
            try:
                if os.path.exists(self.chessboard_config):
                    with open(self.chessboard_config, 'r') as f:
                        import json
                        config_data = json.load(f)
                        pattern_id = config_data.get('pattern_id', '')
                        if pattern_id:  # Only mark as loaded if pattern_id exists
                            # Map pattern_id to display name
                            pattern_map = {
                                'standard_chessboard': 'ChessBoard',
                                'charuco_board': 'CharUco',
                                'grid_board': 'GridBoard'
                            }
                            board_type = pattern_id
                            board_type_display = pattern_map.get(pattern_id, pattern_id.replace('_', ' ').title())
                            board_type_loaded = True
            except Exception as e:
                self.get_logger().debug(f"Could not read board type from config: {e}")
            
            return jsonify({
                'has_image': has_image,
                'camera_topic': self.camera_topic,
                'data_dir': self._simplify_path(self.dataset_dir),
                'calibration_data_dir': self._simplify_path(self.calibration_data_dir),
                'has_intrinsic': has_intrinsic,
                'has_extrinsic': has_extrinsic,
                'joint_positions': joint_positions if joint_data_valid else [],
                'tcp_pose': tcp_pose if tcp_data_valid else None,
                'freedrive_active': freedrive_active,
                'robot_connected': robot_connected,
                'board_type': board_type,
                'board_type_display': board_type_display,
                'board_type_loaded': board_type_loaded
            })
        
        @self.app.route('/get_web_logs', methods=['GET'])
        def get_web_logs():
            """Get new web log messages."""
            from flask import request, jsonify
            
            try:
                # Get the last message index from query parameter
                last_index = int(request.args.get('last_index', -1))
                
                with self.web_log_lock:
                    # Return messages after the last_index
                    new_messages = []
                    if last_index < len(self.web_log_messages) - 1:
                        start_index = max(0, last_index + 1)
                        new_messages = self.web_log_messages[start_index:]
                    
                    return jsonify({
                        'success': True,
                        'messages': new_messages,
                        'current_index': len(self.web_log_messages) - 1
                    })
            except Exception as e:
                return jsonify({
                    'success': False,
                    'message': str(e)
                })
        
        @self.app.route('/get_workflow_files', methods=['GET'])
        def get_workflow_files():
            """Get list of JSON files from temp/workflow_files directory."""
            from flask import jsonify
            from common.workspace_utils import get_workspace_root
            import glob
            
            try:
                # Get workspace root
                workspace_root = get_workspace_root()
                if not workspace_root:
                    return jsonify({
                        'status': 'error',
                        'message': 'Could not find workspace root',
                        'files': []
                    }), 500
                
                # Build path to workflow_files directory
                workflow_dir = os.path.join(workspace_root, 'temp', 'workflow_files')
                
                # Create directory if it doesn't exist
                if not os.path.exists(workflow_dir):
                    self.get_logger().info(f"Creating workflow directory: {workflow_dir}")
                    os.makedirs(workflow_dir, exist_ok=True)
                
                # Get all JSON files
                json_files = glob.glob(os.path.join(workflow_dir, '*.json'))
                # Extract just the filenames
                file_names = [os.path.basename(f) for f in json_files]
                file_names.sort()  # Sort alphabetically
                
                self.get_logger().info(f"Found {len(file_names)} workflow files in {workflow_dir}")
                
                return jsonify({
                    'status': 'success',
                    'files': file_names,
                    'directory': workflow_dir
                })
                
            except Exception as e:
                self.get_logger().error(f"Error getting workflow files: {e}")
                return jsonify({
                    'status': 'error',
                    'message': str(e),
                    'files': []
                }), 500
        
        @self.app.route('/run_workflow', methods=['POST'])
        def run_workflow():
            """Run a workflow using ros2 run ur15_workflow run_workflow.py."""
            from flask import request, jsonify
            from common.workspace_utils import get_workspace_root
            import subprocess
            import threading
            
            try:
                data = request.get_json()
                workflow_file = data.get('workflow_file')
                
                if not workflow_file:
                    return jsonify({
                        'status': 'error',
                        'message': 'No workflow file specified'
                    }), 400
                
                # Get workspace root
                workspace_root = get_workspace_root()
                if not workspace_root:
                    return jsonify({
                        'status': 'error',
                        'message': 'Could not find workspace root'
                    }), 500
                
                # Build full path to workflow file
                workflow_dir = os.path.join(workspace_root, 'temp', 'workflow_files')
                # Create directory if it doesn't exist
                os.makedirs(workflow_dir, exist_ok=True)
                
                workflow_path = os.path.join(workflow_dir, workflow_file)
                
                # Check if file exists
                if not os.path.exists(workflow_path):
                    return jsonify({
                        'status': 'error',
                        'message': f'Workflow file not found: {workflow_file}'
                    }), 404
                
                # Build ros2 run command
                command = ['ros2', 'run', 'ur15_workflow', 'run_workflow.py', '--config', workflow_path]
                
                self.get_logger().info(f"Running workflow command: {' '.join(command)}")
                
                # Run command in background thread to avoid blocking
                def run_command():
                    try:
                        process = subprocess.Popen(
                            command,
                            stdout=subprocess.PIPE,
                            stderr=subprocess.PIPE,
                            text=True
                        )
                        self.get_logger().info(f"Workflow process started with PID: {process.pid}")
                        
                        # Wait for process to complete
                        stdout, stderr = process.communicate()
                        return_code = process.returncode
                        
                        # Log completion to web
                        if return_code == 0:
                            self.push_web_log(f"Workflow completed successfully: {workflow_file}", 'success')
                            self.get_logger().info(f"Workflow completed successfully: {workflow_file}")
                        else:
                            self.push_web_log(f"Workflow failed with exit code {return_code}: {workflow_file}", 'error')
                            self.get_logger().error(f"Workflow failed with exit code {return_code}: {workflow_file}")
                            if stderr:
                                self.get_logger().error(f"Workflow stderr: {stderr}")
                    except Exception as e:
                        self.get_logger().error(f"Error running workflow process: {e}")
                        self.push_web_log(f"Error running workflow: {str(e)}", 'error')
                
                thread = threading.Thread(target=run_command, daemon=True)
                thread.start()
                
                return jsonify({
                    'status': 'success',
                    'message': f'Workflow started: {workflow_file}',
                    'workflow_file': workflow_file,
                    'workflow_path': workflow_path
                })
                
            except Exception as e:
                self.get_logger().error(f"Error running workflow: {e}")
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                }), 500
        
        @self.app.route('/workflow_config_center_url', methods=['GET'])
        def workflow_config_center_url():
            """Return the URL for workflow configuration center."""
            from flask import jsonify, request
            
            # Use the same hostname as the request to build workflow config center URL
            host = request.host.split(':')[0]  # Extract hostname without port
            workflow_url = f'http://{host}:8008'
            
            self.get_logger().info(f"Redirecting to workflow config center: {workflow_url}")
            
            return jsonify({
                'status': 'success',
                'url': workflow_url
            })
        
        @self.app.route('/prepare_ref_img_1', methods=['POST'])
        def prepare_ref_img_1():
            """Prepare ref_img_1.jpg for labeling based on current operation name."""
            from flask import request, jsonify, url_for
            from urllib.parse import quote
            from common.workspace_utils import get_workspace_root
            
            try:
                data = request.get_json()
                operation_name = data.get('operation_name')
                
                if not operation_name:
                    return jsonify({
                        'status': 'error',
                        'message': 'Operation name is required'
                    }), 400
                
                # Get workspace root (robot_dc directory)
                workspace_root = get_workspace_root()
                if workspace_root is None:
                    return jsonify({
                        'status': 'error',
                        'message': 'Could not determine workspace root directory'
                    }), 500
                
                # Construct ref_img_1 path: workspace_root/dataset/operation_name/ref_img_1.jpg
                operation_dir = os.path.join(workspace_root, 'dataset', operation_name)
                image_filename = 'ref_img_1.jpg'
                image_path = os.path.join(operation_dir, image_filename)
                
                # Check if image exists
                if not os.path.exists(image_path):
                    return jsonify({
                        'status': 'error',
                        'message': f'ref_img_1.jpg not found in {operation_dir}'
                    }), 404
                
                # Temporarily store the ref_img_1 path in robot_status for serving
                if not self.status_client.set_status('ur15', 'temp_label_image', image_path):
                    self.get_logger().warning("Failed to set temp_label_image in robot_status")
                
                # Build the image URL that can be accessed from browser
                image_url = url_for('serve_temp_label_image', _external=True)
                
                # Check for corresponding ref_img_1.json file
                json_filename = 'ref_img_1.json'
                json_path = os.path.join(operation_dir, json_filename)
                
                # Use the same hostname as the request to build labeling URL
                host = request.host.split(':')[0]  # Extract hostname without port
                encoded_image_path = quote(image_path)
                
                # Build labeling URL with optional labels parameter
                labeling_url = f'http://{host}:8007?imageUrl={image_url}&imagePath={encoded_image_path}'
                
                # If JSON file exists, add labelsUrl parameter
                if os.path.exists(json_path):
                    labels_url = url_for('serve_temp_labels_json', _external=True)
                    labeling_url += f'&labelsUrl={labels_url}'
                    self.get_logger().info(f"Found existing labels file: {json_path}")
                    self.get_logger().info(f"Labels URL: {labels_url}")
                else:
                    self.get_logger().info(f"No existing labels file found at: {json_path}")
                
                self.get_logger().info(f"Preparing ref_img_1 for operation '{operation_name}': {image_path}")
                self.get_logger().info(f"Image URL: {image_url}")
                self.get_logger().info(f"Labeling URL: {labeling_url}")
                
                return jsonify({
                    'status': 'success',
                    'image_url': image_url,
                    'labeling_url': labeling_url,
                    'image_path': image_path,
                    'json_path': json_path if os.path.exists(json_path) else None
                })
                
            except Exception as e:
                self.get_logger().error(f"Error preparing ref_img_1: {e}")
                return jsonify({
                    'status': 'error',
                    'message': str(e)
                }), 500
        
        @self.app.route('/serve_temp_label_image', methods=['GET'])
        def serve_temp_label_image():
            """Serve the temporary label image (ref_img_1)."""
            from flask import send_file, jsonify, make_response
            
            try:
                # Get temp_label_image path from robot_status service
                try:
                    image_path = self.status_client.get_status('ur15', 'temp_label_image', timeout_sec=2.0)
                except Exception as e:
                    self.get_logger().debug(f"Failed to get temp_label_image: {e}")
                    image_path = None
                
                if image_path is None:
                    return jsonify({'error': 'No temp label image available'}), 404
                
                if not os.path.exists(image_path):
                    return jsonify({'error': 'Image file not found'}), 404
                
                self.get_logger().info(f"Serving temp label image: {image_path}")
                
                # Send the file with CORS headers to allow cross-origin access
                response = make_response(send_file(image_path, mimetype='image/jpeg'))
                response.headers['Access-Control-Allow-Origin'] = '*'
                response.headers['Access-Control-Allow-Methods'] = 'GET, OPTIONS'
                response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
                return response
                
            except Exception as e:
                self.get_logger().error(f"Error serving temp label image: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/serve_temp_labels_json', methods=['GET'])
        def serve_temp_labels_json():
            """Serve the JSON labels file corresponding to the temp label image (ref_img_1.json)."""
            from flask import send_file, jsonify, make_response
            
            try:
                # Get temp_label_image path from robot_status service
                try:
                    image_path = self.status_client.get_status('ur15', 'temp_label_image', timeout_sec=2.0)
                except Exception as e:
                    self.get_logger().debug(f"Failed to get temp_label_image: {e}")
                    image_path = None
                
                if image_path is None:
                    return jsonify({'error': 'No temp label image available'}), 404
                
                # Construct JSON file path
                image_dir = os.path.dirname(image_path)
                image_filename = os.path.basename(image_path)
                json_filename = os.path.splitext(image_filename)[0] + '.json'
                json_path = os.path.join(image_dir, json_filename)
                
                if not os.path.exists(json_path):
                    return jsonify({'error': 'Labels file not found'}), 404
                
                self.get_logger().info(f"Serving temp labels JSON: {json_path}")
                
                # Send the file with CORS headers to allow cross-origin access
                response = make_response(send_file(json_path, mimetype='application/json'))
                response.headers['Access-Control-Allow-Origin'] = '*'
                response.headers['Access-Control-Allow-Methods'] = 'GET, OPTIONS'
                response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
                return response
                
            except Exception as e:
                self.get_logger().error(f"Error serving temp labels JSON: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/save_labels', methods=['POST', 'OPTIONS'])
        def save_labels():
            """Save labels JSON to the same directory as the image on the server."""
            from flask import request, jsonify
            import json
            
            # Handle CORS preflight request
            if request.method == 'OPTIONS':
                response = jsonify({'status': 'ok'})
                response.headers['Access-Control-Allow-Origin'] = '*'
                response.headers['Access-Control-Allow-Methods'] = 'POST, OPTIONS'
                response.headers['Access-Control-Allow-Headers'] = 'Content-Type'
                return response
            
            try:
                data = request.get_json()
                if not data or 'labels' not in data:
                    response = jsonify({'error': 'No labels data provided'})
                    response.headers['Access-Control-Allow-Origin'] = '*'
                    return response, 400
                
                # Get the image path from request data - required, no default
                image_path = data.get('image_path')
                if not image_path:
                    response = jsonify({'error': 'Image path is required'})
                    response.headers['Access-Control-Allow-Origin'] = '*'
                    return response, 400
                
                # Validate that the image path exists
                if not os.path.exists(image_path):
                    response = jsonify({'error': f'Image not found: {image_path}'})
                    response.headers['Access-Control-Allow-Origin'] = '*'
                    return response, 404
                
                # Construct JSON path from image path
                json_path = os.path.splitext(image_path)[0] + '.json'
                
                # Save the labels to JSON file
                with open(json_path, 'w') as f:
                    json.dump(data['labels'], f, indent=2)
                
                self.get_logger().info(f"Labels saved to: {json_path}")
                self.push_web_log(f"Labels saved to: {json_path}", 'success')
                
                response = jsonify({
                    'status': 'success',
                    'message': 'Labels saved to server',
                    'path': json_path
                })
                response.headers['Access-Control-Allow-Origin'] = '*'
                return response
                
            except Exception as e:
                self.get_logger().error(f"Error saving labels: {e}")
                response = jsonify({
                    'status': 'error',
                    'message': str(e)
                })
                response.headers['Access-Control-Allow-Origin'] = '*'
                return response, 500
        
        @self.app.route('/upload_intrinsic', methods=['POST'])
        def upload_intrinsic():
            """Upload intrinsic calibration parameters."""
            from flask import request, jsonify
            import json
            
            try:
                if 'file' not in request.files:
                    return jsonify({'success': False, 'message': 'No file provided'})
                
                file = request.files['file']
                if file.filename == '':
                    return jsonify({'success': False, 'message': 'No file selected'})
                
                if not file.filename.endswith('.json'):
                    return jsonify({'success': False, 'message': 'File must be a JSON file'})
                
                content = file.read().decode('utf-8')
                data = json.loads(content)
                
                if 'camera_matrix' not in data:
                    return jsonify({'success': False, 'message': 'Missing camera_matrix in JSON'})
                if 'distortion_coefficients' not in data:
                    return jsonify({'success': False, 'message': 'Missing distortion_coefficients in JSON'})
                
                camera_matrix = np.array(data['camera_matrix'], dtype=np.float64)
                distortion_coefficients = np.array(data['distortion_coefficients'], dtype=np.float64)
                
                if camera_matrix.shape != (3, 3):
                    return jsonify({'success': False, 'message': 'camera_matrix must be 3x3'})
                if distortion_coefficients.shape[0] < 4:
                    return jsonify({'success': False, 'message': 'distortion_coefficients must have at least 4 elements'})
                
                with self.calibration_lock:
                    self.camera_matrix = camera_matrix
                    self.distortion_coefficients = distortion_coefficients
                
                self.get_logger().info(f"Intrinsic parameters loaded from {file.filename}")
                return jsonify({'success': True, 'message': 'Intrinsic parameters loaded successfully'})
                
            except json.JSONDecodeError as e:
                return jsonify({'success': False, 'message': f'Invalid JSON format: {str(e)}'})
            except Exception as e:
                self.get_logger().error(f"Error loading intrinsic parameters: {e}")
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/upload_extrinsic', methods=['POST'])
        def upload_extrinsic():
            """Upload extrinsic calibration parameters."""
            from flask import request, jsonify
            import json
            
            try:
                if 'file' not in request.files:
                    return jsonify({'success': False, 'message': 'No file provided'})
                
                file = request.files['file']
                if file.filename == '':
                    return jsonify({'success': False, 'message': 'No file selected'})
                
                if not file.filename.endswith('.json'):
                    return jsonify({'success': False, 'message': 'File must be a JSON file'})
                
                content = file.read().decode('utf-8')
                data = json.loads(content)
                
                if 'cam2end_matrix' not in data:
                    return jsonify({'success': False, 'message': 'Missing cam2end_matrix in JSON'})
                if 'target2base_matrix' not in data:
                    return jsonify({'success': False, 'message': 'Missing target2base_matrix in JSON'})
                
                cam2end_matrix = np.array(data['cam2end_matrix'], dtype=np.float64)
                target2base_matrix = np.array(data['target2base_matrix'], dtype=np.float64)
                
                if cam2end_matrix.shape != (4, 4):
                    return jsonify({'success': False, 'message': 'cam2end_matrix must be 4x4'})
                if target2base_matrix.shape != (4, 4):
                    return jsonify({'success': False, 'message': 'target2base_matrix must be 4x4'})
                
                with self.calibration_lock:
                    self.cam2end_matrix = cam2end_matrix
                    self.target2base_matrix = target2base_matrix
                
                self.get_logger().info(f"Extrinsic parameters loaded from {file.filename}")
                return jsonify({'success': True, 'message': 'Extrinsic parameters loaded successfully'})
                
            except json.JSONDecodeError as e:
                return jsonify({'success': False, 'message': f'Invalid JSON format: {str(e)}'})
            except Exception as e:
                self.get_logger().error(f"Error loading extrinsic parameters: {e}")
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/change_data_dir', methods=['POST'])
        def change_data_dir():
            """Change dataset directory."""
            from flask import request, jsonify
            
            try:
                data = request.get_json()
                if not data or 'data_dir' not in data:
                    return jsonify({'success': False, 'message': 'No data_dir provided'})
                
                new_dir = data['data_dir'].strip()
                if not new_dir:
                    return jsonify({'success': False, 'message': 'Directory path cannot be empty'})
                
                # Expand user home directory if needed
                new_dir = os.path.expanduser(new_dir)
                
                # Create directory if it doesn't exist
                try:
                    os.makedirs(new_dir, exist_ok=True)
                    self.dataset_dir = new_dir
                    self.get_logger().info(f"Dataset directory changed to: {new_dir}")
                    return jsonify({
                        'success': True, 
                        'message': 'Directory changed successfully',
                        'data_dir': self._simplify_path(new_dir)
                    })
                except Exception as e:
                    return jsonify({
                        'success': False, 
                        'message': f'Failed to create/access directory: {str(e)}'
                    })
                
            except Exception as e:
                self.get_logger().error(f"Error changing data directory: {e}")
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/change_calibration_dir', methods=['POST'])
        def change_calibration_dir():
            """Change calibration data directory."""
            from flask import request, jsonify
            
            try:
                data = request.get_json()
                if not data or 'calibration_dir' not in data:
                    return jsonify({'success': False, 'message': 'No calibration_dir provided'})
                
                new_dir = data['calibration_dir'].strip()
                if not new_dir:
                    return jsonify({'success': False, 'message': 'Directory path cannot be empty'})
                
                # Expand user home directory if needed
                new_dir = os.path.expanduser(new_dir)
                
                # Create directory if it doesn't exist
                try:
                    os.makedirs(new_dir, exist_ok=True)
                    self.calibration_data_dir = new_dir
                    self.get_logger().info(f"Calibration data directory changed to: {new_dir}")
                    return jsonify({
                        'success': True, 
                        'message': 'Calibration directory changed successfully',
                        'calibration_data_dir': self._simplify_path(new_dir)
                    })
                except Exception as e:
                    return jsonify({
                        'success': False, 
                        'message': f'Failed to create/access directory: {str(e)}'
                    })
                
            except Exception as e:
                self.get_logger().error(f"Error changing calibration directory: {e}")
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/change_chessboard_config', methods=['POST'])
        def change_chessboard_config():
            """Change chessboard configuration file path."""
            from flask import request, jsonify
            
            try:
                data = request.get_json()
                if not data or 'chessboard_config' not in data:
                    return jsonify({'success': False, 'message': 'No chessboard_config provided'})
                
                new_path = data['chessboard_config'].strip()
                if not new_path:
                    return jsonify({'success': False, 'message': 'File path cannot be empty'})
                
                # Expand user home directory if needed
                new_path = os.path.expanduser(new_path)
                
                # Check if file exists or can be created
                config_dir = os.path.dirname(new_path)
                if config_dir and not os.path.exists(config_dir):
                    try:
                        os.makedirs(config_dir, exist_ok=True)
                    except Exception as e:
                        return jsonify({
                            'success': False, 
                            'message': f'Failed to create directory: {str(e)}'
                        })
                
                # Update the configuration path
                self.chessboard_config = new_path
                self.get_logger().info(f"Chessboard config file changed to: {new_path}")
                
                return jsonify({
                    'success': True, 
                    'message': 'Chessboard config file path changed successfully',
                    'chessboard_config': self._simplify_path(new_path)
                })
                
            except Exception as e:
                self.get_logger().error(f"Error changing chessboard config: {e}")
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/load_chessboard_config', methods=['POST'])
        def load_chessboard_config():
            """Load chessboard configuration from file."""
            from flask import request, jsonify
            import json
            
            try:
                data = request.get_json()
                config_path = data.get('config_path', self.chessboard_config)
                
                # Expand user home directory if needed
                config_path = os.path.expanduser(config_path)
                
                if not os.path.exists(config_path):
                    error_msg = f'Config file not found: {config_path}'
                    self.push_web_log(error_msg, 'error')
                    return jsonify({
                        'success': False,
                        'message': error_msg
                    })
                
                # Read and parse the config file
                with open(config_path, 'r') as f:
                    config_data = json.load(f)
                
                pattern_id = config_data.get('pattern_id', 'Unknown')
                self.get_logger().info(f"Loaded chessboard config from: {config_path}")
                self.get_logger().info(f"Pattern ID: {pattern_id}")
                
                # Push success message to web log
                self.push_web_log(f'Chessboard config loaded successfully (Pattern: {pattern_id})', 'success')
                
                return jsonify({
                    'success': True,
                    'message': 'Config loaded successfully',
                    'config': config_data
                })
                
            except json.JSONDecodeError as e:
                error_msg = f'Invalid JSON format: {str(e)}'
                self.get_logger().error(f"Invalid JSON in config file: {e}")
                self.push_web_log(error_msg, 'error')
                return jsonify({
                    'success': False,
                    'message': error_msg
                })
            except Exception as e:
                error_msg = f'Error loading chessboard config: {str(e)}'
                self.get_logger().error(error_msg)
                self.push_web_log(error_msg, 'error')
                return jsonify({
                    'success': False,
                    'message': error_msg
                })
        
        @self.app.route('/change_operation_path', methods=['POST'])
        def change_operation_path():
            """Change operation path."""
            from flask import request, jsonify
            
            try:
                data = request.get_json()
                if not data or 'operation_path' not in data:
                    return jsonify({'success': False, 'message': 'No operation_path provided'})
                
                new_path = data['operation_path'].strip()
                if not new_path:
                    return jsonify({'success': False, 'message': 'Path cannot be empty'})
                
                # Expand user home directory if needed
                new_path = os.path.expanduser(new_path)
                
                # Get operation name from request or extract from path
                operation_name = data.get('operation_name', '').strip()
                if not operation_name:
                    # Fallback: extract from path if not provided
                    operation_name = os.path.basename(new_path)
                
                # Create directory if it doesn't exist
                try:
                    os.makedirs(new_path, exist_ok=True)
                    self.get_logger().info(f"Operation path changed to: {new_path}")
                    self.get_logger().info(f"Operation name: {operation_name}")
                    
                    # Set operation name to robot_status (only if operation_name is valid)
                    if operation_name and operation_name != 'input operation name':
                        # robot_status_redis doesn't need node parameter
                        if set_to_status('ur15', 'last_operation_name', operation_name):
                            self.get_logger().info(f"Successfully set last_operation_name to robot_status: {operation_name}")
                        else:
                            self.get_logger().warning(f"Failed to set last_operation_name to robot_status")
                    else:
                        self.get_logger().info("Operation name not set (empty or default value)")
                    
                    return jsonify({
                        'success': True, 
                        'message': 'Operation path changed successfully',
                        'operation_path': self._simplify_path(new_path),
                        'operation_name': operation_name
                    })
                except Exception as e:
                    return jsonify({
                        'success': False, 
                        'message': f'Failed to create/access directory: {str(e)}'
                    })
                
            except Exception as e:
                self.get_logger().error(f"Error changing operation path: {e}")
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/set_operating_unit', methods=['POST'])
        def set_operating_unit():
            """Set operating unit to robot_status."""
            from flask import request, jsonify
            
            try:
                data = request.get_json()
                if not data or 'operating_unit' not in data:
                    return jsonify({'success': False, 'message': 'No operating_unit provided'})
                
                operating_unit = data['operating_unit']
                
                # Validate the operating unit value
                if not isinstance(operating_unit, int) or operating_unit < 1:
                    return jsonify({'success': False, 'message': 'Operating unit must be an integer >= 1'})
                
                self.get_logger().info(f"Setting operating unit to: {operating_unit}")
                
                # Set to robot_status (robot_status_redis doesn't need node parameter)
                if set_to_status('ur15', 'rack_operating_unit_id', operating_unit):
                    self.get_logger().info(f"Successfully set rack_operating_unit_id to robot_status: {operating_unit}")
                    return jsonify({
                        'success': True,
                        'message': 'Operating unit set successfully',
                        'operating_unit': operating_unit
                    })
                else:
                    self.get_logger().warning("Failed to set rack_operating_unit_id to robot_status")
                    return jsonify({
                        'success': False,
                        'message': 'Failed to set operating unit to robot_status'
                    })
                
            except Exception as e:
                self.get_logger().error(f"Error setting operating unit: {e}")
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/update_robot_state', methods=['POST'])
        def update_robot_state():
            """Update joint positions and TCP pose to robot_status."""
            from flask import request, jsonify
            
            try:
                data = request.get_json()
                if not data:
                    return jsonify({'success': False, 'message': 'No data provided'})
                
                success_count = 0
                failed_items = []
                
                # Update joint positions if provided
                if 'joint_positions' in data:
                    joint_positions = data['joint_positions']
                    if isinstance(joint_positions, list) and len(joint_positions) == 6:
                        # Convert to numpy array
                        joint_positions_array = np.array(joint_positions, dtype=np.float64)
                        # robot_status_redis doesn't need node parameter
                        if set_to_status('ur15', 'joint_positions', joint_positions_array):
                            success_count += 1
                            self.get_logger().debug(f"Updated joint_positions to robot_status as numpy array")
                        else:
                            failed_items.append('joint_positions')
                            self.get_logger().warning("Failed to set joint_positions to robot_status")
                
                # Update TCP pose if provided
                if 'tcp_pose' in data:
                    tcp_pose = data['tcp_pose']
                    if isinstance(tcp_pose, list) and len(tcp_pose) == 6:
                        # Convert to numpy array in order [x, y, z, rx, ry, rz]
                        tcp_pose_array = np.array(tcp_pose, dtype=np.float64)
                        # robot_status_redis doesn't need node parameter
                        if set_to_status('ur15', 'tcp_pose', tcp_pose_array):
                            success_count += 1
                            self.get_logger().debug(f"Updated tcp_pose to robot_status as numpy array [x, y, z, rx, ry, rz]")
                        else:
                            failed_items.append('tcp_pose')
                            self.get_logger().warning("Failed to set tcp_pose to robot_status")
                
                if success_count > 0:
                    return jsonify({
                        'success': True,
                        'message': f'Successfully updated {success_count} item(s)',
                        'failed_items': failed_items
                    })
                else:
                    return jsonify({
                        'success': False,
                        'message': 'No valid data to update or all updates failed',
                        'failed_items': failed_items
                    })
                
            except Exception as e:
                self.get_logger().error(f"Error updating robot state: {e}")
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/get_pose_count', methods=['GET'])
        def get_pose_count():
            """Get the number of pose JSON files in calibration data directory."""
            from flask import jsonify
            
            try:
                if not hasattr(self, 'calibration_data_dir') or not self.calibration_data_dir:
                    return jsonify({'success': False, 'message': 'Calibration directory not set'})
                
                calib_dir = self.calibration_data_dir
                
                # Check if directory exists
                if not os.path.exists(calib_dir):
                    return jsonify({'success': True, 'count': 0})
                
                # Count JSON files with numeric names (0.json, 1.json, etc.)
                count = 0
                for filename in os.listdir(calib_dir):
                    if filename.endswith('.json'):
                        try:
                            # Try to parse filename as integer (e.g., "0.json" -> 0)
                            int(os.path.splitext(filename)[0])
                            count += 1
                        except ValueError:
                            # Skip non-numeric JSON files
                            continue
                
                self.get_logger().debug(f"Pose count in {calib_dir}: {count}")
                return jsonify({'success': True, 'count': count})
                
            except Exception as e:
                self.get_logger().error(f"Error getting pose count: {e}")
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/toggle_freedrive', methods=['POST'])
        def toggle_freedrive():
            """Toggle freedrive mode on/off."""
            from flask import jsonify
            
            try:
                with self.ur15_lock:
                    if not self.ur15_robot:
                        return jsonify({
                            'success': False, 
                            'message': 'UR15 robot not connected',
                            'freedrive_active': False
                        })
                    
                    if not self.freedrive_active:
                        result = self.ur15_robot.freedrive_mode()
                        if result == 0:
                            self.freedrive_active = True
                            self.get_logger().info("Freedrive mode activated")
                            return jsonify({
                                'success': True, 
                                'message': 'Freedrive mode activated',
                                'freedrive_active': True
                            })
                        else:
                            return jsonify({
                                'success': False, 
                                'message': 'Failed to activate freedrive mode',
                                'freedrive_active': False
                            })
                    else:
                        result = self.ur15_robot.end_freedrive_mode()
                        if result == 0:
                            self.freedrive_active = False
                            self.get_logger().info("Freedrive mode deactivated")
                            return jsonify({
                                'success': True, 
                                'message': 'Freedrive mode deactivated',
                                'freedrive_active': False
                            })
                        else:
                            return jsonify({
                                'success': False, 
                                'message': 'Failed to deactivate freedrive mode',
                                'freedrive_active': self.freedrive_active
                            })
                            
            except Exception as e:
                self.get_logger().error(f"Error toggling freedrive: {e}")
                return jsonify({
                    'success': False, 
                    'message': str(e),
                    'freedrive_active': self.freedrive_active
                })
        
        @self.app.route('/toggle_validation', methods=['POST'])
        def toggle_validation():
            """Toggle calibration validation display on/off."""
            from flask import jsonify
            
            try:
                with self.calibration_lock:
                    has_intrinsic = self.camera_matrix is not None and self.distortion_coefficients is not None
                    has_extrinsic = self.cam2end_matrix is not None and self.target2base_matrix is not None
                
                if not (has_intrinsic and has_extrinsic):
                    return jsonify({
                        'success': False,
                        'message': 'Both intrinsic and extrinsic parameters must be loaded first',
                        'validation_active': False
                    })
                
                self.validation_active = not self.validation_active
                status = 'activated' if self.validation_active else 'deactivated'
                self.get_logger().info(f"Validation display {status}")
                
                return jsonify({
                    'success': True,
                    'message': f'Validation display {status}',
                    'validation_active': self.validation_active
                })
                
            except Exception as e:
                self.get_logger().error(f"Error toggling validation: {e}")
                return jsonify({
                    'success': False,
                    'message': str(e),
                    'validation_active': self.validation_active
                })
        
        @self.app.route('/toggle_corner_detection', methods=['POST'])
        def toggle_corner_detection():
            """Toggle corner detection mode."""
            from flask import jsonify, request
            
            try:
                data = request.get_json()
                enable = data.get('enable', False)
                
                if enable:
                    # Get pattern JSON configuration
                    pattern_json = data.get('pattern_json', None)
                    
                    if pattern_json is None:
                        return jsonify({
                            'success': False,
                            'message': 'No pattern_json provided',
                            'enabled': False
                        })
                    
                    # Store corner detection parameters
                    self.corner_detection_enabled = True
                    self.corner_detection_params = {
                        'pattern_json': pattern_json
                    }
                    
                    pattern_name = pattern_json.get('name', 'Pattern')
                    self.get_logger().info(f"Corner detection enabled: {pattern_name}")
                    return jsonify({
                        'success': True,
                        'message': f'Corner detection enabled: {pattern_name}',
                        'enabled': True
                    })
                else:
                    # Disable corner detection
                    self.corner_detection_enabled = False
                    self.corner_detection_params = {}
                    
                    self.get_logger().info("Corner detection disabled")
                    return jsonify({
                        'success': True,
                        'message': 'Corner detection disabled',
                        'enabled': False
                    })
                    
            except Exception as e:
                self.get_logger().error(f"Error toggling corner detection: {e}")
                return jsonify({
                    'success': False,
                    'message': str(e),
                    'enabled': False
                })
        
        @self.app.route('/toggle_draw_rack', methods=['POST'])
        def toggle_draw_rack():
            """Toggle GB200 rack drawing on/off."""
            from flask import jsonify, request
            
            try:
                data = request.get_json()
                enable = data.get('enable', False)
                
                self.draw_rack_enabled = enable
                
                if enable:
                    self.get_logger().info("GB200 rack drawing enabled")
                    return jsonify({
                        'success': True,
                        'message': 'GB200 rack drawing enabled',
                        'enabled': True
                    })
                else:
                    self.get_logger().info("GB200 rack drawing disabled")
                    return jsonify({
                        'success': True,
                        'message': 'GB200 rack drawing disabled',
                        'enabled': False
                    })
                    
            except Exception as e:
                self.get_logger().error(f"Error toggling rack drawing: {e}")
                return jsonify({
                    'success': False,
                    'message': str(e),
                    'enabled': False
                })
        
        @self.app.route('/toggle_draw_keypoints', methods=['POST'])
        def toggle_draw_keypoints():
            """Toggle keypoints drawing on/off."""
            from flask import jsonify, request
            
            try:
                data = request.get_json()
                enable = data.get('enable', False)
                
                self.draw_keypoints_enabled = enable
                
                if enable:
                    self.get_logger().info("Keypoints drawing enabled")
                    return jsonify({
                        'success': True,
                        'message': 'Keypoints drawing enabled',
                        'enabled': True
                    })
                else:
                    self.get_logger().info("Keypoints drawing disabled")
                    return jsonify({
                        'success': True,
                        'message': 'Keypoints drawing disabled',
                        'enabled': False
                    })
                    
            except Exception as e:
                self.get_logger().error(f"Error toggling keypoints drawing: {e}")
                return jsonify({
                    'success': False,
                    'message': str(e),
                    'enabled': False
                })
        
        @self.app.route('/take_screenshot', methods=['POST'])
        def take_screenshot():
            """Take a screenshot of current camera feed and save robot pose."""
            from flask import jsonify, request
            from datetime import datetime
            import json
            
            try:
                # Get save directory from request, default to calibration_data_dir
                request_data = request.get_json() or {}
                save_dir = request_data.get('save_dir', self.calibration_data_dir)
                
                # Expand user home directory if present
                save_dir = os.path.expanduser(save_dir)
                
                # Create directory if it doesn't exist
                os.makedirs(save_dir, exist_ok=True)
                
                # Check if we have camera image
                with self.image_lock:
                    if self.current_image is None:
                        return jsonify({
                            'success': False,
                            'message': 'No camera image available'
                        })
                    
                    # Copy the current image (already in OpenCV format)
                    cv_image = self.current_image.copy()
                
                # Check if robot is connected
                with self.ur15_lock:
                    if self.ur15_robot is None:
                        return jsonify({
                            'success': False,
                            'message': 'Robot not connected'
                        })
                    
                    # Read actual joint positions and TCP pose
                    try:
                        joint_positions = self.ur15_robot.get_actual_joint_positions()
                        if joint_positions is None:
                            return jsonify({
                                'success': False,
                                'message': 'Failed to read joint positions'
                            })
                        
                        tcp_pose = self.ur15_robot.get_actual_tcp_pose()
                        if tcp_pose is None:
                            return jsonify({
                                'success': False,
                                'message': 'Failed to read TCP pose'
                            })
                    except Exception as e:
                        return jsonify({
                            'success': False,
                            'message': f'Failed to read robot state: {str(e)}'
                        })
                
                # Find next available file number in the save directory
                counter = self._get_next_file_number_in_dir(save_dir)
                
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
                
                # Save JSON file
                json_filename = os.path.join(save_dir, f"{counter}.json")
                with open(json_filename, 'w') as f:
                    json.dump(data, f, indent=2)
                
                # Save image file
                image_filename = os.path.join(save_dir, f"{counter}.jpg")
                cv2.imwrite(image_filename, cv_image)
                
                self.get_logger().info(f"Screenshot saved: {image_filename}")
                
                return jsonify({
                    'success': True,
                    'message': 'Screenshot saved successfully',
                    'file_path': image_filename,
                    'filename': f"{counter}.jpg"
                })
                
            except Exception as e:
                self.get_logger().error(f"Error taking screenshot: {e}")
                return jsonify({
                    'success': False,
                    'message': str(e)
                })
        
        @self.app.route('/get_image_count', methods=['POST'])
        def get_image_count():
            """Get the count of images in the calibration directory."""
            from flask import jsonify, request
            import glob
            
            try:
                request_data = request.get_json() or {}
                directory = request_data.get('directory', self.calibration_data_dir)
                directory = os.path.expanduser(directory)
                
                if not os.path.exists(directory):
                    return jsonify({
                        'success': True,
                        'count': 0
                    })
                
                # Count JSON files (assuming each image has a corresponding JSON, excluding chessboard_config.json)
                json_files = [f for f in glob.glob(os.path.join(directory, '*.json')) 
                             if not f.endswith('chessboard_config.json')]
                count = len(json_files)
                
                return jsonify({
                    'success': True,
                    'count': count
                })
                
            except Exception as e:
                self.get_logger().error(f"Error getting image count: {e}")
                return jsonify({
                    'success': False,
                    'message': str(e),
                    'count': 0
                })
        
        @self.app.route('/delete_calibration_image', methods=['POST'])
        def delete_calibration_image():
            """Delete specific calibration image(s) and renumber remaining files."""
            from flask import jsonify, request
            import glob
            import re
            
            try:
                request_data = request.get_json() or {}
                directory = request_data.get('directory', self.calibration_data_dir)
                directory = os.path.expanduser(directory)
                image_number = request_data.get('image_number')
                delete_all = request_data.get('delete_all', False)
                
                if not os.path.exists(directory):
                    return jsonify({
                        'success': False,
                        'message': 'Directory does not exist'
                    })
                
                # Get all JSON files (excluding chessboard_config.json)
                json_files = [f for f in glob.glob(os.path.join(directory, '*.json')) 
                             if not f.endswith('chessboard_config.json')]
                
                if delete_all:
                    # Delete all images and JSON files (excluding chessboard_config.json)
                    jpg_files = glob.glob(os.path.join(directory, '*.jpg'))
                    total_deleted = len(json_files) + len(jpg_files)
                    
                    for file in json_files + jpg_files:
                        try:
                            os.remove(file)
                        except Exception as e:
                            self.get_logger().error(f"Error deleting {file}: {e}")
                    
                    return jsonify({
                        'success': True,
                        'message': f'Successfully deleted all {total_deleted} calibration files (chessboard_config.json preserved)'
                    })
                
                # Delete specific image
                json_file = os.path.join(directory, f"{image_number}.json")
                jpg_file = os.path.join(directory, f"{image_number}.jpg")
                
                if not os.path.exists(json_file) and not os.path.exists(jpg_file):
                    return jsonify({
                        'success': False,
                        'message': f'Image #{image_number} does not exist'
                    })
                
                # Delete the specific files
                deleted_files = []
                if os.path.exists(json_file):
                    os.remove(json_file)
                    deleted_files.append(f"{image_number}.json")
                if os.path.exists(jpg_file):
                    os.remove(jpg_file)
                    deleted_files.append(f"{image_number}.jpg")
                
                # Get all remaining files and renumber them (excluding chessboard_config.json)
                json_files = [f for f in glob.glob(os.path.join(directory, '*.json')) 
                             if not f.endswith('chessboard_config.json')]
                jpg_files = glob.glob(os.path.join(directory, '*.jpg'))
                
                # Extract numbers from filenames
                def extract_number(filepath):
                    filename = os.path.basename(filepath)
                    match = re.match(r'^(\d+)\.(json|jpg)$', filename)
                    return int(match.group(1)) if match else None
                
                # Create list of (number, json_path, jpg_path) tuples
                file_pairs = {}
                for json_file in json_files:
                    num = extract_number(json_file)
                    if num is not None:
                        if num not in file_pairs:
                            file_pairs[num] = {'json': None, 'jpg': None}
                        file_pairs[num]['json'] = json_file
                
                for jpg_file in jpg_files:
                    num = extract_number(jpg_file)
                    if num is not None:
                        if num not in file_pairs:
                            file_pairs[num] = {'json': None, 'jpg': None}
                        file_pairs[num]['jpg'] = jpg_file
                
                # Sort by number and renumber sequentially
                sorted_numbers = sorted(file_pairs.keys())
                for new_number, old_number in enumerate(sorted_numbers):
                    if new_number != old_number:
                        pair = file_pairs[old_number]
                        
                        # Rename JSON file
                        if pair['json']:
                            new_json_path = os.path.join(directory, f"{new_number}.json")
                            os.rename(pair['json'], new_json_path)
                        
                        # Rename JPG file
                        if pair['jpg']:
                            new_jpg_path = os.path.join(directory, f"{new_number}.jpg")
                            os.rename(pair['jpg'], new_jpg_path)
                
                return jsonify({
                    'success': True,
                    'message': f'Successfully deleted image #{image_number} ({", ".join(deleted_files)}) and renumbered {len(sorted_numbers)} remaining files'
                })
                
            except Exception as e:
                self.get_logger().error(f"Error deleting calibration image: {e}")
                return jsonify({
                    'success': False,
                    'message': str(e)
                })
        
        @self.app.route('/auto_collect_data', methods=['POST'])
        def auto_collect_data():
            """Execute auto collect calibration data script."""
            from flask import jsonify
            import subprocess
            import threading
            
            try:
                # Get the scripts directory using common workspace utilities
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    return jsonify({
                        'success': False,
                        'message': 'Could not find scripts directory'
                    })
                
                script_path = os.path.join(scripts_dir, 'ur_auto_collect_data.py')
                
                # Check if script exists
                if not os.path.exists(script_path):
                    return jsonify({
                        'success': False,
                        'message': f'Script not found: {script_path}'
                    })
                
                # Prepare command with parameters
                cmd = ['python3', script_path]
                
                # Add robot IP parameter
                if hasattr(self, 'ur15_ip') and self.ur15_ip:
                    cmd.extend(['--robot-ip', self.ur15_ip])
                
                # Add data directory parameter (only pass data-dir, no positions-file)
                if hasattr(self, 'calibration_data_dir') and self.calibration_data_dir:
                    cmd.extend(['--data-dir', self.calibration_data_dir])
                
                self.get_logger().info(f"Auto collect command: {' '.join(cmd)}")
                self.get_logger().info(f"Robot IP: {self.ur15_ip if hasattr(self, 'ur15_ip') else 'default'}")
                self.get_logger().info(f"Data directory: {self.calibration_data_dir if hasattr(self, 'calibration_data_dir') else 'default'}")
                self.get_logger().info("Will use existing JSON files in data directory for positions")
                
                # Prepare log file path
                log_file_path = os.path.join(self.calibration_data_dir if hasattr(self, 'calibration_data_dir') else '/tmp', 'auto_collect_log.txt')
                
                # Define a function to monitor the auto collection process
                def monitor_auto_collect():
                    """Monitor auto collect process and report completion status."""
                    try:
                        # Execute the script and wait for completion
                        with open(log_file_path, 'w') as log_file:
                            process = subprocess.Popen(
                                cmd,
                                cwd=os.path.dirname(script_path),
                                stdout=log_file,
                                stderr=subprocess.STDOUT
                            )
                            
                            # Track this process for cleanup
                            with self.process_lock:
                                self.child_processes.append(process)
                            
                            self.get_logger().info(f"Started auto collect data script with PID: {process.pid}")
                            
                            # Wait for the process to complete
                            return_code = process.wait()
                            
                            self.get_logger().info(f"Auto collect script finished with return code: {return_code}")
                            
                            # Push completion message to web log
                            if return_code == 0:
                                self.push_web_log("📸 Auto collection completed successfully!", 'success')
                                # Update pose count and notify
                                try:
                                    import glob
                                    image_count = len(glob.glob(os.path.join(self.calibration_data_dir, '*.jpg')))
                                    self.push_web_log(f"📊 Total images collected: {image_count}", 'info')
                                except:
                                    pass
                                self.push_web_log("✅ Ready for calibration", 'success')
                            else:
                                self.push_web_log(f"❌ Auto collection failed with return code: {return_code}", 'error')
                                self.push_web_log(f"📄 Check log file: {self._simplify_path(log_file_path)}", 'info')
                    
                    except Exception as e:
                        self.get_logger().error(f"Error in auto collect monitor thread: {e}")
                        self.push_web_log(f"❌ Auto collection error: {str(e)}", 'error')
                
                # Start monitoring in a background thread
                monitor_thread = threading.Thread(target=monitor_auto_collect, daemon=True)
                monitor_thread.start()
                
                self.get_logger().info(f"Started auto collect monitoring thread")
                self.get_logger().info(f"Script output will be logged to: {log_file_path}")
                
                # Push start message to web log
                self.push_web_log("🤖 Starting auto collection process...", 'info')
                self.push_web_log(f"🎯 Robot IP: {self.ur15_ip if hasattr(self, 'ur15_ip') else 'default'}", 'info')
                self.push_web_log(f"📁 Data directory: {self._simplify_path(self.calibration_data_dir)}", 'info')
                
                return jsonify({
                    'success': True,
                    'message': 'Auto collect data script started',
                    'log_file': self._simplify_path(log_file_path)
                })
                
            except Exception as e:
                self.get_logger().error(f"Error starting auto collect script: {e}")
                self.push_web_log(f"❌ Failed to start auto collection: {str(e)}", 'error')
                return jsonify({
                    'success': False,
                    'message': str(e)
                })
        
        @self.app.route('/calibrate_cam', methods=['POST'])
        def calibrate_cam():
            """Execute camera calibration script."""
            from flask import jsonify
            import subprocess
            import threading
            
            try:
                # Get the scripts directory using common workspace utilities
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    return jsonify({
                        'success': False,
                        'message': 'Could not find scripts directory'
                    })
                
                script_path = os.path.join(scripts_dir, 'ur_cam_calibrate.py')
                
                # Check if script exists
                if not os.path.exists(script_path):
                    return jsonify({
                        'success': False,
                        'message': f'Script not found: {script_path}'
                    })
                
                self.get_logger().info(f"Calibration data dir: {self.calibration_data_dir}")
                self.get_logger().info(f"Output dir: {self.calibration_result_dir}")
                
                # Prepare log file path
                log_file_path = os.path.join(self.calibration_data_dir if hasattr(self, 'calibration_data_dir') else '/tmp', 'calibrate_cam_log.txt')
                
                # Use chessboard config from parameter
                config_file_path = self.chessboard_config if hasattr(self, 'chessboard_config') else os.path.join(self.calibration_data_dir, 'chessboard_config.json')
                
                # Prepare command with config-file parameter
                cmd = ['python3', script_path, 
                       '--data-dir', self.calibration_data_dir,
                       '--output-dir', self.calibration_result_dir,
                       '--config-file', config_file_path,
                       '--verbose']
                
                self.get_logger().info(f"Camera calibration command: {' '.join(cmd)}")
                
                # Define a function to monitor the calibration process
                def monitor_calibration():
                    """Monitor calibration process and auto-load results when done."""
                    try:
                        # Execute the script and wait for completion
                        with open(log_file_path, 'w') as log_file:
                            process = subprocess.Popen(
                                cmd,
                                cwd=os.path.dirname(script_path),
                                stdout=log_file,
                                stderr=subprocess.STDOUT
                            )
                            
                            # Track this process for cleanup
                            with self.process_lock:
                                self.child_processes.append(process)
                            
                            self.get_logger().info(f"Started camera calibration script with PID: {process.pid}")
                            
                            # Wait for the process to complete
                            return_code = process.wait()
                            
                            self.get_logger().info(f"Calibration script finished with return code: {return_code}")
                            
                            # Push calibration completion message to web log
                            if return_code == 0:
                                self.push_web_log("🎯 Camera calibration completed successfully!", 'success')
                            else:
                                self.push_web_log(f"❌ Camera calibration failed with return code: {return_code}", 'error')
                            
                            # If successful, auto-load the calibration results
                            if return_code == 0:
                                self.get_logger().info("Calibration successful, auto-loading results...")
                                self.push_web_log("📥 Auto-loading calibration parameters...", 'info')
                                
                                # Load intrinsic parameters
                                camera_params_dir = os.path.join(self.calibration_result_dir, 'ur15_camera_parameters')
                                intrinsic_file = os.path.join(camera_params_dir, 'ur15_cam_calibration_result.json')
                                extrinsic_file = os.path.join(camera_params_dir, 'ur15_cam_eye_in_hand_result.json')
                                
                                intrinsic_loaded = False
                                extrinsic_loaded = False
                                
                                # Load intrinsic calibration result
                                if os.path.exists(intrinsic_file):
                                    try:
                                        with open(intrinsic_file, 'r') as f:
                                            intrinsic_data = json.load(f)
                                        
                                        if intrinsic_data.get('success', False):
                                            camera_matrix = np.array(intrinsic_data['camera_matrix'], dtype=np.float64)
                                            distortion_coeffs = np.array(intrinsic_data['distortion_coefficients'], dtype=np.float64)
                                            
                                            with self.calibration_lock:
                                                self.camera_matrix = camera_matrix
                                                self.distortion_coefficients = distortion_coeffs
                                            
                                            rms_error = intrinsic_data.get('rms_error', 'N/A')
                                            self.get_logger().info("✅ Auto-loaded intrinsic parameters")
                                            self.push_web_log(f"✅ Intrinsic parameters loaded (RMS: {rms_error} pixels)", 'success')
                                            intrinsic_loaded = True
                                        else:
                                            self.get_logger().warning("Intrinsic calibration result indicates failure")
                                            self.push_web_log("⚠️ Intrinsic calibration failed", 'warning')
                                    except Exception as e:
                                        self.get_logger().error(f"Failed to auto-load intrinsic parameters: {e}")
                                        self.push_web_log(f"❌ Failed to load intrinsic parameters: {e}", 'error')
                                else:
                                    self.get_logger().warning(f"Intrinsic result file not found: {intrinsic_file}")
                                    self.push_web_log("⚠️ Intrinsic result file not found", 'warning')
                                
                                # Load extrinsic calibration result (eye-in-hand)
                                if os.path.exists(extrinsic_file):
                                    try:
                                        with open(extrinsic_file, 'r') as f:
                                            extrinsic_data = json.load(f)
                                        
                                        if extrinsic_data.get('success', False):
                                            cam2end_matrix = np.array(extrinsic_data['cam2end_matrix'], dtype=np.float64)
                                            target2base_matrix = np.array(extrinsic_data['target2base_matrix'], dtype=np.float64)
                                            
                                            with self.calibration_lock:
                                                self.cam2end_matrix = cam2end_matrix
                                                self.target2base_matrix = target2base_matrix
                                            
                                            rms_error = extrinsic_data.get('rms_error', 'N/A')
                                            self.get_logger().info("✅ Auto-loaded extrinsic parameters (eye-in-hand)")
                                            self.push_web_log(f"✅ Extrinsic parameters loaded (RMS: {rms_error} pixels)", 'success')
                                            extrinsic_loaded = True
                                        else:
                                            self.get_logger().warning("Extrinsic calibration result indicates failure")
                                            self.push_web_log("⚠️ Extrinsic calibration failed", 'warning')
                                    except Exception as e:
                                        self.get_logger().error(f"Failed to auto-load extrinsic parameters: {e}")
                                        self.push_web_log(f"❌ Failed to load extrinsic parameters: {e}", 'error')
                                else:
                                    self.get_logger().warning(f"Extrinsic result file not found: {extrinsic_file}")
                                    self.push_web_log("⚠️ Extrinsic result file not found", 'warning')
                                
                                # Final summary
                                if intrinsic_loaded and extrinsic_loaded:
                                    self.push_web_log("🎉 Calibration completed! All parameters loaded successfully", 'success')
                                    
                                    # Send calibration parameters to robot_status
                                    try:
                                        self.push_web_log("📤 Sending calibration parameters to robot_status...", 'info')
                                        
                                        # Prepare and send calibration data
                                        with self.calibration_lock:
                                            params = {
                                                'camera_matrix': self.camera_matrix,
                                                'distortion_coefficients': self.distortion_coefficients,
                                                'cam2end_matrix': self.cam2end_matrix,
                                                'target2base_matrix': self.target2base_matrix
                                            }
                                        
                                        # Send all parameters
                                        success_count = 0
                                        for key, value in params.items():
                                            if self.status_client.set_status('ur15', key, value):
                                                success_count += 1
                                        
                                        if success_count == len(params):
                                            self.get_logger().info("✅ Calibration parameters sent to robot_status")
                                            self.push_web_log("✅ Calibration parameters saved to robot_status", 'success')
                                        else:
                                            self.get_logger().warning(f"Only {success_count}/{len(params)} parameters saved to robot_status")
                                            self.push_web_log(f"⚠️ Only {success_count}/{len(params)} parameters saved", 'warning')
                                        
                                    except Exception as e:
                                        self.get_logger().error(f"Failed to send calibration to robot_status: {e}")
                                        self.push_web_log(f"⚠️ Failed to send to robot_status: {e}", 'warning')
                                    
                                elif intrinsic_loaded or extrinsic_loaded:
                                    self.push_web_log("⚠️ Calibration partially completed", 'warning')
                                else:
                                    self.push_web_log("❌ Calibration completed but failed to load parameters", 'error')
                                
                                self.get_logger().info("🎉 Calibration completed and parameters auto-loaded!")
                            else:
                                self.get_logger().error(f"Calibration script failed with return code: {return_code}")
                                self.get_logger().info(f"Check log file for details: {log_file_path}")
                                self.push_web_log(f"📄 Check log file: {self._simplify_path(log_file_path)}", 'info')
                    
                    except Exception as e:
                        self.get_logger().error(f"Error in calibration monitor thread: {e}")
                
                # Start monitoring in a background thread
                monitor_thread = threading.Thread(target=monitor_calibration, daemon=True)
                monitor_thread.start()
                
                self.get_logger().info(f"Started camera calibration monitoring thread")
                self.get_logger().info(f"  Data directory: {self.calibration_data_dir}")
                self.get_logger().info(f"  Config file: {config_file_path}")
                self.get_logger().info(f"  Output directory: {self.calibration_result_dir}")
                self.get_logger().info(f"  Script output will be logged to: {log_file_path}")
                
                # Push start message to web log
                self.push_web_log("🚀 Starting camera calibration process...", 'info')
                self.push_web_log(f"📁 Data directory: {self._simplify_path(self.calibration_data_dir)}", 'info')
                self.push_web_log(f"⚙️ Config file: {self._simplify_path(config_file_path)}", 'info')
                
                return jsonify({
                    'success': True,
                    'message': 'Camera calibration script started. Results will be auto-loaded when complete.',
                    'config_file': self._simplify_path(config_file_path),
                    'output_dir': self._simplify_path(self.calibration_result_dir),
                    'log_file': self._simplify_path(log_file_path)
                })
                
            except Exception as e:
                self.get_logger().error(f"Error starting camera calibration script: {e}")
                return jsonify({
                    'success': False,
                    'message': str(e)
                })
        
        @self.app.route('/capture_task_data', methods=['POST'])
        def capture_task_data():
            """Capture current image, robot pose, and camera parameters for a task."""
            from flask import jsonify, request
            import time
            import json
            
            try:
                data = request.get_json()
                task_name = data.get('task_name')
                task_path = data.get('task_path')
                calibration_data_dir = data.get('calibration_data_dir')
                
                if not task_name or not task_path:
                    return jsonify({
                        'success': False, 
                        'message': 'Missing task_name or task_path'
                    })
                
                if not calibration_data_dir:
                    return jsonify({
                        'success': False, 
                        'message': 'Missing calibration_data_dir'
                    })
                
                # Expand ~ in paths and ensure task directory exists
                expanded_task_path = os.path.expanduser(task_path)
                os.makedirs(expanded_task_path, exist_ok=True)
                
                # Get next available file number for capture
                capture_number = self._get_next_capture_file_number(expanded_task_path)
                
                # 1. Capture current image as ref_img_N.jpg
                with self.image_lock:
                    if self.current_image is None:
                        return jsonify({
                            'success': False, 
                            'message': 'No camera image available'
                        })
                    
                    image_filename = f'ref_img_{capture_number}.jpg'
                    image_path = os.path.join(expanded_task_path, image_filename)
                    cv2.imwrite(image_path, self.current_image)
                
                # 2. Get current robot pose and save as ref_pose.json
                current_tcp_pose = None
                with self.ur15_lock:
                    if self.ur15_robot is None:
                        return jsonify({
                            'success': False,
                            'message': 'Robot not connected'
                        })
                    
                    try:
                        # Read both joint positions and TCP pose like in screenshot
                        joint_positions = self.ur15_robot.get_actual_joint_positions()
                        if joint_positions is None:
                            return jsonify({
                                'success': False,
                                'message': 'Failed to read joint positions'
                            })
                        
                        tcp_pose = self.ur15_robot.get_actual_tcp_pose()
                        if tcp_pose is None:
                            return jsonify({
                                'success': False,
                                'message': 'Failed to read TCP pose'
                            })
                        
                        # Calculate end2base transformation matrix like in screenshot
                        end2base = self._pose_to_matrix(tcp_pose)
                        
                        # Format pose data exactly like screenshot endpoint
                        from datetime import datetime
                        current_tcp_pose = {
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
                    except Exception as e:
                        return jsonify({
                            'success': False,
                            'message': f'Failed to read robot pose: {str(e)}'
                        })
                
                if current_tcp_pose is None:
                    return jsonify({
                        'success': False, 
                        'message': 'No robot pose data available'
                    })

                # 3. Get camera parameters from calibration results
                camera_params = self._get_camera_parameters_from_status(calibration_data_dir)
                if not camera_params['success']:
                    return jsonify({
                        'success': False, 
                        'message': camera_params['message']
                    })

                # Merge all data into flat structure and save as ref_img_N_pose.json
                combined_data = {}
                
                # Add pose data directly to top level (excluding timestamp first)
                pose_data_without_timestamp = {k: v for k, v in current_tcp_pose.items() if k != 'timestamp'}
                combined_data.update(pose_data_without_timestamp)
                
                # Add camera parameters directly to top level (flattened)
                camera_data = camera_params['data']
                if 'intrinsics' in camera_data:
                    combined_data.update(camera_data['intrinsics'])
                if 'extrinsics' in camera_data:
                    combined_data.update(camera_data['extrinsics'])
                
                # Add timestamp at the end
                if 'timestamp' in current_tcp_pose:
                    combined_data['timestamp'] = current_tcp_pose['timestamp']
                
                pose_filename = f'ref_img_{capture_number}_pose.json'
                pose_path = os.path.join(expanded_task_path, pose_filename)
                with open(pose_path, 'w') as f:
                    json.dump(combined_data, f, indent=2)                # Add web log message
                self.push_web_log(f'Captured image and pose data: {image_filename}, {pose_filename}', 'success')
                
                return jsonify({
                    'success': True,
                    'message': f'Successfully captured image and pose data',
                    'image_path': image_path,
                    'pose_path': pose_path,
                    'image_file': image_filename,
                    'pose_file': pose_filename,
                    'calibration_source': camera_params.get('source', 'Unknown')
                })
                
            except Exception as e:
                error_msg = f'Failed to capture task data: {str(e)}'
                self.get_logger().error(error_msg)
                self.push_web_log(error_msg, 'error')
                return jsonify({'success': False, 'message': error_msg})
        
        @self.app.route('/capture_ref_img_1', methods=['POST'])
        def capture_ref_img_1():
            """Capture current image and robot pose, always saving as ref_img_1.jpg and ref_img_1_pose.json."""
            from flask import jsonify, request
            import time
            import json
            
            try:
                data = request.get_json()
                task_name = data.get('task_name')
                task_path = data.get('task_path')
                calibration_data_dir = data.get('calibration_data_dir')
                
                if not task_name or not task_path:
                    return jsonify({
                        'success': False, 
                        'message': 'Missing task_name or task_path'
                    })
                
                if not calibration_data_dir:
                    return jsonify({
                        'success': False, 
                        'message': 'Missing calibration_data_dir'
                    })
                
                # Expand ~ in paths and ensure task directory exists
                expanded_task_path = os.path.expanduser(task_path)
                os.makedirs(expanded_task_path, exist_ok=True)
                
                # Fixed filenames
                image_filename = 'ref_img_1.jpg'
                pose_filename = 'ref_img_1_pose.json'
                
                # 1. Capture current image as ref_img_1.jpg
                with self.image_lock:
                    if self.current_image is None:
                        return jsonify({
                            'success': False, 
                            'message': 'No camera image available'
                        })
                    
                    image_path = os.path.join(expanded_task_path, image_filename)
                    cv2.imwrite(image_path, self.current_image)
                
                # 2. Get current robot pose
                current_tcp_pose = None
                with self.ur15_lock:
                    if self.ur15_robot is None:
                        return jsonify({
                            'success': False,
                            'message': 'Robot not connected'
                        })
                    
                    try:
                        # Read both joint positions and TCP pose
                        joint_positions = self.ur15_robot.get_actual_joint_positions()
                        if joint_positions is None:
                            return jsonify({
                                'success': False,
                                'message': 'Failed to read joint positions'
                            })
                        
                        tcp_pose = self.ur15_robot.get_actual_tcp_pose()
                        if tcp_pose is None:
                            return jsonify({
                                'success': False,
                                'message': 'Failed to read TCP pose'
                            })
                        
                        # Calculate end2base transformation matrix
                        end2base = self._pose_to_matrix(tcp_pose)
                        
                        # Format pose data
                        from datetime import datetime
                        current_tcp_pose = {
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
                    except Exception as e:
                        return jsonify({
                            'success': False,
                            'message': f'Failed to read robot pose: {str(e)}'
                        })
                
                if current_tcp_pose is None:
                    return jsonify({
                        'success': False, 
                        'message': 'No robot pose data available'
                    })

                # 3. Get camera parameters from calibration results
                camera_params = self._get_camera_parameters_from_status(calibration_data_dir)
                if not camera_params['success']:
                    return jsonify({
                        'success': False, 
                        'message': camera_params['message']
                    })

                # Merge all data into flat structure and save as ref_img_1_pose.json
                combined_data = {}
                
                # Add pose data directly to top level (excluding timestamp first)
                pose_data_without_timestamp = {k: v for k, v in current_tcp_pose.items() if k != 'timestamp'}
                combined_data.update(pose_data_without_timestamp)
                
                # Add camera parameters directly to top level (flattened)
                camera_data = camera_params['data']
                if 'intrinsics' in camera_data:
                    combined_data.update(camera_data['intrinsics'])
                if 'extrinsics' in camera_data:
                    combined_data.update(camera_data['extrinsics'])
                
                # Add timestamp at the end
                if 'timestamp' in current_tcp_pose:
                    combined_data['timestamp'] = current_tcp_pose['timestamp']
                
                pose_path = os.path.join(expanded_task_path, pose_filename)
                with open(pose_path, 'w') as f:
                    json.dump(combined_data, f, indent=2)
                
                # Add web log message
                self.push_web_log(f'Captured ref_img_1: {image_filename}, {pose_filename}', 'success')
                
                # Upload the absolute path of ref_img_1.jpg to robot_status
                try:
                    abs_ref_img_1_path = os.path.abspath(image_path)
                    if self.status_client.set_status('ur15', 'last_picture', abs_ref_img_1_path):
                        self.get_logger().info(f"✅ Uploaded last_picture path to robot_status: {abs_ref_img_1_path}")
                        self.push_web_log(f"✅ Saved last_picture path: {abs_ref_img_1_path}", 'success')
                    else:
                        self.get_logger().warning("⚠️ Failed to upload last_picture path to robot_status")
                        self.push_web_log("⚠️ Failed to save last_picture path", 'warning')
                except Exception as e:
                    self.get_logger().error(f"❌ Error uploading last_picture to robot_status: {e}")
                    self.push_web_log(f"⚠️ Error saving last_picture: {e}", 'warning')
                
                return jsonify({
                    'success': True,
                    'message': f'Successfully captured ref_img_1',
                    'image_path': image_path,
                    'pose_path': pose_path,
                    'image_file': image_filename,
                    'pose_file': pose_filename,
                    'calibration_source': camera_params.get('source', 'Unknown')
                })
                
            except Exception as e:
                error_msg = f'Failed to capture ref_img_1: {str(e)}'
                self.get_logger().error(error_msg)
                self.push_web_log(error_msg, 'error')
                return jsonify({'success': False, 'message': error_msg})
        
        @self.app.route('/set_task_path', methods=['POST'])
        def set_task_path():
            """Set path for a specific task."""
            from flask import jsonify, request
            
            try:
                data = request.get_json()
                task_name = data.get('task_name')
                task_path = data.get('task_path')
                
                if not task_name or not task_path:
                    return jsonify({
                        'success': False,
                        'message': 'Missing task_name or task_path'
                    })
                
                # Expand ~ in path and create directory if it doesn't exist
                try:
                    expanded_task_path = os.path.expanduser(task_path)
                    os.makedirs(expanded_task_path, exist_ok=True)
                    self.get_logger().info(f"Task path set for {task_name}: {expanded_task_path}")
                    self.push_web_log(f'Path set for task "{task_name}": {self._simplify_path(expanded_task_path)}', 'success')
                    
                    return jsonify({
                        'success': True,
                        'message': f'Successfully set path for task {task_name}',
                        'task_name': task_name,
                        'task_path': expanded_task_path
                    })
                
                except OSError as e:
                    error_msg = f'Failed to create directory: {str(e)}'
                    self.get_logger().error(f"Failed to create directory {expanded_task_path}: {e}")
                    return jsonify({
                        'success': False,
                        'message': error_msg
                    })
                    
            except Exception as e:
                error_msg = f'Failed to set task path: {str(e)}'
                self.get_logger().error(error_msg)
                return jsonify({'success': False, 'message': error_msg})
        
        @self.app.route('/movej', methods=['POST'])
        def movej():
            """Move robot to specified joint positions."""
            from flask import jsonify, request
            
            try:
                # Get joint positions from request
                request_data = request.get_json()
                if not request_data:
                    return jsonify({
                        'success': False,
                        'message': 'No data provided'
                    })
                
                joint_positions = request_data.get('joint_positions')
                if not joint_positions:
                    return jsonify({
                        'success': False,
                        'message': 'Joint positions not provided'
                    })
                
                # Get blocking parameter (default: False)
                blocking = request_data.get('blocking', False)
                
                # Validate joint positions
                if not isinstance(joint_positions, list) or len(joint_positions) != 6:
                    return jsonify({
                        'success': False,
                        'message': 'Joint positions must be a list of 6 values'
                    })
                
                # Convert to radians and validate range
                try:
                    joint_positions_rad = []
                    for i, pos in enumerate(joint_positions):
                        pos_rad = float(pos)
                        # Basic joint limits check (approximate UR15 limits)
                        if abs(pos_rad) > 6.28:  # About 2*pi radians
                            return jsonify({
                                'success': False,
                                'message': f'Joint {i+1} position {pos_rad:.3f} rad is out of range'
                            })
                        joint_positions_rad.append(pos_rad)
                except (ValueError, TypeError):
                    return jsonify({
                        'success': False,
                        'message': 'Invalid joint position values'
                    })
                
                # Check robot connection
                with self.ur15_lock:
                    if self.ur15_robot is None:
                        return jsonify({
                            'success': False,
                            'message': 'Robot not connected'
                        })
                    
                    # Check if robot is in freedrive mode
                    if self.freedrive_active:
                        return jsonify({
                            'success': False,
                            'message': 'Cannot move robot while in freedrive mode'
                        })
                    
                    # Send movej command
                    try:
                        self.get_logger().info(f"Moving robot to joint positions: {joint_positions_rad}, blocking: {blocking}")
                        result = self.ur15_robot.movej(
                            joint_positions_rad,
                            a=0.5,  # acceleration 0.5 rad/s^2
                            v=0.3,  # velocity 0.3 rad/s
                            blocking=blocking  # Use blocking parameter from request
                        )
                        
                        if result == 0:
                            # Don't push to web log to avoid duplicate logs from frontend
                            return jsonify({
                                'success': True,
                                'message': 'Robot movement started',
                                'joint_positions': joint_positions_rad
                            })
                        else:
                            return jsonify({
                                'success': False,
                                'message': 'Failed to send movement command'
                            })
                    except Exception as e:
                        self.get_logger().error(f"Error sending movej command: {e}")
                        return jsonify({
                            'success': False,
                            'message': f'Robot movement error: {str(e)}'
                        })
                
            except Exception as e:
                self.get_logger().error(f"Error in movej endpoint: {e}")
                return jsonify({
                    'success': False,
                    'message': str(e)
                })
        
        @self.app.route('/movel', methods=['POST'])
        def movel():
            """Move robot to specified TCP pose."""
            from flask import jsonify, request
            
            try:
                # Get TCP pose from request
                request_data = request.get_json()
                if not request_data:
                    return jsonify({
                        'success': False,
                        'message': 'No data provided'
                    })
                
                tcp_pose = request_data.get('tcp_pose')
                if not tcp_pose:
                    return jsonify({
                        'success': False,
                        'message': 'TCP pose not provided'
                    })
                
                # Validate TCP pose
                if not isinstance(tcp_pose, list) or len(tcp_pose) != 6:
                    return jsonify({
                        'success': False,
                        'message': 'TCP pose must be a list of 6 values [x,y,z,rx,ry,rz]'
                    })
                
                # Validate and convert units
                try:
                    tcp_pose_meters_radians = []
                    for i, value in enumerate(tcp_pose):
                        val = float(value)
                        if i < 3:  # Position values (x, y, z)
                            # Convert from mm to meters
                            val_m = val / 1000.0
                            # Basic position limits check (approximate workspace limits)
                            if abs(val_m) > 2.0:  # 2 meter workspace limit
                                return jsonify({
                                    'success': False,
                                    'message': f'Position {"XYZ"[i]} value {val}mm is out of workspace range'
                                })
                            tcp_pose_meters_radians.append(val_m)
                        else:  # Rotation values (rx, ry, rz)
                            # Convert from degrees to radians
                            val_rad = val * np.pi / 180.0
                            # Basic rotation limits check
                            if abs(val_rad) > 6.28:  # About 2*pi radians
                                return jsonify({
                                    'success': False,
                                    'message': f'Rotation {"XYZ"[i-3]} value {val}° is out of range'
                                })
                            tcp_pose_meters_radians.append(val_rad)
                            
                except (ValueError, TypeError):
                    return jsonify({
                        'success': False,
                        'message': 'Invalid TCP pose values'
                    })
                
                # Check robot connection
                with self.ur15_lock:
                    if self.ur15_robot is None:
                        return jsonify({
                            'success': False,
                            'message': 'Robot not connected'
                        })
                    
                    # Check if robot is in freedrive mode
                    if self.freedrive_active:
                        return jsonify({
                            'success': False,
                            'message': 'Cannot move robot while in freedrive mode'
                        })
                    
                    # Send movel command
                    try:
                        self.get_logger().info(f"Moving robot to TCP pose: {tcp_pose_meters_radians}")
                        result = self.ur15_robot.movel(
                            tcp_pose_meters_radians,
                            a=0.5,  # acceleration 0.5 m/s^2
                            v=0.5,  # velocity 0.1 m/s
                            blocking=False  # Don't block web interface
                        )
                        
                        if result == 0:
                            # Convert back for display (meters to mm, radians to degrees)
                            display_pose = []
                            for i, val in enumerate(tcp_pose_meters_radians):
                                if i < 3:
                                    display_pose.append(f'{val * 1000:.1f}mm')  # Position in mm
                                else:
                                    display_pose.append(f'{val * 180 / np.pi:.2f}°')  # Rotation in degrees
                            
                            self.push_web_log(f"🎯 Moving to TCP pose: [{', '.join(display_pose)}]", 'info')
                            return jsonify({
                                'success': True,
                                'message': 'TCP movement started',
                                'tcp_pose': tcp_pose_meters_radians
                            })
                        else:
                            return jsonify({
                                'success': False,
                                'message': 'Failed to send movement command'
                            })
                    except Exception as e:
                        self.get_logger().error(f"Error sending movel command: {e}")
                        return jsonify({
                            'success': False,
                            'message': f'TCP movement error: {str(e)}'
                        })
                
            except Exception as e:
                self.get_logger().error(f"Error in movel endpoint: {e}")
                return jsonify({
                    'success': False,
                    'message': str(e)
                })
        
        @self.app.route('/locate_last_operation', methods=['POST'])
        def locate_last_operation():
            """Execute locate last operation script (ur_3d_positioning.py)."""
            from flask import request, jsonify
            import subprocess
            import threading
            
            try:
                # Get last_operation_name from robot_status (robot_status_redis doesn't need node parameter)
                operation_name = get_from_status('ur15', 'last_operation_name')
                
                if not operation_name:
                    return jsonify({
                        'success': False,
                        'message': 'No last_operation_name found in robot_status. Please set an operation name first.'
                    })
                
                operation_name = operation_name.strip()
                if not operation_name or operation_name == 'input operation name':
                    return jsonify({
                        'success': False,
                        'message': 'Invalid operation name in robot_status'
                    })
                
                # Get the scripts directory
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    return jsonify({
                        'success': False,
                        'message': 'Could not find scripts directory'
                    })
                
                script_path = os.path.join(scripts_dir, 'ur_3d_positioning.py')
                
                # Check if script exists
                if not os.path.exists(script_path):
                    return jsonify({
                        'success': False,
                        'message': f'Script not found: {script_path}'
                    })
                
                # Prepare command
                cmd = ['python3', script_path, '--operation-name', operation_name]
                
                self.get_logger().info(f"Locate last operation command: {' '.join(cmd)}")
                
                # Define a function to monitor the process
                def monitor_locate_last_operation():
                    """Monitor locate last operation process and report completion status."""
                    try:
                        process = subprocess.Popen(
                            cmd,
                            cwd=os.path.dirname(script_path),
                            stdout=subprocess.PIPE,
                            stderr=subprocess.STDOUT,
                            text=True,
                            bufsize=1
                        )
                        
                        # Track this process for cleanup
                        with self.process_lock:
                            self.child_processes.append(process)
                        
                        self.get_logger().info(f"Started locate last operation script with PID: {process.pid}")
                        
                        # Read and log output in real-time
                        for line in process.stdout:
                            line = line.rstrip()
                            if line:
                                self.get_logger().info(f"[ur_3d_positioning] {line}")
                                # Also push important messages to web log
                                if '✓' in line or '✗' in line or 'Step' in line or 'Error' in line:
                                    if '✗' in line or 'Error' in line or 'failed' in line.lower():
                                        self.push_web_log(line, 'error')
                                    elif '✓' in line:
                                        self.push_web_log(line, 'success')
                                    else:
                                        self.push_web_log(line, 'info')
                        
                        # Wait for the process to complete
                        return_code = process.wait()
                        
                        self.get_logger().info(f"Locate last operation script finished with return code: {return_code}")
                        
                        # Push completion message to web log
                        if return_code == 0:
                            self.push_web_log(f"✅ Locate last operation '{operation_name}' completed successfully!", 'success')
                        else:
                            self.push_web_log(f"❌ Locate last operation '{operation_name}' failed with return code: {return_code}", 'error')
                    except Exception as e:
                        self.get_logger().error(f"Error in locate last operation monitor thread: {e}")
                        self.push_web_log(f"❌ Locate last operation error: {str(e)}", 'error')
                
                # Start monitoring thread
                monitor_thread = threading.Thread(target=monitor_locate_last_operation, daemon=True)
                monitor_thread.start()
                
                self.get_logger().info(f"Started locate last operation monitoring thread")
                
                # Push start message to web log
                self.push_web_log(f"🎯 Starting locate last operation '{operation_name}'...", 'info')
                
                return jsonify({
                    'success': True,
                    'message': f'Locate last operation script started for: {operation_name}'
                })
                
            except Exception as e:
                self.get_logger().error(f"Error starting locate last operation script: {e}")
                self.push_web_log(f"❌ Failed to start locate last operation: {str(e)}", 'error')
                return jsonify({
                    'success': False,
                    'message': str(e)
                })
        
        @self.app.route('/locate_unlock_knob', methods=['POST'])
        def locate_unlock_knob():
            """Execute locate unlock knob script (ur_locate.py --operation-name unlock_knob)."""
            from flask import jsonify
            import subprocess
            import threading
            
            try:
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    return jsonify({'success': False, 'message': 'Could not find scripts directory'})
                
                script_path = os.path.join(scripts_dir, 'ur_locate.py')
                if not os.path.exists(script_path):
                    return jsonify({'success': False, 'message': f'Script not found: {script_path}'})
                
                cmd = ['python3', script_path, '--operation-name', 'unlock_knob']
                self.get_logger().info(f"Locate unlock knob command: {' '.join(cmd)}")
                
                def monitor_process():
                    try:
                        process = subprocess.Popen(cmd, cwd=os.path.dirname(script_path),
                                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                        with self.process_lock:
                            self.child_processes.append(process)
                        
                        self.get_logger().info(f"Started locate unlock knob script with PID: {process.pid}")
                        return_code = process.wait()
                        self.get_logger().info(f"Locate unlock knob script finished with return code: {return_code}")
                        
                        if return_code == 0:
                            self.push_web_log("✅ Locate unlock knob completed successfully!", 'success')
                        else:
                            self.push_web_log(f"❌ Locate unlock knob failed with return code: {return_code}", 'error')
                    except Exception as e:
                        self.get_logger().error(f"Error in locate unlock knob monitor thread: {e}")
                        self.push_web_log(f"❌ Locate unlock knob error: {str(e)}", 'error')
                
                monitor_thread = threading.Thread(target=monitor_process, daemon=True)
                monitor_thread.start()
                self.push_web_log("🔓 Starting locate unlock knob process...", 'info')
                
                return jsonify({'success': True, 'message': 'Locate unlock knob script started'})
            except Exception as e:
                self.get_logger().error(f"Error starting locate unlock knob script: {e}")
                self.push_web_log(f"❌ Failed to start locate unlock knob: {str(e)}", 'error')
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/locate_open_handle', methods=['POST'])
        def locate_open_handle():
            """Execute locate open handle script (ur_locate.py --operation-name open_handle)."""
            from flask import jsonify
            import subprocess
            import threading
            
            try:
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    return jsonify({'success': False, 'message': 'Could not find scripts directory'})
                
                script_path = os.path.join(scripts_dir, 'ur_locate.py')
                if not os.path.exists(script_path):
                    return jsonify({'success': False, 'message': f'Script not found: {script_path}'})
                
                cmd = ['python3', script_path, '--operation-name', 'open_handle']
                self.get_logger().info(f"Locate open handle command: {' '.join(cmd)}")
                
                def monitor_process():
                    try:
                        process = subprocess.Popen(cmd, cwd=os.path.dirname(script_path),
                                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                        with self.process_lock:
                            self.child_processes.append(process)
                        
                        self.get_logger().info(f"Started locate open handle script with PID: {process.pid}")
                        return_code = process.wait()
                        self.get_logger().info(f"Locate open handle script finished with return code: {return_code}")
                        
                        if return_code == 0:
                            self.push_web_log("✅ Locate open handle completed successfully!", 'success')
                        else:
                            self.push_web_log(f"❌ Locate open handle failed with return code: {return_code}", 'error')
                    except Exception as e:
                        self.get_logger().error(f"Error in locate open handle monitor thread: {e}")
                        self.push_web_log(f"❌ Locate open handle error: {str(e)}", 'error')
                
                monitor_thread = threading.Thread(target=monitor_process, daemon=True)
                monitor_thread.start()
                self.push_web_log("🕹️ Starting locate open handle process...", 'info')
                
                return jsonify({'success': True, 'message': 'Locate open handle script started'})
            except Exception as e:
                self.get_logger().error(f"Error starting locate open handle script: {e}")
                self.push_web_log(f"❌ Failed to start locate open handle: {str(e)}", 'error')
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/locate_close_left', methods=['POST'])
        def locate_close_left():
            """Execute locate close left script (ur_locate.py --operation-name close_left)."""
            from flask import jsonify
            import subprocess
            import threading
            
            try:
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    return jsonify({'success': False, 'message': 'Could not find scripts directory'})
                
                script_path = os.path.join(scripts_dir, 'ur_locate.py')
                if not os.path.exists(script_path):
                    return jsonify({'success': False, 'message': f'Script not found: {script_path}'})
                
                cmd = ['python3', script_path, '--operation-name', 'close_left']
                self.get_logger().info(f"Locate close left command: {' '.join(cmd)}")
                
                def monitor_process():
                    try:
                        process = subprocess.Popen(cmd, cwd=os.path.dirname(script_path),
                                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                        with self.process_lock:
                            self.child_processes.append(process)
                        
                        self.get_logger().info(f"Started locate close left script with PID: {process.pid}")
                        return_code = process.wait()
                        self.get_logger().info(f"Locate close left script finished with return code: {return_code}")
                        
                        if return_code == 0:
                            self.push_web_log("✅ Locate close left completed successfully!", 'success')
                        else:
                            self.push_web_log(f"❌ Locate close left failed with return code: {return_code}", 'error')
                    except Exception as e:
                        self.get_logger().error(f"Error in locate close left monitor thread: {e}")
                        self.push_web_log(f"❌ Locate close left error: {str(e)}", 'error')
                
                monitor_thread = threading.Thread(target=monitor_process, daemon=True)
                monitor_thread.start()
                self.push_web_log("⬅️ Starting locate close left process...", 'info')
                
                return jsonify({'success': True, 'message': 'Locate close left script started'})
            except Exception as e:
                self.get_logger().error(f"Error starting locate close left script: {e}")
                self.push_web_log(f"❌ Failed to start locate close left: {str(e)}", 'error')
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/locate_close_right', methods=['POST'])
        def locate_close_right():
            """Execute locate close right script (ur_locate.py --operation-name close_right)."""
            from flask import jsonify
            import subprocess
            import threading
            
            try:
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    return jsonify({'success': False, 'message': 'Could not find scripts directory'})
                
                script_path = os.path.join(scripts_dir, 'ur_locate.py')
                if not os.path.exists(script_path):
                    return jsonify({'success': False, 'message': f'Script not found: {script_path}'})
                
                cmd = ['python3', script_path, '--operation-name', 'close_right']
                self.get_logger().info(f"Locate close right command: {' '.join(cmd)}")
                
                def monitor_process():
                    try:
                        process = subprocess.Popen(cmd, cwd=os.path.dirname(script_path),
                                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                        with self.process_lock:
                            self.child_processes.append(process)
                        
                        self.get_logger().info(f"Started locate close right script with PID: {process.pid}")
                        return_code = process.wait()
                        self.get_logger().info(f"Locate close right script finished with return code: {return_code}")
                        
                        if return_code == 0:
                            self.push_web_log("✅ Locate close right completed successfully!", 'success')
                        else:
                            self.push_web_log(f"❌ Locate close right failed with return code: {return_code}", 'error')
                    except Exception as e:
                        self.get_logger().error(f"Error in locate close right monitor thread: {e}")
                        self.push_web_log(f"❌ Locate close right error: {str(e)}", 'error')
                
                monitor_thread = threading.Thread(target=monitor_process, daemon=True)
                monitor_thread.start()
                self.push_web_log("➡️ Starting locate close right process...", 'info')
                
                return jsonify({'success': True, 'message': 'Locate close right script started'})
            except Exception as e:
                self.get_logger().error(f"Error starting locate close right script: {e}")
                self.push_web_log(f"❌ Failed to start locate close right: {str(e)}", 'error')
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/execute_unlock_knob', methods=['POST'])
        def execute_unlock_knob():
            """Execute unlock knob script (ur_op_unlock_knob.py)."""
            from flask import jsonify
            import subprocess
            import threading
            
            try:
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    return jsonify({'success': False, 'message': 'Could not find scripts directory'})
                
                script_path = os.path.join(scripts_dir, 'ur_op_unlock_knob.py')
                if not os.path.exists(script_path):
                    return jsonify({'success': False, 'message': f'Script not found: {script_path}'})
                
                cmd = ['python3', script_path]
                self.get_logger().info(f"Execute unlock knob command: {' '.join(cmd)}")
                
                def monitor_process():
                    try:
                        process = subprocess.Popen(cmd, cwd=os.path.dirname(script_path),
                                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                        with self.process_lock:
                            self.child_processes.append(process)
                        
                        self.get_logger().info(f"Started execute unlock knob script with PID: {process.pid}")
                        return_code = process.wait()
                        self.get_logger().info(f"Execute unlock knob script finished with return code: {return_code}")
                        
                        if return_code == 0:
                            self.push_web_log("✅ Execute unlock knob completed successfully!", 'success')
                        else:
                            self.push_web_log(f"❌ Execute unlock knob failed with return code: {return_code}", 'error')
                    except Exception as e:
                        self.get_logger().error(f"Error in execute unlock knob monitor thread: {e}")
                        self.push_web_log(f"❌ Execute unlock knob error: {str(e)}", 'error')
                
                monitor_thread = threading.Thread(target=monitor_process, daemon=True)
                monitor_thread.start()
                self.push_web_log("🔧 Starting execute unlock knob process...", 'info')
                
                return jsonify({'success': True, 'message': 'Execute unlock knob script started'})
            except Exception as e:
                self.get_logger().error(f"Error starting execute unlock knob script: {e}")
                self.push_web_log(f"❌ Failed to start execute unlock knob: {str(e)}", 'error')
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/execute_open_handle', methods=['POST'])
        def execute_open_handle():
            """Execute open handle script (ur_op_open_handle.py)."""
            from flask import jsonify
            import subprocess
            import threading
            
            try:
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    return jsonify({'success': False, 'message': 'Could not find scripts directory'})
                
                script_path = os.path.join(scripts_dir, 'ur_op_open_handle.py')
                if not os.path.exists(script_path):
                    return jsonify({'success': False, 'message': f'Script not found: {script_path}'})
                
                cmd = ['python3', script_path]
                self.get_logger().info(f"Execute open handle command: {' '.join(cmd)}")
                
                def monitor_process():
                    try:
                        process = subprocess.Popen(cmd, cwd=os.path.dirname(script_path),
                                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                        with self.process_lock:
                            self.child_processes.append(process)
                        
                        self.get_logger().info(f"Started execute open handle script with PID: {process.pid}")
                        return_code = process.wait()
                        self.get_logger().info(f"Execute open handle script finished with return code: {return_code}")
                        
                        if return_code == 0:
                            self.push_web_log("✅ Execute open handle completed successfully!", 'success')
                        else:
                            self.push_web_log(f"❌ Execute open handle failed with return code: {return_code}", 'error')
                    except Exception as e:
                        self.get_logger().error(f"Error in execute open handle monitor thread: {e}")
                        self.push_web_log(f"❌ Execute open handle error: {str(e)}", 'error')
                
                monitor_thread = threading.Thread(target=monitor_process, daemon=True)
                monitor_thread.start()
                self.push_web_log("👜 Starting execute open handle process...", 'info')
                
                return jsonify({'success': True, 'message': 'Execute open handle script started'})
            except Exception as e:
                self.get_logger().error(f"Error starting execute open handle script: {e}")
                self.push_web_log(f"❌ Failed to start execute open handle: {str(e)}", 'error')
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/execute_close_left', methods=['POST'])
        def execute_close_left():
            """Execute close left script (ur_op_close_left.py)."""
            from flask import jsonify
            import subprocess
            import threading
            
            try:
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    return jsonify({'success': False, 'message': 'Could not find scripts directory'})
                
                script_path = os.path.join(scripts_dir, 'ur_op_close_left.py')
                if not os.path.exists(script_path):
                    return jsonify({'success': False, 'message': f'Script not found: {script_path}'})
                
                cmd = ['python3', script_path]
                self.get_logger().info(f"Execute close left command: {' '.join(cmd)}")
                
                def monitor_process():
                    try:
                        process = subprocess.Popen(cmd, cwd=os.path.dirname(script_path),
                                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                        with self.process_lock:
                            self.child_processes.append(process)
                        
                        self.get_logger().info(f"Started execute close left script with PID: {process.pid}")
                        return_code = process.wait()
                        self.get_logger().info(f"Execute close left script finished with return code: {return_code}")
                        
                        if return_code == 0:
                            self.push_web_log("✅ Execute close left completed successfully!", 'success')
                        else:
                            self.push_web_log(f"❌ Execute close left failed with return code: {return_code}", 'error')
                    except Exception as e:
                        self.get_logger().error(f"Error in execute close left monitor thread: {e}")
                        self.push_web_log(f"❌ Execute close left error: {str(e)}", 'error')
                
                monitor_thread = threading.Thread(target=monitor_process, daemon=True)
                monitor_thread.start()
                self.push_web_log("◀️ Starting execute close left process...", 'info')
                
                return jsonify({'success': True, 'message': 'Execute close left script started'})
            except Exception as e:
                self.get_logger().error(f"Error starting execute close left script: {e}")
                self.push_web_log(f"❌ Failed to start execute close left: {str(e)}", 'error')
                return jsonify({'success': False, 'message': str(e)})
        
        @self.app.route('/execute_close_right', methods=['POST'])
        def execute_close_right():
            """Execute close right script (ur_op_close_right.py)."""
            from flask import jsonify
            import subprocess
            import threading
            
            try:
                scripts_dir = get_scripts_directory()
                if scripts_dir is None:
                    return jsonify({'success': False, 'message': 'Could not find scripts directory'})
                
                script_path = os.path.join(scripts_dir, 'ur_op_close_right.py')
                if not os.path.exists(script_path):
                    return jsonify({'success': False, 'message': f'Script not found: {script_path}'})
                
                cmd = ['python3', script_path]
                self.get_logger().info(f"Execute close right command: {' '.join(cmd)}")
                
                def monitor_process():
                    try:
                        process = subprocess.Popen(cmd, cwd=os.path.dirname(script_path),
                                                  stdout=subprocess.PIPE, stderr=subprocess.STDOUT, text=True)
                        with self.process_lock:
                            self.child_processes.append(process)
                        
                        self.get_logger().info(f"Started execute close right script with PID: {process.pid}")
                        return_code = process.wait()
                        self.get_logger().info(f"Execute close right script finished with return code: {return_code}")
                        
                        if return_code == 0:
                            self.push_web_log("✅ Execute close right completed successfully!", 'success')
                        else:
                            self.push_web_log(f"❌ Execute close right failed with return code: {return_code}", 'error')
                    except Exception as e:
                        self.get_logger().error(f"Error in execute close right monitor thread: {e}")
                        self.push_web_log(f"❌ Execute close right error: {str(e)}", 'error')
                
                monitor_thread = threading.Thread(target=monitor_process, daemon=True)
                monitor_thread.start()
                self.push_web_log("▶️ Starting execute close right process...", 'info')
                
                return jsonify({'success': True, 'message': 'Execute close right script started'})
            except Exception as e:
                self.get_logger().error(f"Error starting execute close right script: {e}")
                self.push_web_log(f"❌ Failed to start execute close right: {str(e)}", 'error')
                return jsonify({'success': False, 'message': str(e)})
        
        # Asset route must come BEFORE the main report route to avoid conflicts
        @self.app.route('/calibration_report/<report_type>/<path:filename>')
        def serve_calibration_assets(report_type, filename):
            """Serve images and other assets from calibration report directories."""
            from flask import send_from_directory, jsonify
            from common.workspace_utils import get_temp_directory
            
            try:
                if report_type == 'eye_in_hand':
                    report_dir = os.path.join(self.calibration_result_dir, 
                                            'ur15_eye_in_hand_calibration_report')
                elif report_type == 'intrinsic':
                    report_dir = os.path.join(self.calibration_result_dir, 
                                            'ur15_intrinsic_calibration_report')
                else:
                    return jsonify({'error': 'Invalid report type'}), 404
                
                if not os.path.exists(report_dir):
                    return jsonify({'error': 'Report directory not found'}), 404
                
                self.get_logger().info(f"Serving calibration asset: {filename} from {report_dir}")
                return send_from_directory(report_dir, filename)
                
            except Exception as e:
                self.get_logger().error(f"Error serving calibration asset: {e}")
                return jsonify({'error': str(e)}), 500
        
        @self.app.route('/calibration_report/<report_type>')
        def serve_calibration_report(report_type):
            """Serve calibration report HTML with base tag injection."""
            from flask import jsonify, Response
            from common.workspace_utils import get_temp_directory
            
            try:
                if report_type == 'eye_in_hand':
                    report_path = os.path.join(self.calibration_result_dir, 
                                             'ur15_eye_in_hand_calibration_report', 
                                             'calibration_report.html')
                elif report_type == 'intrinsic':
                    report_path = os.path.join(self.calibration_result_dir,
                                             'ur15_intrinsic_calibration_report', 
                                             'calibration_report.html')
                else:
                    return jsonify({'error': 'Invalid report type'}), 404
                
                if not os.path.exists(report_path):
                    self.get_logger().warn(f"Calibration report not found: {report_path}")
                    return jsonify({'error': 'Calibration report not found'}), 404
                
                self.get_logger().info(f"Serving calibration report: {report_path}")
                
                # Read the HTML and inject a base tag to fix relative paths
                with open(report_path, 'r', encoding='utf-8') as f:
                    html_content = f.read()
                
                # Inject base tag after <head> to make relative URLs work
                base_tag = f'<base href="/calibration_report/{report_type}/">'
                if '<head>' in html_content:
                    html_content = html_content.replace('<head>', f'<head>\n    {base_tag}', 1)
                
                return Response(html_content, mimetype='text/html')
                
            except Exception as e:
                self.get_logger().error(f"Error serving calibration report: {e}")
                return jsonify({'error': str(e)}), 500
    
    def _get_camera_calib_params(self):
        """
        Get camera intrinsic, distortion, and extrinsic matrices.
        
        Returns:
            dict: Dictionary with keys 'intrinsic', 'distortion', 'extrinsic', 'cam2end_matrix', 'end2base_matrix'
                  Returns None if calibration parameters are not available or TCP pose is missing
        """
        try:
            # Check if calibration parameters are available
            with self.calibration_lock:
                if self.camera_matrix is None or self.distortion_coefficients is None:
                    return None
                if self.cam2end_matrix is None:
                    return None
                camera_matrix = self.camera_matrix.copy()
                distortion_coefficients = self.distortion_coefficients.copy()
                cam2end_matrix = self.cam2end_matrix.copy()
            
            # Get current TCP pose
            with self.robot_data_lock:
                if self.tcp_pose is None:
                    return None
                x = self.tcp_pose['x'] / 1000.0  # Convert mm to m
                y = self.tcp_pose['y'] / 1000.0
                z = self.tcp_pose['z'] / 1000.0
                qx = self.tcp_pose['qx']
                qy = self.tcp_pose['qy']
                qz = self.tcp_pose['qz']
                qw = self.tcp_pose['qw']
            
            # Convert quaternion to rotation matrix
            R = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
            ])
            
            # Build end2base transformation matrix
            end2base_matrix = np.eye(4)
            end2base_matrix[:3, :3] = R
            end2base_matrix[:3, 3] = [x, y, z]
            
            # Calculate camera to base transformation
            cam2base_matrix = end2base_matrix @ cam2end_matrix
            
            # Calculate extrinsic matrix (base to camera)
            base2cam_matrix = np.linalg.inv(cam2base_matrix)
            
            return {
                'intrinsic': camera_matrix,
                'distortion': distortion_coefficients,
                'extrinsic': base2cam_matrix,
                'cam2end_matrix': cam2end_matrix,
                'end2base_matrix': end2base_matrix
            }
            
        except Exception as e:
            self.get_logger().error(f"Error getting camera extrinsic parameters: {e}")
            return None
    
    def project_base_origin_to_image(self, frame):
        """Project base coordinate system and curves to image using draw_curves_on_image."""
        try:
            # Get camera parameters using helper function
            params = self._get_camera_calib_params()
            if params is None:
                return frame
            
            # Draw UR15 base model
            frame = draw_utils.draw_model_on_image(
                frame,
                intrinsic=params['intrinsic'],
                extrinsic=params['extrinsic'],
                model=self.ur15_base_model,
                distortion=params['distortion'],
                thickness=2
            )
            
        except Exception as e:
            self.get_logger().error(f"Error projecting curves to image: {e}")
        
        return frame
    
    def project_rack_to_image(self, frame):
        """Project GB200 rack to image using draw_curves_on_image."""
        try:
            # Get camera parameters using helper function
            params = self._get_camera_calib_params()
            if params is None:
                self.get_logger().warn("Camera calibration parameters not available for rack drawing")
                return frame
            
            # Get rack work object parameters - client handles caching internally
            try:
                rack2base_matrix = self.status_client.get_status('ur15', 'rack2base_matrix')
            except Exception as e:
                self.get_logger().debug(f"Failed to get rack2base_matrix: {e}")
                return frame
            
            if rack2base_matrix is None:
                self.get_logger().debug("rack2base_matrix is None, skipping rack drawing")
                return frame
                        
            # Calculate rack2camera transformation
            # base2camera = params['extrinsic']
            # rack2camera = base2camera @ rack2base
            base2camera = params['extrinsic']
            rack2camera = base2camera @ rack2base_matrix
            
            # Draw GB200 rack model with rack2camera extrinsic
            frame = draw_utils.draw_model_on_image(
                frame,
                intrinsic=params['intrinsic'],
                extrinsic=rack2camera,
                model=self.gb200rack_model,
                distortion=params['distortion'],
                thickness=2
            )

            # Calculate server2camera transformation for GB200 server
            # Get Operating Unit value from robot_status
            try:
                operating_unit_id = self.status_client.get_status('ur15', 'rack_operating_unit_id')
            except Exception as e:
                self.get_logger().debug(f"Failed to get rack_operating_unit_id: {e}")
                operating_unit_id = None
            
            if operating_unit_id is None:
                operating_unit_id = 14  # Default to 14 if not set
                self.get_logger().debug(f"rack_operating_unit_id not found, using default: {operating_unit_id}")
            
            # Generate server frame in rack coordinate system using class instance
            if self.server_frame_generator is None:
                self.get_logger().warn("Server frame generator not initialized")
                return frame
            
            result = self.server_frame_generator.generate_server_frame_in_rack(index=operating_unit_id)
            
            if result is not None:
                # Get server2rack transformation matrix
                server2rack_matrix = result['target_server_transformation_matrix_in_rack']
                
                # Calculate server2camera transformation
                # server2camera = base2camera @ rack2base @ server2rack
                server2camera = base2camera @ rack2base_matrix @ server2rack_matrix
                
                # Draw GB200 server model with server2camera extrinsic
                frame = draw_utils.draw_model_on_image(
                    frame,
                    intrinsic=params['intrinsic'],
                    extrinsic=server2camera,
                    model=self.gb200server_model,
                    distortion=params['distortion'],
                    thickness=2
                )
            else:
                self.get_logger().warn(f"Failed to generate server frame for unit {operating_unit_id}")


        except Exception as e:
            self.get_logger().error(f"Error projecting rack to image: {e}")
        
        return frame
    
    def draw_keypoints_on_image(self, frame):
        """Draw 3D keypoints from robot_status on image using draw_utils."""
        try:
            # Get camera parameters using helper function
            params = self._get_camera_calib_params()
            if params is None:
                return frame
            
            # Get 3D points from robot_status - client handles caching internally
            try:
                points_3d = self.status_client.get_status('ur15', 'last_points_3d')
            except Exception as e:
                self.get_logger().debug(f"Failed to get last_points_3d: {e}")
                return frame
            
            # Check if points are available
            if points_3d is None:
                return frame
            
            # Convert to numpy array
            points_3d = np.array(points_3d, dtype=np.float64)
            
            if len(points_3d) == 0:
                return frame
            
            # Draw keypoints using draw_utils function
            frame = draw_utils.draw_keypoints_on_image(
                frame,
                intrinsic=params['intrinsic'],
                extrinsic=params['extrinsic'],
                points_3d=points_3d,
                distortion=params['distortion'],
                radius=6,
                color=(0, 0, 255),  # Red
                thickness=2,
                draw_labels=True,
                label_color=(255, 255, 255)
            )
            
            # Draw info text
            info_text = f"Keypoints: {len(points_3d)}"
            cv2.putText(frame, info_text, (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
            
        except Exception as e:
            self.get_logger().error(f"Error drawing keypoints on image: {e}")
        
        return frame
    
    def apply_corner_detection(self, frame):
        """Apply corner detection visualization to frame using calibration toolkit."""
        if not CALIBRATION_TOOLKIT_AVAILABLE:
            cv2.putText(frame, 'Calibration toolkit not available', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            return frame
        
        try:
            # Get the pattern JSON configuration
            pattern_json = self.corner_detection_params.get('pattern_json', None)
            
            if pattern_json is None:
                cv2.putText(frame, 'No pattern configuration', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
                return frame
            
            # Create or reuse pattern instance from JSON
            if not hasattr(self, '_current_pattern') or \
               getattr(self, '_current_pattern_json', None) != pattern_json:
                
                self._current_pattern = create_pattern_from_json(pattern_json)
                self._current_pattern_json = pattern_json
                
                pattern_id = pattern_json.get('pattern_id', 'unknown')
                params = pattern_json.get('parameters', {})
                self.get_logger().info(f"Created pattern from JSON: {pattern_id}, params: {params}")
            
            # Detect corners using toolkit
            ret, corners, point_ids = self._current_pattern.detect_corners(frame)
            
            if ret and corners is not None:
                # Draw corners using toolkit
                frame = self._current_pattern.draw_corners(frame, corners, point_ids)
                
                # Add status text
                pattern_id = pattern_json.get('pattern_id', 'Unknown')
                pattern_name = pattern_json.get('name', pattern_id)
                corner_count = len(corners) if corners is not None else 0
                cv2.putText(frame, f'{pattern_name}: {corner_count} points FOUND', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 0), 2)
                
                if pattern_id == 'charuco_board' and point_ids is not None:
                    cv2.putText(frame, f'ChArUco IDs: {len(point_ids)}', (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
                elif pattern_id == 'grid_board' and point_ids is not None:
                    cv2.putText(frame, f'ArUco Markers: {len(point_ids)//4}', (10, 60),
                               cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 255), 2)
            else:
                pattern_name = pattern_json.get('name', 'Pattern')
                cv2.putText(frame, f'{pattern_name}: NOT FOUND', (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
                           
        except Exception as e:
            self.get_logger().error(f'Error in toolkit corner detection: {e}')
            cv2.putText(frame, f'Detection error: {str(e)[:40]}', (10, 30),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        return frame
    
    def generate_frames(self):
        """Generate video frames for streaming."""
        while True:
            try:
                with self.image_lock:
                    if self.current_image is not None:
                        frame = self.current_image.copy()
                        
                        # Apply corner detection if enabled
                        if self.corner_detection_enabled:
                            frame = self.apply_corner_detection(frame)
                        
                        # Draw UR15 base if enabled
                        if self.validation_active:
                            frame = self.project_base_origin_to_image(frame)
                        
                        # Draw GB200 rack if enabled
                        if self.draw_rack_enabled:
                            frame = self.project_rack_to_image(frame)
                        
                        # Draw keypoints if enabled
                        if self.draw_keypoints_enabled:
                            frame = self.draw_keypoints_on_image(frame)
                        
                        # Scale down for web display while maintaining aspect ratio
                        height, width = frame.shape[:2]
                        max_display_width = 800  # Maximum width for web display
                        
                        if width > max_display_width:
                            scale = max_display_width / width
                            new_width = int(width * scale)
                            new_height = int(height * scale)
                            frame = cv2.resize(frame, (new_width, new_height), interpolation=cv2.INTER_LANCZOS4)
                        
                        encode_param = [cv2.IMWRITE_JPEG_QUALITY, 95]
                        _, buffer = cv2.imencode('.jpg', frame, encode_param)
                        frame_bytes = buffer.tobytes()
                        
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    else:
                        placeholder = self.create_placeholder_image()
                        _, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 95])
                        frame_bytes = buffer.tobytes()
                        
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                
                threading.Event().wait(0.1)
                
            except Exception as e:
                self.get_logger().error(f"Error in video stream: {e}")
                error_img = self.create_error_image(str(e))
                _, buffer = cv2.imencode('.jpg', error_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                frame_bytes = buffer.tobytes()
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                
                threading.Event().wait(1.0)
    
    def create_placeholder_image(self):
        """Create a placeholder image when no camera feed is available."""
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img.fill(50)
        
        text_lines = [
            "Waiting for camera data...",
            f"Topic: {self.camera_topic}",
            "Please check if camera is running"
        ]
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.7
        color = (255, 255, 255)
        thickness = 2
        
        y_start = 150
        for i, line in enumerate(text_lines):
            text_size = cv2.getTextSize(line, font, font_scale, thickness)[0]
            x = (img.shape[1] - text_size[0]) // 2
            y = y_start + i * 40
            cv2.putText(img, line, (x, y), font, font_scale, color, thickness)
        
        return img
    
    def create_error_image(self, error_msg):
        """Create an error image."""
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img.fill(50)
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        cv2.putText(img, "Error:", (50, 200), font, 1, (0, 0, 255), 2)
        
        words = error_msg.split()
        lines = []
        current_line = ""
        for word in words:
            test_line = current_line + " " + word if current_line else word
            text_size = cv2.getTextSize(test_line, font, 0.5, 1)[0]
            if text_size[0] < 540:
                current_line = test_line
            else:
                if current_line:
                    lines.append(current_line)
                current_line = word
        if current_line:
            lines.append(current_line)
        
        y = 250
        for line in lines[:5]:
            cv2.putText(img, line, (50, y), font, 0.5, (255, 255, 255), 1)
            y += 30
        
        return img

    def _get_camera_parameters_from_status(self, calibration_data_dir):
        """Get camera parameters from robot_status (ur15 namespace)."""
        try:
            # Read camera parameters from robot_status
            try:
                camera_matrix = self.status_client.get_status('ur15', 'camera_matrix')
            except Exception as e:
                self.get_logger().debug(f"Failed to get camera_matrix: {e}")
                camera_matrix = None
            
            try:
                distortion_coefficients = self.status_client.get_status('ur15', 'distortion_coefficients')
            except Exception as e:
                self.get_logger().debug(f"Failed to get distortion_coefficients: {e}")
                distortion_coefficients = None
            
            try:
                cam2end_matrix = self.status_client.get_status('ur15', 'cam2end_matrix')
            except Exception as e:
                self.get_logger().debug(f"Failed to get cam2end_matrix: {e}")
                cam2end_matrix = None
            
            self.get_logger().info(f"Reading calibration parameters from robot_status (ur15 namespace)")
            
            # Check if we have required parameters
            if camera_matrix is None or distortion_coefficients is None:
                return {
                    'success': False,
                    'message': 'Camera intrinsic parameters (camera_matrix, distortion_coefficients) not found in robot_status. Please run calibration first.'
                }
            
            # Convert numpy arrays to lists for JSON serialization
            if isinstance(camera_matrix, np.ndarray):
                camera_matrix = camera_matrix.tolist()
            if isinstance(distortion_coefficients, np.ndarray):
                distortion_coefficients = distortion_coefficients.tolist()
            
            # Build intrinsics dictionary
            intrinsics = {
                'camera_matrix': camera_matrix,
                'distortion_coefficients': distortion_coefficients
            }
            self.get_logger().info("✅ Successfully loaded intrinsic parameters from robot_status")
            
            # Build camera parameters dictionary
            camera_params = {
                'intrinsics': intrinsics
            }
            
            # Add extrinsics if available
            if cam2end_matrix is not None:
                # Convert to list if it's a numpy array
                if isinstance(cam2end_matrix, np.ndarray):
                    cam2end_matrix = cam2end_matrix.tolist()
                
                extrinsics = {
                    'cam2end_matrix': cam2end_matrix
                }
                camera_params['extrinsics'] = extrinsics
                self.get_logger().info("✅ Successfully loaded cam2end_matrix from robot_status")
            else:
                self.get_logger().warning("cam2end_matrix not found in robot_status")
            
            return {
                'success': True, 
                'data': camera_params,
                'source': 'robot_status (ur15 namespace)'
            }
            
        except Exception as e:
            return {
                'success': False,
                'message': f'Error reading calibration parameters from robot_status: {str(e)}'
            }


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    node = None
    
    try:
        node = UR15WebNode()
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\nShutting down...")
    except Exception as e:
        print(f"Error: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if node is not None:
            print("Cleaning up child processes...")
            node.cleanup()
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()