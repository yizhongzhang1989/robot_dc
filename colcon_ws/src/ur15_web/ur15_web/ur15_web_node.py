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
from flask import Flask, render_template_string, Response
from ament_index_python.packages import get_package_share_directory

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


def find_available_port(start_port=8030, max_attempts=10):
    """Find an available port starting from start_port."""
    for port in range(start_port, start_port + max_attempts):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(('localhost', port))
                return port
        except OSError:
            continue
    return None


class UR15WebNode(Node):
    """ROS2 node for UR15 web interface with camera calibration validation."""
    
    def __init__(self):
        super().__init__('ur15_web_node')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/ur15_camera/image_raw')
        self.declare_parameter('web_port', 8030)
        self.declare_parameter('ur15_ip', '192.168.1.15')
        self.declare_parameter('ur15_port', 30002)
        self.declare_parameter('data_dir', '/tmp/dataset')
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        web_port = self.get_parameter('web_port').value
        self.ur15_ip = self.get_parameter('ur15_ip').value
        self.ur15_port = self.get_parameter('ur15_port').value
        self.data_dir = self.get_parameter('data_dir').value
        
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
        self.ur15_lock = threading.Lock()
        self._init_ur15_connection()
        
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
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir, exist_ok=True)
            return 0
        
        existing_files = [f for f in os.listdir(self.data_dir) if f.endswith('.json')]
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
        
    def _run_flask_server(self):
        """Run Flask server with proper error handling."""
        try:
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
                html_content = html_content.replace('{{ data_dir }}', self.data_dir)
                
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
            
            # Debug log for camera status
            if hasattr(self, '_debug_counter'):
                self._debug_counter += 1
            else:
                self._debug_counter = 1
            
            if self._debug_counter % 20 == 0:  # Log every 10 seconds (20 * 500ms)
                self.get_logger().info(f"Camera status check: has_image={has_image}, current_image type: {type(self.current_image)}")
            
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
            
            return jsonify({
                'has_image': has_image,
                'camera_topic': self.camera_topic,
                'data_dir': self.data_dir,
                'has_intrinsic': has_intrinsic,
                'has_extrinsic': has_extrinsic,
                'joint_positions': joint_positions if joint_data_valid else [],
                'tcp_pose': tcp_pose if tcp_data_valid else None,
                'freedrive_active': freedrive_active,
                'robot_connected': robot_connected
            })
        
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
                    self.data_dir = new_dir
                    self.get_logger().info(f"Dataset directory changed to: {new_dir}")
                    return jsonify({
                        'success': True, 
                        'message': 'Directory changed successfully',
                        'data_dir': new_dir
                    })
                except Exception as e:
                    return jsonify({
                        'success': False, 
                        'message': f'Failed to create/access directory: {str(e)}'
                    })
                
            except Exception as e:
                self.get_logger().error(f"Error changing data directory: {e}")
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
        
        @self.app.route('/take_screenshot', methods=['POST'])
        def take_screenshot():
            """Take a screenshot of current camera feed and save robot pose."""
            from flask import jsonify
            from datetime import datetime
            import json
            
            try:
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
                
                # Find next available file number
                counter = self._get_next_file_number()
                
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
                json_filename = os.path.join(self.data_dir, f"{counter}.json")
                with open(json_filename, 'w') as f:
                    json.dump(data, f, indent=2)
                
                # Save image file
                image_filename = os.path.join(self.data_dir, f"{counter}.jpg")
                cv2.imwrite(image_filename, cv_image)
                
                self.get_logger().info(f"Screenshot saved: {counter}.jpg and {counter}.json")
                
                return jsonify({
                    'success': True,
                    'message': 'Screenshot saved successfully',
                    'filename': f"{counter}.jpg"
                })
                
            except Exception as e:
                self.get_logger().error(f"Error taking screenshot: {e}")
                return jsonify({
                    'success': False,
                    'message': str(e)
                })
    
    def project_base_origin_to_image(self, frame):
        """Project base coordinate system origin to image and draw it."""
        try:
            with self.calibration_lock:
                if self.camera_matrix is None or self.distortion_coefficients is None:
                    return frame
                if self.cam2end_matrix is None or self.target2base_matrix is None:
                    return frame
                camera_matrix = self.camera_matrix.copy()
                distortion_coefficients = self.distortion_coefficients.copy()
                cam2end_matrix = self.cam2end_matrix.copy()
                target2base_matrix = self.target2base_matrix.copy()
            
            with self.robot_data_lock:
                if self.tcp_pose is None:
                    return frame
                x = self.tcp_pose['x'] / 1000.0
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
            
            end2base_matrix = np.eye(4)
            end2base_matrix[:3, :3] = R
            end2base_matrix[:3, 3] = [x, y, z]
            
            cam2base_matrix = end2base_matrix @ cam2end_matrix
            base_origin_base = np.array([0.0, 0.0, 0.0, 1.0])
            base2cam_matrix = np.linalg.inv(cam2base_matrix)
            base_origin_cam = base2cam_matrix @ base_origin_base
            point_3d = base_origin_cam[:3]
            
            if point_3d[2] <= 0:
                return frame
            
            point_3d_reshaped = point_3d.reshape(1, 1, 3)
            rvec = np.zeros((3, 1))
            tvec = np.zeros((3, 1))
            
            image_points, _ = cv2.projectPoints(
                point_3d_reshaped,
                rvec, tvec,
                camera_matrix,
                distortion_coefficients
            )
            
            pixel_x = int(image_points[0][0][0])
            pixel_y = int(image_points[0][0][1])
            
            height, width = frame.shape[:2]
            if 0 <= pixel_x < width and 0 <= pixel_y < height:
                color = (0, 0, 255)
                thickness = 3
                length = 20
                
                cv2.line(frame, (pixel_x - length, pixel_y), (pixel_x + length, pixel_y), color, thickness)
                cv2.line(frame, (pixel_x, pixel_y - length), (pixel_x, pixel_y + length), color, thickness)
                cv2.circle(frame, (pixel_x, pixel_y), 10, color, thickness)
                
                label = "Base Origin"
                font = cv2.FONT_HERSHEY_SIMPLEX
                font_scale = 0.7
                label_thickness = 2
                
                (text_width, text_height), baseline = cv2.getTextSize(label, font, font_scale, label_thickness)
                
                text_x = pixel_x + 15
                text_y = pixel_y - 10
                cv2.rectangle(frame, 
                            (text_x - 5, text_y - text_height - 5),
                            (text_x + text_width + 5, text_y + baseline + 5),
                            (0, 0, 0), -1)
                
                cv2.putText(frame, label, (text_x, text_y), font, font_scale, color, label_thickness)
                
                distance = np.linalg.norm(point_3d)
                distance_text = f"Dist: {distance*1000:.1f}mm"
                cv2.putText(frame, distance_text, (text_x, text_y + 25), 
                           font, 0.5, (255, 255, 255), 1)
            
            # Draw a circle with radius 102mm at z=0 plane centered at base origin
            radius_mm = 102.0  # mm
            radius_m = radius_mm / 1000.0  # Convert to meters
            num_points = 72  # Number of points to draw the circle (every 5 degrees)
            
            circle_points_3d = []
            for i in range(num_points):
                angle = 2 * np.pi * i / num_points
                # Point on circle in base coordinates (x, y, z=0)
                circle_x = radius_m * np.cos(angle)
                circle_y = radius_m * np.sin(angle)
                circle_z = 0.0
                
                # Create homogeneous coordinate
                point_base = np.array([circle_x, circle_y, circle_z, 1.0])
                
                # Transform to camera coordinates
                point_cam = base2cam_matrix @ point_base
                
                # Check if point is in front of camera
                if point_cam[2] > 0:
                    circle_points_3d.append(point_cam[:3])
            
            # Project circle points to image
            if len(circle_points_3d) > 0:
                circle_points_3d_array = np.array(circle_points_3d).reshape(-1, 1, 3)
                rvec = np.zeros((3, 1))
                tvec = np.zeros((3, 1))
                
                circle_image_points, _ = cv2.projectPoints(
                    circle_points_3d_array,
                    rvec, tvec,
                    camera_matrix,
                    distortion_coefficients
                )
                
                # Draw circle points
                circle_color = (0, 0, 255)  # Red color (BGR format)
                point_radius = 4
                
                for point in circle_image_points:
                    px = int(point[0][0])
                    py = int(point[0][1])
                    
                    # Check if point is within image bounds
                    if 0 <= px < width and 0 <= py < height:
                        cv2.circle(frame, (px, py), point_radius, circle_color, -1)
            
        except Exception as e:
            self.get_logger().error(f"Error projecting base origin: {e}")
        
        return frame
    
    def generate_frames(self):
        """Generate video frames for streaming."""
        while True:
            try:
                with self.image_lock:
                    if self.current_image is not None:
                        frame = self.current_image.copy()
                        
                        # Only project validation if validation is active
                        if self.validation_active:
                            frame = self.project_base_origin_to_image(frame)
                        
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


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
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
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()