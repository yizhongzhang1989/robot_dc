#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import glob
import threading
import argparse
import socket
import struct
import time
import math
import tempfile
import json
import subprocess
import signal
import atexit
from flask import Flask, render_template_string, Response, jsonify, request


def tcp_pose_to_transform_matrix(tcp_pose):
    """
    Convert TCP pose (X, Y, Z, Rx, Ry, Rz) to 4x4 transformation matrix.
    
    Args:
        tcp_pose: List of 6 values [X, Y, Z, Rx, Ry, Rz] where:
                 - X, Y, Z are in meters
                 - Rx, Ry, Rz are rotation angles in radians
    
    Returns:
        4x4 numpy array representing the transformation matrix
    """
    x, y, z, rx, ry, rz = tcp_pose
    
    # Create rotation matrices for each axis
    cos_rx, sin_rx = math.cos(rx), math.sin(rx)
    cos_ry, sin_ry = math.cos(ry), math.sin(ry)
    cos_rz, sin_rz = math.cos(rz), math.sin(rz)
    
    # Rotation matrix around X axis
    R_x = np.array([
        [1, 0, 0],
        [0, cos_rx, -sin_rx],
        [0, sin_rx, cos_rx]
    ])
    
    # Rotation matrix around Y axis
    R_y = np.array([
        [cos_ry, 0, sin_ry],
        [0, 1, 0],
        [-sin_ry, 0, cos_ry]
    ])
    
    # Rotation matrix around Z axis
    R_z = np.array([
        [cos_rz, -sin_rz, 0],
        [sin_rz, cos_rz, 0],
        [0, 0, 1]
    ])
    
    # Combined rotation matrix (ZYX order)
    R = R_z @ R_y @ R_x
    
    # Create 4x4 transformation matrix
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    
    return T


def find_available_port(start_port=8020, max_attempts=10):
    """Find an available port starting from start_port."""
    for port in range(start_port, start_port + max_attempts):
        try:
            with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                s.bind(('localhost', port))
                return port
        except OSError:
            continue
    return None


class CameraCalibrationNode(Node):
    """ROS2 node for camera calibration with web interface."""
    
    def __init__(self, camera_topic='/robot_arm_camera/image_raw', save_dir='./calibration_images', web_port=8020):
        super().__init__('camera_calibration_node')
        
        # Parameters
        self.camera_topic = camera_topic
        self.save_dir = save_dir
        
        # Initialize camera node process
        self.camera_process = None
        self._start_camera_node()
        
        # Register cleanup on exit
        atexit.register(self._cleanup_camera_node)
        
        # Find available port if the requested port is occupied
        if web_port:
            # Check if the requested port is available
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(('localhost', web_port))
                    self.web_port = web_port
            except OSError:
                # Port is occupied, find an alternative
                available_port = find_available_port(web_port, 10)
                if available_port:
                    self.web_port = available_port
                    self.get_logger().warning(f"Port {web_port} is occupied, using port {available_port} instead")
                else:
                    raise RuntimeError(f"No available ports found starting from {web_port}")
        else:
            self.web_port = find_available_port()
            if not self.web_port:
                raise RuntimeError("No available ports found")
        
        # Create save directory if it doesn't exist
        os.makedirs(self.save_dir, exist_ok=True)
        
        # Initialize CV bridge
        self.bridge = CvBridge()
        
        # Current image storage
        self.current_image = None
        self.image_lock = threading.Lock()
        
        # Image counter for naming - initialize based on existing files
        self.image_counter = self._initialize_image_counter()
        
        # Corner detection mode
        self.corner_detection_enabled = False
        self.chessboard_size = (11, 8)  # Default corner size (corners, not grid squares)
        self.chessboard_grid_size = (12, 9)  # Default grid size for display
        self.corner_lock = threading.Lock()
        
        # Robot joint angle monitoring
        self.robot_host = '192.168.1.10'
        self.robot_port = 2001
        self.joint_angles = [0.0] * 6  # Store 6 joint angles (we'll use first 6 of 7)
        self.tcp_pose = [0.0] * 6  # Store TCP pose (X, Y, Z, Rx, Ry, Rz)
        self.joint_lock = threading.Lock()
        self.robot_connected = False
        self.robot_last_update = None
        
        # Start robot monitoring thread
        self.robot_thread = threading.Thread(target=self._monitor_robot, daemon=True)
        self.robot_thread.start()
        
    # Create QoS profile optimized for real-time video streaming
    # Using RELIABLE delivery helps downstream consumers keep synchronized frames
        video_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1
        )
        
        # Create subscriber for camera images
        self.image_subscriber = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            video_qos
        )
        
        # Initialize Flask app with better shutdown handling
        self.app = Flask(__name__)
        self.app.config['THREADED'] = True
        
        self.setup_flask_routes()
        
        self.get_logger().info("Camera Calibration Node started")
        self.get_logger().info(f"Subscribing to: {self.camera_topic}")
        self.get_logger().info(f"Save directory: {os.path.abspath(self.save_dir)}")
        self.get_logger().info(f"Found {self.image_counter} existing screenshots, next will be {self.image_counter:04d}")
        self.get_logger().info(f"Web interface: http://localhost:{self.web_port}")
        
        # Flag to control Flask server
        self.flask_running = True
        
        # Start Flask server in a separate thread with better error handling
        self.flask_thread = threading.Thread(
            target=self._run_flask_server,
            daemon=True
        )
        self.flask_thread.start()
        
        # Give Flask a moment to start
        time.sleep(1)
    
    def _start_camera_node(self):
        """Start the camera node using ros2 launch."""
        try:
            # Find the workspace root
            try:
                from common.workspace_utils import get_workspace_root
                workspace_root = get_workspace_root()
            except:
                current_dir = os.path.dirname(os.path.abspath(__file__))
                workspace_root = os.path.dirname(current_dir)  # Go up one level from scripts
            colcon_ws = os.path.join(workspace_root, 'colcon_ws')
            
            # Check if colcon workspace exists
            if not os.path.exists(colcon_ws):
                self.get_logger().error(f"Colcon workspace not found at: {colcon_ws}")
                return
            
            # Setup environment and launch camera node
            env = os.environ.copy()
            
            # Source the workspace setup
            setup_bash = os.path.join(colcon_ws, 'install', 'setup.bash')
            if os.path.exists(setup_bash):
                # Use bash to source setup and run launch
                launch_cmd = f"source {setup_bash} && ros2 launch camera_node robot_arm_cam_launch.py"
                self.camera_process = subprocess.Popen(
                    launch_cmd,
                    shell=True,
                    executable='/bin/bash',
                    env=env,
                    stdout=subprocess.PIPE,
                    stderr=subprocess.PIPE,
                    preexec_fn=os.setsid  # Create new process group
                )
                self.get_logger().info("Camera node launched successfully")
                
                # Give camera node time to start
                time.sleep(3)
            else:
                self.get_logger().error(f"Setup file not found at: {setup_bash}")
                self.get_logger().info("Please make sure to build the colcon workspace first:")
                self.get_logger().info(f"cd {colcon_ws} && colcon build")
                
        except Exception as e:
            self.get_logger().error(f"Failed to start camera node: {e}")
    
    def _cleanup_camera_node(self):
        """Clean up camera node process."""
        if self.camera_process:
            try:
                # Terminate the entire process group
                os.killpg(os.getpgid(self.camera_process.pid), signal.SIGTERM)
                self.camera_process.wait(timeout=5)
                self.get_logger().info("Camera node stopped successfully")
            except subprocess.TimeoutExpired:
                # Force kill if it doesn't stop gracefully
                os.killpg(os.getpgid(self.camera_process.pid), signal.SIGKILL)
                self.get_logger().warning("Camera node force killed")
            except Exception as e:
                self.get_logger().error(f"Error stopping camera node: {e}")
            finally:
                self.camera_process = None
    
    def destroy_node(self):
        """Override destroy_node to clean up camera process."""
        self._cleanup_camera_node()
        super().destroy_node()
    
    def _run_flask_server(self):
        """Run Flask server with proper error handling."""
        try:
            # Simple Flask server startup
            self.app.run(
                host='0.0.0.0', 
                port=self.web_port, 
                debug=False, 
                use_reloader=False,
                threaded=True
            )
        except Exception as e:
            self.get_logger().error(f"Flask server error: {e}")
    
    def shutdown_flask(self):
        """Gracefully shutdown Flask server."""
        self.flask_running = False
        # Give some time for connections to close
        time.sleep(0.5)
    
    def _initialize_image_counter(self):
        """Initialize image counter based on existing image files in the new format."""
        try:
            # Get all existing image files in new format (序号.jpg)
            image_files = glob.glob(os.path.join(self.save_dir, '*.jpg'))
            
            if not image_files:
                return 0
            
            # Extract counter numbers from filenames
            # Expected format: 序号.jpg (simple numeric filename)
            counters = []
            for file_path in image_files:
                filename = os.path.basename(file_path)
                # Remove .jpg extension and try to parse as integer
                name_without_ext = filename.replace('.jpg', '')
                try:
                    counter = int(name_without_ext)
                    counters.append(counter)
                except ValueError:
                    # Skip files that don't match the numeric format
                    continue
            
            if counters:
                # Return the next counter number (max + 1)
                return max(counters) + 1
            else:
                # If no valid numeric files found, start from 0
                return 0
                
        except Exception as e:
            self.get_logger().warning(f"Failed to initialize image counter: {e}")
            return 0
    
    def _get_current_image_count(self):
        """Get current count of image files in the save directory."""
        try:
            # Get all existing image files in new format (序号.jpg)
            image_files = glob.glob(os.path.join(self.save_dir, '*.jpg'))
            
            if not image_files:
                return 0
            
            # Count only numeric filename files
            valid_count = 0
            for file_path in image_files:
                filename = os.path.basename(file_path)
                # Remove .jpg extension and try to parse as integer
                name_without_ext = filename.replace('.jpg', '')
                try:
                    int(name_without_ext)  # Check if it's a valid integer
                    valid_count += 1
                except ValueError:
                    # Skip files that don't match the numeric format
                    continue
            
            return valid_count
                
        except Exception as e:
            self.get_logger().warning(f"Failed to get current image count: {e}")
            return 0
    
    def _get_next_sequence_number(self):
        """Get the next available sequence number for saving files."""
        try:
            # Get all existing image files in new format (序号.jpg)
            image_files = glob.glob(os.path.join(self.save_dir, '*.jpg'))
            
            if not image_files:
                return 0
            
            # Extract counter numbers from filenames
            used_numbers = set()
            for file_path in image_files:
                filename = os.path.basename(file_path)
                # Remove .jpg extension and try to parse as integer
                name_without_ext = filename.replace('.jpg', '')
                try:
                    counter = int(name_without_ext)
                    used_numbers.add(counter)
                except ValueError:
                    # Skip files that don't match the numeric format
                    continue
            
            if not used_numbers:
                return 0
            
            # Find the next available number starting from 0
            next_number = 0
            while next_number in used_numbers:
                next_number += 1
            
            return next_number
                
        except Exception as e:
            self.get_logger().warning(f"Failed to get next sequence number: {e}")
            # Fallback to current counter
            return self.image_counter
        
    def image_callback(self, msg):
        """Callback function for receiving camera images."""
        try:
            # Convert ROS Image to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Store the current image with thread safety
            with self.image_lock:
                self.current_image = cv_image.copy()
            
            # Log first image received
            if not hasattr(self, '_first_image_logged'):
                self.get_logger().info(f"First image received! Size: {cv_image.shape}")
                self._first_image_logged = True
                
        except Exception as e:
            self.get_logger().error(f"Error converting image: {e}")
    
    def _monitor_robot(self):
        """Monitor robot joint angles from port 2001."""
        while True:
            try:
                # Create socket and connect to robot
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.settimeout(5.0)  # 5 second timeout
                sock.connect((self.robot_host, self.robot_port))
                
                self.get_logger().info(f"Connected to robot at {self.robot_host}:{self.robot_port}")
                self.robot_connected = True
                
                frame_size = 1468
                
                while True:
                    # Read exactly frame_size bytes from the socket
                    data = b""
                    while len(data) < frame_size:
                        packet = sock.recv(frame_size - len(data))
                        if not packet:
                            break
                        data += packet
                    
                    if len(data) == frame_size:
                        # Parse joint angles (first 28 bytes = 7 floats, we use first 6)
                        joint_positions = list(struct.unpack("7f", data[0:28]))
                        
                        # Parse TCP pose (bytes 368-391 = 6 floats: X,Y,Z,Rx,Ry,Rz)
                        tcp_positions = list(struct.unpack("6f", data[368:392]))
                        
                        # Convert radians to degrees and store only first 6 joints
                        joint_angles_deg = [math.degrees(angle) for angle in joint_positions[:6]]
                        
                        # TCP pose: position in mm, rotation in degrees
                        tcp_pose_converted = [
                            tcp_positions[0] * 1000,  # X in mm
                            tcp_positions[1] * 1000,  # Y in mm  
                            tcp_positions[2] * 1000,  # Z in mm
                            math.degrees(tcp_positions[3]),  # Rx in degrees
                            math.degrees(tcp_positions[4]),  # Ry in degrees
                            math.degrees(tcp_positions[5])   # Rz in degrees
                        ]
                        
                        with self.joint_lock:
                            self.joint_angles = joint_angles_deg
                            self.tcp_pose = tcp_pose_converted
                            self.robot_last_update = time.time()
                    else:
                        break
                        
            except socket.timeout:
                self.get_logger().warning("Robot connection timeout, retrying...")
                self.robot_connected = False
            except ConnectionRefusedError:
                self.get_logger().warning(f"Cannot connect to robot at {self.robot_host}:{self.robot_port}, retrying in 5 seconds...")
                self.robot_connected = False
                time.sleep(5)
            except Exception as e:
                self.get_logger().error(f"Robot monitoring error: {e}")
                self.robot_connected = False
            finally:
                try:
                    sock.close()
                except Exception:
                    pass
                time.sleep(1)  # Wait before retry
    
    def setup_flask_routes(self):
        """Setup Flask web routes."""
        
        @self.app.route('/')
        def index():
            """Main web interface."""
            html_template = """
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>DUCO Camera Calibration Tool</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f0f0f0;
        }
        .container {
            max-width: 1400px;
            margin: 0 auto;
            background-color: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
            display: flex;
            gap: 20px;
            align-items: flex-start;
        }
        .main-content {
            flex: 1;
            display: flex;
            flex-direction: column;
        }
        .sidebar {
            width: 200px;
            background: linear-gradient(135deg, #f8f9fa 0%, #e9ecef 100%);
            border: 1px solid #dee2e6;
            border-radius: 12px;
            padding: 15px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
            display: flex;
            flex-direction: column;
            align-self: stretch;
        }
        .sidebar h4 {
            margin-top: 0;
            margin-bottom: 15px;
            color: #495057;
            border-bottom: 2px solid #007bff;
            padding-bottom: 8px;
            font-size: 1em;
            text-align: center;
        }
        .thumbnail-grid {
            display: flex;
            flex-direction: column;
            gap: 6px;
            flex: 1;
            overflow-y: auto;
        }
        .thumbnail-item {
            position: relative;
            border-radius: 8px;
            overflow: hidden;
            box-shadow: 0 2px 4px rgba(0,0,0,0.1);
            transition: transform 0.2s ease;
            cursor: pointer;
        }
        .thumbnail-item:hover {
            transform: scale(1.05);
            box-shadow: 0 4px 8px rgba(0,0,0,0.2);
        }
        .thumbnail-img {
            width: 100%;
            height: 75px;
            object-fit: cover;
            border-radius: 8px;
        }
        .thumbnail-label {
            position: absolute;
            bottom: 0;
            left: 0;
            right: 0;
            background: linear-gradient(to top, rgba(0,0,0,0.8), transparent);
            color: white;
            padding: 8px 6px 4px;
            font-size: 0.7em;
            text-align: center;
        }
        .header {
            text-align: center;
            color: #333;
            margin-bottom: 20px;
        }
        .video-container {
            text-align: center;
            margin-bottom: 20px;
        }
        .video-frame {
            border: 2px solid #ddd;
            border-radius: 8px;
            max-width: 100%;
            height: auto;
        }
        .controls {
            display: flex;
            justify-content: center;
            gap: 10px;
            margin-bottom: 20px;
            flex-wrap: wrap;
        }
        .btn {
            padding: 12px 24px;
            border: none;
            border-radius: 6px;
            cursor: pointer;
            font-size: 16px;
            font-weight: bold;
            transition: all 0.3s ease;
        }
        .btn-primary {
            background-color: #007bff;
            color: white;
        }
        .btn-primary:hover {
            background-color: #0056b3;
        }
        .btn-success {
            background-color: #28a745;
            color: white;
        }
        .btn-success:hover {
            background-color: #1e7e34;
        }
        .btn-warning {
            background-color: #ffc107;
            color: black;
        }
        .btn-warning:hover {
            background-color: #e0a800;
        }
        .btn-danger {
            background-color: #dc3545;
            color: white;
        }
        .btn-danger:hover {
            background-color: #c82333;
        }
        .status {
            text-align: center;
            padding: 10px;
            margin: 10px 0;
            border-radius: 4px;
            font-weight: bold;
        }
        .status-success {
            background-color: #d4edda;
            color: #155724;
            border: 1px solid #c3e6cb;
        }
        .status-error {
            background-color: #f8d7da;
            color: #721c24;
            border: 1px solid #f5c6cb;
        }
        .status-info {
            background-color: #d1ecf1;
            color: #0c5460;
            border: 1px solid #bee5eb;
        }
        .info-panel {
            background: linear-gradient(135deg, #f8f9fa 0%, #e9ecef 100%);
            border: 1px solid #dee2e6;
            border-radius: 12px;
            padding: 20px;
            margin-top: 20px;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        }
        .info-panel h3 {
            margin-top: 0;
            margin-bottom: 20px;
            color: #495057;
            border-bottom: 2px solid #007bff;
            padding-bottom: 10px;
            font-size: 1.2em;
        }
        .info {
            display: grid;
            grid-template-columns: repeat(8, 1fr);
            gap: 15px;
        }
        .info-item {
            background-color: white;
            padding: 12px 16px;
            border-radius: 8px;
            border: 1px solid #e9ecef;
            box-shadow: 0 1px 3px rgba(0,0,0,0.1);
            transition: transform 0.2s ease;
        }
        .info-item:hover {
            transform: translateY(-2px);
            box-shadow: 0 2px 6px rgba(0,0,0,0.15);
        }
        .info-label {
            display: block;
            font-weight: 600;
            color: #495057;
            font-size: 0.8em;
            margin-bottom: 3px;
        }
        .info-value {
            display: block;
            font-weight: 500;
            color: #212529;
            font-size: 0.85em;
            word-break: break-all;
            line-height: 1.3;
        }
        #cameraStatus {
            font-weight: 600;
        }
        .first-row-item {
            /* 第一行的4个卡片各占2列，总共8列 */
            grid-column: span 2;
        }
        .robot-connection {
            /* Robot Connection占2列 */
            grid-column: span 2;
        }
        .pose-item {
            /* Joint Angles和TCP Pose各占3列，给予更多空间显示完整数组 */
            grid-column: span 3;
        }
        .pose-item .info-value {
            font-family: 'Courier New', monospace;
            font-size: 0.75em;
            /* 尽量保持单行显示 */
            white-space: nowrap;
            overflow: hidden;
            text-overflow: ellipsis;
            line-height: 1.2;
        }
        .hidden {
            display: none;
        }
        
        /* Modal styles for corner detection settings */
        .modal {
            display: none;
            position: fixed;
            z-index: 1000;
            left: 0;
            top: 0;
            width: 100%;
            height: 100%;
            background-color: rgba(0,0,0,0.5);
            backdrop-filter: blur(5px);
        }
        
        .modal-content {
            background-color: #fefefe;
            margin: 10% auto;
            padding: 0;
            border-radius: 15px;
            box-shadow: 0 10px 30px rgba(0,0,0,0.3);
            width: 90%;
            max-width: 500px;
            animation: modalSlideIn 0.3s ease-out;
        }
        
        @keyframes modalSlideIn {
            from {
                transform: translateY(-50px);
                opacity: 0;
            }
            to {
                transform: translateY(0);
                opacity: 1;
            }
        }
        
        .modal-header {
            background: linear-gradient(135deg, #007bff, #0056b3);
            color: white;
            padding: 20px 25px;
            border-radius: 15px 15px 0 0;
            text-align: center;
            position: relative;
        }
        
        .modal-header h2 {
            margin: 0;
            font-size: 1.4em;
            font-weight: 600;
        }
        
        .close {
            position: absolute;
            right: 15px;
            top: 50%;
            transform: translateY(-50%);
            color: rgba(255,255,255,0.8);
            font-size: 28px;
            font-weight: bold;
            cursor: pointer;
            transition: color 0.3s ease;
        }
        
        .close:hover {
            color: white;
        }
        
        .modal-body {
            padding: 30px 25px;
        }
        
        .form-group {
            margin-bottom: 25px;
        }
        
        .form-label {
            display: block;
            margin-bottom: 8px;
            font-weight: 600;
            color: #333;
            font-size: 1em;
        }
        
        .form-input {
            width: 100%;
            padding: 12px 15px;
            border: 2px solid #e0e0e0;
            border-radius: 8px;
            font-size: 16px;
            transition: border-color 0.3s ease;
            box-sizing: border-box;
        }
        
        .form-input:focus {
            outline: none;
            border-color: #007bff;
            box-shadow: 0 0 0 3px rgba(0,123,255,0.1);
        }
        
        .form-help {
            font-size: 0.85em;
            color: #666;
            margin-top: 5px;
        }
        
        .modal-footer {
            padding: 20px 25px;
            border-top: 1px solid #e0e0e0;
            display: flex;
            justify-content: flex-end;
            gap: 10px;
        }
        
        .modal-btn {
            padding: 12px 20px;
            border: none;
            border-radius: 6px;
            cursor: pointer;
            font-size: 14px;
            font-weight: 600;
            transition: all 0.3s ease;
            min-width: 80px;
        }
        
        .modal-btn-cancel {
            background: #f8f9fa;
            color: #6c757d;
            border: 1px solid #dee2e6;
        }
        
        .modal-btn-cancel:hover {
            background: #e2e6ea;
        }
        
        .modal-btn-confirm {
            background: #007bff;
            color: white;
        }
        
        .modal-btn-confirm:hover {
            background: #0056b3;
        }
        
        @media (max-width: 768px) {
            .container {
                flex-direction: column;
            }
            .sidebar {
                display: none;
            }
            .controls {
                flex-direction: column;
                align-items: center;
            }
            .btn {
                width: 80%;
                max-width: 300px;
            }
            .info {
                grid-template-columns: 1fr;
            }
            .info-panel {
                padding: 15px;
            }
            .first-row-item, .robot-connection, .pose-item {
                /* 在小屏幕上取消跨列 */
                grid-column: span 1;
            }
            .pose-item .info-value {
                /* 在小屏幕上允许换行 */
                white-space: normal;
                word-break: break-word;
            }
            .modal-content {
                margin: 5% auto;
                width: 95%;
            }
            .modal-body {
                padding: 20px 15px;
            }
        }
        @media (min-width: 1200px) {
            .info {
                /* 在大屏幕上保持8列布局 */
                grid-template-columns: repeat(8, 1fr);
            }
        }
    </style>
</head>
<body>
        <div class="container">
        <div class="main-content">
            <div class="header">
                                <h1>📷 DUCO Camera Calibration Tool</h1>
                <p>Real-time Camera View & Screenshot Capture</p>
            </div>
            
            <div class="video-container">
                <img id="videoFeed" class="video-frame" src="/video_feed" alt="Camera Feed">
            </div>
            
            <div class="controls">
                <button class="btn btn-success" onclick="takeScreenshot()">📸 Take Screenshot</button>
                <button class="btn btn-primary" onclick="refreshFeed()">🔄 Refresh Feed</button>
                <button id="cornerDetectionBtn" class="btn btn-warning" onclick="toggleCornerDetection()">🎯 Corner Detection</button>
                <button class="btn btn-danger" onclick="clearImages()">🗑️ Clear Images</button>
            </div>
            
            <div id="statusMessage" class="status hidden"></div>
            
            <!-- Corner Detection Settings Modal -->
            <div id="cornerDetectionModal" class="modal">
                <div class="modal-content">
                    <div class="modal-header">
                        <h2>🎯 Chessboard Settings</h2>
                        <span class="close" onclick="closeCornerDetectionModal()">&times;</span>
                    </div>
                    <div class="modal-body">
                        <div class="form-group">
                            <label class="form-label" for="chessboardWidth">Chessboard Width (Grid Count)</label>
                            <input type="number" id="chessboardWidth" class="form-input" value="12" min="2" max="50">
                            <div class="form-help">Number of squares in horizontal direction</div>
                        </div>
                        
                        <div class="form-group">
                            <label class="form-label" for="chessboardHeight">Chessboard Height (Grid Count)</label>
                            <input type="number" id="chessboardHeight" class="form-input" value="9" min="2" max="50">
                            <div class="form-help">Number of squares in vertical direction</div>
                        </div>
                    </div>
                    <div class="modal-footer">
                        <button class="modal-btn modal-btn-cancel" onclick="closeCornerDetectionModal()">Cancel</button>
                        <button class="modal-btn modal-btn-confirm" onclick="confirmCornerDetection()">Start Detection</button>
                    </div>
                </div>
            </div>
            
            <div class="info-panel">
                <h3>📊 System Status</h3>
                <div class="info">
                    <div class="info-item first-row-item">
                        <span class="info-label">Camera Status:</span>
                        <span id="cameraStatus" class="info-value">Waiting for connection...</span>
                    </div>
                    <div class="info-item first-row-item">
                        <span class="info-label">Camera Topic:</span>
                        <span id="cameraTopic" class="info-value">{{ camera_topic }}</span>
                    </div>
                    <div class="info-item first-row-item">
                        <span class="info-label">Save Directory:</span>
                        <span id="saveDirectory" class="info-value">{{ save_dir }}</span>
                    </div>
                    <div class="info-item first-row-item">
                        <span class="info-label">Screenshot Count:</span>
                        <span id="imageCount" class="info-value">0</span>
                    </div>
                    <div class="info-item robot-connection">
                        <span class="info-label">Robot Connection:</span>
                        <span id="robotStatus" class="info-value">Connecting...</span>
                    </div>
                    <div class="info-item pose-item">
                        <span class="info-label">Joint Angles:</span>
                        <span id="jointAngles" class="info-value">[--°, --°, --°, --°, --°, --°]</span>
                    </div>
                    <div class="info-item pose-item">
                        <span class="info-label">TCP Pose:</span>
                        <span id="tcpPose" class="info-value">[-- mm, -- mm, -- mm, --°, --°, --°]</span>
                    </div>
                </div>
            </div>
        </div>
        
        <div class="sidebar">
            <h4>Recent Screenshots</h4>
            <div id="thumbnailGrid" class="thumbnail-grid">
                <!-- Thumbnails will be populated here -->
            </div>
        </div>
    </div>

    <script>
        let imageCount = 0;
        let cornerDetectionEnabled = false;

        function showStatus(message, type = 'info') {
            const statusElement = document.getElementById('statusMessage');
            statusElement.textContent = message;
            statusElement.className = `status status-${type}`;
            statusElement.classList.remove('hidden');
            
            setTimeout(() => {
                statusElement.classList.add('hidden');
            }, 3000);
        }

        function toggleCornerDetection() {
            if (!cornerDetectionEnabled) {
                // Show modal instead of prompt
                showCornerDetectionModal();
            } else {
                // Disable corner detection
                fetch('/toggle_corner_detection', {
                    method: 'POST',
                    headers: {
                        'Content-Type': 'application/json',
                    },
                    body: JSON.stringify({enable: false})
                })
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        cornerDetectionEnabled = false;
                        document.getElementById('cornerDetectionBtn').textContent = '🎯 Corner Detection';
                        document.getElementById('cornerDetectionBtn').className = 'btn btn-warning';
                        showStatus('Corner detection disabled', 'info');
                    } else {
                        showStatus('Failed to disable corner detection: ' + data.message, 'error');
                    }
                });
            }
        }

        function showCornerDetectionModal() {
            document.getElementById('cornerDetectionModal').style.display = 'block';
            // Focus on first input
            setTimeout(() => {
                document.getElementById('chessboardWidth').focus();
            }, 100);
        }

        function closeCornerDetectionModal() {
            document.getElementById('cornerDetectionModal').style.display = 'none';
        }

        function confirmCornerDetection() {
            const width = parseInt(document.getElementById('chessboardWidth').value);
            const height = parseInt(document.getElementById('chessboardHeight').value);
            
            if (isNaN(width) || isNaN(height) || width <= 1 || height <= 1) {
                showStatus('Please enter valid chessboard size (greater than 1)', 'error');
                return;
            }
            
            // Convert grid size to corner size (grid_size - 1)
            const cornerWidth = width - 1;
            const cornerHeight = height - 1;
            
            // Enable corner detection
            fetch('/toggle_corner_detection', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                },
                body: JSON.stringify({
                    enable: true,
                    chessboard_width: cornerWidth,
                    chessboard_height: cornerHeight,
                    grid_width: width,
                    grid_height: height
                })
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    cornerDetectionEnabled = true;
                    document.getElementById('cornerDetectionBtn').textContent = '🛑 Stop Detection';
                    document.getElementById('cornerDetectionBtn').className = 'btn btn-danger';
                    showStatus(`Corner detection enabled (${width}×${height} grid, ${cornerWidth}×${cornerHeight} corners)`, 'success');
                    closeCornerDetectionModal();
                } else {
                    showStatus('Failed to enable corner detection: ' + data.message, 'error');
                }
            })
            .catch(error => {
                showStatus('Network request failed: ' + error.message, 'error');
            });
        }

        // Close modal when clicking outside of it
        window.onclick = function(event) {
            const modal = document.getElementById('cornerDetectionModal');
            if (event.target === modal) {
                closeCornerDetectionModal();
            }
        }

        // Handle Enter key in modal inputs
        document.addEventListener('DOMContentLoaded', function() {
            const inputs = document.querySelectorAll('#cornerDetectionModal input');
            inputs.forEach(input => {
                input.addEventListener('keypress', function(e) {
                    if (e.key === 'Enter') {
                        confirmCornerDetection();
                    }
                });
            });
        });

        function takeScreenshot() {
            fetch('/take_screenshot', {method: 'POST'})
                .then(response => response.json())
                .then(data => {
                    if (data.success) {
                        showStatus('Screenshot saved: ' + data.filename, 'success');
                        updateStatus();
                    } else {
                        showStatus('Screenshot failed: ' + data.message, 'error');
                    }
                });
        }

        function refreshFeed() {
            showStatus('Feed refreshed', 'info');
            location.reload();
        }

        function clearImages() {
            if (confirm('Are you sure you want to clear all images?')) {
                fetch('/clear_images', {method: 'POST'})
                    .then(response => response.json())
                    .then(data => {
                        if (data.success) {
                            showStatus('Images cleared successfully', 'success');
                            updateStatus();
                        } else {
                            showStatus('Clear failed: ' + data.message, 'error');
                        }
                    });
            }
        }

        function updateThumbnails() {
            fetch('/get_thumbnails')
            .then(response => response.json())
            .then(data => {
                const thumbnailGrid = document.getElementById('thumbnailGrid');
                thumbnailGrid.innerHTML = '';
                
                if (data.thumbnails && data.thumbnails.length > 0) {
                    data.thumbnails.forEach((thumbnail, index) => {
                        const thumbnailItem = document.createElement('div');
                        thumbnailItem.className = 'thumbnail-item';
                        thumbnailItem.onclick = () => openImageModal(thumbnail.filename);
                        
                        // 从文件名中提取序号 (格式: 序号.jpg)
                        let sequenceNumber = '0';
                        const nameWithoutExt = thumbnail.filename.replace('.jpg', '');
                        if (/^\d+$/.test(nameWithoutExt)) {
                            sequenceNumber = nameWithoutExt;
                        }
                        
                        thumbnailItem.innerHTML = `
                            <img src="/thumbnail/${thumbnail.filename}" alt="${thumbnail.filename}" class="thumbnail-img">
                            <div class="thumbnail-label">${sequenceNumber}</div>
                        `;
                        
                        thumbnailGrid.appendChild(thumbnailItem);
                    });
                } else {
                    thumbnailGrid.innerHTML = '<div style="text-align: center; color: #6c757d; font-size: 0.9em; padding: 20px;">No screenshots yet</div>';
                }
            })
            .catch(error => {
                console.error('Failed to fetch thumbnails:', error);
            });
        }

        function openImageModal(filename) {
            // Simple modal to view full image
            const modal = document.createElement('div');
            modal.style.cssText = `
                position: fixed; top: 0; left: 0; width: 100%; height: 100%;
                background: rgba(0,0,0,0.8); display: flex; justify-content: center;
                align-items: center; z-index: 1000; cursor: pointer;
            `;
            
            const img = document.createElement('img');
            img.src = `/image/${filename}`;
            img.style.cssText = 'max-width: 90%; max-height: 90%; border-radius: 8px;';
            
            modal.appendChild(img);
            modal.onclick = () => document.body.removeChild(modal);
            document.body.appendChild(modal);
        }

        function updateStatus() {
            fetch('/get_status')
            .then(response => response.json())
            .then(data => {
                document.getElementById('imageCount').textContent = data.image_count;
                imageCount = data.image_count;
                
                // Update camera status
                const cameraStatusElement = document.getElementById('cameraStatus');
                if (data.has_image) {
                    cameraStatusElement.textContent = `Connected`;
                    cameraStatusElement.style.color = 'green';
                } else {
                    cameraStatusElement.textContent = 'Waiting for connection... - Check camera_node';
                    cameraStatusElement.style.color = 'red';
                }
                
                // Update camera topic and save directory
                document.getElementById('cameraTopic').textContent = data.camera_topic;
                // Extract directory name from full path
                const fullPath = data.save_directory;
                const directoryName = fullPath.split('/').pop() || fullPath;
                document.getElementById('saveDirectory').textContent = directoryName;
                
                // Update robot status and joint angles
                const robotStatusElement = document.getElementById('robotStatus');
                const jointAnglesElement = document.getElementById('jointAngles');
                const tcpPoseElement = document.getElementById('tcpPose');
                
                if (data.robot_connected) {
                    robotStatusElement.textContent = 'Connected';
                    robotStatusElement.style.color = 'green';
                    
                    // Update joint angles in compact format
                    if (data.joint_angles && data.joint_angles.length >= 6) {
                        const jointText = '[' + data.joint_angles.slice(0, 6).map(angle => 
                            angle.toFixed(1) + '°'
                        ).join(', ') + ']';
                        jointAnglesElement.textContent = jointText;
                    } else {
                        jointAnglesElement.textContent = '[--°, --°, --°, --°, --°, --°]';
                    }
                    
                    // Update TCP pose in compact format
                    if (data.tcp_pose && data.tcp_pose.length >= 6) {
                        const tcpText = '[' + 
                            data.tcp_pose.slice(0, 3).map(pos => pos.toFixed(1) + ' mm').join(', ') + ', ' +
                            data.tcp_pose.slice(3, 6).map(rot => rot.toFixed(1) + '°').join(', ') + ']';
                        tcpPoseElement.textContent = tcpText;
                    } else {
                        tcpPoseElement.textContent = '[-- mm, -- mm, -- mm, --°, --°, --°]';
                    }
                } else {
                    robotStatusElement.textContent = 'Disconnected';
                    robotStatusElement.style.color = 'red';
                    jointAnglesElement.textContent = '[--°, --°, --°, --°, --°, --°]';
                    tcpPoseElement.textContent = '[-- mm, -- mm, -- mm, --°, --°, --°]';
                }
                
                // Update corner detection status
                if (data.corner_detection_enabled !== undefined) {
                    cornerDetectionEnabled = data.corner_detection_enabled;
                    const cornerBtn = document.getElementById('cornerDetectionBtn');
                    if (cornerDetectionEnabled) {
                        cornerBtn.textContent = '🛑 Stop Detection';
                        cornerBtn.className = 'btn btn-danger';
                    } else {
                        cornerBtn.textContent = '🎯 Corner Detection';
                        cornerBtn.className = 'btn btn-warning';
                    }
                }
                
                // Update thumbnails
                updateThumbnails();
            })
            .catch(error => {
                console.error('Status fetch failed:', error);
                document.getElementById('cameraStatus').textContent = 'Connection Error';
                document.getElementById('cameraStatus').style.color = 'red';
            });
        }

        // 定期更新统计信息
        setInterval(() => {
            updateStatus();
        }, 2000);

        // 页面加载时立即更新一次状态
        window.onload = function() {
            updateStatus();
            updateThumbnails();
        };
    </script>
</body>
</html>
            """
            return render_template_string(html_template, 
                                        save_dir=self.save_dir, 
                                        camera_topic=self.camera_topic)
        
        @self.app.route('/video_feed')
        def video_feed():
            """Video streaming route."""
            return Response(self.generate_frames(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/take_screenshot', methods=['POST'])
        def take_screenshot():
            """Take screenshot endpoint."""
            result = self.save_screenshot()
            return jsonify(result)
        
        @self.app.route('/clear_images', methods=['POST'])
        def clear_images():
            """Clear all calibration images and JSON data files."""
            try:
                # Clear image files (new format: 序号.jpg)
                image_files = glob.glob(os.path.join(self.save_dir, '*.jpg'))
                # Clear JSON files (序号.json)
                json_files = glob.glob(os.path.join(self.save_dir, '*.json'))
                
                # Filter to only remove numeric named files and their corresponding data files
                numeric_image_files = []
                for file_path in image_files:
                    filename = os.path.basename(file_path)
                    name_without_ext = filename.replace('.jpg', '')
                    try:
                        int(name_without_ext)  # Check if it's a numeric filename
                        numeric_image_files.append(file_path)
                    except ValueError:
                        # Skip non-numeric files
                        continue
                
                # Filter JSON files to only remove numeric named ones
                numeric_json_files = []
                for file_path in json_files:
                    filename = os.path.basename(file_path)
                    name_without_ext = filename.replace('.json', '')
                    try:
                        int(name_without_ext)  # Check if it's a numeric filename
                        numeric_json_files.append(file_path)
                    except ValueError:
                        # Skip non-numeric files
                        continue
                
                all_files_to_remove = numeric_image_files + numeric_json_files
                
                for file_path in all_files_to_remove:
                    os.remove(file_path)
                    
                self.image_counter = 0
                return jsonify({
                    'success': True,
                    'message': f'Deleted {len(numeric_image_files)} images and {len(numeric_json_files)} JSON files'
                })
            except Exception as e:
                return jsonify({
                    'success': False,
                    'message': f'Delete failed: {str(e)}'
                })
        
        @self.app.route('/toggle_corner_detection', methods=['POST'])
        def toggle_corner_detection():
            """Toggle corner detection mode."""
            try:
                data = request.get_json()
                enable = data.get('enable', False)
                
                with self.corner_lock:
                    if enable:
                        # Get corner dimensions (actual corner count for detection)
                        chessboard_width = data.get('chessboard_width', 11)
                        chessboard_height = data.get('chessboard_height', 8)
                        
                        # Get grid dimensions (for display purposes)
                        grid_width = data.get('grid_width', 12)
                        grid_height = data.get('grid_height', 9)
                        
                        self.chessboard_size = (chessboard_width, chessboard_height)
                        self.chessboard_grid_size = (grid_width, grid_height)
                        self.corner_detection_enabled = True
                        message = f'Corner detection enabled: {grid_width}x{grid_height} grid ({chessboard_width}x{chessboard_height} corners)'
                    else:
                        self.corner_detection_enabled = False
                        message = 'Corner detection disabled'
                
                return jsonify({
                    'success': True,
                    'enabled': self.corner_detection_enabled,
                    'chessboard_size': self.chessboard_size,
                    'chessboard_grid_size': self.chessboard_grid_size,
                    'message': message
                })
            except Exception as e:
                return jsonify({
                    'success': False,
                    'message': f'Toggle failed: {str(e)}'
                })
        
        @self.app.route('/get_status')
        def get_status():
            """Get current status."""
            has_image = self.current_image is not None
            image_shape = None
            if has_image:
                image_shape = self.current_image.shape
            
            # Get robot joint angles with thread safety
            with self.joint_lock:
                joint_angles = self.joint_angles.copy()
                tcp_pose = self.tcp_pose.copy()
                robot_connected = self.robot_connected
                last_update = self.robot_last_update
            
            # Check if robot data is recent (within last 5 seconds)
            current_time = time.time()
            if last_update and (current_time - last_update) > 5:
                robot_connected = False
            
            # Get corner detection status
            with self.corner_lock:
                corner_detection_enabled = self.corner_detection_enabled
                chessboard_size = self.chessboard_size
                chessboard_grid_size = self.chessboard_grid_size
            
            # Get actual current image count from filesystem (real-time)
            actual_image_count = self._get_current_image_count()
            
            return jsonify({
                'image_count': actual_image_count,
                'has_image': has_image,
                'image_shape': image_shape,
                'camera_topic': self.camera_topic,
                'save_directory': os.path.abspath(self.save_dir),
                'robot_connected': robot_connected,
                'joint_angles': joint_angles,
                'tcp_pose': tcp_pose,
                'corner_detection_enabled': corner_detection_enabled,
                'chessboard_size': chessboard_size,
                'chessboard_grid_size': chessboard_grid_size
            })
        
        @self.app.route('/get_thumbnails')
        def get_thumbnails():
            """Get list of recent screenshots for thumbnails."""
            try:
                # Get all image files in new format (序号.jpg)
                image_files = glob.glob(os.path.join(self.save_dir, '*.jpg'))
                
                # Filter and sort by numeric sequence (oldest first)
                def extract_sequence_number(file_path):
                    filename = os.path.basename(file_path)
                    name_without_ext = filename.replace('.jpg', '')
                    try:
                        return int(name_without_ext)  # Get the numeric part
                    except ValueError:
                        return -1  # Non-numeric files go to the beginning
                
                # Filter out non-numeric files and sort
                numeric_files = []
                for file_path in image_files:
                    seq_num = extract_sequence_number(file_path)
                    if seq_num >= 0:  # Only include numeric files
                        numeric_files.append(file_path)
                
                numeric_files.sort(key=extract_sequence_number, reverse=False)  # Oldest first
                # Get only the last 10 files (most recent 10 by sequence number)
                recent_files = numeric_files[-10:] if len(numeric_files) > 10 else numeric_files
                
                thumbnails = []
                for file_path in recent_files:
                    filename = os.path.basename(file_path)
                    thumbnails.append({
                        'filename': filename,
                        'path': file_path
                    })
                
                return jsonify({'thumbnails': thumbnails})
            except Exception as e:
                return jsonify({'thumbnails': [], 'error': str(e)})
        
        @self.app.route('/thumbnail/<filename>')
        def serve_thumbnail(filename):
            """Serve thumbnail image."""
            try:
                from flask import send_file
                import tempfile
                
                file_path = os.path.join(self.save_dir, filename)
                if not os.path.exists(file_path):
                    return "File not found", 404
                
                # Create thumbnail
                img = cv2.imread(file_path)
                if img is None:
                    return "Cannot read image", 404
                
                # Resize to thumbnail size (width=200, maintain aspect ratio)
                height, width = img.shape[:2]
                thumb_width = 200
                thumb_height = int(height * thumb_width / width)
                thumbnail = cv2.resize(img, (thumb_width, thumb_height))
                
                # Save to temporary file
                temp_file = tempfile.NamedTemporaryFile(suffix='.jpg', delete=False)
                cv2.imwrite(temp_file.name, thumbnail, [cv2.IMWRITE_JPEG_QUALITY, 85])
                temp_file.close()
                
                return send_file(temp_file.name, mimetype='image/jpeg')
            except Exception as e:
                return f"Error: {str(e)}", 500
        
        @self.app.route('/image/<filename>')
        def serve_full_image(filename):
            """Serve full-size image."""
            try:
                from flask import send_file
                file_path = os.path.join(self.save_dir, filename)
                if not os.path.exists(file_path):
                    return "File not found", 404
                return send_file(file_path, mimetype='image/jpeg')
            except Exception as e:
                return f"Error: {str(e)}", 500
    
    def generate_frames(self):
        """Generate video frames for streaming."""
        while True:
            try:
                with self.image_lock:
                    if self.current_image is not None:
                        frame = self.current_image.copy()
                        
                        # Apply corner detection if enabled
                        with self.corner_lock:
                            if self.corner_detection_enabled:
                                frame = self.apply_corner_detection(frame)
                        
                        # Resize frame if too large for better streaming performance
                        height, width = frame.shape[:2]
                        if width > 800:
                            scale = 800.0 / width
                            new_width = int(width * scale)
                            new_height = int(height * scale)
                            frame = cv2.resize(frame, (new_width, new_height))
                        
                        # Encode frame as JPEG
                        encode_param = [cv2.IMWRITE_JPEG_QUALITY, 85]
                        _, buffer = cv2.imencode('.jpg', frame, encode_param)
                        frame_bytes = buffer.tobytes()
                        
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    else:
                        # If no image available, create a placeholder
                        placeholder = self.create_placeholder_image()
                        _, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        frame_bytes = buffer.tobytes()
                        
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                
                # Small delay to prevent excessive CPU usage
                threading.Event().wait(0.1)  # 10 FPS to reduce load
                
            except Exception as e:
                self.get_logger().error(f"Error in video stream: {e}")
                # Create error image
                error_img = self.create_error_image(str(e))
                _, buffer = cv2.imencode('.jpg', error_img, [cv2.IMWRITE_JPEG_QUALITY, 85])
                frame_bytes = buffer.tobytes()
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                
                threading.Event().wait(1.0)  # Wait longer on error
    
    def apply_corner_detection(self, frame):
        """Apply corner detection to the frame."""
        try:
            # Convert to grayscale for corner detection
            gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
            
            # Find chessboard corners
            ret, corners = cv2.findChessboardCorners(
                gray, 
                self.chessboard_size, 
                flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
            )
            
            # Draw corners if found
            if ret:
                # Refine corners for better accuracy
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners_refined = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                # Draw the corners
                cv2.drawChessboardCorners(frame, self.chessboard_size, corners_refined, ret)
                
                # Add status text with grid and corner information
                grid_info = f'{self.chessboard_grid_size[0]}x{self.chessboard_grid_size[1]} grid'
                corner_info = f'{self.chessboard_size[0]}x{self.chessboard_size[1]} corners'
                cv2.putText(frame, f'Corners Found: {len(corners)} ({grid_info}, {corner_info})', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            else:
                # Add status text when no corners found
                grid_info = f'{self.chessboard_grid_size[0]}x{self.chessboard_grid_size[1]} grid'
                corner_info = f'{self.chessboard_size[0]}x{self.chessboard_size[1]} corners'
                cv2.putText(frame, f'No Corners Found ({grid_info}, {corner_info})', 
                           (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
            
            # Add corner detection indicator
            cv2.putText(frame, 'Corner Detection: ON', 
                       (10, frame.shape[0] - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 0), 2)
            
        except Exception as e:
            # Add error text
            cv2.putText(frame, f'Corner Detection Error: {str(e)[:50]}', 
                       (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 1)
        
        return frame
    
    def create_placeholder_image(self):
        """Create a placeholder image when no camera feed is available."""
        img = np.zeros((480, 640, 3), dtype=np.uint8)
        img.fill(50)  # Dark gray background
        
        # Add text
        text_lines = [
            "Waiting for camera data...",
            f"Topic: {self.camera_topic}",
            "Please check if camera_node is running"
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
        img[:, :, 2] = 50  # Red tint
        
        # Add error text
        text_lines = [
            "Video Stream Error",
            f"Error: {error_msg[:50]}..."
        ]
        
        font = cv2.FONT_HERSHEY_SIMPLEX
        font_scale = 0.6
        color = (0, 255, 255)  # Yellow color
        thickness = 2
        
        y_start = 180
        for i, line in enumerate(text_lines):
            text_size = cv2.getTextSize(line, font, font_scale, thickness)[0]
            x = (img.shape[1] - text_size[0]) // 2
            y = y_start + i * 40
            cv2.putText(img, line, (x, y), font, font_scale, color, thickness)
        
        return img
    
    def save_screenshot(self):
        """Save the current camera image as a screenshot with joint angles in the new format."""
        with self.image_lock:
            if self.current_image is not None:
                # Get current joint angles and TCP pose
                with self.joint_lock:
                    joint_angles = self.joint_angles.copy()
                    tcp_pose = self.tcp_pose.copy()
                    robot_connected = self.robot_connected
                
                # Find the next available sequence number
                next_sequence = self._get_next_sequence_number()
                
                # Generate filenames with simple numeric format
                image_filename = f"{next_sequence}.jpg"
                json_filename = f"{next_sequence}.json"
                
                image_filepath = os.path.join(self.save_dir, image_filename)
                json_filepath = os.path.join(self.save_dir, json_filename)
                
                # Save the image
                success = cv2.imwrite(image_filepath, self.current_image)
                
                if success:
                    # Save joint angles and TCP pose data to JSON only
                    try:
                        if robot_connected and len(joint_angles) >= 6 and len(tcp_pose) >= 6:
                            # TCP pose: convert position from mm to meters and rotation from degrees to radians
                            tcp_pose_converted = [
                                tcp_pose[0] / 1000.0,  # X in meters
                                tcp_pose[1] / 1000.0,  # Y in meters  
                                tcp_pose[2] / 1000.0,  # Z in meters
                                math.radians(tcp_pose[3]),  # Rx in radians
                                math.radians(tcp_pose[4]),  # Ry in radians
                                math.radians(tcp_pose[5])   # Rz in radians
                            ]
                            
                            # Convert joint angles from degrees to radians
                            joint_angles_rad = [math.radians(angle) for angle in joint_angles[:6]]
                            
                            # Create 4x4 transformation matrix
                            transform_matrix = tcp_pose_to_transform_matrix(tcp_pose_converted)
                            
                            # Save TCP pose data and transformation matrix to JSON
                            json_data = {
                                "joint_angles": joint_angles_rad,  # Joint angles in radians
                                "end_xyzrpy": {
                                    "x": tcp_pose_converted[0],
                                    "y": tcp_pose_converted[1], 
                                    "z": tcp_pose_converted[2],
                                    "rx": tcp_pose_converted[3],
                                    "ry": tcp_pose_converted[4],
                                    "rz": tcp_pose_converted[5]
                                },
                                "end2base": transform_matrix.tolist()
                            }
                            
                            with open(json_filepath, 'w') as json_file:
                                json.dump(json_data, json_file, indent=2)
                            
                            self.get_logger().info(f"Screenshot saved: {image_filepath}")
                            self.get_logger().info(f"Robot data saved: {json_filepath}")
                            
                        else:
                            # If robot not connected, save dummy JSON data
                            json_data = {
                                "joint_angles": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],  # Dummy joint angles
                                "end_xyzrpy": {
                                    "x": 0.0,
                                    "y": 0.0,
                                    "z": 0.0,
                                    "rx": 0.0,
                                    "ry": 0.0,
                                    "rz": 0.0
                                },
                                "end2base": np.eye(4).tolist()
                            }
                            
                            with open(json_filepath, 'w') as json_file:
                                json.dump(json_data, json_file, indent=2)
                            
                            self.get_logger().warning(f"Robot disconnected - saved dummy data for {image_filename}")
                    
                    except Exception as e:
                        self.get_logger().error(f"Failed to save JSON data: {e}")
                        # If JSON save fails, at least we have the image
                    
                    # Update image_counter to next available sequence number for subsequent saves
                    self.image_counter = self._get_next_sequence_number()
                    
                    # Create result message
                    if robot_connected and len(joint_angles) >= 6:
                        angles_str = '[' + ', '.join([f'{angle:.1f}°' for angle in joint_angles[:6]]) + ']'
                        if len(tcp_pose) >= 6:
                            tcp_str = f'[{tcp_pose[0]:.0f},{tcp_pose[1]:.0f},{tcp_pose[2]:.0f}mm; {tcp_pose[3]:.1f},{tcp_pose[4]:.1f},{tcp_pose[5]:.1f}°]'
                            message = f'Data saved: {image_filename}, {json_filename} - Angles: {angles_str}, TCP: {tcp_str}'
                        else:
                            message = f'Data saved: {image_filename}, {json_filename} - Angles: {angles_str}'
                    else:
                        message = f'Data saved: {image_filename}, {json_filename} (no robot data - dummy data saved)'
                    
                    return {
                        'success': True,
                        'filename': image_filename,
                        'message': message
                    }
                else:
                    self.get_logger().error(f"Failed to save screenshot: {image_filepath}")
                    return {
                        'success': False,
                        'message': 'Failed to save screenshot'
                    }
            else:
                self.get_logger().warning("No image available to save")
                return {
                    'success': False,
                    'message': 'No image available'
                }


def main():
    """Main function."""
    parser = argparse.ArgumentParser(description='DUCO Camera Screenshot Tool - Web Interface')
    parser.add_argument('--topic', '-t', default='/robot_arm_camera/image_raw',
                       help='Camera image topic name (default: /robot_arm_camera/image_raw)')
    parser.add_argument('--save-dir', '-d', default='./calibration_images',
                       help='Directory to save screenshot images (default: ./calibration_images)')
    parser.add_argument('--port', '-p', type=int, default=8020,
                       help='Web server port (default: 8020)')
    
    args = parser.parse_args()
    
    # Initialize ROS2
    rclpy.init()
    
    calibration_node = None
    
    def signal_handler(sig, frame):
        """Handle Ctrl+C signal."""
        print("\n👋 User requested shutdown")
        if calibration_node:
            print("🔄 Shutting down camera node...")
            calibration_node._cleanup_camera_node()
            print("🔄 Shutting down web server...")
            calibration_node.shutdown_flask()
            calibration_node.destroy_node()
        rclpy.shutdown()
        print("🔄 Cleanup completed")
        exit(0)
    
    # Register signal handler
    signal.signal(signal.SIGINT, signal_handler)
    
    try:
        # Create and run the calibration node
        calibration_node = CameraCalibrationNode(
            camera_topic=args.topic,
            save_dir=args.save_dir,
            web_port=args.port
        )
        
        print("\n🚀 DUCO Camera Calibration Tool Started!")
        print(f"📷 Camera Topic: {args.topic}")
        print(f"💾 Save Directory: {os.path.abspath(args.save_dir)}")
        print(f"🌐 Web Interface: http://localhost:{calibration_node.web_port}")
        print("� Camera node automatically started")
        print("�💡 Open the above link in your browser to start using")
        print("⏹️  Press Ctrl+C to exit\n")
        
        # Keep the node running
        rclpy.spin(calibration_node)
        
    except KeyboardInterrupt:
        print("\n👋 User requested shutdown")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        # Cleanup
        if calibration_node:
            print("🔄 Shutting down camera node...")
            calibration_node._cleanup_camera_node()
            print("🔄 Shutting down web server...")
            calibration_node.shutdown_flask()
            calibration_node.destroy_node()
        rclpy.shutdown()
        print("🔄 Cleanup completed")


if __name__ == '__main__':
    main()
