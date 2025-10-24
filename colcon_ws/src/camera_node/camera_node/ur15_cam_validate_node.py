#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
UR15 Camera Calibration Validation Node

This node provides a web interface for validating camera calibration with UR15 robot.
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
        os.path.dirname(get_package_share_directory('camera_node')),
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


class UR15CameraValidateNode(Node):
    """ROS2 node for UR15 camera calibration validation with web interface."""
    
    def __init__(self):
        super().__init__('ur15_cam_validate_node')
        
        # Declare parameters
        self.declare_parameter('camera_topic', '/ur15_camera/image_raw')
        self.declare_parameter('web_port', 8030)
        self.declare_parameter('ur15_ip', '192.168.1.15')
        self.declare_parameter('ur15_port', 30002)
        
        # Get parameters
        self.camera_topic = self.get_parameter('camera_topic').value
        web_port = self.get_parameter('web_port').value
        self.ur15_ip = self.get_parameter('ur15_ip').value
        self.ur15_port = self.get_parameter('ur15_port').value
        
        # Find available port if the requested port is occupied
        if web_port:
            try:
                with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
                    s.bind(('localhost', web_port))
                    self.web_port = web_port
            except OSError:
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
        self.app = Flask(__name__)
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
        
        # Import the HTML template from the original script
        from pathlib import Path
        
        @self.app.route('/')
        def index():
            """Main web interface."""
            html_template = """
<!DOCTYPE html>
<html lang="zh-CN">
<head>
    <meta charset="UTF-8">
    <meta name="viewport" content="width=device-width, initial-scale=1.0">
    <title>UR15 Camera Calibration Validator</title>
    <style>
        body {
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f0f0f0;
        }
        .container {
            max-width: 1200px;
            margin: 0 auto;
            background-color: white;
            border-radius: 10px;
            padding: 20px;
            box-shadow: 0 4px 6px rgba(0,0,0,0.1);
        }
        .header {
            text-align: center;
            color: #333;
            margin-bottom: 20px;
        }
        .header h1 {
            margin: 0 0 10px 0;
        }
        .header p {
            margin: 0;
            color: #666;
        }
        .video-container {
            text-align: center;
            margin-bottom: 20px;
        }
        .video-frame {
            border: 2px solid #ddd;
            border-radius: 8px;
            width: 1280px;
            max-width: 100%;
            height: auto;
            box-shadow: 0 2px 8px rgba(0,0,0,0.1);
        }
        .controls {
            display: flex;
            justify-content: center;
            gap: 10px;
            margin-bottom: 20px;
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
        .btn-freedrive {
            background-color: #28a745;
            color: white;
        }
        .btn-freedrive:hover {
            background-color: #218838;
            transform: translateY(-2px);
            box-shadow: 0 4px 8px rgba(40,167,69,0.3);
        }
        .btn-freedrive.active {
            background-color: #dc3545;
        }
        .btn-freedrive.active:hover {
            background-color: #c82333;
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
            margin-bottom: 15px;
            color: #495057;
            border-bottom: 2px solid #007bff;
            padding-bottom: 10px;
            font-size: 1.2em;
        }
        .info {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(250px, 1fr));
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
            font-size: 0.9em;
            margin-bottom: 5px;
        }
        .info-value {
            display: block;
            font-weight: 500;
            color: #212529;
            font-size: 1em;
        }
        #cameraStatus {
            font-weight: 600;
        }
        .robot-info {
            display: flex;
            flex-direction: column;
            gap: 15px;
        }
        .robot-info-item {
            background-color: white;
            padding: 15px;
            border-radius: 8px;
            border: 1px solid #e9ecef;
            box-shadow: 0 1px 3px rgba(0,0,0,0.1);
        }
        .robot-value {
            font-family: 'Courier New', monospace;
            font-size: 0.9em;
            color: #212529;
            margin-top: 8px;
            padding: 10px;
            background-color: #f8f9fa;
            border-radius: 4px;
            border-left: 3px solid #007bff;
            word-break: break-word;
            white-space: pre-wrap;
        }
        .upload-section {
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(300px, 1fr));
            gap: 20px;
            margin-top: 10px;
        }
        .upload-item {
            background-color: white;
            padding: 20px;
            border-radius: 8px;
            border: 2px dashed #dee2e6;
            text-align: center;
            transition: all 0.3s ease;
        }
        .upload-item:hover {
            border-color: #007bff;
            box-shadow: 0 2px 8px rgba(0,123,255,0.2);
        }
        .upload-label {
            display: flex;
            flex-direction: column;
            align-items: center;
            gap: 15px;
        }
        .upload-title {
            font-weight: 600;
            color: #495057;
            font-size: 1em;
        }
        .btn-upload {
            background-color: #007bff;
            color: white;
            padding: 10px 20px;
            border: none;
            border-radius: 6px;
            cursor: pointer;
            font-size: 14px;
            font-weight: bold;
            transition: all 0.3s ease;
        }
        .btn-upload:hover {
            background-color: #0056b3;
            transform: translateY(-2px);
            box-shadow: 0 4px 8px rgba(0,123,255,0.3);
        }
        .file-name {
            margin-top: 10px;
            font-size: 0.9em;
            color: #6c757d;
            font-style: italic;
        }
        .message-modal {
            display: none;
            position: fixed;
            top: 20px;
            left: 50%;
            transform: translateX(-50%);
            z-index: 2000;
            animation: slideDown 0.3s ease-out;
        }
        @keyframes slideDown {
            from {
                opacity: 0;
                transform: translateX(-50%) translateY(-20px);
            }
            to {
                opacity: 1;
                transform: translateX(-50%) translateY(0);
            }
        }
        .message-content {
            background-color: white;
            padding: 15px 30px;
            border-radius: 8px;
            box-shadow: 0 4px 12px rgba(0,0,0,0.3);
            border-left: 4px solid #007bff;
        }
        .message-content.success {
            border-left-color: #28a745;
        }
        .message-content.error {
            border-left-color: #dc3545;
        }
        .message-text {
            font-size: 1em;
            font-weight: 500;
            color: #333;
        }
        @media (max-width: 768px) {
            .info {
                grid-template-columns: 1fr;
            }
            .upload-section {
                grid-template-columns: 1fr;
            }
        }
    </style>
</head>
<body>
    <div class="container">
        <div class="header">
            <h1>📷 UR15 Camera Calibration Validator</h1>
            <p>Real-time Camera View & Calibration Validation</p>
        </div>
        
        <div class="video-container">
            <img id="videoFeed" class="video-frame" src="/video_feed" alt="Camera Feed">
        </div>
        
        <div class="controls">
            <button id="freedriveBtn" class="btn btn-freedrive" onclick="toggleFreedrive()">
                🎮 Enter Freedrive Mode
            </button>
        </div>
        
        <div class="info-panel">
            <h3>📊 Camera Status</h3>
            <div class="info">
                <div class="info-item">
                    <span class="info-label">Camera Status:</span>
                    <span id="cameraStatus" class="info-value">Waiting for connection...</span>
                </div>
                <div class="info-item">
                    <span class="info-label">Camera Topic:</span>
                    <span id="cameraTopic" class="info-value">{{ camera_topic }}</span>
                </div>
                <div class="info-item">
                    <span class="info-label">Intrinsic Calibration:</span>
                    <span id="intrinsicStatus" class="info-value">Not Loaded</span>
                </div>
                <div class="info-item">
                    <span class="info-label">Extrinsic Calibration:</span>
                    <span id="extrinsicStatus" class="info-value">Not Loaded</span>
                </div>
            </div>
        </div>
        
        <div class="info-panel">
            <h3>🤖 Robot State</h3>
            <div class="robot-info">
                <div class="robot-info-item">
                    <span class="info-label">Joint Positions (°):</span>
                    <div id="jointPositions" class="robot-value">Waiting for data...</div>
                </div>
                <div class="robot-info-item">
                    <span class="info-label">TCP Pose:</span>
                    <div id="tcpPoseDisplay" class="robot-value">Waiting for data...</div>
                </div>
            </div>
        </div>
        
        <div class="info-panel">
            <h3>📁 Load Calibration Data</h3>
            <div class="upload-section">
                <div class="upload-item">
                    <label class="upload-label">
                        <span class="upload-title">📐 Intrinsic Parameters (JSON)</span>
                        <input type="file" id="intrinsicFile" accept=".json" style="display: none;" onchange="uploadIntrinsic()">
                        <button class="btn btn-upload" onclick="document.getElementById('intrinsicFile').click()">Choose File</button>
                    </label>
                    <span id="intrinsicFileName" class="file-name">No file selected</span>
                </div>
                
                <div class="upload-item">
                    <label class="upload-label">
                        <span class="upload-title">🎯 Extrinsic Parameters (JSON)</span>
                        <input type="file" id="extrinsicFile" accept=".json" style="display: none;" onchange="uploadExtrinsic()">
                        <button class="btn btn-upload" onclick="document.getElementById('extrinsicFile').click()">Choose File</button>
                    </label>
                    <span id="extrinsicFileName" class="file-name">No file selected</span>
                </div>
            </div>
        </div>
    </div>
    
    <div id="messageModal" class="message-modal">
        <div class="message-content">
            <span class="message-text" id="messageText"></span>
        </div>
    </div>

    <script>
        let freedriveActive = false;

        function showMessage(message, type = 'info') {
            const modal = document.getElementById('messageModal');
            const content = modal.querySelector('.message-content');
            const text = document.getElementById('messageText');
            
            text.textContent = message;
            content.className = 'message-content ' + type;
            modal.style.display = 'block';
            
            setTimeout(() => {
                modal.style.display = 'none';
            }, 3000);
        }

        function toggleFreedrive() {
            const btn = document.getElementById('freedriveBtn');
            btn.disabled = true;
            
            fetch('/toggle_freedrive', {
                method: 'POST',
                headers: {
                    'Content-Type': 'application/json',
                }
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    freedriveActive = data.freedrive_active;
                    updateFreedriveBtnUI();
                    showMessage(data.message, 'success');
                } else {
                    showMessage('❌ ' + data.message, 'error');
                }
                btn.disabled = false;
            })
            .catch(error => {
                showMessage('❌ Request failed: ' + error.message, 'error');
                btn.disabled = false;
            });
        }

        function updateFreedriveBtnUI() {
            const btn = document.getElementById('freedriveBtn');
            if (freedriveActive) {
                btn.textContent = '🛑 Exit Freedrive Mode';
                btn.className = 'btn btn-freedrive active';
            } else {
                btn.textContent = '🎮 Enter Freedrive Mode';
                btn.className = 'btn btn-freedrive';
            }
        }

        function uploadIntrinsic() {
            const fileInput = document.getElementById('intrinsicFile');
            const file = fileInput.files[0];
            
            if (!file) {
                return;
            }
            
            document.getElementById('intrinsicFileName').textContent = file.name;
            
            const formData = new FormData();
            formData.append('file', file);
            
            fetch('/upload_intrinsic', {
                method: 'POST',
                body: formData
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showMessage('✅ Intrinsic parameters loaded successfully', 'success');
                    updateStatus();
                } else {
                    showMessage('❌ Failed to load intrinsic parameters: ' + data.message, 'error');
                }
            })
            .catch(error => {
                showMessage('❌ Upload failed: ' + error.message, 'error');
            });
        }

        function uploadExtrinsic() {
            const fileInput = document.getElementById('extrinsicFile');
            const file = fileInput.files[0];
            
            if (!file) {
                return;
            }
            
            document.getElementById('extrinsicFileName').textContent = file.name;
            
            const formData = new FormData();
            formData.append('file', file);
            
            fetch('/upload_extrinsic', {
                method: 'POST',
                body: formData
            })
            .then(response => response.json())
            .then(data => {
                if (data.success) {
                    showMessage('✅ Extrinsic parameters loaded successfully', 'success');
                    updateStatus();
                } else {
                    showMessage('❌ Failed to load extrinsic parameters: ' + data.message, 'error');
                }
            })
            .catch(error => {
                showMessage('❌ Upload failed: ' + error.message, 'error');
            });
        }

        function updateStatus() {
            fetch('/get_status')
            .then(response => response.json())
            .then(data => {
                // Update camera status
                const cameraStatusElement = document.getElementById('cameraStatus');
                if (data.has_image) {
                    cameraStatusElement.textContent = 'Connected';
                    cameraStatusElement.style.color = 'green';
                } else {
                    cameraStatusElement.textContent = 'Waiting for connection...';
                    cameraStatusElement.style.color = 'red';
                }
                
                // Update camera topic
                document.getElementById('cameraTopic').textContent = data.camera_topic;
                
                // Update intrinsic calibration status
                const intrinsicStatusElement = document.getElementById('intrinsicStatus');
                if (data.has_intrinsic) {
                    intrinsicStatusElement.textContent = 'Loaded ✓';
                    intrinsicStatusElement.style.color = 'green';
                } else {
                    intrinsicStatusElement.textContent = 'Not Loaded';
                    intrinsicStatusElement.style.color = 'red';
                }
                
                // Update extrinsic calibration status
                const extrinsicStatusElement = document.getElementById('extrinsicStatus');
                if (data.has_extrinsic) {
                    extrinsicStatusElement.textContent = 'Loaded ✓';
                    extrinsicStatusElement.style.color = 'green';
                } else {
                    extrinsicStatusElement.textContent = 'Not Loaded';
                    extrinsicStatusElement.style.color = 'red';
                }
                
                // Update joint positions
                const jointPositionsElement = document.getElementById('jointPositions');
                if (data.joint_positions && data.joint_positions.length > 0) {
                    const jointText = data.joint_positions.map((pos, idx) => 
                        `J${idx + 1}: ${pos.toFixed(2)}°`
                    ).join('\\n');
                    jointPositionsElement.textContent = jointText;
                    jointPositionsElement.style.color = '#212529';
                } else {
                    jointPositionsElement.textContent = 'Waiting for data...';
                    jointPositionsElement.style.color = '#6c757d';
                }
                
                // Update TCP pose
                const tcpPoseElement = document.getElementById('tcpPoseDisplay');
                if (data.tcp_pose) {
                    const pose = data.tcp_pose;
                    const poseText = `Position (mm):\\n  X: ${pose.x.toFixed(2)}\\n  Y: ${pose.y.toFixed(2)}\\n  Z: ${pose.z.toFixed(2)}\\n\\nOrientation (quaternion):\\n  X: ${pose.qx.toFixed(4)}\\n  Y: ${pose.qy.toFixed(4)}\\n  Z: ${pose.qz.toFixed(4)}\\n  W: ${pose.qw.toFixed(4)}`;
                    tcpPoseElement.textContent = poseText;
                    tcpPoseElement.style.color = '#212529';
                } else {
                    tcpPoseElement.textContent = 'Waiting for data...';
                    tcpPoseElement.style.color = '#6c757d';
                }
                
                // Update freedrive button state
                if (data.freedrive_active !== undefined) {
                    freedriveActive = data.freedrive_active;
                    updateFreedriveBtnUI();
                }
            })
            .catch(error => {
                console.error('Status fetch failed:', error);
                document.getElementById('cameraStatus').textContent = 'Connection Error';
                document.getElementById('cameraStatus').style.color = 'red';
            });
        }

        // Update status every 500ms
        setInterval(() => {
            updateStatus();
        }, 500);

        // Update status on page load
        window.onload = function() {
            updateStatus();
        };
    </script>
</body>
</html>
            """
            return render_template_string(html_template, camera_topic=self.camera_topic)
        
        @self.app.route('/video_feed')
        def video_feed():
            """Video streaming route."""
            return Response(self.generate_frames(),
                          mimetype='multipart/x-mixed-replace; boundary=frame')
        
        @self.app.route('/get_status')
        def get_status():
            """Get current status."""
            from flask import jsonify
            has_image = self.current_image is not None
            
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
            
            # Check if data is recent (within last 2 seconds)
            current_time = time.time()
            joint_data_valid = last_joint_update and (current_time - last_joint_update) < 2.0
            tcp_data_valid = last_tcp_update and (current_time - last_tcp_update) < 2.0
            
            return jsonify({
                'has_image': has_image,
                'camera_topic': self.camera_topic,
                'has_intrinsic': has_intrinsic,
                'has_extrinsic': has_extrinsic,
                'joint_positions': joint_positions if joint_data_valid else [],
                'tcp_pose': tcp_pose if tcp_data_valid else None,
                'freedrive_active': freedrive_active
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
                        frame = self.project_base_origin_to_image(frame)
                        
                        height, width = frame.shape[:2]
                        if width > 800:
                            scale = 800.0 / width
                            new_width = int(width * scale)
                            new_height = int(height * scale)
                            frame = cv2.resize(frame, (new_width, new_height))
                        
                        encode_param = [cv2.IMWRITE_JPEG_QUALITY, 85]
                        _, buffer = cv2.imencode('.jpg', frame, encode_param)
                        frame_bytes = buffer.tobytes()
                        
                        yield (b'--frame\r\n'
                               b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                    else:
                        placeholder = self.create_placeholder_image()
                        _, buffer = cv2.imencode('.jpg', placeholder, [cv2.IMWRITE_JPEG_QUALITY, 85])
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
        node = UR15CameraValidateNode()
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
