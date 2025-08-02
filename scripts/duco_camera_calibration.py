#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np
import os
import threading
from datetime import datetime
import argparse
import socket
import struct
import time
import math
from flask import Flask, render_template_string, Response, jsonify


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
        super().__init__('camera_node')
        
        # Parameters
        self.camera_topic = camera_topic
        self.save_dir = save_dir
        
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
        
        # Image counter for naming
        self.image_counter = 0
        
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
        
        # Create subscriber for camera images
        self.image_subscriber = self.create_subscription(
            Image,
            self.camera_topic,
            self.image_callback,
            10
        )
        
        # Initialize Flask app with better shutdown handling
        self.app = Flask(__name__)
        self.app.config['THREADED'] = True
        
        self.setup_flask_routes()
        
        self.get_logger().info("Camera Calibration Node started")
        self.get_logger().info(f"Subscribing to: {self.camera_topic}")
        self.get_logger().info(f"Save directory: {os.path.abspath(self.save_dir)}")
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
            gap: 10px;
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
            height: 120px;
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
                <button class="btn btn-danger" onclick="clearImages()">🗑️ Clear Images</button>
            </div>
            
            <div id="statusMessage" class="status hidden"></div>
            
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
            <h4>📸 Recent Screenshots</h4>
            <div id="thumbnailGrid" class="thumbnail-grid">
                <!-- Thumbnails will be populated here -->
            </div>
        </div>
    </div>

    <script>
        let imageCount = 0;

        function showStatus(message, type = 'info') {
            const statusElement = document.getElementById('statusMessage');
            statusElement.textContent = message;
            statusElement.className = `status status-${type}`;
            statusElement.classList.remove('hidden');
            
            setTimeout(() => {
                statusElement.classList.add('hidden');
            }, 3000);
        }

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
                        
                        // 使用序号而不是时间戳，从最新的开始编号
                        const sequenceNumber = String(index).padStart(4, '0');
                        
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
            """Clear all calibration images and angle files."""
            try:
                import glob
                # Clear image files
                image_files = glob.glob(os.path.join(self.save_dir, 'screenshot_*.jpg'))
                # Clear angle files
                angle_files = glob.glob(os.path.join(self.save_dir, 'screenshot_*_angles.txt'))
                
                for file_path in image_files + angle_files:
                    os.remove(file_path)
                    
                self.image_counter = 0
                return jsonify({
                    'success': True,
                    'message': f'Deleted {len(image_files)} images and {len(angle_files)} angle files'
                })
            except Exception as e:
                return jsonify({
                    'success': False,
                    'message': f'Delete failed: {str(e)}'
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
            
            return jsonify({
                'image_count': self.image_counter,
                'has_image': has_image,
                'image_shape': image_shape,
                'camera_topic': self.camera_topic,
                'save_directory': os.path.abspath(self.save_dir),
                'robot_connected': robot_connected,
                'joint_angles': joint_angles,
                'tcp_pose': tcp_pose
            })
        
        @self.app.route('/get_thumbnails')
        def get_thumbnails():
            """Get list of recent screenshots for thumbnails."""
            try:
                import glob
                image_files = glob.glob(os.path.join(self.save_dir, 'screenshot_*.jpg'))
                # Sort by modification time, newest first
                image_files.sort(key=os.path.getmtime, reverse=True)
                # Get only the last 8 files
                recent_files = image_files[:8]
                
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
        """Save the current camera image as a screenshot with joint angles."""
        with self.image_lock:
            if self.current_image is not None:
                # Get current joint angles and TCP pose
                with self.joint_lock:
                    joint_angles = self.joint_angles.copy()
                    tcp_pose = self.tcp_pose.copy()
                    robot_connected = self.robot_connected
                
                # Generate filename with timestamp
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = f"screenshot_{timestamp}_{self.image_counter:04d}.jpg"
                filepath = os.path.join(self.save_dir, filename)
                
                # Save the image
                success = cv2.imwrite(filepath, self.current_image)
                
                if success:
                    # Save joint angles and TCP pose to text file
                    data_filename = f"screenshot_{timestamp}_{self.image_counter:04d}_pose_data.txt"
                    data_filepath = os.path.join(self.save_dir, data_filename)
                    
                    try:
                        with open(data_filepath, 'w', encoding='utf-8') as f:
                            f.write(f"Screenshot: {filename}\n")
                            f.write(f"Timestamp: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}\n")
                            f.write(f"Robot Connected: {robot_connected}\n")
                            
                            if robot_connected and len(joint_angles) >= 6:
                                # Write joint angles
                                f.write("\nJoint Angles (degrees):\n")
                                for i, angle in enumerate(joint_angles[:6]):
                                    f.write(f"  Joint {i+1}: {angle:.3f}°\n")
                                angles_str = '[' + ', '.join([f'{angle:.3f}°' for angle in joint_angles[:6]]) + ']'
                                f.write(f"Joint Angles Array: {angles_str}\n")
                                
                                # Write TCP pose
                                if len(tcp_pose) >= 6:
                                    f.write("\nTCP Pose:\n")
                                    f.write(f"  X: {tcp_pose[0]:.1f} mm\n")
                                    f.write(f"  Y: {tcp_pose[1]:.1f} mm\n")
                                    f.write(f"  Z: {tcp_pose[2]:.1f} mm\n")
                                    f.write(f"  Rx: {tcp_pose[3]:.3f}°\n")
                                    f.write(f"  Ry: {tcp_pose[4]:.3f}°\n")
                                    f.write(f"  Rz: {tcp_pose[5]:.3f}°\n")
                                    tcp_str = '[' + ', '.join([f'{tcp_pose[i]:.1f} mm' if i < 3 else f'{tcp_pose[i]:.3f}°' for i in range(6)]) + ']'
                                    f.write(f"TCP Pose Array: {tcp_str}\n")
                                else:
                                    f.write("\nTCP Pose: Not available\n")
                            else:
                                f.write("Joint Angles: Not available (robot disconnected)\n")
                                f.write("TCP Pose: Not available (robot disconnected)\n")
                    except Exception as e:
                        self.get_logger().warning(f"Failed to save pose data: {e}")
                    
                    self.image_counter += 1
                    
                    # Create result message
                    if robot_connected and len(joint_angles) >= 6:
                        angles_str = '[' + ', '.join([f'{angle:.1f}°' for angle in joint_angles[:6]]) + ']'
                        if len(tcp_pose) >= 6:
                            tcp_str = f'[{tcp_pose[0]:.0f},{tcp_pose[1]:.0f},{tcp_pose[2]:.0f}mm; {tcp_pose[3]:.1f},{tcp_pose[4]:.1f},{tcp_pose[5]:.1f}°]'
                            message = f'Screenshot saved: {filename} with angles: {angles_str} and TCP: {tcp_str}'
                        else:
                            message = f'Screenshot saved: {filename} with angles: {angles_str}'
                    else:
                        message = f'Screenshot saved: {filename} (no robot data)'
                    
                    self.get_logger().info(f"Screenshot saved: {filepath}")
                    if robot_connected:
                        if len(joint_angles) >= 6:
                            angles_str = '[' + ', '.join([f'{angle:.1f}°' for angle in joint_angles[:6]]) + ']'
                            self.get_logger().info(f"Joint angles recorded: {angles_str}")
                        if len(tcp_pose) >= 6:
                            tcp_str = f'TCP: [{tcp_pose[0]:.0f},{tcp_pose[1]:.0f},{tcp_pose[2]:.0f}mm; {tcp_pose[3]:.1f},{tcp_pose[4]:.1f},{tcp_pose[5]:.1f}°]'
                            self.get_logger().info(f"TCP pose recorded: {tcp_str}")
                    
                    return {
                        'success': True,
                        'filename': filename,
                        'message': message
                    }
                else:
                    self.get_logger().error(f"Failed to save screenshot: {filepath}")
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
        print("💡 Open the above link in your browser to start using")
        print("⏹️  Press Ctrl+C to exit\n")
        
        # Keep the node running
        rclpy.spin(calibration_node)
        
    except KeyboardInterrupt:
        print("\n👋 User requested shutdown")
    except Exception as e:
        print(f"❌ Error: {e}")
    finally:
        # Cleanup
        if 'calibration_node' in locals():
            print("🔄 Shutting down web server...")
            calibration_node.shutdown_flask()
            calibration_node.destroy_node()
        rclpy.shutdown()
        print("🔄 Cleanup completed")


if __name__ == '__main__':
    main()
