#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import threading
import time
from flask import Flask, render_template_string, Response
from flask_socketio import SocketIO, emit
import base64


class ImageWebViewerNode(Node):
    def __init__(self):
        super().__init__('image_web_viewer_node')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Parameters
        self.declare_parameter('raw_topic', '/camera/image_raw')
        self.declare_parameter('processed_topic', '/camera/image_undistorted')
        self.declare_parameter('resized_compressed_topic', '/camera/image_undistorted_resize_jpeg')
        self.declare_parameter('web_port', 8080)
        
        self.raw_topic = self.get_parameter('raw_topic').value
        self.processed_topic = self.get_parameter('processed_topic').value
        self.resized_compressed_topic = self.get_parameter('resized_compressed_topic').value
        self.web_port = self.get_parameter('web_port').value
        
        # Image storage
        self.raw_image = None
        self.processed_image = None
        self.resized_compressed_image = None
        self.image_lock = threading.Lock()
        
        # Create subscribers
        self.raw_subscription = self.create_subscription(
            Image,
            self.raw_topic,
            self.raw_image_callback,
            10
        )
        
        self.processed_subscription = self.create_subscription(
            Image,
            self.processed_topic,
            self.processed_image_callback,
            10
        )
        
        self.resized_compressed_subscription = self.create_subscription(
            CompressedImage,
            self.resized_compressed_topic,
            self.resized_compressed_image_callback,
            10
        )
        
        # Initialize Flask app and SocketIO
        self.app = Flask(__name__)
        self.app.config['SECRET_KEY'] = 'image_viewer_secret'
        self.socketio = SocketIO(self.app, cors_allowed_origins="*")
        self.setup_flask_routes()
        self.setup_socketio_events()
        
        # Start web server in a separate thread
        self.web_thread = threading.Thread(target=self.start_web_server, daemon=True)
        self.web_thread.start()
        
        self.get_logger().info('Image web viewer node started with WebSocket support')
        self.get_logger().info(f'Raw image topic: {self.raw_topic}')
        self.get_logger().info(f'Processed image topic: {self.processed_topic}')
        self.get_logger().info(f'Resized compressed image topic: {self.resized_compressed_topic}')
        self.get_logger().info(f'Web interface available at: http://localhost:{self.web_port}')
        
    def raw_image_callback(self, msg):
        """Callback for raw image topic"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.image_lock:
                self.raw_image = cv_image.copy()
            
            # Send via WebSocket
            image_b64 = self.get_image_base64(cv_image)
            if image_b64:
                self.socketio.emit('raw_image_update', {'image': image_b64})
        except Exception as e:
            self.get_logger().error(f'Error processing raw image: {str(e)}')
            
    def processed_image_callback(self, msg):
        """Callback for processed image topic"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.image_lock:
                self.processed_image = cv_image.copy()
            
            # Send via WebSocket
            image_b64 = self.get_image_base64(cv_image)
            if image_b64:
                self.socketio.emit('processed_image_update', {'image': image_b64})
        except Exception as e:
            self.get_logger().error(f'Error processing undistorted image: {str(e)}')
            
    def resized_compressed_image_callback(self, msg):
        """Callback for resized compressed image topic"""
        try:
            # Decode compressed image
            import numpy as np
            np_arr = np.frombuffer(msg.data, np.uint8)
            cv_image = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
            
            if cv_image is not None:
                with self.image_lock:
                    self.resized_compressed_image = cv_image.copy()
                
                # Send via WebSocket
                image_b64 = self.get_image_base64(cv_image)
                if image_b64:
                    self.socketio.emit('resized_image_update', {'image': image_b64})
            else:
                self.get_logger().error('Failed to decode compressed image')
        except Exception as e:
            self.get_logger().error(f'Error processing resized compressed image: {str(e)}')
            
    def get_image_base64(self, image):
        """Convert OpenCV image to base64 string for web display"""
        if image is None:
            return None
            
        # Resize image for web display (optional, for better performance)
        height, width = image.shape[:2]
        if width > 800:
            scale = 800 / width
            new_width = 800
            new_height = int(height * scale)
            image = cv2.resize(image, (new_width, new_height))
        
        # Encode image as JPEG
        _, buffer = cv2.imencode('.jpg', image, [cv2.IMWRITE_JPEG_QUALITY, 85])
        
        # Convert to base64
        image_base64 = base64.b64encode(buffer).decode('utf-8')
        return image_base64
    
    def setup_socketio_events(self):
        """Setup SocketIO event handlers"""
        
        @self.socketio.on('connect')
        def handle_connect():
            self.get_logger().info('Client connected to WebSocket')
            
        @self.socketio.on('disconnect')
        def handle_disconnect():
            self.get_logger().info('Client disconnected from WebSocket')
        
    def setup_flask_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            """Main page showing real-time images"""
            html_template = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>Real-time Camera Image Viewer</title>
                <script src="https://cdnjs.cloudflare.com/ajax/libs/socket.io/4.0.1/socket.io.js"></script>
                <style>
                    body { 
                        font-family: Arial, sans-serif; 
                        margin: 20px;
                        background-color: #f0f0f0;
                    }
                    .container {
                        max-width: 1800px;
                        margin: 0 auto;
                    }
                    .header {
                        text-align: center;
                        margin-bottom: 30px;
                        background-color: white;
                        padding: 20px;
                        border-radius: 10px;
                        box-shadow: 0 2px 5px rgba(0,0,0,0.1);
                    }
                    .connection-info {
                        text-align: center;
                        margin-bottom: 20px;
                        padding: 10px;
                        background-color: white;
                        border-radius: 5px;
                        box-shadow: 0 1px 3px rgba(0,0,0,0.1);
                    }
                    .connection-status {
                        display: inline-block;
                        width: 10px;
                        height: 10px;
                        border-radius: 50%;
                        margin-right: 8px;
                    }
                    .connected {
                        background-color: #4CAF50;
                    }
                    .disconnected {
                        background-color: #f44336;
                    }
                    .images-container {
                        display: flex;
                        gap: 20px;
                        flex-wrap: wrap;
                        justify-content: center;
                    }
                    .image-box {
                        background-color: white;
                        padding: 20px;
                        border-radius: 10px;
                        box-shadow: 0 2px 5px rgba(0,0,0,0.1);
                        text-align: center;
                        flex: 1;
                        min-width: 300px;
                        max-width: 450px;
                    }
                    .image-title {
                        font-size: 18px;
                        font-weight: bold;
                        margin-bottom: 15px;
                        color: #333;
                    }
                    .topic-name {
                        font-size: 12px;
                        color: #666;
                        font-family: monospace;
                        margin-bottom: 10px;
                    }
                    img {
                        max-width: 100%;
                        height: auto;
                        border: 2px solid #ddd;
                        border-radius: 5px;
                        transition: opacity 0.3s ease;
                        display: none;
                    }
                    .no-image {
                        width: 400px;
                        height: 300px;
                        background-color: #f8f8f8;
                        border: 2px dashed #ccc;
                        display: flex;
                        align-items: center;
                        justify-content: center;
                        color: #999;
                        font-style: italic;
                        border-radius: 5px;
                    }
                    .status {
                        margin-top: 10px;
                        font-size: 12px;
                        color: #666;
                    }
                </style>
                <script>
                    const socket = io();
                    
                    // Connection status
                    socket.on('connect', function() {
                        console.log('Connected to server');
                        document.getElementById('connection-status').className = 'connection-status connected';
                        document.getElementById('status-text').textContent = 'Connected - Real-time streaming active';
                    });
                    
                    socket.on('disconnect', function() {
                        console.log('Disconnected from server');
                        document.getElementById('connection-status').className = 'connection-status disconnected';
                        document.getElementById('status-text').textContent = 'Disconnected - Trying to reconnect...';
                    });
                    
                    // Image updates
                    socket.on('raw_image_update', function(data) {
                        const img = document.getElementById('raw-image');
                        const placeholder = document.getElementById('raw-placeholder');
                        const status = document.getElementById('raw-status');
                        if (img && data.image) {
                            img.src = 'data:image/jpeg;base64,' + data.image;
                            img.style.display = 'block';
                            placeholder.style.display = 'none';
                            status.innerHTML = '✅ Live stream active';
                        }
                    });
                    
                    socket.on('processed_image_update', function(data) {
                        const img = document.getElementById('processed-image');
                        const placeholder = document.getElementById('processed-placeholder');
                        const status = document.getElementById('processed-status');
                        if (img && data.image) {
                            img.src = 'data:image/jpeg;base64,' + data.image;
                            img.style.display = 'block';
                            placeholder.style.display = 'none';
                            status.innerHTML = '✅ Live stream active';
                        }
                    });
                    
                    socket.on('resized_image_update', function(data) {
                        const img = document.getElementById('resized-image');
                        const placeholder = document.getElementById('resized-placeholder');
                        const status = document.getElementById('resized-status');
                        if (img && data.image) {
                            img.src = 'data:image/jpeg;base64,' + data.image;
                            img.style.display = 'block';
                            placeholder.style.display = 'none';
                            status.innerHTML = '✅ Live stream active';
                        }
                    });
                </script>
            </head>
            <body>
                <div class="container">
                    <div class="header">
                        <h1>🎥 Real-time Camera Image Viewer</h1>
                        <p>WebSocket-powered live streaming without page refresh</p>
                    </div>
                    
                    <div class="connection-info">
                        <span id="connection-status" class="connection-status disconnected"></span>
                        <span id="status-text">Connecting...</span>
                    </div>
                    
                    <div class="images-container">
                        <div class="image-box">
                            <div class="image-title">🔸 Raw Image</div>
                            <div class="topic-name">{{ raw_topic }}</div>
                            <img id="raw-image" alt="Raw Image">
                            <div id="raw-placeholder" class="no-image">Waiting for live stream...</div>
                            <div id="raw-status" class="status">⏳ Waiting for data...</div>
                        </div>
                        
                        <div class="image-box">
                            <div class="image-title">🔹 Undistorted Image</div>
                            <div class="topic-name">{{ processed_topic }}</div>
                            <img id="processed-image" alt="Undistorted Image">
                            <div id="processed-placeholder" class="no-image">Waiting for live stream...</div>
                            <div id="processed-status" class="status">⏳ Waiting for data...</div>
                        </div>
                        
                        <div class="image-box">
                            <div class="image-title">🔸 Resized Compressed Image</div>
                            <div class="topic-name">{{ resized_compressed_topic }}</div>
                            <img id="resized-image" alt="Resized Compressed Image">
                            <div id="resized-placeholder" class="no-image">Waiting for live stream...</div>
                            <div id="resized-status" class="status">⏳ Waiting for data...</div>
                        </div>
                    </div>
                </div>
            </body>
            </html>
            """
            
            return render_template_string(
                html_template,
                raw_topic=self.raw_topic,
                processed_topic=self.processed_topic,
                resized_compressed_topic=self.resized_compressed_topic
            )
    
    def start_web_server(self):
        """Start the web server"""
        try:
            self.socketio.run(self.app, host='0.0.0.0', port=self.web_port, debug=False, allow_unsafe_werkzeug=True)
        except Exception as e:
            self.get_logger().error(f'Failed to start web server: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    image_web_viewer_node = ImageWebViewerNode()
    
    try:
        rclpy.spin(image_web_viewer_node)
    except KeyboardInterrupt:
        pass
    finally:
        image_web_viewer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
