#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
import threading
import time
from flask import Flask, render_template_string, Response
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
        
        # Initialize Flask app
        self.app = Flask(__name__)
        self.setup_flask_routes()
        
        # Start web server in a separate thread
        self.web_thread = threading.Thread(target=self.start_web_server, daemon=True)
        self.web_thread.start()
        
        self.get_logger().info('Image web viewer node started')
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
        except Exception as e:
            self.get_logger().error(f'Error processing raw image: {str(e)}')
            
    def processed_image_callback(self, msg):
        """Callback for processed image topic"""
        try:
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            with self.image_lock:
                self.processed_image = cv_image.copy()
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
        
    def setup_flask_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            """Main page showing both images"""
            html_template = """
            <!DOCTYPE html>
            <html>
            <head>
                <title>Camera Image Viewer</title>
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
                    .refresh-info {
                        text-align: center;
                        margin-top: 20px;
                        color: #666;
                        font-size: 14px;
                    }
                </style>
                <script>
                    // Auto-refresh every 2 seconds
                    setTimeout(function(){
                        location.reload();
                    }, 2000);
                </script>
            </head>
            <body>
                <div class="container">
                    <div class="header">
                        <h1>🎥 Camera Image Viewer</h1>
                        <p>Real-time comparison of raw, undistorted, and resized camera images</p>
                    </div>
                    
                    <div class="images-container">
                        <div class="image-box">
                            <div class="image-title">🔸 Raw Image</div>
                            <div class="topic-name">{{ raw_topic }}</div>
                            {% if raw_image %}
                                <img src="data:image/jpeg;base64,{{ raw_image }}" alt="Raw Image">
                                <div class="status">✅ Receiving data</div>
                            {% else %}
                                <div class="no-image">No image received yet</div>
                                <div class="status">⏳ Waiting for data...</div>
                            {% endif %}
                        </div>
                        
                        <div class="image-box">
                            <div class="image-title">🔹 Undistorted Image</div>
                            <div class="topic-name">{{ processed_topic }}</div>
                            {% if processed_image %}
                                <img src="data:image/jpeg;base64,{{ processed_image }}" alt="Undistorted Image">
                                <div class="status">✅ Receiving data</div>
                            {% else %}
                                <div class="no-image">No image received yet</div>
                                <div class="status">⏳ Waiting for data...</div>
                            {% endif %}
                        </div>
                        
                        <div class="image-box">
                            <div class="image-title">🔸 Resized Compressed Image</div>
                            <div class="topic-name">{{ resized_compressed_topic }}</div>
                            {% if resized_compressed_image %}
                                <img src="data:image/jpeg;base64,{{ resized_compressed_image }}" alt="Resized Compressed Image">
                                <div class="status">✅ Receiving data</div>
                            {% else %}
                                <div class="no-image">No image received yet</div>
                                <div class="status">⏳ Waiting for data...</div>
                            {% endif %}
                        </div>
                    </div>
                    
                    <div class="refresh-info">
                        🔄 Page auto-refreshes every 2 seconds | Current time: {{ current_time }}
                    </div>
                </div>
            </body>
            </html>
            """
            
            with self.image_lock:
                raw_image_b64 = self.get_image_base64(self.raw_image)
                processed_image_b64 = self.get_image_base64(self.processed_image)
                resized_compressed_image_b64 = self.get_image_base64(self.resized_compressed_image)
            
            return render_template_string(
                html_template,
                raw_image=raw_image_b64,
                processed_image=processed_image_b64,
                resized_compressed_image=resized_compressed_image_b64,
                raw_topic=self.raw_topic,
                processed_topic=self.processed_topic,
                resized_compressed_topic=self.resized_compressed_topic,
                current_time=time.strftime('%Y-%m-%d %H:%M:%S')
            )
            
        @self.app.route('/raw')
        def raw_stream():
            """Stream raw image"""
            def generate():
                while True:
                    with self.image_lock:
                        if self.raw_image is not None:
                            _, buffer = cv2.imencode('.jpg', self.raw_image)
                            yield (b'--frame\r\n'
                                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                    time.sleep(0.1)
            return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
            
        @self.app.route('/processed')
        def processed_stream():
            """Stream processed image"""
            def generate():
                while True:
                    with self.image_lock:
                        if self.processed_image is not None:
                            _, buffer = cv2.imencode('.jpg', self.processed_image)
                            yield (b'--frame\r\n'
                                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                    time.sleep(0.1)
            return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')
            
    def start_web_server(self):
        """Start the Flask web server"""
        try:
            self.app.run(host='0.0.0.0', port=self.web_port, debug=False, use_reloader=False)
        except Exception as e:
            self.get_logger().error(f'Web server error: {str(e)}')


def main(args=None):
    rclpy.init(args=args)
    
    web_viewer_node = ImageWebViewerNode()
    
    try:
        rclpy.spin(web_viewer_node)
    except KeyboardInterrupt:
        pass
    finally:
        web_viewer_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
