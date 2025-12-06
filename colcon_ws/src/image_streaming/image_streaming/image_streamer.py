#!/usr/bin/env python3
"""
Image Streaming Node - Subscribes to an image topic and serves it via HTTP.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import threading
from http.server import HTTPServer, BaseHTTPRequestHandler
from urllib.parse import urlparse
import json
import os
import time
from ament_index_python.packages import get_package_share_directory


class ImageStreamer(Node):
    """ROS 2 node that streams images from a topic to a web interface."""

    def __init__(self):
        super().__init__('image_streamer')
        
        # Declare parameters
        self.declare_parameter('image_topic', '/camera/image_raw')
        self.declare_parameter('port', 8080)
        self.declare_parameter('quality', 85)
        
        # Get parameters
        self.image_topic = self.get_parameter('image_topic').value
        self.port = self.get_parameter('port').value
        self.quality = self.get_parameter('quality').value
        
        self.get_logger().info(f'Starting image streamer on port {self.port}')
        self.get_logger().info(f'Subscribing to topic: {self.image_topic}')
        
        # Initialize CV Bridge
        self.bridge = CvBridge()
        
        # Store latest image
        self.latest_image = None
        self.lock = threading.Lock()
        
        # FPS and resolution tracking
        self.frame_count = 0
        self.last_fps_time = time.time()
        self.current_fps = 0.0
        self.image_width = 0
        self.image_height = 0
        
        # Create subscriber
        self.subscription = self.create_subscription(
            Image,
            self.image_topic,
            self.image_callback,
            10
        )
        
        # Start HTTP server in a separate thread
        self.server_thread = threading.Thread(target=self.start_server, daemon=True)
        self.server_thread.start()

    def image_callback(self, msg):
        """Callback for image messages."""
        try:
            # Convert ROS Image message to OpenCV image
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # Update resolution
            height, width = cv_image.shape[:2]
            with self.lock:
                self.image_height = height
                self.image_width = width
            
            # Calculate FPS
            self.frame_count += 1
            current_time = time.time()
            time_diff = current_time - self.last_fps_time
            
            if time_diff >= 1.0:  # Update FPS every second
                with self.lock:
                    self.current_fps = self.frame_count / time_diff
                self.frame_count = 0
                self.last_fps_time = current_time
            
            # Encode as JPEG
            _, buffer = cv2.imencode('.jpg', cv_image, 
                                    [int(cv2.IMWRITE_JPEG_QUALITY), self.quality])
            
            # Store the encoded image
            with self.lock:
                self.latest_image = buffer.tobytes()
                
        except Exception as e:
            self.get_logger().error(f'Error processing image: {str(e)}')

    def get_latest_image(self):
        """Get the latest image frame."""
        with self.lock:
            return self.latest_image
    
    def get_stream_info(self):
        """Get current stream information."""
        with self.lock:
            return {
                'fps': round(self.current_fps, 1),
                'width': self.image_width,
                'height': self.image_height,
                'topic': self.image_topic,
                'quality': self.quality
            }

    def start_server(self):
        """Start the HTTP server."""
        # Create a reference to self for the handler
        node = self
        
        class StreamHandler(BaseHTTPRequestHandler):
            """HTTP request handler for streaming images."""
            
            def log_message(self, format, *args):
                """Suppress default logging."""
                pass
            
            def do_GET(self):
                """Handle GET requests."""
                # Parse path without query parameters
                parsed_path = urlparse(self.path).path
                
                if parsed_path == '/':
                    # Serve the main page
                    self.send_response(200)
                    self.send_header('Content-type', 'text/html')
                    self.end_headers()
                    html = self.get_html_page()
                    self.wfile.write(html.encode())
                
                elif parsed_path == '/info':
                    # Serve stream info as JSON
                    self.send_response(200)
                    self.send_header('Content-type', 'application/json')
                    self.end_headers()
                    info = node.get_stream_info()
                    self.wfile.write(json.dumps(info).encode())
                    
                elif parsed_path == '/stream':
                    # Serve the MJPEG stream
                    self.send_response(200)
                    self.send_header('Content-type', 'multipart/x-mixed-replace; boundary=frame')
                    self.end_headers()
                    
                    try:
                        while True:
                            image = node.get_latest_image()
                            if image is not None:
                                self.wfile.write(b'--frame\r\n')
                                self.send_header('Content-type', 'image/jpeg')
                                self.send_header('Content-length', str(len(image)))
                                self.end_headers()
                                self.wfile.write(image)
                                self.wfile.write(b'\r\n')
                            else:
                                # Wait a bit if no image is available
                                threading.Event().wait(0.1)
                    except (ConnectionResetError, BrokenPipeError):
                        pass
                        
                elif parsed_path == '/image.jpg':
                    # Serve a single image
                    image = node.get_latest_image()
                    if image is not None:
                        self.send_response(200)
                        self.send_header('Content-type', 'image/jpeg')
                        self.send_header('Content-length', str(len(image)))
                        self.end_headers()
                        self.wfile.write(image)
                    else:
                        self.send_response(503)
                        self.end_headers()
                        self.wfile.write(b'No image available yet')
                else:
                    self.send_response(404)
                    self.end_headers()
            
            def get_html_page(self):
                """Generate the HTML page for viewing the stream."""
                return f"""
<!DOCTYPE html>
<html>
<head>
    <title>ROS 2 Image Stream - {node.image_topic}</title>
    <style>
        body {{
            font-family: Arial, sans-serif;
            margin: 0;
            padding: 20px;
            background-color: #f0f0f0;
            display: flex;
            flex-direction: column;
            align-items: center;
        }}
        h1 {{
            color: #333;
            margin-bottom: 10px;
        }}
        .info {{
            color: #666;
            margin-bottom: 15px;
            background-color: #f8f9fa;
            padding: 15px;
            border-radius: 6px;
            border-left: 4px solid #007bff;
        }}
        .info-row {{
            display: grid;
            grid-template-columns: repeat(auto-fit, minmax(200px, 1fr));
            gap: 10px;
            margin-top: 10px;
        }}
        .info-item {{
            display: flex;
            flex-direction: column;
        }}
        .info-label {{
            font-size: 12px;
            color: #888;
            text-transform: uppercase;
            letter-spacing: 0.5px;
        }}
        .info-value {{
            font-size: 18px;
            font-weight: bold;
            color: #333;
            margin-top: 2px;
        }}
        .fps {{
            color: #28a745;
        }}
        .resolution {{
            color: #007bff;
        }}
        .container {{
            background-color: white;
            border-radius: 8px;
            box-shadow: 0 2px 10px rgba(0,0,0,0.1);
            padding: 20px;
            max-width: 1200px;
        }}
        img {{
            max-width: 100%;
            height: auto;
            border: 2px solid #ddd;
            border-radius: 4px;
        }}
        .controls {{
            margin-top: 20px;
            display: flex;
            gap: 10px;
            align-items: center;
            flex-wrap: wrap;
        }}
        button {{
            padding: 10px 20px;
            font-size: 16px;
            cursor: pointer;
            background-color: #4CAF50;
            color: white;
            border: none;
            border-radius: 4px;
        }}
        button:hover {{
            background-color: #45a049;
        }}
        .status {{
            padding: 8px 16px;
            border-radius: 4px;
            font-weight: bold;
        }}
        .status.connected {{
            background-color: #d4edda;
            color: #155724;
        }}
        .status.disconnected {{
            background-color: #f8d7da;
            color: #721c24;
        }}
    </style>
</head>
<body>
    <div class="container">
        <h1>ROS 2 Image Stream</h1>
        <div class="info">
            <div><strong>Topic:</strong> {node.image_topic}</div>
            <div class="info-row">
                <div class="info-item">
                    <span class="info-label">Resolution</span>
                    <span class="info-value resolution" id="resolution">--</span>
                </div>
                <div class="info-item">
                    <span class="info-label">FPS</span>
                    <span class="info-value fps" id="fps">--</span>
                </div>
                <div class="info-item">
                    <span class="info-label">Quality</span>
                    <span class="info-value">{node.quality}%</span>
                </div>
            </div>
        </div>
        <img id="stream" src="/stream" alt="Image stream">
        <div class="controls">
            <button onclick="location.reload()">Refresh</button>
            <button onclick="window.open('/image.jpg', '_blank')">View Single Frame</button>
            <span id="status" class="status connected">Connected</span>
        </div>
    </div>
    <script>
        const img = document.getElementById('stream');
        const status = document.getElementById('status');
        const fpsElement = document.getElementById('fps');
        const resolutionElement = document.getElementById('resolution');
        
        // Update stream info
        function updateInfo() {{
            fetch('/info?' + new Date().getTime())
                .then(response => {{
                    if (!response.ok) throw new Error('Network response was not ok');
                    return response.json();
                }})
                .then(data => {{
                    fpsElement.textContent = data.fps + ' fps';
                    resolutionElement.textContent = data.width + ' × ' + data.height;
                    console.log('Info updated:', data);
                }})
                .catch(error => {{
                    console.error('Error fetching info:', error);
                    fpsElement.textContent = 'Error';
                    resolutionElement.textContent = 'Error';
                }});
        }}
        
        // Update info immediately and then every second
        updateInfo();
        setInterval(updateInfo, 1000);
        
        img.onerror = function() {{
            status.textContent = 'Disconnected';
            status.className = 'status disconnected';
            // Try to reconnect after 2 seconds
            setTimeout(() => {{
                img.src = '/stream?' + new Date().getTime();
            }}, 2000);
        }};
        
        img.onload = function() {{
            status.textContent = 'Connected';
            status.className = 'status connected';
        }};
    </script>
</body>
</html>
                """
        
        try:
            server = HTTPServer(('0.0.0.0', node.port), StreamHandler)
            node.get_logger().info(f'HTTP server started on http://0.0.0.0:{node.port}')
            server.serve_forever()
        except Exception as e:
            node.get_logger().error(f'Failed to start HTTP server: {str(e)}')


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = ImageStreamer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
