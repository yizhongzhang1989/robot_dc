import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import subprocess
import threading
import numpy as np
import cv2
import time
import os
import json
import yaml
from flask import Flask, Response, jsonify
from cv_bridge import CvBridge
from ament_index_python.packages import get_package_share_directory


def get_stream_resolution(url, max_retries=3, retry_delay=2):
    """get the resolution of the RTSP stream using ffprobe with retries"""
    for attempt in range(max_retries):
        try:
            cmd = [
                "ffprobe",
                "-v", "error",
                "-select_streams", "v:0",
                "-show_entries", "stream=width,height",
                "-of", "json",
                url
            ]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
            
            if result.returncode != 0:
                print(f"ffprobe failed for {url}, attempt {attempt + 1}/{max_retries}: {result.stderr}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    print(f"Using default resolution for {url}")
                    return 1920, 1080
            
            info = json.loads(result.stdout)
            
            if "streams" not in info or len(info["streams"]) == 0:
                print(f"No streams found for {url}, attempt {attempt + 1}/{max_retries}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    print(f"Using default resolution for {url}")
                    return 1920, 1080
                    
            stream = info["streams"][0]
            if 'width' not in stream or 'height' not in stream:
                print(f"No resolution info found for {url}, attempt {attempt + 1}/{max_retries}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    print(f"Using default resolution for {url}")
                    return 1920, 1080
                    
            w = stream["width"]
            h = stream["height"]
            return w, h
            
        except (json.JSONDecodeError, KeyError, subprocess.TimeoutExpired) as e:
            print(f"Error getting resolution for {url}, attempt {attempt + 1}/{max_retries}: {e}")
            if attempt < max_retries - 1:
                time.sleep(retry_delay)
                continue
    
    return 1920, 1080


class CameraCalibration:
    """Camera calibration manager for loading calibration data and undistorting images"""
    
    def __init__(self, calibration_file_path):
        self.calibration_file_path = calibration_file_path
        self.camera_matrix = None
        self.dist_coeffs = None
        self.new_camera_matrix = None
        self.roi = None
        self.image_size = None
        self.map1 = None
        self.map2 = None
        self.load_calibration()
    
    def load_calibration(self):
        """Load camera calibration from YAML file"""
        try:
            with open(self.calibration_file_path, 'r') as file:
                calib_data = yaml.safe_load(file)
            
            # Extract calibration parameters
            self.image_size = (calib_data['image_width'], calib_data['image_height'])
            
            # Camera matrix
            cam_matrix_data = calib_data['camera_matrix']['data']
            self.camera_matrix = np.array(cam_matrix_data).reshape(3, 3)
            
            # Distortion coefficients
            dist_coeffs_data = calib_data['distortion_coefficients']['data']
            self.dist_coeffs = np.array(dist_coeffs_data)
            
            # Get optimal new camera matrix
            self.new_camera_matrix, self.roi = cv2.getOptimalNewCameraMatrix(
                self.camera_matrix, self.dist_coeffs, self.image_size, 1, self.image_size
            )
            
            # Precompute undistortion maps for better performance
            self.map1, self.map2 = cv2.initUndistortRectifyMap(
                self.camera_matrix, self.dist_coeffs, None, 
                self.new_camera_matrix, self.image_size, cv2.CV_16SC2
            )
            
            print(f"Camera calibration loaded successfully from {self.calibration_file_path}")
            print(f"Image size: {self.image_size}")
            print(f"Camera matrix shape: {self.camera_matrix.shape}")
            print(f"Distortion coefficients: {self.dist_coeffs}")
            
        except Exception as e:
            print(f"Error loading calibration file {self.calibration_file_path}: {e}")
            self.camera_matrix = None
            self.dist_coeffs = None
    
    def undistort_image(self, image):
        """Undistort an image using the loaded calibration parameters"""
        if self.camera_matrix is None or self.dist_coeffs is None:
            return image
        
        try:
            # Use precomputed maps for faster undistortion
            undistorted = cv2.remap(image, self.map1, self.map2, cv2.INTER_LINEAR)
            
            # Crop the image using ROI if needed
            x, y, w, h = self.roi
            if w > 0 and h > 0:
                undistorted = undistorted[y:y+h, x:x+w]
            
            return undistorted
        except Exception as e:
            print(f"Error undistorting image: {e}")
            return image


class RTSPStreamCalibrate:
    """RTSP stream handler with calibration support"""
    
    def __init__(self, name, url, calibration, node=None):
        self.name = name
        self.url = url
        self.calibration = calibration
        self.frame = None
        self.undistorted_frame = None
        self.running = True
        self.lock = threading.Lock()
        self.node = node
        
        # Viewer tracking
        self.viewer_count = 0
        self.viewer_lock = threading.Lock()
        
        # ROS publishing
        self.ros_publish_enabled = False
        self.ros_publisher = None
        self.undistorted_publisher = None
        self.cv_bridge = None
        self.last_published_frame_id = -1
        self.frame_ready_event = threading.Event()
        
        self.log_info(f"Getting resolution for {name} stream: {url}")
        self.width, self.height = get_stream_resolution(self.url)
        self.log_info(f"Resolution for {name}: {self.width}x{self.height}")
        
        # Frame tracking
        self.frame_count = 0
        self.last_frame_update = time.time()
        
        self.log_info(f"Starting FFmpeg process for {name}...")
        max_ffmpeg_retries = 3
        self.proc = None
        self.available = False
        
        for attempt in range(max_ffmpeg_retries):
            try:
                self.proc = subprocess.Popen(
                    [
                        "ffmpeg",
                        "-rtsp_transport", "tcp",
                        "-fflags", "nobuffer",
                        "-flags", "low_delay",
                        "-an",
                        "-i", self.url,
                        "-f", "rawvideo",
                        "-pix_fmt", "bgr24",
                        "-"
                    ],
                    stdout=subprocess.PIPE,
                    stderr=subprocess.DEVNULL
                )
                time.sleep(3)
                if self.proc.poll() is None:
                    self.available = True
                    self.log_info(f"FFmpeg process started for {name}")
                    break
                else:
                    self.log_warn(f"FFmpeg process failed for {name}, attempt {attempt + 1}/{max_ffmpeg_retries}")
                    if attempt < max_ffmpeg_retries - 1:
                        time.sleep(2)
                        continue
            except Exception as e:
                self.log_error(f"Error starting FFmpeg for {name}, attempt {attempt + 1}/{max_ffmpeg_retries}: {e}")
                if attempt < max_ffmpeg_retries - 1:
                    time.sleep(2)
                    continue
        
        if not self.available:
            self.log_error(f"Failed to start FFmpeg for {name} after {max_ffmpeg_retries} attempts.")
            self.proc = None
        
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()

    def log_info(self, message):
        if self.node:
            self.node.get_logger().info(message)
        else:
            print(f"[INFO] {message}")

    def log_warn(self, message):
        if self.node:
            self.node.get_logger().warn(message)
        else:
            print(f"[WARN] {message}")

    def log_error(self, message):
        if self.node:
            self.node.get_logger().error(message)
        else:
            print(f"[ERROR] {message}")

    def update(self):
        """Continuously read frames and process them"""
        if not self.available or self.proc is None:
            return
            
        frame_size = self.width * self.height * 3
        consecutive_failures = 0
        max_consecutive_failures = 10
        
        while self.running and self.available:
            try:
                if self.proc.stdout is None:
                    self.log_error(f"FFmpeg stdout is None for {self.name}")
                    self.available = False
                    break
                    
                raw = self.proc.stdout.read(frame_size)
                if len(raw) != frame_size:
                    consecutive_failures += 1
                    if consecutive_failures >= max_consecutive_failures:
                        self.log_error(f"Too many consecutive frame read failures for {self.name}")
                        self.available = False
                        break
                    continue
                
                consecutive_failures = 0
                frame = np.frombuffer(raw, np.uint8).reshape((self.height, self.width, 3))
                
                # Apply calibration to get undistorted frame
                undistorted_frame = self.calibration.undistort_image(frame)
                
                with self.lock:
                    self.frame = frame.copy()
                    self.undistorted_frame = undistorted_frame.copy()
                    self.frame_count += 1
                    self.last_frame_update = time.time()
                
                # Trigger ROS publishing
                if self.ros_publish_enabled:
                    self.frame_ready_event.set()
                    
            except Exception as e:
                consecutive_failures += 1
                self.log_error(f"Error reading frame for {self.name}: {e}")
                if consecutive_failures >= max_consecutive_failures:
                    self.available = False
                    break
                time.sleep(0.1)

    def get_frame(self):
        """Get the latest original frame"""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def get_undistorted_frame(self):
        """Get the latest undistorted frame"""
        with self.lock:
            return self.undistorted_frame.copy() if self.undistorted_frame is not None else None

    def get_combined_frame(self, max_width=1200):
        """Get combined frame showing original and undistorted images side by side"""
        with self.lock:
            if self.frame is None:
                return None
            
            original = self.frame.copy()
            undistorted = self.undistorted_frame.copy() if self.undistorted_frame is not None else original
            
            # Resize frames for web display - each image gets half the max width with some margin
            single_max_width = max_width // 2 - 10  # Leave space for margin
            if original.shape[1] > single_max_width:
                scale = single_max_width / original.shape[1]
                new_width = int(original.shape[1] * scale)
                new_height = int(original.shape[0] * scale)
                original = cv2.resize(original, (new_width, new_height))
                undistorted = cv2.resize(undistorted, (new_width, new_height))
            
            # Create a margin between images (20 pixels wide, same height as images)
            margin = np.zeros((original.shape[0], 20, 3), dtype=np.uint8)
            margin.fill(64)  # Dark gray margin
            
            # Combine horizontally with margin
            combined = np.hstack([original, margin, undistorted])
            return combined

    def add_viewer(self):
        with self.viewer_lock:
            self.viewer_count += 1

    def remove_viewer(self):
        with self.viewer_lock:
            self.viewer_count = max(0, self.viewer_count - 1)

    def has_viewers(self):
        with self.viewer_lock:
            return self.viewer_count > 0

    def stop(self):
        self.running = False
        if self.proc:
            self.proc.terminate()


class CameraCalibrateNode(Node):
    """ROS2 node for camera calibration and undistortion"""
    
    def __init__(self):
        super().__init__('camera_calibrate_node')
        
        # Declare parameters
        self.declare_parameter('camera_name', 'RobotArmCameraCalibrate')
        self.declare_parameter('rtsp_url_main', 'rtsp://admin:123456@192.168.1.102/stream0')
        self.declare_parameter('camera_ip', '192.168.1.102')
        self.declare_parameter('server_port', 8013)
        self.declare_parameter('stream_fps', 25)
        self.declare_parameter('jpeg_quality', 75)
        self.declare_parameter('max_width', 800)
        self.declare_parameter('publish_ros_image', True)
        self.declare_parameter('ros_topic_name', '/robot_arm_camera/image_calibrated')
        self.declare_parameter('calibration_file', 'config/ost.yaml')
        self.declare_parameter('show_undistorted', True)
        self.declare_parameter('undistorted_topic_name', '/robot_arm_camera/image_undistorted')
        
        # Get parameters
        self.camera_name = self.get_parameter('camera_name').value
        self.rtsp_url_main = self.get_parameter('rtsp_url_main').value
        self.camera_ip = self.get_parameter('camera_ip').value
        self.server_port = self.get_parameter('server_port').value
        self.stream_fps = self.get_parameter('stream_fps').value
        self.jpeg_quality = self.get_parameter('jpeg_quality').value
        self.max_width = self.get_parameter('max_width').value
        self.publish_ros_image = self.get_parameter('publish_ros_image').value
        self.ros_topic_name = self.get_parameter('ros_topic_name').value
        self.calibration_file = self.get_parameter('calibration_file').value
        self.show_undistorted = self.get_parameter('show_undistorted').value
        self.undistorted_topic_name = self.get_parameter('undistorted_topic_name').value
        
        # Load calibration file
        try:
            package_share_directory = get_package_share_directory('camera_node')
            calibration_file_path = os.path.join(package_share_directory, self.calibration_file)
            if not os.path.exists(calibration_file_path):
                # Try relative path from current directory
                calibration_file_path = self.calibration_file
            self.calibration = CameraCalibration(calibration_file_path)
        except Exception as e:
            self.get_logger().error(f"Failed to load calibration: {e}")
            self.calibration = CameraCalibration(self.calibration_file)
        
        # Initialize stream
        self.stream = RTSPStreamCalibrate("main", self.rtsp_url_main, self.calibration, self)
        
        # ROS2 publishers
        if self.publish_ros_image:
            self.image_publisher = self.create_publisher(Image, self.ros_topic_name, 10)
            self.undistorted_publisher = self.create_publisher(Image, self.undistorted_topic_name, 10)
            self.cv_bridge = CvBridge()
            
            # Setup ROS publishing for stream
            self.stream.ros_publish_enabled = True
            self.stream.ros_publisher = self.image_publisher
            self.stream.undistorted_publisher = self.undistorted_publisher
            self.stream.cv_bridge = self.cv_bridge
            
            # Start ROS publishing thread
            self.ros_publish_thread = threading.Thread(target=self.ros_publish_loop, daemon=True)
            self.ros_publish_thread.start()
        
        # Flask web server
        self.app = Flask(__name__)
        self.setup_routes()
        
        # Start Flask server in a separate thread
        self.flask_thread = threading.Thread(
            target=lambda: self.app.run(host='0.0.0.0', port=self.server_port, debug=False),
            daemon=True
        )
        self.flask_thread.start()
        
        self.get_logger().info(f"Camera Calibrate Node started on port {self.server_port}")
        self.get_logger().info(f"Web interface available at http://localhost:{self.server_port}")

    def setup_routes(self):
        """Setup Flask routes"""
        
        @self.app.route('/')
        def index():
            return f'''
            <!DOCTYPE html>
            <html>
            <head>
                <title>{self.camera_name}</title>
                <style>
                    body {{ 
                        font-family: Arial, sans-serif; 
                        margin: 20px; 
                        background-color: #f5f5f5;
                    }}
                    .container {{ 
                        max-width: 1600px; 
                        margin: 0 auto; 
                        background-color: white;
                        padding: 20px;
                        border-radius: 8px;
                        box-shadow: 0 2px 10px rgba(0,0,0,0.1);
                    }}
                    .header {{
                        text-align: center;
                        margin-bottom: 20px;
                        padding-bottom: 15px;
                        border-bottom: 2px solid #e0e0e0;
                    }}
                    .video-container {{ 
                        text-align: center; 
                        margin: 20px 0; 
                        background-color: #fafafa;
                        padding: 20px;
                        border-radius: 8px;
                    }}
                    .camera-feed {{
                        max-width: 100%; 
                        height: auto; 
                        border: 2px solid #ddd;
                        border-radius: 8px;
                        box-shadow: 0 4px 8px rgba(0,0,0,0.1);
                    }}
                    .image-labels {{
                        display: flex;
                        justify-content: space-between;
                        margin-top: 15px;
                        padding: 0 50px;
                    }}
                    .image-label {{
                        flex: 1;
                        text-align: center;
                        font-size: 1.2em;
                        font-weight: bold;
                        color: #333;
                    }}
                    .info-section {{ 
                        margin-top: 30px;
                        padding-top: 20px;
                        border-top: 1px solid #e0e0e0;
                    }}
                    .info-line {{
                        margin: 8px 0;
                        color: #666;
                        font-size: 0.9em;
                    }}
                    .comparison-title {{
                        font-size: 1.8em;
                        color: #333;
                        margin-bottom: 10px;
                        text-align: center;
                        font-weight: bold;
                    }}
                </style>
                <script>
                    function refreshImage() {{
                        var img = document.getElementById('camera-feed');
                        img.src = '/video_feed?' + new Date().getTime();
                    }}
                    setInterval(refreshImage, {1000//self.stream_fps});
                </script>
            </head>
            <body>
                <div class="container">
                    <div class="header">
                        <h1>original image vs distorted image</h1>
                    </div>
                    
                    <div class="video-container">
                        <img id="camera-feed" class="camera-feed" src="/video_feed" alt="Camera feed loading...">
                        <div class="image-labels">
                            <div class="image-label">Original Image</div>
                            <div class="image-label">Undistorted Image</div>
                        </div>
                    </div>
                    
                    <div class="info-section">
                        <div class="info-line"><strong>Camera Name:</strong> {self.camera_name}</div>
                        <div class="info-line"><strong>Original Topic:</strong> {self.ros_topic_name}</div>
                        <div class="info-line"><strong>Undistorted Topic:</strong> {self.undistorted_topic_name}</div>
                        <div class="info-line"><strong>Calibration File:</strong> {self.calibration_file}</div>
                    </div>
                </div>
            </body>
            </html>
            '''

        @self.app.route('/video_feed')
        def video_feed():
            def generate():
                self.stream.add_viewer()
                try:
                    while True:
                        frame = self.stream.get_combined_frame(self.max_width)
                        if frame is not None:
                            _, buffer = cv2.imencode('.jpg', frame, 
                                                   [cv2.IMWRITE_JPEG_QUALITY, self.jpeg_quality])
                            yield (b'--frame\r\n'
                                   b'Content-Type: image/jpeg\r\n\r\n' + buffer.tobytes() + b'\r\n')
                        time.sleep(1.0 / self.stream_fps)
                finally:
                    self.stream.remove_viewer()
            
            return Response(generate(), mimetype='multipart/x-mixed-replace; boundary=frame')

        @self.app.route('/status')
        def status():
            return jsonify({
                'available': self.stream.available,
                'viewers': self.stream.viewer_count,
                'frame_count': self.stream.frame_count,
                'camera_name': self.camera_name,
                'calibration_loaded': self.calibration.camera_matrix is not None
            })

    def ros_publish_loop(self):
        """ROS publishing loop"""
        while rclpy.ok():
            try:
                # Wait for new frame
                self.stream.frame_ready_event.wait(timeout=1.0)
                self.stream.frame_ready_event.clear()
                
                if not self.stream.available:
                    continue
                
                # Publish original image
                original_frame = self.stream.get_frame()
                if original_frame is not None:
                    ros_image = self.cv_bridge.cv2_to_imgmsg(original_frame, encoding='bgr8')
                    ros_image.header.stamp = self.get_clock().now().to_msg()
                    ros_image.header.frame_id = f"{self.camera_name}_frame"
                    self.image_publisher.publish(ros_image)
                
                # Publish undistorted image
                undistorted_frame = self.stream.get_undistorted_frame()
                if undistorted_frame is not None:
                    ros_undistorted = self.cv_bridge.cv2_to_imgmsg(undistorted_frame, encoding='bgr8')
                    ros_undistorted.header.stamp = self.get_clock().now().to_msg()
                    ros_undistorted.header.frame_id = f"{self.camera_name}_undistorted_frame"
                    self.undistorted_publisher.publish(ros_undistorted)
                    
            except Exception as e:
                self.get_logger().error(f"Error in ROS publishing: {e}")
                time.sleep(0.1)

    def destroy_node(self):
        """Clean up resources"""
        if hasattr(self, 'stream'):
            self.stream.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CameraCalibrateNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
