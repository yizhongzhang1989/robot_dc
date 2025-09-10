import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse, StreamingResponse
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, Response
from ament_index_python.packages import get_package_share_directory
import os
import sys
import threading
import json
import asyncio
from typing import List
import xml.etree.ElementTree as ET
import base64
import socket
import time
import cv2
import subprocess
import numpy as np

def get_stream_resolution(url):
    """获取RTSP流的分辨率，带错误处理"""
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
            print(f"ffprobe failed with return code {result.returncode}")
            print(f"stderr: {result.stderr}")
            raise Exception(f"ffprobe 执行失败: {result.stderr}")
        
        info = json.loads(result.stdout)
        
        if 'streams' not in info or len(info['streams']) == 0:
            raise Exception("未找到视频流信息")
            
        stream = info["streams"][0]
        if 'width' not in stream or 'height' not in stream:
            raise Exception("无法获取分辨率信息")
            
        w = stream["width"]
        h = stream["height"]
        return w, h
        
    except subprocess.TimeoutExpired:
        raise Exception("获取流信息超时")
    except json.JSONDecodeError as e:
        raise Exception(f"解析流信息失败: {e}")
    except Exception as e:
        raise Exception(f"获取分辨率失败: {e}")


class RTSPStream:
    def __init__(self, name, url):
        self.name = name
        self.url = url
        self.frame = None
        self.running = False
        self.lock = threading.Lock()
        self.proc = None
        self.thread = None
        self.width = None
        self.height = None
        
        # 启动时自动开始流
        self.start()

    def start(self):
        """开始视频流"""
        if self.running:
            return
            
        try:
            print(f"[{self.name}] Connecting to: {self.url}")
            self.width, self.height = get_stream_resolution(self.url)
            print(f"[{self.name}] Stream resolution: {self.width}x{self.height}")
        except Exception as e:
            print(f"[{self.name}] Error getting resolution: {e}")
            print(f"[{self.name}] Use default 1920x1080")
            # 使用默认分辨率
            self.width, self.height = 1920, 1080

        try:
            self.running = True
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

            self.thread = threading.Thread(target=self.update, daemon=True)
            self.thread.start()
            print(f"[{self.name}] Stream started successfully")
        except Exception as e:
            self.running = False
            print(f"[{self.name}] Failed to start stream: {e}")
            raise e

    def update(self):
        frame_size = self.width * self.height * 3
        while self.running:
            try:
                raw = self.proc.stdout.read(frame_size)
                if len(raw) != frame_size:
                    continue
                frame = np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 3))
                with self.lock:
                    self.frame = frame
            except Exception as e:
                if self.running:
                    print(f"[{self.name}] Error reading frame: {e}")
                break

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        """停止视频流"""
        if not self.running:
            return
            
        self.running = False
        if self.proc:
            self.proc.kill()
        if self.thread:
            self.thread.join()
        with self.lock:
            self.frame = None
        print(f"[{self.name}] Stream stopped")
    
    def restart(self):
        """重启视频流"""
        print(f"[{self.name}] Restarting stream...")
        self.stop()
        time.sleep(1)  # 等待1秒
        self.start()
    
    def change_url(self, new_url):
        """切换到新的RTSP URL"""
        print(f"[{self.name}] Changing URL to: {new_url}")
        self.url = new_url
        self.restart()
    
    def is_running(self):
        """检查流是否正在运行"""
        return self.running


# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        """Send message to all connected clients"""
        disconnected_clients = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except RuntimeError:
                # Mark connection for removal if it's closed
                disconnected_clients.append(connection)
        
        # Clean up disconnected clients
        for client in disconnected_clients:
            self.disconnect(client)


class RobotArmWebServer(Node):
    def __init__(self):
        super().__init__('robot_arm_web_server')
        
        # Declare parameters
        self.declare_parameter('device_id', 1)
        self.declare_parameter('port', 8080)
        
        # Dynamic URDF file path - try to find the URDF file using ROS2 package discovery
        default_urdf_path = self.find_urdf_file()
        self.declare_parameter('urdf_file', default_urdf_path)
        
        self.device_id = self.get_parameter('device_id').value
        self.port = self.get_parameter('port').value
        self.urdf_file = self.get_parameter('urdf_file').value
        
        # If parameter is empty, use auto-discovery
        if not self.urdf_file or self.urdf_file.strip() == '':
            self.urdf_file = default_urdf_path
        
        # WebSocket connection manager
        self.connection_manager = ConnectionManager()
        
        # Create event for notifying the websocket broadcaster
        self.new_state_event = threading.Event()
        self.broadcast_task = None
        
        # Create publisher for robot arm commands
        self.cmd_publisher = self.create_publisher(
            String, 
            f'/arm{self.device_id}/cmd', 
            10
        )
        
        # Create subscription for robot state
        self.robot_state_subscription = self.create_subscription(
            String,
            f'/arm{self.device_id}/robot_state',
            self.robot_state_callback,
            10
        )
        
        # Store latest robot state
        self.latest_robot_state = None
        self.robot_state_lock = threading.Lock()
        
        # Camera image from ROS topic (instead of direct RTSP)
        self.latest_camera_image = None
        self.camera_image_lock = threading.Lock()
        
        # Subscribe to camera image topic
        self.camera_subscription = self.create_subscription(
            Image,
            '/robot_arm_camera/image_raw',
            self.camera_image_callback,
            1
        )
        
        # CV Bridge for image conversion
        self.bridge = CvBridge()
        
        # Camera calibration attributes
        self.corner_detection_enabled = False
        self.chessboard_size = (11, 8)  # Default chessboard size (corners)
        self.latest_image = None  # For calibration API compatibility
        
        self.get_logger().info("Subscribed to camera topic: /robot_arm_camera/image_raw")
        
        # FT Sensor UDP data
        self.latest_ft_data = None
        self.ft_data_lock = threading.Lock()
        self.ft_connection_manager = ConnectionManager()
        
        # Load URDF functionality
        self.load_urdf_data()
        
        # Start UDP receiver for FT sensor data
        self.start_ft_udp_receiver()
        
        self.get_logger().info(f'Robot arm web server initialized for device {self.device_id}')
        self.get_logger().info(f'Publishing to topic: /arm{self.device_id}/cmd')
        self.get_logger().info(f'Subscribing to topic: /arm{self.device_id}/robot_state')
        
        # Start FastAPI server in a separate thread
        self.start_web_server()
    
    def camera_image_callback(self, msg):
        """Handle incoming camera image messages"""
        try:
            # Convert ROS Image to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Process corner detection if enabled
            processed_image = self.process_corner_detection(cv_image.copy())
            
            with self.camera_image_lock:
                self.latest_camera_image = processed_image
                # Store both formats for compatibility
                self.latest_image = msg  # Keep original ROS message for calibration API
                
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')
    
    def process_corner_detection(self, image):
        """Process corner detection and draw corners on image if enabled"""
        if not hasattr(self, 'corner_detection_enabled') or not self.corner_detection_enabled:
            return image
            
        try:
            # Convert to grayscale for corner detection
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # Get chessboard size (default if not set)
            chessboard_size = getattr(self, 'chessboard_size', (11, 8))
            
            # Find chessboard corners
            ret, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
            
            if ret:
                # Refine corner positions for better accuracy
                criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
                corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
                
                # Draw corners on the image
                cv2.drawChessboardCorners(image, chessboard_size, corners, ret)
                
                # Add status text
                cv2.putText(image, f'Corners: {len(corners)} found', (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
            else:
                # No corners found
                cv2.putText(image, 'No corners detected', (10, 30), 
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                           
            # Add chessboard size info
            cv2.putText(image, f'Target: {chessboard_size[0]}x{chessboard_size[1]}', (10, 70), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 0), 2)
                       
        except Exception as e:
            self.get_logger().error(f'Error in corner detection: {e}')
            # Add error text to image
            cv2.putText(image, 'Corner detection error', (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
        
        return image
    
    def add_scripts_to_path(self):
        """Add the scripts directory to Python path for importing FTC modules"""
        # Find the robot_dc2 directory and add scripts to path
        current_file = os.path.abspath(__file__)
        self.get_logger().info(f"Current file path: {current_file}")
        
        scripts_dir = None
        path_parts = current_file.split(os.sep)
        self.get_logger().info(f"Path parts: {path_parts}")
        
        for i, part in enumerate(path_parts):
            if part == 'robot_dc2':
                scripts_dir = os.sep.join(path_parts[:i+1] + ['scripts'])
                break
        
        if scripts_dir is None:
            # Fallback: try different approach 
            # Look for robot_dc2 in the path more flexibly
            for i, part in enumerate(path_parts):
                if 'robot_dc2' in part:
                    base_dir = os.sep.join(path_parts[:i+1])
                    if part != 'robot_dc2':
                        # If the directory contains robot_dc2 but has different name
                        base_dir = os.path.dirname(base_dir)
                    scripts_dir = os.path.join(base_dir, 'scripts')
                    break
        
        self.get_logger().info(f"Computed scripts directory: {scripts_dir}")
        
        if scripts_dir and os.path.exists(scripts_dir):
            if scripts_dir not in sys.path:
                sys.path.insert(0, scripts_dir)
            self.get_logger().info(f"Added scripts directory to path: {scripts_dir}")
            self.get_logger().info(f"Files in scripts directory: {os.listdir(scripts_dir)}")
            return True
        else:
            self.get_logger().error(f"Scripts directory not found or doesn't exist: {scripts_dir}")
            return False
    
    def robot_state_callback(self, msg):
        """Handle incoming robot state messages"""
        with self.robot_state_lock:
            try:
                # Parse JSON string and store as dictionary
                self.latest_robot_state = json.loads(msg.data)
                # Trigger websocket broadcast
                self.new_state_event.set()
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Error parsing robot state JSON: {e}')
    
    def generate_camera_stream(self):
        """生成视频流，使用来自ROS话题的图像数据"""
        while True:
            with self.camera_image_lock:
                frame = self.latest_camera_image.copy() if self.latest_camera_image is not None else None
            
            if frame is not None:
                # Resize frame if needed (optional)
                h, w = frame.shape[:2]
                if w > 800:
                    scale = 800 / w
                    frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
                
                # Encode frame as JPEG
                _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                frame_bytes = buffer.tobytes()
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            else:
                # 如果没有图像数据，返回一个黑色图像
                black_image = np.zeros((480, 640, 3), dtype=np.uint8)
                _, buffer = cv2.imencode('.jpg', black_image)
                frame_bytes = buffer.tobytes()
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            
            time.sleep(0.04)  # 大约25Hz，与摄像头帧率保持一致
    
    def get_latest_camera_frame(self):
        """Get the latest camera frame as JPEG bytes from ROS topic"""
        with self.camera_image_lock:
            frame = self.latest_camera_image.copy() if self.latest_camera_image is not None else None
        
        if frame is not None:
            # Resize frame if needed
            h, w = frame.shape[:2]
            if w > 800:
                scale = 800 / w
                frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
            
            # Encode frame as JPEG
            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
            return buffer.tobytes()
        return None
    
    def start_ft_udp_receiver(self):
        """Start UDP receiver for FT sensor data in a separate thread"""
        def udp_receiver():
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            try:
                sock.bind(('0.0.0.0', 5566))  # Port 5566 for FT sensor data
                self.get_logger().info('FT Sensor UDP receiver started on port 5566')
                
                while True:
                    try:
                        data, addr = sock.recvfrom(4096)
                        message = data.decode('utf-8').strip()
                        ft_data = json.loads(message)
                        
                        # Store latest FT data
                        with self.ft_data_lock:
                            self.latest_ft_data = ft_data
                        
                        # Note: We'll handle WebSocket broadcasting in the main event loop
                        # For now, just store the data and let WebSocket clients poll for it
                        
                    except json.JSONDecodeError:
                        self.get_logger().warning(f'Invalid JSON from FT sensor: {message[:100]}')
                    except Exception as e:
                        self.get_logger().error(f'Error in FT UDP receiver: {e}')
                        
            except Exception as e:
                self.get_logger().error(f'Failed to start FT UDP receiver: {e}')
            finally:
                sock.close()
        
        # Start UDP receiver in daemon thread
        udp_thread = threading.Thread(target=udp_receiver, daemon=True)
        udp_thread.start()
    
    def get_latest_ft_data(self):
        """Get the latest FT sensor data"""
        with self.ft_data_lock:
            return self.latest_ft_data
    
    def get_latest_robot_state(self):
        """Get the latest robot state data"""
        with self.robot_state_lock:
            return self.latest_robot_state
    
    def send_command(self, command):
        """Send command to robot arm"""
        msg = String()
        msg.data = command
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f'Sent command: {command}')
        return {"status": "success", "command": command}
    
    def find_urdf_file(self):
        """Find URDF file using multiple strategies for cross-machine compatibility"""
        # Strategy 1: Try to use ROS2 package discovery
        try:
            package_share_dir = get_package_share_directory('duco_gcr5_910_urdf')
            urdf_path = os.path.join(package_share_dir, 'urdf', 'duco_gcr5_910_urdf.urdf')
            if os.path.exists(urdf_path):
                self.get_logger().info(f"Found URDF using package discovery: {urdf_path}")
                return urdf_path
        except Exception as e:
            self.get_logger().warn(f"Package discovery failed: {e}")
        
        # Strategy 2: Search relative to current package
        try:
            # Get the directory of this Python file
            current_dir = os.path.dirname(os.path.abspath(__file__))
            # Navigate to workspace src directory and look for the URDF package
            workspace_src = os.path.join(current_dir, '..', '..', '..', 'src')
            urdf_package_dir = os.path.join(workspace_src, 'duco_gcr5_910_urdf')
            urdf_path = os.path.join(urdf_package_dir, 'urdf', 'duco_gcr5_910_urdf.urdf')
            
            if os.path.exists(urdf_path):
                urdf_path = os.path.abspath(urdf_path)
                self.get_logger().info(f"Found URDF using relative path: {urdf_path}")
                return urdf_path
        except Exception as e:
            self.get_logger().warn(f"Relative path search failed: {e}")
        
        # Strategy 3: Search in common workspace locations
        possible_bases = [
            os.path.expanduser('~/Documents/robot_dc/colcon_ws/src'),
            os.path.expanduser('~/robot_dc/colcon_ws/src'),
            os.path.expanduser('~/workspace/robot_dc/colcon_ws/src'),
            '/opt/ros/workspace/src',
        ]
        
        for base_dir in possible_bases:
            urdf_path = os.path.join(base_dir, 'duco_gcr5_910_urdf', 'urdf', 'duco_gcr5_910_urdf.urdf')
            if os.path.exists(urdf_path):
                self.get_logger().info(f"Found URDF using common path search: {urdf_path}")
                return urdf_path
        
        # Strategy 4: Search for the file in the entire system (as a last resort)
        try:
            import subprocess
            result = subprocess.run(['find', os.path.expanduser('~'), '-name', 'duco_gcr5_910_urdf.urdf', '-type', 'f'], 
                                  capture_output=True, text=True, timeout=10)
            if result.returncode == 0 and result.stdout.strip():
                urdf_path = result.stdout.strip().split('\n')[0]  # Take the first match
                self.get_logger().info(f"Found URDF using system search: {urdf_path}")
                return urdf_path
        except Exception as e:
            self.get_logger().warn(f"System search failed: {e}")
        
        # Fallback: Return a default path (will cause an error later if file doesn't exist)
        fallback_path = '/tmp/duco_gcr5_910_urdf.urdf'
        self.get_logger().error(f"Could not find URDF file! Using fallback: {fallback_path}")
        return fallback_path
    
    def load_urdf_data(self):
        """Load URDF file and parse joint information"""
        try:
            self.get_logger().info(f"Loading URDF from: {self.urdf_file}")
            with open(self.urdf_file, 'r') as f:
                self.urdf_content = f.read()
            self.get_logger().info("Successfully loaded URDF file")
            
            # Parse joints from URDF
            self.joints = self.parse_joints()
            
            # Load mesh files
            self.meshes = {}
            self.load_meshes()
            
            # Joint state tracking
            self.joint_states = {joint: 0.0 for joint in self.joints.keys()}
            
        except Exception as e:
            self.get_logger().error(f"Failed to load URDF file: {e}")
            self.urdf_content = ""
            self.joints = {}
            self.meshes = {}
            self.joint_states = {}
    
    def parse_joints(self):
        """Parse joint information from URDF"""
        joints = {}
        try:
            root = ET.fromstring(self.urdf_content)
            
            for joint in root.findall('joint'):
                joint_name = joint.get('name')
                joint_type = joint.get('type')
                
                if joint_type in ['revolute', 'prismatic', 'continuous']:
                    limit_elem = joint.find('limit')
                    if limit_elem is not None:
                        lower = float(limit_elem.get('lower', '-3.14'))
                        upper = float(limit_elem.get('upper', '3.14'))
                    else:
                        lower, upper = -3.14, 3.14
                    
                    joints[joint_name] = {
                        'type': joint_type,
                        'lower': lower,
                        'upper': upper,
                        'value': 0.0
                    }
            
            self.get_logger().info(f'Found {len(joints)} controllable joints: {list(joints.keys())}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to parse joints: {str(e)}')
            
        return joints
    
    def load_meshes(self):
        """Load mesh files from the URDF package"""
        try:
            # Get the directory containing the URDF file
            urdf_dir = os.path.dirname(self.urdf_file)
            package_dir = os.path.dirname(urdf_dir)  # Go up one level from urdf/ to package root
            mesh_dir = os.path.join(package_dir, 'meshes')
            
            if not os.path.exists(mesh_dir):
                self.get_logger().warn(f"Mesh directory not found: {mesh_dir}")
                return
                
            # Load all STL files
            for root, dirs, files in os.walk(mesh_dir):
                for file in files:
                    if file.endswith('.stl') or file.endswith('.STL'):
                        mesh_path = os.path.join(root, file)
                        try:
                            with open(mesh_path, 'rb') as f:
                                mesh_data = f.read()
                                # Encode as base64 for web transmission
                                self.meshes[file] = base64.b64encode(mesh_data).decode('utf-8')
                            self.get_logger().info(f"Loaded mesh: {file}")
                        except Exception as e:
                            self.get_logger().error(f"Failed to load mesh {file}: {e}")
                            
        except Exception as e:
            self.get_logger().error(f"Error loading meshes: {e}")
    
    def update_joint_value(self, joint_name, value):
        """Update joint value"""
        if joint_name in self.joint_states:
            # Clamp value to joint limits
            joint_info = self.joints[joint_name]
            clamped_value = max(joint_info['lower'], min(joint_info['upper'], value))
            self.joint_states[joint_name] = clamped_value
            self.get_logger().info(f'Updated joint {joint_name} to {clamped_value}')
    
    def start_web_server(self):
        """Start the FastAPI web server"""
        def run_server():
            # Create FastAPI app
            app = FastAPI(title="Robot Arm Web Interface")
            
            # Get web directory
            try:
                web_path = os.path.join(get_package_share_directory('robot_arm_web'), 'web')
                STATIC_DIR = os.path.abspath(web_path)
            except Exception as e:
                # Fallback for development
                self.get_logger().warning(f"Could not get package share directory: {e}")
                STATIC_DIR = os.path.join(os.path.dirname(__file__), '..', 'web')
            
            self.get_logger().info(f"Serving web from: {STATIC_DIR}")
            
            # Routes
            @app.get("/")
            def serve_index():
                return FileResponse(os.path.join(STATIC_DIR, "index.html"))
            
            @app.post("/api/robot_arm/cmd")
            async def send_robot_arm_command(request: Request):
                data = await request.json()
                command = data.get("command")
                if command:
                    return self.send_command(command)
                else:
                    return JSONResponse(content={"error": "No command provided"}, status_code=400)
            
            @app.get("/api/robot_arm/info")
            def get_robot_arm_info():
                return {
                    "device_id": self.device_id,
                    "topic": f"/arm{self.device_id}/cmd",
                    "state_topic": f"/arm{self.device_id}/robot_state",
                    "camera_topic": "/rtsp_camera/image_raw",
                    "available_commands": ["power_on", "power_off", "enable", "disable", "pause", "resume", "stop_program", "task:*"]
                }
            
            @app.get("/api/camera/stream")
            def get_camera_stream():
                """Get current camera image as JPEG"""
                try:
                    image_data = self.get_latest_camera_frame()
                    if image_data is None:
                        return Response(content="No camera image available", status_code=404)
                    
                    return Response(content=image_data, media_type="image/jpeg")
                except Exception as e:
                    self.get_logger().error(f"Error serving camera image: {e}")
                    return Response(content="Camera error", status_code=500)
            
            @app.get("/video_feed")
            def video_feed():
                """Video streaming route - multipart/x-mixed-replace"""
                return StreamingResponse(
                    self.generate_camera_stream(),
                    media_type='multipart/x-mixed-replace; boundary=frame'
                )
            
            @app.get("/api/camera/status")
            def get_camera_status():
                """Get camera status"""
                # Check if we have recent camera data
                with self.camera_image_lock:
                    has_camera_data = self.latest_camera_image is not None
                
                return {
                    "connected": has_camera_data,
                    "rtsp_url": "Using ROS topic /robot_arm_camera/image_raw"
                }
            
            @app.get("/api/robot_arm/state")
            def get_robot_arm_state():
                """Get current robot state"""
                state = self.get_latest_robot_state()
                if state is None:
                    return JSONResponse(content={"error": "No robot state data available"}, status_code=503)
                return JSONResponse(content=state)
            
            # URDF-related endpoints
            @app.get("/api/urdf")
            def get_urdf():
                """Get URDF content"""
                return Response(content=self.urdf_content, media_type="application/xml")
            
            @app.get("/api/joints")
            def get_joints():
                """Get joint information"""
                return JSONResponse(content=self.joints)
            
            @app.get("/api/meshes")
            def get_meshes():
                """Get mesh data"""
                return JSONResponse(content=self.meshes)
            
            @app.get("/api/joint_states")
            def get_joint_states():
                """Get current joint states"""
                return JSONResponse(content=self.joint_states)
            
            @app.post("/api/update_joint")
            async def update_joint(request: Request):
                """Update joint value"""
                try:
                    data = await request.json()
                    joint_name = data['joint_name']
                    value = float(data['value'])
                    
                    self.update_joint_value(joint_name, value)
                    
                    return JSONResponse(content={'status': 'success'})
                except Exception as e:
                    return JSONResponse(content={'error': str(e)}, status_code=400)
            
            # Tool Control API endpoints
            @app.post("/api/tool_control/execute")
            async def tool_control_execute(request: Request):
                """Execute tool control operation via JSON API - simplified version"""
                try:
                    data = await request.json()
                    tool_type = data.get('tool_type')
                    
                    if not tool_type:
                        return JSONResponse(content={'error': 'tool_type is required'}, status_code=400)
                    
                    # Validate tool type
                    valid_tool_types = ['gripper', 'frame', 'stickP', 'stickR', 'homing']
                    if tool_type not in valid_tool_types:
                        return JSONResponse(content={
                            'error': f'Invalid tool_type. Must be one of: {valid_tool_types}'
                        }, status_code=400)
                    
                    # Get current robot state
                    state = self.get_latest_robot_state()
                    if not state or 'boolRegisterOutput' not in state:
                        return JSONResponse(content={
                            'error': 'Robot state not available'
                        }, status_code=503)
                    
                    # Check if program is completed (tool controls are enabled)
                    bool_register_output = state['boolRegisterOutput']
                    is_program_completed = not bool_register_output[4] or bool_register_output[4] == 0
                    
                    if not is_program_completed:
                        return JSONResponse(content={
                            'error': 'Tool controls are disabled - Program must be completed first'
                        }, status_code=400)
                    
                    # Execute tool control operation with default settings (always wait for completion)
                    result = await self.execute_tool_control_operation(tool_type, bool_register_output, wait_completion=True, timeout=100000)
                    
                    return JSONResponse(content=result)
                    
                except Exception as e:
                    self.get_logger().error(f"Error in tool control API: {str(e)}")
                    return JSONResponse(content={'error': str(e)}, status_code=500)
            
            @app.get("/api/tool_control/status")
            async def tool_control_status():
                """Get current tool control status"""
                try:
                    state = self.get_latest_robot_state()
                    if not state or 'boolRegisterOutput' not in state:
                        return JSONResponse(content={
                            'error': 'Robot state not available'
                        }, status_code=503)
                    
                    bool_register_output = state['boolRegisterOutput']
                    
                    # Tool states (first 4 bits)
                    tool_states = {
                        'gripper': bool_register_output[0] or bool_register_output[0] == 1,
                        'frame': bool_register_output[1] or bool_register_output[1] == 1,
                        'stickP': bool_register_output[2] or bool_register_output[2] == 1,
                        'stickR': bool_register_output[3] or bool_register_output[3] == 1,
                    }
                    
                    # Program status (bit 4, inverted logic)
                    is_program_completed = not bool_register_output[4] or bool_register_output[4] == 0
                    
                    # Current state array
                    current_state = [
                        1 if tool_states['gripper'] else 0,
                        1 if tool_states['frame'] else 0,
                        1 if tool_states['stickP'] else 0,
                        1 if tool_states['stickR'] else 0
                    ]
                    
                    return JSONResponse(content={
                        'tool_states': tool_states,
                        'program_completed': is_program_completed,
                        'controls_enabled': is_program_completed,
                        'current_state': current_state,
                        'available_operations': ['gripper', 'frame', 'stickP', 'stickR', 'homing']
                    })
                    
                except Exception as e:
                    self.get_logger().error(f"Error getting tool control status: {str(e)}")
                    return JSONResponse(content={'error': str(e)}, status_code=500)
            
            # FTC-related endpoints
            @app.post("/api/ftc/start")
            async def ftc_start(request: Request):
                """Start FTC"""
                try:
                    if not self.add_scripts_to_path():
                        return JSONResponse(content={'error': 'Scripts directory not found'}, status_code=500)
                    
                    from duco_FTCApiPost import FTC_start
                    FTC_start()
                    self.get_logger().info("FTC start command executed")
                    return JSONResponse(content={'status': 'success', 'message': 'FTC started'})
                except Exception as e:
                    self.get_logger().error(f"Error executing FTC start: {str(e)}")
                    return JSONResponse(content={'error': str(e)}, status_code=500)
            
            @app.post("/api/ftc/stop")
            async def ftc_stop(request: Request):
                """Stop FTC"""
                try:
                    if not self.add_scripts_to_path():
                        return JSONResponse(content={'error': 'Scripts directory not found'}, status_code=500)
                    
                    from duco_FTCApiPost import FTC_stop
                    FTC_stop()
                    self.get_logger().info("FTC stop command executed")
                    return JSONResponse(content={'status': 'success', 'message': 'FTC stopped'})
                except Exception as e:
                    self.get_logger().error(f"Error executing FTC stop: {str(e)}")
                    return JSONResponse(content={'error': str(e)}, status_code=500)
            
            @app.post("/api/ftc/setindex")
            async def ftc_setindex(request: Request):
                """Set FTC index"""
                try:
                    data = await request.json()
                    index = data.get('index', 0)
                    
                    if not self.add_scripts_to_path():
                        return JSONResponse(content={'error': 'Scripts directory not found'}, status_code=500)
                    
                    from duco_FTCApiPost import FTC_SetIndex
                    FTC_SetIndex(index)
                    self.get_logger().info(f"FTC set index command executed with index: {index}")
                    return JSONResponse(content={'status': 'success', 'message': f'FTC index set to {index}'})
                except Exception as e:
                    self.get_logger().error(f"Error executing FTC set index: {str(e)}")
                    return JSONResponse(content={'error': str(e)}, status_code=500)
            
            @app.post("/api/ftc/setdkassemflag")
            async def ftc_setdkassemflag(request: Request):
                """Set FTC DK assembly flag"""
                try:
                    data = await request.json()
                    flag = data.get('flag', 0)
                    
                    if not self.add_scripts_to_path():
                        return JSONResponse(content={'error': 'Scripts directory not found'}, status_code=500)
                    
                    from duco_FTCApiPost import FTC_SetDKAssemFlag
                    FTC_SetDKAssemFlag(flag)
                    action = "enabled" if flag == 1 else "disabled"
                    self.get_logger().info(f"FTC DK assembly flag set to {flag} (program {action})")
                    return JSONResponse(content={'status': 'success', 'message': f'Program {action}'})
                except Exception as e:
                    self.get_logger().error(f"Error executing FTC set DK assembly flag: {str(e)}")
                    return JSONResponse(content={'error': str(e)}, status_code=500)
            
            @app.post("/api/ftc/setftsetallrt")
            async def ftc_setftsetallrt(request: Request):
                """Set FTC Real-Time parameters"""
                try:
                    data = await request.json()
                    
                    if not self.add_scripts_to_path():
                        return JSONResponse(content={'error': 'Scripts directory not found'}, status_code=500)
                    
                    from duco_FTCApiPost import FTC_setFTsetAllRT
                    
                    # Extract all parameters from the request data
                    params = data.get('parameters', {})
                    
                    # Convert ftcSetGroup to array format if it's a string
                    ftcSetGroup = params.get('ftcSetGroup', '17')
                    if isinstance(ftcSetGroup, str):
                        ftcSetGroup = [int(ftcSetGroup)] if ftcSetGroup.isdigit() else [0]
                    
                    FTC_setFTsetAllRT(
                        isProgram=params.get('isProgram', True),
                        ftcProgram=params.get('ftcProgram', None),
                        onlyMonitor=params.get('onlyMonitor', False),
                        graCalcIndex=params.get('graCalcIndex', 0),
                        ftEnabled=params.get('ftEnabled', [True,True,True,True,True,True]),
                        ftSet=params.get('ftSet', [0,0,0,0,0,0]),
                        dead_zone=params.get('deadZone', [1,1,1,0.1,0.1,0.1]),
                        disEndLimit=params.get('disEndLimit', 0.0),
                        timeEndLimit=params.get('timeEndLimit', 0.0),
                        ftEndLimit=params.get('ftEndLimit', [0,0,0,0,0,0]),
                        disAng6D_EndLimit=params.get('disAng6DEndLimit', [0,0,0,0,0,0]),
                        ftcEndType=params.get('ftcEndType', 0),
                        quickSetIndex=params.get('quickSetIndex', [0,0,0,0,0,0]),
                        B=params.get('B', [6000,6000,6000,4500,4500,4500]),
                        M=params.get('M', [20,20,20,25,25,25]),
                        vel_limit=params.get('velLimit', [500,500,500,500,500,500]),
                        cor_pos_limit=params.get('corPosLimit', [10,10,10,5,5,5]),
                        maxForce_1=params.get('maxForce1', [0,0,0,0,0,0]),
                        ifDKStopOnMaxForce_1=params.get('ifDKStopOnMaxForce1', False),
                        ifRobotStopOnMaxForce_1=params.get('ifRobotStopOnMaxForce1', False),
                        maxForce_2=params.get('maxForce2', [0,0,0,0,0,0]),
                        ifDKStopOnMaxForce_2=params.get('ifDKStopOnMaxForce2', False),
                        ifRobotStopOnMaxForce_2=params.get('ifRobotStopOnMaxForce2', False),
                        ifDKStopOnTimeDisMon=params.get('ifDKStopOnTimeDisMon', False),
                        ifRobotStopOnTimeDisMon=params.get('ifRobotStopOnTimeDisMon', False),
                        ifNeedInit=params.get('ifNeedInit', True),
                        withGroup=params.get('withGroup', False),
                        ftcSetGroup=ftcSetGroup,
                        ignoreSensor=params.get('ignoreSensor', False)
                    )
                    
                    self.get_logger().info("FTC setFTSetAllRT command executed with custom parameters")
                    return JSONResponse(content={'status': 'success', 'message': 'FTC RT parameters set successfully'})
                except Exception as e:
                    self.get_logger().error(f"Error executing FTC setFTSetAllRT: {str(e)}")
                    return JSONResponse(content={'error': str(e)}, status_code=500)
            
            # Camera Calibration API endpoints
            @app.post("/api/calibration/screenshot")
            async def take_calibration_screenshot(request: Request):
                """Take a screenshot for camera calibration"""
                try:
                    data = await request.json()
                    joint_angles = data.get('joint_angles', [])
                    tcp_pose = data.get('tcp_pose', [])
                    
                    # Create calibration directory if it doesn't exist
                    # Find workspace root by looking for 'robot_dc2' directory in the path
                    current_file = os.path.abspath(__file__)
                    workspace_root = None
                    
                    # Split the path and find robot_dc2 directory
                    path_parts = current_file.split(os.sep)
                    for i, part in enumerate(path_parts):
                        if part == 'robot_dc2':
                            workspace_root = os.sep.join(path_parts[:i+1])
                            break
                    
                    if workspace_root is None:
                        # Fallback: use a more general approach
                        workspace_root = '/home/a/Documents/robot_dc2'
                    
                    calibration_dir = os.path.join(workspace_root, 'temp')
                    
                    os.makedirs(calibration_dir, exist_ok=True)
                    
                    # Get current camera image
                    if self.latest_image is not None:
                        # Convert ROS image to OpenCV format
                        cv_image = self.bridge.imgmsg_to_cv2(self.latest_image, "bgr8")
                        
                        # Find next available sequence number
                        import glob
                        existing_files = glob.glob(os.path.join(calibration_dir, '*.jpg'))
                        used_numbers = set()
                        for file_path in existing_files:
                            filename = os.path.basename(file_path)
                            name_without_ext = filename.replace('.jpg', '')
                            try:
                                counter = int(name_without_ext)
                                used_numbers.add(counter)
                            except ValueError:
                                # Skip files that don't match the numeric format
                                continue
                        
                        # Find the next available number starting from 0
                        next_sequence = 0
                        while next_sequence in used_numbers:
                            next_sequence += 1
                        
                        # Save image with simple numeric filename (matching duco_camera_collect.py)
                        filename = f"{next_sequence}.jpg"
                        filepath = os.path.join(calibration_dir, filename)
                        
                        cv2.imwrite(filepath, cv_image)
                        
                        # Save robot state data with matching format
                        state_filename = f"{next_sequence}.json"
                        state_filepath = os.path.join(calibration_dir, state_filename)
                        
                        # Convert joint angles and tcp pose to match duco_camera_collect.py format
                        import math
                        import numpy as np
                        
                        # Convert joint_angles from degrees to radians (if they were in degrees)
                        joint_angles_rad = joint_angles if isinstance(joint_angles, list) and len(joint_angles) >= 6 else [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
                        
                        # Convert tcp_pose to proper format (if available)
                        if isinstance(tcp_pose, list) and len(tcp_pose) >= 6:
                            tcp_pose_data = {
                                "x": tcp_pose[0] / 1000.0 if tcp_pose[0] > 100 else tcp_pose[0],  # Convert mm to m if needed
                                "y": tcp_pose[1] / 1000.0 if tcp_pose[1] > 100 else tcp_pose[1],
                                "z": tcp_pose[2] / 1000.0 if tcp_pose[2] > 100 else tcp_pose[2], 
                                "rx": math.radians(tcp_pose[3]) if abs(tcp_pose[3]) > 6.28 else tcp_pose[3],  # Convert degrees to radians if needed
                                "ry": math.radians(tcp_pose[4]) if abs(tcp_pose[4]) > 6.28 else tcp_pose[4],
                                "rz": math.radians(tcp_pose[5]) if abs(tcp_pose[5]) > 6.28 else tcp_pose[5]
                            }
                            
                            # Create 4x4 transformation matrix from tcp pose
                            x, y, z, rx, ry, rz = tcp_pose_data["x"], tcp_pose_data["y"], tcp_pose_data["z"], tcp_pose_data["rx"], tcp_pose_data["ry"], tcp_pose_data["rz"]
                            
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
                            
                            # Combined rotation matrix (order: Z-Y-X)
                            R = R_z @ R_y @ R_x
                            
                            # Create 4x4 transformation matrix
                            transform_matrix = np.eye(4)
                            transform_matrix[:3, :3] = R
                            transform_matrix[:3, 3] = [x, y, z]
                        else:
                            tcp_pose_data = {
                                "x": 0.0, "y": 0.0, "z": 0.0,
                                "rx": 0.0, "ry": 0.0, "rz": 0.0
                            }
                            transform_matrix = np.eye(4)
                        
                        # Create JSON data matching duco_camera_collect.py format
                        state_data = {
                            "joint_angles": joint_angles_rad[:6],
                            "end_xyzrpy": tcp_pose_data,
                            "end2base": transform_matrix.tolist()
                        }
                        with open(state_filepath, 'w') as f:
                            json.dump(state_data, f, indent=2)
                        
                        self.get_logger().info(f"Calibration screenshot saved: {filename}")
                        return JSONResponse(content={
                            'success': True, 
                            'filename': filename,
                            'sequence': next_sequence
                        })
                    else:
                        return JSONResponse(content={
                            'success': False, 
                            'message': 'No camera image available'
                        }, status_code=400)
                        
                except Exception as e:
                    self.get_logger().error(f"Error taking calibration screenshot: {str(e)}")
                    return JSONResponse(content={'success': False, 'message': str(e)}, status_code=500)
            
            @app.post("/api/calibration/corner-detection")
            async def toggle_corner_detection(request: Request):
                """Toggle corner detection for calibration"""
                try:
                    data = await request.json()
                    enable = data.get('enable', False)
                    
                    if enable:
                        chessboard_width = data.get('chessboard_width', 11)
                        chessboard_height = data.get('chessboard_height', 8)
                        
                        # Store corner detection parameters
                        self.corner_detection_enabled = True
                        self.chessboard_size = (chessboard_width, chessboard_height)
                        
                        self.get_logger().info(f"Corner detection enabled: {chessboard_width}x{chessboard_height}")
                        return JSONResponse(content={
                            'success': True,
                            'message': f'Corner detection enabled ({chessboard_width}x{chessboard_height})'
                        })
                    else:
                        self.corner_detection_enabled = False
                        self.get_logger().info("Corner detection disabled")
                        return JSONResponse(content={
                            'success': True,
                            'message': 'Corner detection disabled'
                        })
                        
                except Exception as e:
                    self.get_logger().error(f"Error toggling corner detection: {str(e)}")
                    return JSONResponse(content={'success': False, 'message': str(e)}, status_code=500)
            
            @app.post("/api/calibration/clear")
            async def clear_calibration_images():
                """Clear all calibration images"""
                try:
                    # Get the workspace root directory
                    current_file = os.path.abspath(__file__)
                    workspace_root = None
                    
                    # Split the path and find robot_dc2 directory
                    path_parts = current_file.split(os.sep)
                    for i, part in enumerate(path_parts):
                        if part == 'robot_dc2':
                            workspace_root = os.sep.join(path_parts[:i+1])
                            break
                    
                    if workspace_root is None:
                        workspace_root = '/home/a/Documents/robot_dc2'
                    
                    calibration_dir = os.path.join(workspace_root, 'temp')
                    
                    if os.path.exists(calibration_dir):
                        import glob
                        
                        # Remove all jpg and json files
                        for pattern in ['*.jpg', '*.json']:
                            files = glob.glob(os.path.join(calibration_dir, pattern))
                            for file_path in files:
                                os.remove(file_path)
                        
                        self.get_logger().info("Calibration images cleared")
                        return JSONResponse(content={'success': True, 'message': 'Images cleared successfully'})
                    else:
                        return JSONResponse(content={'success': True, 'message': 'No images to clear'})
                        
                except Exception as e:
                    self.get_logger().error(f"Error clearing calibration images: {str(e)}")
                    return JSONResponse(content={'success': False, 'message': str(e)}, status_code=500)
            
            @app.get("/api/calibration/thumbnails")
            async def get_calibration_thumbnails():
                """Get list of calibration image thumbnails"""
                try:
                    # Get the workspace root directory
                    current_file = os.path.abspath(__file__)
                    workspace_root = None
                    
                    # Split the path and find robot_dc2 directory
                    path_parts = current_file.split(os.sep)
                    for i, part in enumerate(path_parts):
                        if part == 'robot_dc2':
                            workspace_root = os.sep.join(path_parts[:i+1])
                            break
                    
                    if workspace_root is None:
                        workspace_root = '/home/a/Documents/robot_dc2'
                    
                    calibration_dir = os.path.join(workspace_root, 'temp')
                    
                    if not os.path.exists(calibration_dir):
                        return JSONResponse(content={'thumbnails': []})
                    
                    import glob
                    image_files = glob.glob(os.path.join(calibration_dir, '*.jpg'))
                    
                    thumbnails = []
                    for file_path in sorted(image_files):
                        filename = os.path.basename(file_path)
                        thumbnails.append({'filename': filename})
                    
                    return JSONResponse(content={'thumbnails': thumbnails})
                        
                except Exception as e:
                    self.get_logger().error(f"Error getting calibration thumbnails: {str(e)}")
                    return JSONResponse(content={'thumbnails': []}, status_code=500)
            
            @app.get("/api/calibration/thumbnail/{filename}")
            async def get_calibration_thumbnail(filename: str):
                """Get a calibration image thumbnail"""
                try:
                    # Get the workspace root directory
                    current_file = os.path.abspath(__file__)
                    workspace_root = None
                    
                    # Split the path and find robot_dc2 directory
                    path_parts = current_file.split(os.sep)
                    for i, part in enumerate(path_parts):
                        if part == 'robot_dc2':
                            workspace_root = os.sep.join(path_parts[:i+1])
                            break
                    
                    if workspace_root is None:
                        workspace_root = '/home/a/Documents/robot_dc2'
                    
                    calibration_dir = os.path.join(workspace_root, 'temp')
                    file_path = os.path.join(calibration_dir, filename)
                    
                    if os.path.exists(file_path) and os.path.isfile(file_path):
                        # Read and resize image for thumbnail
                        import cv2
                        image = cv2.imread(file_path)
                        if image is not None:
                            # Resize to thumbnail size
                            height, width = image.shape[:2]
                            thumbnail_height = 150
                            thumbnail_width = int(width * thumbnail_height / height)
                            thumbnail = cv2.resize(image, (thumbnail_width, thumbnail_height))
                            
                            # Encode as JPEG
                            _, buffer = cv2.imencode('.jpg', thumbnail)
                            
                            return Response(content=buffer.tobytes(), media_type="image/jpeg")
                        else:
                            return JSONResponse(content={'error': 'Unable to read image'}, status_code=400)
                    else:
                        return JSONResponse(content={'error': 'Image not found'}, status_code=404)
                        
                except Exception as e:
                    self.get_logger().error(f"Error getting calibration thumbnail: {str(e)}")
                    return JSONResponse(content={'error': str(e)}, status_code=500)
            
            @app.get("/api/calibration/image/{filename}")
            async def get_calibration_image(filename: str):
                """Get a full calibration image"""
                try:
                    # Get the workspace root directory
                    current_file = os.path.abspath(__file__)
                    workspace_root = None
                    
                    # Split the path and find robot_dc2 directory
                    path_parts = current_file.split(os.sep)
                    for i, part in enumerate(path_parts):
                        if part == 'robot_dc2':
                            workspace_root = os.sep.join(path_parts[:i+1])
                            break
                    
                    if workspace_root is None:
                        workspace_root = '/home/a/Documents/robot_dc2'
                    
                    calibration_dir = os.path.join(workspace_root, 'temp')
                    file_path = os.path.join(calibration_dir, filename)
                    
                    if os.path.exists(file_path) and os.path.isfile(file_path):
                        return FileResponse(file_path, media_type="image/jpeg")
                    else:
                        return JSONResponse(content={'error': 'Image not found'}, status_code=404)
                        
                except Exception as e:
                    self.get_logger().error(f"Error getting calibration image: {str(e)}")
                    return JSONResponse(content={'error': str(e)}, status_code=500)
            
            @app.post("/api/calibration/intrinsic")
            async def run_intrinsic_calibration(request: Request):
                """Run intrinsic camera calibration using the camera calibration toolkit"""
                try:
                    data = await request.json()
                    chessboard_width = data.get('chessboard_width', 11)
                    chessboard_height = data.get('chessboard_height', 8) 
                    square_size = data.get('square_size', 0.02)  # 20mm
                    
                    # Get workspace root directory
                    current_file = os.path.abspath(__file__)
                    workspace_root = None
                    
                    path_parts = current_file.split(os.sep)
                    for i, part in enumerate(path_parts):
                        if part == 'robot_dc2':
                            workspace_root = os.sep.join(path_parts[:i+1])
                            break
                    
                    if workspace_root is None:
                        workspace_root = '/home/a/Documents/robot_dc2'
                    
                    # Set up paths
                    temp_dir = os.path.join(workspace_root, 'temp')
                    scripts_dir = os.path.join(workspace_root, 'scripts', 'ThirdParty', 'camera_calibration_toolkit')
                    
                    if not os.path.exists(temp_dir):
                        return JSONResponse(content={'success': False, 'message': 'No calibration images found in temp directory'}, status_code=400)
                    
                    if not os.path.exists(scripts_dir):
                        return JSONResponse(content={'success': False, 'message': 'Camera calibration toolkit not found'}, status_code=500)
                    
                    # Check for image files
                    import glob
                    image_files = glob.glob(os.path.join(temp_dir, '*.jpg'))
                    if len(image_files) < 10:
                        return JSONResponse(content={
                            'success': False, 
                            'message': f'Not enough calibration images. Found {len(image_files)} images, need at least 10'
                        }, status_code=400)
                    
                    self.get_logger().info(f"Starting intrinsic calibration with {len(image_files)} images")
                    
                    # Create chessboard config file
                    config_data = {
                        "pattern_id": "standard_chessboard",
                        "name": "Standard Chessboard",
                        "description": "Traditional black and white checkerboard pattern",
                        "is_planar": True,
                        "parameters": {
                            "width": chessboard_width,
                            "height": chessboard_height,
                            "square_size": square_size
                        }
                    }
                    
                    config_path = os.path.join(temp_dir, 'chessboard_config.json')
                    with open(config_path, 'w') as f:
                        json.dump(config_data, f, indent=2)
                    
                    # Run calibration using subprocess to call the calibration script
                    import subprocess
                    import sys
                    
                    # Create a Python script to run the calibration
                    calibration_script = f'''
import os
import sys
import glob
import json
import numpy as np

# Add the camera calibration toolkit to Python path
sys.path.append('{scripts_dir}')

try:
    from core.intrinsic_calibration import IntrinsicCalibrator
    from core.calibration_patterns import load_pattern_from_json
    
    # Set up paths
    temp_dir = '{temp_dir}'
    image_paths = sorted(glob.glob(os.path.join(temp_dir, '*.jpg')))
    config_path = os.path.join(temp_dir, 'chessboard_config.json')
    
    print(f"Found {{len(image_paths)}} images for calibration")
    
    # Load pattern configuration
    with open(config_path, 'r') as f:
        config_data = json.load(f)
    pattern = load_pattern_from_json(config_data)
    
    # Create calibrator
    calibrator = IntrinsicCalibrator(
        image_paths=image_paths,
        calibration_pattern=pattern
    )
    
    # Run calibration
    result = calibrator.calibrate(
        cameraMatrix=None,
        distCoeffs=None,
        flags=0,
        criteria=None,
        verbose=True
    )
    
    # Generate report in temp directory
    report_result = calibrator.generate_calibration_report(os.path.join(temp_dir, "intrinsic_calibration_report"))
    
    # Save results to JSON file for the web interface
    output_data = {{
        'success': True,
        'rms_error': float(result['rms_error']),
        'camera_matrix': result['camera_matrix'].tolist(),
        'distortion_coefficients': result['distortion_coefficients'].flatten().tolist(),
        'image_count': len(image_paths),
        'report_html': report_result.get('html_report', '') if report_result else '',
        'report_json': report_result.get('json_data', '') if report_result else ''
    }}
    
    output_path = os.path.join(temp_dir, 'calibration_result.json')
    with open(output_path, 'w') as f:
        json.dump(output_data, f, indent=2)
    
    print("✅ Calibration completed successfully!")
    print(f"RMS Error: {{result['rms_error']}}")
    
except Exception as e:
    print(f"❌ Calibration failed: {{str(e)}}")
    import traceback
    traceback.print_exc()
    
    # Save error result
    error_data = {{
        'success': False,
        'message': str(e),
        'error_type': type(e).__name__
    }}
    
    output_path = os.path.join('{temp_dir}', 'calibration_result.json')
    with open(output_path, 'w') as f:
        json.dump(error_data, f, indent=2)
'''
                    
                    # Write the calibration script to a temporary file
                    script_path = os.path.join(temp_dir, 'run_calibration.py')
                    with open(script_path, 'w') as f:
                        f.write(calibration_script)
                    
                    # Run the calibration script
                    result_file = os.path.join(temp_dir, 'calibration_result.json')
                    
                    # Remove any existing result file
                    if os.path.exists(result_file):
                        os.remove(result_file)
                    
                    # Run the script with proper Python environment
                    process = subprocess.run([
                        sys.executable, script_path
                    ], 
                    capture_output=True, 
                    text=True, 
                    timeout=300,  # 5 minute timeout
                    cwd=scripts_dir
                    )
                    
                    self.get_logger().info(f"Calibration process stdout: {process.stdout}")
                    if process.stderr:
                        self.get_logger().warning(f"Calibration process stderr: {process.stderr}")
                    
                    # Read the result
                    if os.path.exists(result_file):
                        with open(result_file, 'r') as f:
                            result_data = json.load(f)
                        
                        # Clean up temporary files
                        try:
                            os.remove(script_path)
                            os.remove(config_path)
                        except Exception:
                            pass
                        
                        return JSONResponse(content=result_data)
                    else:
                        return JSONResponse(content={
                            'success': False,
                            'message': f'Calibration process failed. Return code: {process.returncode}',
                            'stdout': process.stdout,
                            'stderr': process.stderr
                        }, status_code=500)
                    
                except subprocess.TimeoutExpired:
                    return JSONResponse(content={
                        'success': False,
                        'message': 'Calibration process timed out (>5 minutes). Please try with fewer images or check the calibration target.'
                    }, status_code=500)
                except Exception as e:
                    self.get_logger().error(f"Error running intrinsic calibration: {str(e)}")
                    return JSONResponse(content={
                        'success': False,
                        'message': f'Server error: {str(e)}'
                    }, status_code=500)
            
            @app.post("/api/calibration/eye-in-hand")
            async def run_eye_in_hand_calibration(request: Request):
                """Run eye-in-hand camera calibration using the camera calibration toolkit"""
                try:
                    data = await request.json()
                    chessboard_width = data.get('chessboard_width', 11)
                    chessboard_height = data.get('chessboard_height', 8) 
                    square_size = data.get('square_size', 0.02)  # 20mm
                    
                    # Get workspace root directory
                    current_file = os.path.abspath(__file__)
                    workspace_root = None
                    
                    path_parts = current_file.split(os.sep)
                    for i, part in enumerate(path_parts):
                        if part == 'robot_dc2':
                            workspace_root = os.sep.join(path_parts[:i+1])
                            break
                    
                    if workspace_root is None:
                        workspace_root = '/home/a/Documents/robot_dc2'
                    
                    # Set up paths
                    temp_dir = os.path.join(workspace_root, 'temp')
                    scripts_dir = os.path.join(workspace_root, 'scripts', 'ThirdParty', 'camera_calibration_toolkit')
                    
                    if not os.path.exists(temp_dir):
                        return JSONResponse(content={'success': False, 'message': 'No calibration images found in temp directory'}, status_code=400)
                    
                    if not os.path.exists(scripts_dir):
                        return JSONResponse(content={'success': False, 'message': 'Camera calibration toolkit not found'}, status_code=500)
                    
                    # Check for image files and corresponding JSON files
                    import glob
                    image_files = glob.glob(os.path.join(temp_dir, '*.jpg'))
                    
                    if len(image_files) < 5:
                        return JSONResponse(content={
                            'success': False, 
                            'message': f'Not enough calibration images. Found {len(image_files)} images, need at least 5'
                        }, status_code=400)
                    
                    # Check if we have pose data for images
                    pose_count = 0
                    for img_file in image_files:
                        img_basename = os.path.basename(img_file).replace('.jpg', '')
                        json_file = os.path.join(temp_dir, f'{img_basename}.json')
                        if os.path.exists(json_file):
                            pose_count += 1
                    
                    if pose_count < 5:
                        return JSONResponse(content={
                            'success': False, 
                            'message': f'Not enough pose data. Found {pose_count} poses, need at least 5'
                        }, status_code=400)
                    
                    self.get_logger().info(f"Starting eye-in-hand calibration with {len(image_files)} images and {pose_count} poses")
                    
                    # Create chessboard config file
                    config_data = {
                        "pattern_id": "standard_chessboard",
                        "name": "Standard Chessboard",
                        "description": "Traditional black and white checkerboard pattern",
                        "is_planar": True,
                        "parameters": {
                            "width": chessboard_width,
                            "height": chessboard_height,
                            "square_size": square_size
                        }
                    }
                    
                    config_path = os.path.join(temp_dir, 'chessboard_config.json')
                    with open(config_path, 'w') as f:
                        json.dump(config_data, f, indent=2)
                    
                    # Create a Python script to run the eye-in-hand calibration
                    calibration_script = f'''
import os
import sys
import glob
import json
import numpy as np
import cv2

# Add the camera calibration toolkit to Python path
sys.path.append('{scripts_dir}')

try:
    from core.eye_in_hand_calibration import EyeInHandCalibrator
    from core.intrinsic_calibration import IntrinsicCalibrator
    from core.calibration_patterns import load_pattern_from_json
    
    # Set up paths
    temp_dir = '{temp_dir}'
    config_path = os.path.join(temp_dir, 'chessboard_config.json')
    
    # Load pattern configuration
    with open(config_path, 'r') as f:
        config_data = json.load(f)
    pattern = load_pattern_from_json(config_data)
    
    # Load images and poses
    image_files = sorted(glob.glob(os.path.join(temp_dir, '*.jpg')))
    images = []
    end2base_matrices = []
    
    for img_file in image_files:
        # Load image
        img = cv2.imread(img_file)
        if img is not None:
            # Load corresponding pose
            img_basename = os.path.basename(img_file).replace('.jpg', '')
            json_file = os.path.join(temp_dir, f'{{img_basename}}.json')
            
            if os.path.exists(json_file):
                with open(json_file, 'r') as f:
                    pose_data = json.load(f)
                end2base_matrix = np.array(pose_data['end2base'])
                
                images.append(img)
                end2base_matrices.append(end2base_matrix)
            else:
                print(f"Warning: No pose file found for {{img_file}}")
    
    print(f"Loaded {{len(images)}} images and {{len(end2base_matrices)}} robot poses")
    
    if len(images) < 5 or len(end2base_matrices) < 5:
        raise Exception(f"Insufficient data: {{len(images)}} images, {{len(end2base_matrices)}} poses")
    
    # Step 1: Perform intrinsic calibration
    print("Performing intrinsic calibration...")
    intrinsic_calibrator = IntrinsicCalibrator(images=images, calibration_pattern=pattern)
    intrinsic_results = intrinsic_calibrator.calibrate(verbose=True)
    
    if not intrinsic_results:
        raise Exception("Intrinsic calibration failed")
    
    camera_matrix = intrinsic_results['camera_matrix']
    distortion_coeffs = intrinsic_results['distortion_coefficients']
    intrinsic_rms = intrinsic_results['rms_error']
    print(f"Intrinsic calibration completed - RMS: {{intrinsic_rms:.4f}} pixels")
    
    # Step 2: Perform eye-in-hand calibration
    print("Performing eye-in-hand calibration...")
    calibrator = EyeInHandCalibrator(
        images=images,
        end2base_matrices=end2base_matrices,
        calibration_pattern=pattern,
        camera_matrix=camera_matrix,
        distortion_coefficients=distortion_coeffs.flatten(),
        verbose=True
    )
    
    result = calibrator.calibrate(verbose=True)
    
    if result is None:
        raise Exception("Eye-in-hand calibration failed")
    
    # Generate report in temp directory
    report_result = calibrator.generate_calibration_report(os.path.join(temp_dir, "eye_in_hand_calibration_report"))
    
    # Save results to JSON file for the web interface
    output_data = {{
        'success': True,
        'rms_error': float(result['rms_error']),
        'intrinsic_rms_error': float(intrinsic_rms),
        'cam2end_matrix': result['cam2end_matrix'].tolist(),
        'target2base_matrix': result['target2base_matrix'].tolist(),
        'camera_matrix': camera_matrix.tolist(),
        'distortion_coefficients': distortion_coeffs.flatten().tolist(),
        'image_count': len(images),
        'pose_count': len(end2base_matrices),
        'report_html': report_result.get('html_report', '') if report_result else '',
        'report_json': report_result.get('json_data', '') if report_result else ''
    }}
    
    output_path = os.path.join(temp_dir, 'eye_in_hand_result.json')
    with open(output_path, 'w') as f:
        json.dump(output_data, f, indent=2)
    
    print("✅ Eye-in-hand calibration completed successfully!")
    print(f"Intrinsic RMS Error: {{intrinsic_rms:.4f}} pixels")
    print(f"Eye-in-hand RMS Error: {{result['rms_error']:.4f}} pixels")
    
except Exception as e:
    print(f"❌ Eye-in-hand calibration failed: {{str(e)}}")
    import traceback
    traceback.print_exc()
    
    # Save error result
    error_data = {{
        'success': False,
        'message': str(e),
        'error_type': type(e).__name__
    }}
    
    output_path = os.path.join('{temp_dir}', 'eye_in_hand_result.json')
    with open(output_path, 'w') as f:
        json.dump(error_data, f, indent=2)
'''
                    
                    # Write the calibration script to a temporary file
                    script_path = os.path.join(temp_dir, 'run_eye_in_hand_calibration.py')
                    with open(script_path, 'w') as f:
                        f.write(calibration_script)
                    
                    # Run the calibration script
                    result_file = os.path.join(temp_dir, 'eye_in_hand_result.json')
                    
                    # Remove any existing result file
                    if os.path.exists(result_file):
                        os.remove(result_file)
                    
                    # Run the script with proper Python environment
                    import subprocess
                    import sys
                    process = subprocess.run([
                        sys.executable, script_path
                    ], 
                    capture_output=True, 
                    text=True, 
                    timeout=600,  # 10 minute timeout for eye-in-hand calibration
                    cwd=scripts_dir
                    )
                    
                    self.get_logger().info(f"Eye-in-hand calibration process stdout: {process.stdout}")
                    if process.stderr:
                        self.get_logger().warning(f"Eye-in-hand calibration process stderr: {process.stderr}")
                    
                    # Read the result
                    if os.path.exists(result_file):
                        with open(result_file, 'r') as f:
                            result_data = json.load(f)
                        
                        # Clean up temporary files
                        try:
                            os.remove(script_path)
                            os.remove(config_path)
                        except Exception:
                            pass
                        
                        return JSONResponse(content=result_data)
                    else:
                        return JSONResponse(content={
                            'success': False,
                            'message': f'Eye-in-hand calibration process failed. Return code: {process.returncode}',
                            'stdout': process.stdout,
                            'stderr': process.stderr
                        }, status_code=500)
                    
                except subprocess.TimeoutExpired:
                    return JSONResponse(content={
                        'success': False,
                        'message': 'Eye-in-hand calibration process timed out (>10 minutes). Please try with fewer images or check the calibration setup.'
                    }, status_code=500)
                except Exception as e:
                    self.get_logger().error(f"Error running eye-in-hand calibration: {str(e)}")
                    return JSONResponse(content={
                        'success': False,
                        'message': f'Server error: {str(e)}'
                    }, status_code=500)
            
            # Static file serving for urdf-loaders library
            @app.get("/third_party/{path:path}")
            async def serve_urdf_loader_files(path: str):
                """Serve static files from urdf_web_viewer third_party directory"""
                try:
                    # Find the workspace src directory more reliably
                    # Look for the colcon_ws directory in the path
                    current_path = os.path.abspath(__file__)
                    while current_path != os.path.dirname(current_path):
                        if os.path.basename(current_path) == 'colcon_ws':
                            workspace_src_dir = os.path.join(current_path, 'src')
                            break
                        current_path = os.path.dirname(current_path)
                    else:
                        # Fallback: try to find src directory
                        workspace_src_dir = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), 'src')
                    
                    third_party_dir = os.path.join(workspace_src_dir, 'urdf_web_viewer', 'third_party')
                    file_path = os.path.join(third_party_dir, path)
                    
                    if os.path.exists(file_path) and os.path.isfile(file_path):
                        return FileResponse(file_path)
                    else:
                        return JSONResponse(content={'error': f'File not found: {file_path}'}, status_code=404)
                except Exception as e:
                    return JSONResponse(content={'error': str(e)}, status_code=500)
            
            # Serve JavaScript file
            @app.get("/js/{filename}")
            async def serve_js_file(filename: str):
                """Serve JavaScript files"""
                js_path = os.path.join(STATIC_DIR, 'js', filename)
                if os.path.exists(js_path) and os.path.isfile(js_path):
                    return FileResponse(js_path, media_type='application/javascript')
                else:
                    return JSONResponse(content={'error': f'File not found: {filename}'}, status_code=404)
            
            # Serve CSS file
            @app.get("/css/{filename}")
            async def serve_css_file(filename: str):
                """Serve CSS files"""
                css_path = os.path.join(STATIC_DIR, 'css', filename)
                if os.path.exists(css_path) and os.path.isfile(css_path):
                    return FileResponse(css_path, media_type='text/css')
                else:
                    return JSONResponse(content={'error': f'File not found: {filename}'}, status_code=404)
            
            # Mount static files
            app.mount("/web", StaticFiles(directory=STATIC_DIR), name="web")
            
            # WebSocket endpoint for real-time updates
            @app.websocket("/ws/robot_state")
            async def websocket_endpoint(websocket: WebSocket):
                await self.connection_manager.connect(websocket)
                try:
                    while True:
                        # Wait for incoming messages (like pong responses)
                        # This keeps the connection alive
                        try:
                            # Use asyncio.wait_for to timeout if no message received
                            message = await asyncio.wait_for(websocket.receive_text(), timeout=30.0)
                            # Echo back any received messages for debugging
                            if message != "ping":
                                await websocket.send_text(f"echo: {message}")
                        except asyncio.TimeoutError:
                            # Send ping to keep connection alive
                            await websocket.send_text("ping")
                        except WebSocketDisconnect:
                            break
                except WebSocketDisconnect:
                    self.connection_manager.disconnect(websocket)
                except Exception as e:
                    self.get_logger().error(f"WebSocket error: {e}")
                    self.connection_manager.disconnect(websocket)
            
            # WebSocket endpoint for FT sensor data
            @app.websocket("/ws/ft_sensor")
            async def ft_sensor_websocket(websocket: WebSocket):
                await self.ft_connection_manager.connect(websocket)
                last_sent_data = None
                try:
                    while True:
                        try:
                            # Get latest FT data
                            latest_data = self.get_latest_ft_data()
                            
                            # Only send if data exists and has changed
                            if latest_data and latest_data != last_sent_data:
                                await websocket.send_json(latest_data)
                                last_sent_data = latest_data.copy() if isinstance(latest_data, dict) else latest_data
                            
                            # Wait 100ms before next check (10Hz instead of 20Hz)
                            await asyncio.sleep(0.1)
                            
                        except WebSocketDisconnect:
                            break
                        except Exception as e:
                            self.get_logger().debug(f"Error sending FT data: {e}")
                            break
                            
                except WebSocketDisconnect:
                    self.ft_connection_manager.disconnect(websocket)
                except Exception as e:
                    self.get_logger().debug(f"FT WebSocket error: {e}")
                    self.ft_connection_manager.disconnect(websocket)
            
            # API endpoint to get latest FT sensor data
            @app.get("/api/ft_sensor/latest")
            async def get_ft_sensor_data():
                latest_data = self.get_latest_ft_data()
                if latest_data:
                    return JSONResponse(content=latest_data)
                else:
                    return JSONResponse(content={"error": "No FT sensor data available"}, status_code=404)

            # Run server
            uvicorn.run(app, host="0.0.0.0", port=self.port)
        
        # Start server in daemon thread
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        
        # Start the state broadcaster thread
        broadcaster_thread = threading.Thread(target=self.run_state_broadcaster, daemon=True)
        broadcaster_thread.start()
        
        self.get_logger().info(f'Web server started on port {self.port}')
        
    def run_state_broadcaster(self):
        """Run the state broadcaster loop in a separate thread"""
        import asyncio
        import time
        
        # Create a new event loop for this thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        # Keep track of last broadcast time to avoid over-broadcasting
        last_broadcast_time = 0
        min_broadcast_interval = 0.05  # 50ms minimum between broadcasts (20Hz max)
        
        async def broadcast_state():
            nonlocal last_broadcast_time
            
            while True:
                # Wait for new state event
                self.new_state_event.wait()
                self.new_state_event.clear()
                
                # Enforce minimum interval between broadcasts
                current_time = time.time()
                if current_time - last_broadcast_time < min_broadcast_interval:
                    # Skip this update if too soon after the last one
                    await asyncio.sleep(0.005)  # Small sleep
                    continue
                
                # Get the latest state and broadcast it
                with self.robot_state_lock:
                    if self.latest_robot_state:
                        await self.connection_manager.broadcast(self.latest_robot_state)
                        last_broadcast_time = time.time()
                
                # Small sleep to avoid CPU overuse
                await asyncio.sleep(0.01)
        
        # Run the broadcast loop
        loop.run_until_complete(broadcast_state())
    
    def destroy_node(self):
        """Clean up resources when shutting down"""
        # No need to stop camera stream since we're using ROS topic subscription
        
        # Call parent destroy_node
        super().destroy_node()

    async def execute_tool_control_operation(self, tool_type, bool_register_output, wait_completion=False, timeout=100000):
        """Execute tool control operation and optionally wait for completion"""
        try:
            # Define target states based on tool type
            target_states = {
                'gripper': [0, 1, 1, 1],      # A: gripper
                'frame': [1, 0, 1, 1],   # B: frame  
                'stickP': [0, 1, 0, 1],   # C: gripper + stickP
                'stickR': [0, 1, 1, 0],   # D: gripper + stickR
                'homing': [1, 1, 1, 1]    # Reset to default state
            }
            
            # Define valid states
            valid_states = {
                '1,1,1,1',  # 1111 - All tools are at home position
                '0,1,1,1',  # 0111 - Get Gripper
                '1,0,1,1',  # 1011 - Get Frame
                '0,1,0,1',  # 0101 - Get StickP(must operate after get gripper)
                '0,1,1,0'   # 0110 - Get StickR(must operate after get gripper)
            }
            
            # Get current state
            gripper_state = 1 if bool_register_output[0] or bool_register_output[0] == 1 else 0
            frame_state = 1 if bool_register_output[1] or bool_register_output[1] == 1 else 0
            stick_p_state = 1 if bool_register_output[2] or bool_register_output[2] == 1 else 0
            stick_r_state = 1 if bool_register_output[3] or bool_register_output[3] == 1 else 0
            
            current_state = [gripper_state, frame_state, stick_p_state, stick_r_state]
            target_state = target_states.get(tool_type, [1, 1, 1, 1])
            
            # Find shortest path using BFS
            path = self.find_shortest_path(current_state, target_state, valid_states)
            
            if not path:
                return {
                    'status': 'success',
                    'message': f'Tool {tool_type} is already at target state',
                    'path': [],
                    'current_state': current_state,
                    'target_state': target_state
                }
            
            if path and path[0] == 'No valid path found':
                return {
                    'status': 'error',
                    'error': 'No valid path found to target state',
                    'current_state': current_state,
                    'target_state': target_state
                }
            
            # Execute the action sequence
            execution_result = await self.execute_action_sequence_backend(path, wait_completion, timeout)
            
            return {
                'status': 'success',
                'message': f'Tool {tool_type} operation executed successfully',
                'path': path,
                'current_state': current_state,
                'target_state': target_state,
                'execution_result': execution_result
            }
            
        except Exception as e:
            self.get_logger().error(f"Error executing tool control operation {tool_type}: {str(e)}")
            return {
                'status': 'error',
                'error': str(e)
            }
    
    def find_shortest_path(self, current_state, target_state, valid_states):
        """Find shortest path using BFS algorithm (similar to duco_test_tool.py)"""
        # Convert states to string for comparison
        current_state_str = ','.join(map(str, current_state))
        target_state_str = ','.join(map(str, target_state))
        
        # If already at target state
        if current_state_str == target_state_str:
            return []
        
        # BFS queue: [state, path]
        queue = [[current_state[:], []]]
        visited = {current_state_str}
        
        # Define all possible actions
        actions = [
            {
                'name': 'zero2gripper',
                'apply': lambda state: [0, state[1], state[2], state[3]] if state[0] == 1 else None
            },
            {
                'name': 'gripper2zero', 
                'apply': lambda state: [1, state[1], state[2], state[3]] if state[0] == 0 else None
            },
            {
                'name': 'zero2frame',
                'apply': lambda state: [state[0], 0, state[2], state[3]] if state[1] == 1 else None
            },
            {
                'name': 'frame2zero',
                'apply': lambda state: [state[0], 1, state[2], state[3]] if state[1] == 0 else None
            },
            {
                'name': 'zero2stickP',
                'apply': lambda state: [state[0], state[1], 0, state[3]] if state[2] == 1 else None
            },
            {
                'name': 'stickP2zero',
                'apply': lambda state: [state[0], state[1], 1, state[3]] if state[2] == 0 else None
            },
            {
                'name': 'zero2stickR',
                'apply': lambda state: [state[0], state[1], state[2], 0] if state[3] == 1 else None
            },
            {
                'name': 'stickR2zero',
                'apply': lambda state: [state[0], state[1], state[2], 1] if state[3] == 0 else None
            }
        ]
        
        while queue:
            current_state, path = queue.pop(0)
            
            # Try all possible actions
            for action in actions:
                new_state = action['apply'](current_state)
                
                if new_state:
                    new_state_str = ','.join(map(str, new_state))
                    
                    # Check if new state is valid and not visited
                    if new_state_str in valid_states and new_state_str not in visited:
                        new_path = path + [action['name']]
                        
                        # If reached target state
                        if new_state_str == target_state_str:
                            return new_path
                        
                        queue.append([new_state, new_path])
                        visited.add(new_state_str)
        
        # No path found
        return ['No valid path found']
    
    async def execute_action_sequence_backend(self, action_path, wait_completion=False, timeout=100000):
        """Execute action sequence by sending task commands"""
        try:
            # Map BFS action names to robot program commands
            action_to_robot_command = {
                'zero2gripper': 'run_program zero2jaw.jspf true',
                'gripper2zero': 'run_program jaw2zero.jspf true',
                'zero2frame': 'run_program zero2holder.jspf true',
                'frame2zero': 'run_program holder2zero.jspf true',
                'zero2stickP': 'run_program zero2stickP.jspf true',
                'stickP2zero': 'run_program stickP2zero.jspf true',
                'zero2stickR': 'run_program zero2stickR.jspf true',
                'stickR2zero': 'run_program stickR2zero.jspf true'
            }
            
            execution_log = []
            
            # Execute each action in sequence
            for i, action in enumerate(action_path):
                robot_command = action_to_robot_command.get(action)
                
                if robot_command:
                    self.get_logger().info(f"Executing step {i + 1}/{len(action_path)}: {action} -> {robot_command}")
                    
                    # Send the robot command
                    command_result = self.send_command(robot_command)
                    execution_log.append({
                        'step': i + 1,
                        'action': action,
                        'robot_command': robot_command,
                        'result': command_result
                    })
                    
                    if wait_completion:
                        # Wait for program to complete before sending next command
                        self.get_logger().info(f"Waiting for program completion after {action}...")
                        await self.wait_for_program_completion_backend(timeout)
                    else:
                        # Small delay between commands
                        await asyncio.sleep(0.5)
                    
                    self.get_logger().info(f"Completed step {i + 1}/{len(action_path)}: {action}")
                else:
                    error_msg = f"Unknown action: {action}"
                    self.get_logger().error(error_msg)
                    execution_log.append({
                        'step': i + 1,
                        'action': action,
                        'error': error_msg
                    })
                    break
            
            return {
                'success': True,
                'execution_log': execution_log,
                'wait_completion': wait_completion
            }
            
        except Exception as e:
            self.get_logger().error(f"Error executing action sequence: {str(e)}")
            return {
                'success': False,
                'error': str(e),
                'execution_log': execution_log if 'execution_log' in locals() else []
            }
    
    async def wait_for_program_completion_backend(self, timeout=100000):
        """Wait for robot program to complete execution by monitoring boolRegisterOutput[4]"""
        start_time = time.time()
        program_started = False
        initial_delay_completed = False
        
        self.get_logger().info('Starting program completion wait...')
        
        while time.time() - start_time < timeout / 1000.0:  # Convert to seconds
            # Add initial delay to allow command to be processed
            if not initial_delay_completed:
                if time.time() - start_time < 0.2:  # Wait 200ms initially
                    await asyncio.sleep(0.05)
                    continue
                else:
                    initial_delay_completed = True
                    self.get_logger().info('Initial delay completed, starting status monitoring...')
            
            # Check if we have robot state data
            state = self.get_latest_robot_state()
            if state and 'boolRegisterOutput' in state:
                bool_register_output = state['boolRegisterOutput']
                
                # Program status logic: 
                # True/1 = program started/running, False/0 = program completed/idle
                bit4_value = bool_register_output[4]
                is_program_active = bit4_value or bit4_value == 1
                is_program_idle = not bit4_value or bit4_value == 0
                
                self.get_logger().debug(f"Program status check: boolRegisterOutput[4] = {bit4_value}, active={is_program_active}, idle={is_program_idle}, started={program_started}")
                
                # Wait for program to start (bit goes to 1)
                if not program_started and is_program_active:
                    program_started = True
                    self.get_logger().info('Program started (bit 4 went to 1)')
                
                # Wait for program to complete (bit goes back to 0) 
                if program_started and is_program_idle:
                    self.get_logger().info('Program completed (bit 4 went to 0)')
                    return True
            else:
                self.get_logger().debug('No robot state data available, waiting...')
            
            # Check again after a short delay
            await asyncio.sleep(0.1)  # Check every 100ms
        
                # Timeout reached
        self.get_logger().error('Timeout waiting for program completion')
        raise Exception('Timeout waiting for program completion')

def main(args=None):
    """Main entry point for the robot arm web server"""
    rclpy.init(args=args)
    
    try:
        node = RobotArmWebServer()
        rclpy.spin(node)
    except KeyboardInterrupt:
        print("Shutting down robot arm web server...")
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
