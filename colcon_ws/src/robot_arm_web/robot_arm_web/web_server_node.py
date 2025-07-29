import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Image
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse, Response
import os
from ament_index_python.packages import get_package_share_directory
import threading
import json
import asyncio
from typing import List
import xml.etree.ElementTree as ET
import base64
import socket
import time
import cv2
from cv_bridge import CvBridge

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
        self.declare_parameter('urdf_file', '/home/a/Documents/robot_dc/colcon_ws/src/duco_gcr5_910_urdf/urdf/duco_gcr5_910_urdf.urdf')
        
        self.device_id = self.get_parameter('device_id').value
        self.port = self.get_parameter('port').value
        self.urdf_file = self.get_parameter('urdf_file').value
        
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
        
        # Create subscription for camera images
        self.camera_subscription = self.create_subscription(
            Image,
            '/rtsp_camera/image_raw',
            self.camera_callback,
            1
        )
        
        # Store latest robot state
        self.latest_robot_state = None
        self.robot_state_lock = threading.Lock()
        
        # Store latest camera image
        self.latest_camera_image = None
        self.camera_image_lock = threading.Lock()
        self.bridge = CvBridge()
        
        # ROS2 Camera Publisher (auto-created when web server starts)
        self.rtsp_url = "rtsp://admin:123456@192.168.1.102/stream0"
        self.camera_cap = None
        self.camera_running = False
        self.camera_thread = None
        self.frame_ready_event = threading.Event()
        
        # Create ROS2 Image Publisher for camera
        self.camera_image_publisher = self.create_publisher(
            Image,
            '/rtsp_camera/image_raw',
            1
        )
        
        # Performance monitoring for camera
        self.camera_frame_count = 0
        self.camera_start_time = time.time()
        self.camera_consecutive_failures = 0
        
        # Start ROS2 camera publisher
        self.start_camera_publisher()
        
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
    
    def add_scripts_to_path(self):
        """Add the scripts directory to Python path for importing FTC modules"""
        import sys
        import os
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
                self.latest_robot_state = json.loads(msg.data)
                # Signal that new state is available
                self.new_state_event.set()
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Error parsing robot state JSON: {e}')
    
    def camera_callback(self, msg):
        """Handle incoming camera image messages"""
        try:
            # Convert ROS Image message to OpenCV format
            cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
            
            # Encode image as JPEG
            _, buffer = cv2.imencode('.jpg', cv_image, [cv2.IMWRITE_JPEG_QUALITY, 80])
            
            # Store the encoded image
            with self.camera_image_lock:
                self.latest_camera_image = buffer.tobytes()
                
        except Exception as e:
            self.get_logger().error(f'Error processing camera image: {e}')
    
    def get_latest_camera_image(self):
        """Get the latest camera image as JPEG bytes"""
        with self.camera_image_lock:
            return self.latest_camera_image
    
    def start_camera_publisher(self):
        """Start ROS2 Camera Image Publisher (auto-created when web server starts)"""
        def camera_publisher_thread():
            """Camera publisher thread function - publishes to ROS2 topic"""
            while self.camera_running:
                try:
                    # Initialize camera if not already done
                    if self.camera_cap is None:
                        self.get_logger().info(f"Starting ROS2 Camera Publisher - Connecting to RTSP: {self.rtsp_url}")
                        self.camera_cap = cv2.VideoCapture(self.rtsp_url, cv2.CAP_FFMPEG)
                        
                        # Set low latency parameters
                        self.camera_cap.set(cv2.CAP_PROP_BUFFERSIZE, 0)
                        self.camera_cap.set(cv2.CAP_PROP_FPS, 30)
                        
                        try:
                            self.camera_cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc('H', '2', '6', '4'))
                        except Exception:
                            pass
                        
                        if not self.camera_cap.isOpened():
                            self.get_logger().error("Cannot open RTSP stream for ROS2 publisher. Please check the URL.")
                            self.camera_cap = None
                            time.sleep(2.0)  # Wait before retry
                            continue
                        else:
                            self.get_logger().info("ROS2 Camera Publisher connected to RTSP successfully")
                    
                    # Read frame
                    ret, frame = self.camera_cap.read()
                    
                    if ret:
                        try:
                            # Convert OpenCV image to ROS Image message
                            image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                            image_msg.header.stamp = self.get_clock().now().to_msg()
                            image_msg.header.frame_id = "camera_frame"
                            
                            # Publish to ROS2 topic
                            self.camera_image_publisher.publish(image_msg)
                            
                            self.camera_frame_count += 1
                            self.camera_consecutive_failures = 0
                            
                            # Also store JPEG encoded version for direct web API access
                            _, buffer = cv2.imencode('.jpg', frame, [cv2.IMWRITE_JPEG_QUALITY, 80])
                            with self.camera_image_lock:
                                self.latest_camera_image = buffer.tobytes()
                            
                            # Trigger frame ready event
                            self.frame_ready_event.set()
                            
                        except Exception as e:
                            self.get_logger().error(f"Error publishing camera image to ROS2: {e}")
                        
                    else:
                        self.camera_consecutive_failures += 1
                        if self.camera_consecutive_failures > 3:
                            self.get_logger().warn("Multiple camera read failures, trying to reconnect...")
                            self._reconnect_camera()
                            self.camera_consecutive_failures = 0
                        time.sleep(0.01)
                        
                except Exception as e:
                    self.get_logger().error(f"Error in camera publisher: {e}")
                    self._reconnect_camera()
                    time.sleep(1.0)
        
        # Start camera publisher
        self.camera_running = True
        self.camera_thread = threading.Thread(target=camera_publisher_thread, daemon=True)
        self.camera_thread.start()
        
        self.get_logger().info("ROS2 Camera Image Publisher started - publishing to /rtsp_camera/image_raw")
        
        # Start performance monitoring timer
        self.camera_monitor_timer = self.create_timer(5.0, self.print_camera_performance)
    
    def print_camera_performance(self):
        """Print camera performance statistics"""
        current_time = time.time()
        elapsed_time = current_time - self.camera_start_time
        
        if elapsed_time > 0:
            frame_fps = self.camera_frame_count / elapsed_time
            
            self.get_logger().info(
                f"Camera Performance: Publishing at {frame_fps:.1f}fps to /rtsp_camera/image_raw"
            )
            
            # Reset counters
            self.camera_frame_count = 0
            self.camera_start_time = current_time
    
    def _reconnect_camera(self):
        """Reconnect to RTSP camera"""
        try:
            if self.camera_cap is not None:
                self.camera_cap.release()
                self.camera_cap = None
            
            self.get_logger().info("Attempting to reconnect to RTSP camera...")
            
        except Exception as e:
            self.get_logger().error(f"Error during camera reconnection: {e}")
    
    def stop_camera_publisher(self):
        """Stop ROS2 camera publisher"""
        self.camera_running = False
        
        if self.camera_thread is not None:
            self.camera_thread.join(timeout=2.0)
        
        if self.camera_cap is not None:
            self.camera_cap.release()
            self.camera_cap = None
        
        # Destroy the camera monitor timer
        if hasattr(self, 'camera_monitor_timer'):
            self.camera_monitor_timer.destroy()
        
        self.get_logger().info("ROS2 Camera Image Publisher stopped")
    
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
            except:
                # Fallback for development
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
                    "available_commands": ["power_on", "power_off", "enable", "disable"]
                }
            
            @app.get("/api/camera/stream")
            def get_camera_stream():
                """Get current camera image as JPEG"""
                try:
                    image_data = self.get_latest_camera_image()
                    if image_data is None:
                        return Response(content="No camera image available", status_code=404)
                    
                    return Response(content=image_data, media_type="image/jpeg")
                except Exception as e:
                    self.get_logger().error(f"Error serving camera image: {e}")
                    return Response(content="Camera error", status_code=500)
            
            @app.get("/api/camera/status")
            def get_camera_status():
                """Get camera status"""
                image_data = self.get_latest_camera_image()
                return {
                    "connected": image_data is not None,
                    "topic": "/rtsp_camera/image_raw"
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
        # Stop ROS2 camera publisher
        self.stop_camera_publisher()
        
        # Call parent destroy_node
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotArmWebServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down robot arm web server...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
