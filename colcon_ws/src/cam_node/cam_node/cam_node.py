import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import threading
import numpy as np
import cv2
import base64
import time
import os
from datetime import datetime
import json

def get_stream_resolution(url, max_retries=3, retry_delay=2):
    """Get the resolution of an RTSP stream using ffprobe with retry mechanism."""
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
                    # Use default resolution if ffprobe fails
                    print(f"Using default resolution for {url}")
                    return 1920, 1080
            
            info = json.loads(result.stdout)
            
            if "streams" not in info or len(info["streams"]) == 0:
                print(f"No streams found for {url}, attempt {attempt + 1}/{max_retries}")
                if attempt < max_retries - 1:
                    time.sleep(retry_delay)
                    continue
                else:
                    # Use default resolution
                    print(f"Using default resolution for {url}")
                    return 1920, 1080
            
            w = info["streams"][0]["width"]
            h = info["streams"][0]["height"]
            print(f"Stream resolution for {url}: {w}x{h}")
            return w, h
            
        except (json.JSONDecodeError, KeyError, subprocess.TimeoutExpired) as e:
            print(f"Error getting stream resolution for {url}, attempt {attempt + 1}/{max_retries}: {e}")
            if attempt < max_retries - 1:
                time.sleep(retry_delay)
                continue
            else:
                # Use default resolution as fallback
                print(f"Using default resolution for {url}")
                return 1920, 1080
    
    # This should never be reached, but just in case
    return 1920, 1080

class RTSPStream:
    """RTSP stream handler using ffmpeg."""
    
    def __init__(self, name, url):
        self.name = name
        self.url = url
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        
        print(f"Getting resolution for {name} stream: {url}")
        self.width, self.height = get_stream_resolution(self.url)
        print(f"Resolution for {name}: {self.width}x{self.height}")
        
        # Add frame tracking for reconnection detection
        self.frame_count = 0
        self.last_frame_update = time.time()
        
        print(f"Starting FFmpeg process for {name}...")
        # Add retry mechanism for FFmpeg process startup
        max_ffmpeg_retries = 3
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
                # Give FFmpeg a moment to start
                time.sleep(1)
                if self.proc.poll() is None:  # Process is still running
                    break
                else:
                    print(f"FFmpeg process failed for {name}, attempt {attempt + 1}/{max_ffmpeg_retries}")
                    if attempt < max_ffmpeg_retries - 1:
                        time.sleep(2)
                        continue
            except Exception as e:
                print(f"Error starting FFmpeg for {name}, attempt {attempt + 1}/{max_ffmpeg_retries}: {e}")
                if attempt < max_ffmpeg_retries - 1:
                    time.sleep(2)
                    continue
                else:
                    raise
        
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()
        print(f"Started {name} stream successfully")

    def update(self):
        frame_size = self.width * self.height * 3
        while self.running:
            if self.proc.stdout is None:
                continue
            raw = self.proc.stdout.read(frame_size)
            if len(raw) != frame_size:
                continue
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 3))
            with self.lock:
                self.frame = frame
                self.frame_count += 1
                self.last_frame_update = time.time()

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None
    
    def get_frame_info(self):
        """Get frame info for reconnection detection."""
        with self.lock:
            return {
                'frame_count': self.frame_count,
                'last_update': self.last_frame_update,
                'has_frame': self.frame is not None
            }

    def stop(self):
        self.running = False
        self.proc.kill()
        self.thread.join()

class CamNode(Node):
    """ROS2 node for camera snapshot service."""
    
    def __init__(self):
        super().__init__('cam_node')
        self.get_logger().info("Initializing CamNode...")
        
        # Declare and read camera parameters similar to motor_node
        self.declare_parameter('camera_name', 'cam100')
        self.camera_name = self.get_parameter('camera_name').value
        self.get_logger().info(f"camera_name from param = {self.camera_name}")
        
        self.declare_parameter('rtsp_url', 'rtsp://admin:123456@192.168.1.100/stream0')
        self.rtsp_url = self.get_parameter('rtsp_url').value
        self.get_logger().info(f"rtsp_url from param = {self.rtsp_url}")
        
        self.declare_parameter('camera_ip', '192.168.1.100')
        self.camera_ip = self.get_parameter('camera_ip').value
        self.get_logger().info(f"camera_ip from param = {self.camera_ip}")
        
        # Camera reconnection detection - use network ping instead of frame counting
        self.camera_status = {}  # Track camera connectivity status
        self.check_interval = 3.0  # Check every 3 seconds
        
        # Initialize single RTSP stream
        self.stream = None
        self.get_logger().info(f"Starting camera stream: {self.rtsp_url}")
        self.stream = RTSPStream(self.camera_name, self.rtsp_url)
        self.camera_status[self.camera_name] = {
            'last_check_time': time.time(),
            'was_disconnected': False,
            'reconnect_detected': False,
            'disconnection_logged': False
        }
        
        # Create snapshot service with camera-specific name
        service_name = f'{self.camera_name}_snapshot'
        self.snapshot_srv = self.create_service(
            Trigger,
            service_name,
            self.handle_snapshot
        )
        self.get_logger().info(f"Created snapshot service: {service_name}")
        
        # Create restart service with camera-specific name
        restart_service_name = f'restart_{self.camera_name}_node'
        self.restart_srv = self.create_service(
            Trigger,
            restart_service_name,
            self.handle_restart
        )
        self.get_logger().info(f"Created restart service: {restart_service_name}")
        
        # Create timer for camera status monitoring
        self.status_timer = self.create_timer(self.check_interval, self.check_camera_reconnection)
        
        self.get_logger().info("CamNode initialized successfully")

    def check_camera_reconnection(self):
        """Check camera connectivity by testing actual network connectivity."""
        current_time = time.time()
        restart_needed = False
        
        # Check single camera status
        status = self.camera_status[self.camera_name]
        
        # Quick ping test (timeout 1 second)
        ping_result = subprocess.run(
            ["ping", "-c", "1", "-W", "1", self.camera_ip], 
            capture_output=True, 
            text=True
        )
        is_reachable = ping_result.returncode == 0
        
        # Update our tracking
        status['last_check_time'] = current_time
        
        if is_reachable:
            # Camera is network reachable
            if status['was_disconnected']:
                # Camera was disconnected but now reachable again
                self.get_logger().error(f"Camera {self.camera_name} ({self.camera_ip}) RECONNECTED! Network is reachable again. Restarting node...")
                status['reconnect_detected'] = True
                restart_needed = True
            
        else:
            # Camera is not reachable
            if not status['was_disconnected']:
                self.get_logger().warning(f"Camera {self.camera_name} ({self.camera_ip}) DISCONNECTED - network unreachable")
                status['was_disconnected'] = True
                status['disconnection_logged'] = True
        
        # If camera reconnected, restart the node
        if restart_needed:
            self.get_logger().error("Camera reconnection detected. Restarting node immediately...")
            self.restart_node()

    def restart_node(self):
        """Restart the node by recreating the camera stream"""
        self.get_logger().error(f"Restarting camera node for camera {self.camera_name}...")
        
        try:
            # Clean up existing stream
            if hasattr(self, 'stream') and self.stream is not None:
                try:
                    self.stream.stop()
                    self.get_logger().info("Stopped existing camera stream")
                except Exception as e:
                    self.get_logger().warning(f"Error stopping stream: {e}")
            
            # Recreate the single stream
            self.stream = RTSPStream(self.camera_name, self.rtsp_url)
            
            # Reset status
            if self.camera_name in self.camera_status:
                self.camera_status[self.camera_name] = {
                    'was_disconnected': False,
                    'reconnect_detected': False,
                    'disconnection_logged': False,
                    'last_check_time': 0
                }
            
            self.get_logger().info(f"Camera node for {self.camera_name} restarted successfully")
            
        except Exception as e:
            self.get_logger().error(f"Failed to restart camera node: {e}")
            
        # Clear last error to allow fresh start
        self.last_error_time = None

    def handle_restart(self, request, response):
        """Handle restart service request from web interface."""
        self.get_logger().info("Manual restart request received from web interface")
        
        # Set response
        response.success = True
        response.message = "Restarting cam_node..."
        
        # Schedule restart in a separate thread to allow response to be sent
        def delayed_restart():
            time.sleep(1)  # Give time for response to be sent
            self.get_logger().info("Executing manual restart...")
            self.restart_node()
        
        restart_thread = threading.Thread(target=delayed_restart)
        restart_thread.start()
        
        return response

    def handle_snapshot(self, request, response):
        """Handle snapshot request for the camera"""
        try:
            if self.stream is None:
                response.success = False
                response.message = "Camera stream not available"
                self.get_logger().error(f"Snapshot failed: Camera stream not available for {self.camera_name}")
                return response
            
            # Try to get a frame from the camera
            frame = self.stream.get_frame()
            if frame is None:
                response.success = False
                response.message = "Failed to capture image from camera"
                self.get_logger().error(f"Snapshot failed: Could not get frame from {self.camera_name}")
                return response
            
            # Save the image to file
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"{self.camera_name}_snapshot_{timestamp}.jpg"
            filepath = os.path.join("/tmp", filename)
            
            # Encode image as base64 for web interface
            import cv2
            import base64
            _, buffer = cv2.imencode('.jpg', frame)
            img_b64 = base64.b64encode(buffer.tobytes()).decode('utf-8')
            
            # Save file and return base64 data
            success = cv2.imwrite(filepath, frame)
            if success:
                response.success = True
                # Return JSON string with both file path and base64 data
                result_data = {
                    "img": img_b64,
                    "timestamp": timestamp,
                    "filepath": filepath,
                    "camera": self.camera_name
                }
                response.message = str(result_data)
                self.get_logger().info(f"Snapshot saved for {self.camera_name}: {filepath}")
            else:
                response.success = False
                response.message = "Failed to save image"
                self.get_logger().error(f"Failed to save snapshot for {self.camera_name}")
                
        except Exception as e:
            response.success = False
            response.message = f"Error taking snapshot: {str(e)}"
            self.get_logger().error(f"Exception in snapshot for {self.camera_name}: {e}")
        
        return response

    def destroy_node(self):
        """Clean up resources when destroying the node."""
        self.get_logger().info("Destroying CamNode...")
        if hasattr(self, 'stream') and self.stream is not None:
            self.stream.stop()
        super().destroy_node()

def main():
    """Main function to run the cam node."""
    rclpy.init()
    node = CamNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
