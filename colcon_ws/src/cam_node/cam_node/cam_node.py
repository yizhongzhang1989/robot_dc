import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
import subprocess
import threading
import numpy as np
import cv2
import base64
import time
import datetime
import json

HIGH_URL = "rtsp://admin:123456@192.168.1.100/stream0"
LOW_URL = "rtsp://admin:123456@192.168.1.101/stream0"

RTSP_URLS = {
    'high': HIGH_URL,
    'low': LOW_URL
}

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
        
        # Camera reconnection detection - use network ping instead of frame counting
        self.camera_status = {}  # Track camera connectivity status
        self.check_interval = 3.0  # Check every 3 seconds
        
        # Initialize RTSP streams
        self.streams = {}
        for key, url in RTSP_URLS.items():
            self.get_logger().info(f"Starting {key} stream: {url}")
            self.streams[key] = RTSPStream(key, url)
            self.camera_status[key] = {
                'last_check_time': time.time(),
                'was_disconnected': False,
                'reconnect_detected': False,
                'disconnection_logged': False
            }
        
        # Create snapshot service
        self.snapshot_srv = self.create_service(
            Trigger,
            'snapshot',
            self.handle_snapshot
        )
        
        # Create restart service
        self.restart_srv = self.create_service(
            Trigger,
            'restart_cam_node',
            self.handle_restart
        )
        
        # Create timer for camera status monitoring
        self.status_timer = self.create_timer(self.check_interval, self.check_camera_reconnection)
        
        self.get_logger().info("CamNode initialized successfully")

    def check_camera_reconnection(self):
        """Check camera connectivity by testing actual network connectivity."""
        current_time = time.time()
        restart_needed = False
        
        for key, stream in self.streams.items():
            status = self.camera_status[key]
            
            # Test actual network connectivity using ping
            camera_ip = "192.168.1.100" if key == 'high' else "192.168.1.101"
            
            # Quick ping test (timeout 1 second)
            ping_result = subprocess.run(
                ["ping", "-c", "1", "-W", "1", camera_ip], 
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
                    self.get_logger().error(f"Camera {key} ({camera_ip}) RECONNECTED! Network is reachable again. Restarting node...")
                    status['reconnect_detected'] = True
                    restart_needed = True
                
            else:
                # Camera is not reachable
                if not status['was_disconnected']:
                    self.get_logger().warning(f"Camera {key} ({camera_ip}) DISCONNECTED - network unreachable")
                    status['was_disconnected'] = True
                    status['disconnection_logged'] = True
        
        # If any camera reconnected, restart the entire node
        if restart_needed:
            self.get_logger().error("Camera reconnection detected. Restarting node immediately...")
            self.restart_node()

    def restart_node(self):
        """Restart the cam node streams without restarting the entire process."""
        self.get_logger().info("Shutting down existing streams...")
        
        # Stop existing streams
        for stream in self.streams.values():
            stream.stop()
        
        # Clear existing streams
        self.streams.clear()
        
        # Wait for cameras to stabilize after reconnection
        self.get_logger().info("Waiting for cameras to stabilize...")
        time.sleep(3)  # Reduced from 5 to 3 seconds since we're not restarting the process
        
        # Reinitialize streams
        self.get_logger().info("Reinitializing camera streams...")
        for key, url in RTSP_URLS.items():
            self.get_logger().info(f"Restarting {key} stream: {url}")
            try:
                self.streams[key] = RTSPStream(key, url)
                # Reset camera status
                self.camera_status[key] = {
                    'last_check_time': time.time(),
                    'was_disconnected': False,
                    'reconnect_detected': False,
                    'disconnection_logged': False
                }
                self.get_logger().info(f"Successfully restarted {key} stream")
            except Exception as e:
                self.get_logger().error(f"Failed to restart {key} stream: {e}")
        
        self.get_logger().info("Camera node restart completed")

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
        """Handle snapshot service request."""
        self.get_logger().info("Snapshot request received")
        
        result = {}
        successful_snapshots = 0
        
        for key, stream in self.streams.items():
            timestamp = datetime.datetime.now().isoformat()
            
            frame = stream.get_frame()
            if frame is not None:
                try:
                    _, buffer = cv2.imencode('.jpg', frame)
                    img_b64 = base64.b64encode(buffer.tobytes()).decode('utf-8')
                    result[key] = {'img': img_b64, 'timestamp': timestamp, 'status': 'success'}
                    successful_snapshots += 1
                    self.get_logger().info(f"Snapshot captured for {key}")
                except Exception as e:
                    self.get_logger().error(f"Error encoding frame for {key}: {e}")
                    result[key] = {
                        'img': '', 
                        'timestamp': timestamp, 
                        'status': 'encoding_error',
                        'error': str(e)
                    }
            else:
                self.get_logger().warning(f"No frame available for {key}")
                result[key] = {
                    'img': '', 
                    'timestamp': timestamp, 
                    'status': 'no_frame'
                }
        
        # Determine overall success
        response.success = successful_snapshots > 0
        if successful_snapshots == 0:
            self.get_logger().error("No snapshots captured from any camera")
        elif successful_snapshots < len(self.streams):
            self.get_logger().warning(f"Only {successful_snapshots}/{len(self.streams)} cameras provided snapshots")
            
        response.message = str(result)
        return response

    def destroy_node(self):
        """Clean up resources when destroying the node."""
        self.get_logger().info("Destroying CamNode...")
        for stream in self.streams.values():
            stream.stop()
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
