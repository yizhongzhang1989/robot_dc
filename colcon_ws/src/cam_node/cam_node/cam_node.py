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

def get_stream_resolution(url):
    """Get the resolution of an RTSP stream using ffprobe."""
    cmd = [
        "ffprobe",
        "-v", "error",
        "-select_streams", "v:0",
        "-show_entries", "stream=width,height",
        "-of", "json",
        url
    ]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
        if result.returncode != 0:
            print(f"ffprobe failed for {url}: {result.stderr}")
            return 640, 480  # Default resolution
        info = json.loads(result.stdout)
        w = info["streams"][0]["width"]
        h = info["streams"][0]["height"]
        return w, h
    except Exception as e:
        print(f"Error getting stream resolution for {url}: {e}")
        return 640, 480  # Default resolution

class RTSPStream:
    """RTSP stream handler using ffmpeg."""
    
    def __init__(self, name, url):
        self.name = name
        self.url = url
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        self.width, self.height = get_stream_resolution(self.url)
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

    def update(self):
        """Continuously read frames from ffmpeg."""
        frame_size = self.width * self.height * 3
        while self.running:
            try:
                if self.proc.stdout is None:
                    continue
                raw = self.proc.stdout.read(frame_size)
                if len(raw) != frame_size:
                    continue
                frame = np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 3))
                with self.lock:
                    self.frame = frame
            except Exception as e:
                print(f"[{self.name}] Error reading frame: {e}")
                continue

    def get_frame(self):
        """Get the current frame."""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        """Stop the stream."""
        self.running = False
        if self.proc:
            self.proc.kill()
        if self.thread.is_alive():
            self.thread.join()

class CamNode(Node):
    """ROS2 node for camera snapshot service."""
    
    def __init__(self):
        super().__init__('cam_node')
        self.get_logger().info("Initializing CamNode...")
        
        # Initialize RTSP streams
        self.streams = {}
        for key, url in RTSP_URLS.items():
            self.get_logger().info(f"Starting {key} stream: {url}")
            self.streams[key] = RTSPStream(key, url)
        
        # Create snapshot service
        self.snapshot_srv = self.create_service(
            Trigger,
            'snapshot',
            self.handle_snapshot
        )
        
        self.get_logger().info("CamNode initialized successfully")

    def handle_snapshot(self, request, response):
        """Handle snapshot service request."""
        self.get_logger().info("Snapshot request received")
        
        result = {}
        
        for key, stream in self.streams.items():
            timestamp = datetime.datetime.now().isoformat()
            frame = stream.get_frame()
            if frame is not None:
                try:
                    _, buffer = cv2.imencode('.jpg', frame)
                    img_b64 = base64.b64encode(buffer.tobytes()).decode('utf-8')
                    result[key] = {'img': img_b64, 'timestamp': timestamp}
                    self.get_logger().info(f"Snapshot captured for {key}")
                except Exception as e:
                    self.get_logger().error(f"Error encoding frame for {key}: {e}")
                    result[key] = {'img': '', 'timestamp': timestamp}
            else:
                self.get_logger().warning(f"No frame available for {key}")
                result[key] = {'img': '', 'timestamp': timestamp}
        
        response.success = True
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
