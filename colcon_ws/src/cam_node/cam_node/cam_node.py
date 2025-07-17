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
import os

# 设置显示环境变量，确保Jetson本地显示
if 'DISPLAY' not in os.environ:
    os.environ['DISPLAY'] = ':0'

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
            return 640, 480
        info = json.loads(result.stdout)
        w = info["streams"][0]["width"]
        h = info["streams"][0]["height"]
        print(f"Stream resolution for {url}: {w}x{h}")
        return w, h
    except Exception as e:
        print(f"Error getting stream resolution for {url}: {e}")
        return 640, 480  # Default resolution

class RTSPStream:
    """RTSP stream handler using ffmpeg - exact copy of cam.py"""
    
    def __init__(self, name, url):
        self.name = name
        self.url = url
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        
        print(f"[{self.name}] Getting resolution...")
        self.width, self.height = get_stream_resolution(self.url)
        print(f"[{self.name}] Resolution: {self.width}x{self.height}")
        
        print(f"[{self.name}] Starting ffmpeg process...")
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
        
        print(f"[{self.name}] Starting update thread...")
        self.thread = threading.Thread(target=self.update, daemon=True)
        self.thread.start()
        print(f"[{self.name}] Thread started successfully")

    def update(self):
        """Continuously read frames from ffmpeg - exact copy of cam.py"""
        frame_size = self.width * self.height * 3
        print(f"[{self.name}] Starting update loop, frame_size: {frame_size}")
        
        while self.running:
            if self.proc.stdout is None:
                print(f"[{self.name}] No stdout, waiting...")
                continue
            
            raw = self.proc.stdout.read(frame_size)
            if len(raw) != frame_size:
                print(f"[{self.name}] Incomplete frame: {len(raw)}/{frame_size}")
                continue
            
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 3))
            with self.lock:
                self.frame = frame
            
            # Simple frame count for debugging
            if not hasattr(self, '_frame_count'):
                self._frame_count = 0
            self._frame_count += 1
            
            if self._frame_count == 1:
                print(f"[{self.name}] First frame received successfully")
            elif self._frame_count % 30 == 0:
                print(f"[{self.name}] Frame {self._frame_count} received")

    def get_frame(self):
        """Get the current frame - exact copy of cam.py"""
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        """Stop the stream - exact copy of cam.py"""
        self.running = False
        if self.proc:
            self.proc.kill()
        if self.thread.is_alive():
            self.thread.join()

class CamNode(Node):
    """ROS2 node for camera snapshot service."""
    
    def __init__(self, display=True):
        super().__init__('cam_node')
        self.display = display
        self.get_logger().info("Initializing CamNode...")
        # 初始化两个摄像头流
        self.streams = {}
        for key, url in RTSP_URLS.items():
            self.get_logger().info(f"Starting {key} stream: {url}")
            self.streams[key] = RTSPStream(key, url)
        # 创建快照服务
        self.snapshot_srv = self.create_service(
            Trigger,
            'snapshot',
            self.handle_snapshot
        )
        self.get_logger().info("CamNode initialized successfully")
        print("Camera display enabled. Press 'q' in any camera window to quit.")
        print("Press 's' in any camera window to save a snapshot.")
        print("High resolution camera should be on the left, low resolution on the right.")

    def handle_snapshot(self, request, response):
        """Handle snapshot service request - simplified like cam.py"""
        self.get_logger().info("Snapshot request received")
        
        result = {}
        
        for key, stream in self.streams.items():
            timestamp = datetime.datetime.now().isoformat()
            frame = stream.get_frame()
            if frame is not None:
                try:
                    # Encode as JPEG - same as cam.py
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
    node = CamNode(display=True)
    try:
        # 主动spin，保证OpenCV窗口实时刷新
        import time
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
