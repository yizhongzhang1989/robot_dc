import cv2
import threading
import subprocess
import numpy as np
import json
from flask import Flask, Response, stream_with_context

app = Flask(__name__)

MAX_WINDOW_WIDTH = 800


def get_stream_resolution(url):
    """Get stream resolution with error handling for unavailable streams."""
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
            print(f"ffprobe failed for {url}: {result.stderr}")
            return None, None
            
        info = json.loads(result.stdout)
        
        if "streams" not in info or len(info["streams"]) == 0:
            print(f"No streams found for {url}")
            return None, None
            
        w = info["streams"][0]["width"]
        h = info["streams"][0]["height"]
        print(f"Stream resolution for {url}: {w}x{h}")
        return w, h
        
    except (json.JSONDecodeError, KeyError, subprocess.TimeoutExpired) as e:
        print(f"Error getting stream resolution for {url}: {e}")
        return None, None


class RTSPStream:
    def __init__(self, name, url):
        self.name = name
        self.url = url
        self.frame = None
        self.running = True
        self.lock = threading.Lock()
        self.available = False
        self.error_message = None

        # Try to get stream resolution
        self.width, self.height = get_stream_resolution(self.url)
        
        if self.width is None or self.height is None:
            print(f"[{self.name}] Stream not available: {self.url}")
            self.available = False
            self.error_message = f"Stream not available: {self.url}"
            return
        
        print(f"[{self.name}] Stream resolution: {self.width}x{self.height}")
        
        # Try to start FFmpeg process
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
            import time
            time.sleep(1)
            
            if self.proc.poll() is None:  # Process is still running
                self.available = True
                self.thread = threading.Thread(target=self.update, daemon=True)
                self.thread.start()
                print(f"[{self.name}] Stream started successfully")
            else:
                print(f"[{self.name}] FFmpeg process failed to start")
                self.available = False
                self.error_message = "FFmpeg process failed to start"
                
        except Exception as e:
            print(f"[{self.name}] Error starting stream: {e}")
            self.available = False
            self.error_message = f"Error starting stream: {str(e)}"

    def update(self):
        if not self.available:
            return
            
        frame_size = self.width * self.height * 3
        while self.running and self.available:
            try:
                raw = self.proc.stdout.read(frame_size)
                if len(raw) != frame_size:
                    continue
                frame = np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 3))
                with self.lock:
                    self.frame = frame
            except Exception as e:
                print(f"[{self.name}] Error reading frame: {e}")
                break

    def get_frame(self):
        if not self.available:
            return None
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        if self.available and hasattr(self, 'proc'):
            self.proc.kill()
            if hasattr(self, 'thread'):
                self.thread.join()


def resize_frame(frame, max_width):
    h, w = frame.shape[:2]
    if w > max_width:
        scale = max_width / w
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
    return frame


# Setup RTSP URLs
STREAM1_URL = "rtsp://admin:123456@192.168.1.100/stream0"
STREAM2_URL = "rtsp://admin:123456@192.168.1.101/stream0"

# Start streams with error handling
print("Initializing streams...")
stream1 = RTSPStream("Stream1", STREAM1_URL)
stream2 = RTSPStream("Stream2", STREAM2_URL)

print(f"Stream1 available: {stream1.available}")
print(f"Stream2 available: {stream2.available}")


def generate_stream(stream: RTSPStream):
    try:
        if not stream.available:
            # Generate a simple error image
            import time
            while True:
                # Create a simple error image
                error_img = np.zeros((240, 320, 3), dtype=np.uint8)
                error_img[:] = (50, 50, 50)  # Dark gray background
                
                # Add text (simplified - just a colored rectangle as placeholder)
                cv2.rectangle(error_img, (10, 100), (310, 140), (0, 0, 255), -1)
                cv2.putText(error_img, f"{stream.name} Stream", (20, 80), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(error_img, "Not Available", (20, 125), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
                cv2.putText(error_img, stream.error_message or "Unknown error", (20, 170), 
                           cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
                
                _, buffer = cv2.imencode('.jpg', error_img)
                frame_bytes = buffer.tobytes()
                
                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                time.sleep(1)  # Update error message every second
        else:
            while True:
                frame = stream.get_frame()
                if frame is not None:
                    frame = resize_frame(frame, MAX_WINDOW_WIDTH)
                    _, buffer = cv2.imencode('.jpg', frame)
                    frame_bytes = buffer.tobytes()

                    yield (b'--frame\r\n'
                           b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
                else:
                    import time
                    time.sleep(0.05)
    except Exception as e:
        print(f"Error in generate_stream for {stream.name}: {e}")
    finally:
        if stream.available:
            stream.stop()


@app.route('/')
def index():
    stream1_status = "Available" if stream1.available else f"Not Available: {stream1.error_message}"
    stream2_status = "Available" if stream2.available else f"Not Available: {stream2.error_message}"
    
    return f'''
    <html>
        <head><title>Dual RTSP Streams</title></head>
        <body>
            <h1>Stream1 ({stream1_status})</h1>
            <img src="/video_feed/stream1" style="max-width:800px;">
            <h1>Stream2 ({stream2_status})</h1>
            <img src="/video_feed/stream2" style="max-width:800px;">
            <div style="margin-top:20px; padding:10px; background-color:#f0f0f0;">
                <h3>Stream Information:</h3>
                <p>Stream1 URL: {STREAM1_URL}</p>
                <p>Stream2 URL: {STREAM2_URL}</p>
                <p>Server running on port 8011</p>
            </div>
        </body>
    </html>
    '''


@app.route('/video_feed/stream1')
def video_feed_stream1():
    return Response(stream_with_context(generate_stream(stream1)),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/video_feed/stream2')
def video_feed_stream2():
    return Response(stream_with_context(generate_stream(stream2)),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8011, threaded=True)
