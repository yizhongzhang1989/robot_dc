import cv2
import threading
import subprocess
import numpy as np
import json
from flask import Flask, Response, stream_with_context

app = Flask(__name__)

MAX_WINDOW_WIDTH = 800


def get_stream_resolution(url):
    cmd = [
        "ffprobe",
        "-v", "error",
        "-select_streams", "v:0",
        "-show_entries", "stream=width,height",
        "-of", "json",
        url
    ]
    result = subprocess.run(cmd, capture_output=True, text=True)
    info = json.loads(result.stdout)
    w = info["streams"][0]["width"]
    h = info["streams"][0]["height"]
    return w, h


class RTSPStream:
    def __init__(self, name, url):
        self.name = name
        self.url = url
        self.frame = None
        self.running = True
        self.lock = threading.Lock()

        self.width, self.height = get_stream_resolution(self.url)
        print(f"[{self.name}] Stream resolution: {self.width}x{self.height}")

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
        frame_size = self.width * self.height * 3
        while self.running:
            raw = self.proc.stdout.read(frame_size)
            if len(raw) != frame_size:
                continue
            frame = np.frombuffer(raw, dtype=np.uint8).reshape((self.height, self.width, 3))
            with self.lock:
                self.frame = frame

    def get_frame(self):
        with self.lock:
            return self.frame.copy() if self.frame is not None else None

    def stop(self):
        self.running = False
        self.proc.kill()
        self.thread.join()


def resize_frame(frame, max_width):
    h, w = frame.shape[:2]
    if w > max_width:
        scale = max_width / w
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
    return frame


# Setup RTSP URLs
HIGH_URL = "rtsp://admin:123456@192.168.1.100/stream0"
LOW_URL = "rtsp://admin:123456@192.168.1.101/stream0"

# Start streams
stream_high = RTSPStream("High", HIGH_URL)
stream_low = RTSPStream("Low", LOW_URL)


def generate_stream(stream: RTSPStream):
    try:
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
    finally:
        stream.stop()


@app.route('/')
def index():
    return '''
    <html>
        <head><title>Dual RTSP Streams</title></head>
        <body>
            <h1>High Stream</h1>
            <img src="/video_feed/high">
            <h1>Low Stream</h1>
            <img src="/video_feed/low">
        </body>
    </html>
    '''


@app.route('/video_feed/high')
def video_feed_high():
    return Response(stream_with_context(generate_stream(stream_high)),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/video_feed/low')
def video_feed_low():
    return Response(stream_with_context(generate_stream(stream_low)),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8011, threaded=True)
