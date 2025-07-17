import cv2
import threading
import subprocess
import numpy as np
import json
from flask import Flask, render_template_string, request, jsonify
import base64

app = Flask(__name__)

HIGH_URL = "rtsp://admin:123456@192.168.1.100/stream0"
LOW_URL = "rtsp://admin:123456@192.168.1.101/stream0"


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
            if self.proc.stdout is None:
                continue
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


stream_high = RTSPStream("High", HIGH_URL)
stream_low = RTSPStream("Low", LOW_URL)


HTML = '''
<!DOCTYPE html>
<html lang="en">
<head>
    <meta charset="UTF-8">
    <title>RTSP Snapshot</title>
    <style>
        #snapshot-area { display: flex; gap: 2em; margin-top: 2em; }
        img { max-width: 400px; max-height: 300px; border: 1px solid #ccc; }
    </style>
</head>
<body>
    <h1>RTSP Snapshot Demo</h1>
    <button id="snapshot-btn">Get Snapshot</button>
    <div id="snapshot-area" style="display:none;">
        <div>
            <img id="img-high" src="">
            <div id="time-high"></div>
        </div>
        <div>
            <img id="img-low" src="">
            <div id="time-low"></div>
        </div>
    </div>
    <script>
    document.getElementById('snapshot-btn').onclick = async function() {
        const btn = document.getElementById('snapshot-btn');
        btn.disabled = true;
        try {
            const res = await fetch('/snapshot', {method: 'POST'});
            const data = await res.json();
            if (data.high && data.high.img) {
                document.getElementById('img-high').src = 'data:image/jpeg;base64,' + data.high.img;
                document.getElementById('time-high').innerText = data.high.timestamp || '';
            }
            if (data.low && data.low.img) {
                document.getElementById('img-low').src = 'data:image/jpeg;base64,' + data.low.img;
                document.getElementById('time-low').innerText = data.low.timestamp || '';
            }
            document.getElementById('snapshot-area').style.display = 'flex';
        } catch (err) {
            alert('Snapshot failed');
        } finally {
            btn.disabled = false;
        }
    };
    </script>
</body>
</html>
'''

@app.route('/')
def index():
    return render_template_string(HTML)

@app.route('/snapshot', methods=['POST'])
def snapshot():
    import datetime
    result = {}
    for key, stream in zip(['high', 'low'], [stream_high, stream_low]):
        frame = stream.get_frame()
        if frame is not None:
            _, buffer = cv2.imencode('.jpg', frame)
            img_b64 = base64.b64encode(buffer.tobytes()).decode('utf-8')
            ts = datetime.datetime.now().isoformat()
            result[key] = {'img': img_b64, 'timestamp': ts}
        else:
            result[key] = {'img': '', 'timestamp': ''}
    return jsonify(result)

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8020, threaded=True) 