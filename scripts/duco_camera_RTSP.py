import cv2
import threading
import subprocess
import numpy as np
import json
import os
from flask import Flask, Response, stream_with_context

app = Flask(__name__)

MAX_WINDOW_WIDTH = 800


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
        import time
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


def resize_frame(frame, max_width):
    h, w = frame.shape[:2]
    if w > max_width:
        scale = max_width / w
        frame = cv2.resize(frame, (int(w * scale), int(h * scale)))
    return frame


# Setup RTSP URL - 支持多种分辨率
CAMERA_STREAMS = {
    "1080p": "rtsp://admin:123456@192.168.1.102/stream0",  # 主码流 1920x1080
    "360p": "rtsp://admin:123456@192.168.1.102/stream1"    # 子码流 640x360
}

# 当前分辨率
current_resolution = "1080p"

# 最新截图路径
latest_snapshot = None

# Start stream
camera_stream = RTSPStream("Camera", CAMERA_STREAMS[current_resolution])


def generate_stream():
    """生成视频流，每次都检查当前流状态"""
    while True:
        if camera_stream.is_running():
            frame = camera_stream.get_frame()
            if frame is not None:
                frame = resize_frame(frame, MAX_WINDOW_WIDTH)
                _, buffer = cv2.imencode('.jpg', frame)
                frame_bytes = buffer.tobytes()

                yield (b'--frame\r\n'
                       b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            else:
                import time
                time.sleep(0.05)
        else:
            # 如果摄像头停止，返回一个黑色图像
            black_frame = np.zeros((480, 640, 3), dtype=np.uint8)
            cv2.putText(black_frame, 'Camera Stopped', (200, 240), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            _, buffer = cv2.imencode('.jpg', black_frame)
            frame_bytes = buffer.tobytes()
            
            yield (b'--frame\r\n'
                   b'Content-Type: image/jpeg\r\n\r\n' + frame_bytes + b'\r\n')
            import time
            time.sleep(0.5)


@app.route('/')
def index():
    return '''
    <html>
        <head>
            <title>RTSP Camera Stream</title>
            <style>
                body {
                    font-family: Arial, sans-serif;
                    max-width: 1000px;
                    margin: 0 auto;
                    padding: 20px;
                }
                .controls {
                    margin-bottom: 20px;
                }
                button {
                    padding: 10px 20px;
                    margin: 5px;
                    font-size: 16px;
                    cursor: pointer;
                    border: none;
                    border-radius: 5px;
                }
                .start-btn { background-color: #4CAF50; color: white; }
                .stop-btn { background-color: #f44336; color: white; }
                select {
                    padding: 8px 12px;
                    margin: 5px;
                    font-size: 14px;
                    border: 1px solid #ddd;
                    border-radius: 4px;
                }
                label {
                    font-weight: bold;
                    margin-right: 10px;
                }
                .status { 
                    padding: 10px; 
                    margin: 10px 0;
                    border-radius: 5px;
                    font-weight: bold;
                }
                .status.running { background-color: #d4edda; color: #155724; }
                .status.stopped { background-color: #f8d7da; color: #721c24; }
                #camera-container {
                    text-align: center;
                    border: 2px solid #ddd;
                    padding: 10px;
                    border-radius: 5px;
                }
            </style>
        </head>
        <body>
            <h1>Robot Arm Camera</h1>
            
            <div class="controls">
                <button class="start-btn" onclick="startCamera()">Start Camera</button>
                <button class="stop-btn" onclick="stopCamera()">Stop Camera</button>
                <button onclick="takeSnapshot()">Snapshot</button>
            </div>
            
            <div class="controls">
                <label for="resolution">Resolution: </label>
                <select id="resolution" onchange="changeResolution()">
                    <option value="1080p">1920x1080</option>
                    <option value="360p">640x360</option>
                </select>
            </div>

            <div id="status" class="status">Checking status...</div>

            <div id="camera-container">
                <img id="camera-feed" src="/video_feed" style="max-width: 100%; height: auto;">
            </div>

            <script>
                function startCamera() {
                    fetch('/start', {method: 'POST'})
                        .then(response => response.json())
                        .then(data => {
                            updateStatus(data.message, data.status);
                            if (data.status === 'success') {
                                // 等待一下再刷新图像
                                setTimeout(() => {
                                    refreshImage();
                                }, 1000);
                            }
                        });
                }
                
                function stopCamera() {
                    fetch('/stop', {method: 'POST'})
                        .then(response => response.json())
                        .then(data => {
                            updateStatus(data.message, data.status);
                        });
                }
                
                function changeResolution() {
                    const resolution = document.getElementById('resolution').value;
                    updateStatus('Changing resolution...', 'running');
                    
                    fetch('/change_resolution', {
                        method: 'POST',
                        headers: {
                            'Content-Type': 'application/json',
                        },
                        body: JSON.stringify({resolution: resolution})
                    })
                    .then(response => response.json())
                    .then(data => {
                        updateStatus(data.message, data.status);
                        if (data.status === 'success') {
                            setTimeout(() => {
                                refreshImage();
                            }, 2000);
                        }
                    });
                }
                
                function takeSnapshot() {
                    updateStatus('Taking snapshot...', 'running');

                    fetch('/snapshot', {method: 'POST'})
                        .then(response => response.json())
                        .then(data => {
                            updateStatus(data.message, data.status);
                            if (data.status === 'success') {
                                // 创建下载链接
                                const a = document.createElement('a');
                                a.href = '/download_snapshot';
                                a.download = data.filename;
                                a.click();
                            }
                        });
                }
                
                function refreshImage() {
                    const img = document.getElementById('camera-feed');
                    img.src = '/video_feed?' + new Date().getTime();
                }
                
                function getStatus() {
                    fetch('/status')
                        .then(response => response.json())
                        .then(data => {
                            updateStatus(data.message, data.is_running ? 'running' : 'stopped');
                        });
                }
                
                function updateStatus(message, status) {
                    const statusDiv = document.getElementById('status');
                    statusDiv.textContent = message;
                    statusDiv.className = 'status ' + (status === 'success' || status === 'running' ? 'running' : 'stopped');
                }
                
                // 页面加载时检查状态
                window.onload = function() {
                    getStatus();
                };
                
                // 定期检查状态
                setInterval(getStatus, 5000);
            </script>
        </body>
    </html>
    '''


@app.route('/video_feed')
def video_feed():
    return Response(stream_with_context(generate_stream()),
                    mimetype='multipart/x-mixed-replace; boundary=frame')


@app.route('/start', methods=['POST'])
def start_camera():
    """开启摄像头"""
    try:
        camera_stream.start()
        return {'status': 'success', 'message': 'Camera started successfully'}
    except Exception as e:
        return {'status': 'error', 'message': f'Failed to start camera: {str(e)}'}


@app.route('/stop', methods=['POST']) 
def stop_camera():
    """停止摄像头"""
    try:
        camera_stream.stop()
        return {'status': 'success', 'message': 'Camera stopped successfully'}
    except Exception as e:
        return {'status': 'error', 'message': f'Failed to stop camera: {str(e)}'}


@app.route('/status')
def get_camera_status():
    """获取摄像头状态"""
    is_running = camera_stream.is_running()
    message = 'Camera is running' if is_running else 'Camera is stopped'
    return {'is_running': is_running, 'message': message}


@app.route('/change_resolution', methods=['POST'])
def change_resolution():
    """切换摄像头分辨率"""
    try:
        from flask import request
        data = request.get_json()
        resolution = data.get('resolution', '1080p')
        
        global current_resolution
        if resolution in CAMERA_STREAMS:
            current_resolution = resolution
            new_url = CAMERA_STREAMS[resolution]
            camera_stream.change_url(new_url)
            return {'status': 'success', 'message': f'Exchanged to {resolution} Resolution'}
        else:
            return {'status': 'error', 'message': 'Unsupported resolution'}
    except Exception as e:
        return {'status': 'error', 'message': f'Failed to change resolution: {str(e)}'}


@app.route('/snapshot', methods=['POST'])
def take_snapshot():
    """截图保存"""
    try:
        frame = camera_stream.get_frame()
        if frame is not None:
            import os
            from datetime import datetime
            
            # 创建截图目录
            snapshot_dir = "snapshots"
            if not os.path.exists(snapshot_dir):
                os.makedirs(snapshot_dir)
            
            # 生成文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"snapshot_{timestamp}_{current_resolution}.jpg"
            filepath = os.path.join(snapshot_dir, filename)
            
            # 保存图片
            cv2.imwrite(filepath, frame)
            
            # 保存最新截图路径
            global latest_snapshot
            latest_snapshot = filepath

            return {'status': 'success', 'message': f'Snapshot saved: {filename}', 'filename': filename}
        else:
            return {'status': 'error', 'message': 'Can not get current frame'}
    except Exception as e:
        return {'status': 'error', 'message': f'Failed to take snapshot: {str(e)}'}


@app.route('/download_snapshot')
def download_snapshot():
    """下载最新截图"""
    try:
        from flask import send_file
        if 'latest_snapshot' in globals() and os.path.exists(latest_snapshot):
            return send_file(latest_snapshot, as_attachment=True)
        else:
            return {'error': '没有可用的截图'}, 404
    except Exception as e:
        return {'error': str(e)}, 500


if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8011, threaded=True)
