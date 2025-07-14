import time
import threading
import base64
import cv2
import numpy as np
from flask import Flask, Response, render_template_string, request, jsonify
import datetime
import pytz
import rclpy
from rclpy.node import Node

RTSP_URLS = {
    'cam100': 'rtsp://admin:123456@192.168.1.100/stream0',
    'cam101': 'rtsp://admin:123456@192.168.1.101/stream0'
}

HTML_PAGE = '''
<html>
<head><title>RTSP Snapshot</title></head>
<body>
    <h1>RTSP Snapshot</h1>
    <div style="display:flex;gap:40px;">
        <div>
            <button onclick="takePhoto('cam100')">拍照（100）</button>
            <div id="img-area-cam100">
                <img id="snapshot-cam100" src="" style="max-width:400px;max-height:300px;"/>
                <div id="snap-time-cam100" style="margin-top:10px;font-size:16px;color:#333;"></div>
            </div>
        </div>
        <div>
            <button onclick="takePhoto('cam101')">拍照（101）</button>
            <div id="img-area-cam101">
                <img id="snapshot-cam101" src="" style="max-width:400px;max-height:300px;"/>
                <div id="snap-time-cam101" style="margin-top:10px;font-size:16px;color:#333;"></div>
            </div>
        </div>
    </div>
    <script>
    function takePhoto(cam) {
        fetch('/snapshot', {
            method: 'POST',
            headers: {'Content-Type': 'application/json'},
            body: JSON.stringify({cam: cam})
        })
        .then(response => response.json())
        .then(data => {
            if(data.status === 'ok') {
                document.getElementById('snapshot-' + data.cam).src = 'data:image/jpeg;base64,' + data.img;
                document.getElementById('snap-time-' + data.cam).innerText = '拍照时间：' + data.time;
            } else {
                alert('拍照失败: ' + data.error);
            }
        });
    }
    </script>
</body>
</html>
'''

def get_rtsp_snapshot(rtsp_url):
    cap = cv2.VideoCapture(rtsp_url)
    if not cap.isOpened():
        return None, '无法打开RTSP流'
    ret, frame = cap.read()
    cap.release()
    if not ret:
        return None, '无法读取帧'
    _, buffer = cv2.imencode('.jpg', frame)
    img_b64 = base64.b64encode(buffer.tobytes()).decode('utf-8')
    return img_b64, None

# Flask服务
app = Flask(__name__)

@app.route('/')
def index():
    return render_template_string(HTML_PAGE)

@app.route('/snapshot', methods=['POST'])
def snapshot():
    data = request.get_json()
    cam = data.get('cam', 'cam100')
    rtsp_url = RTSP_URLS.get(cam)
    if not rtsp_url:
        return jsonify({'status': 'fail', 'error': '未知摄像头', 'cam': cam})
    img_b64, err = get_rtsp_snapshot(rtsp_url)
    if img_b64:
        tz = pytz.timezone('Asia/Shanghai')
        now = datetime.datetime.now(tz)
        time_str = now.strftime('%Y-%m-%d %H:%M:%S')
        return jsonify({'status': 'ok', 'img': img_b64, 'time': time_str, 'cam': cam})
    else:
        return jsonify({'status': 'fail', 'error': err, 'cam': cam})

# ROS2节点
class CamNode(Node):
    def __init__(self):
        super().__init__('cam_node')
        self.cam_ids = ['cam100', 'cam101']
        self.timer_period = 30.0  # seconds
        timers = []
        for cam_id in self.cam_ids:
            timer = self.create_timer(self.timer_period, lambda cam_id=cam_id: self.take_snapshot(cam_id))
            timers.append(timer)

    def take_snapshot(self, cam_id, retry=3, retry_interval=2):
        rtsp_url = RTSP_URLS[cam_id]
        for i in range(retry):
            img_b64, err = get_rtsp_snapshot(rtsp_url)
            if img_b64:
                self.get_logger().info(f"[{cam_id}] 拍照成功")
                return img_b64
            else:
                self.get_logger().warn(f"[{cam_id}] 拍照失败: {err}")
            time.sleep(retry_interval)
        self.get_logger().error(f"[{cam_id}] 多次尝试后仍失败，可能需要重启摄像头！")
        self.restart_camera(cam_id)
        return None

    def restart_camera(self, cam_id):
        self.get_logger().warn(f"[{cam_id}] 正在尝试重启摄像头...")
        # 这里可集成实际硬件重启命令
        time.sleep(5)
        self.get_logger().info(f"[{cam_id}] 重启完成，等待摄像头恢复...")
        time.sleep(10)

def flask_thread():
    app.run(host='0.0.0.0', port=8012, threaded=True)

def main():
    # 启动Flask服务线程
    t = threading.Thread(target=flask_thread, daemon=True)
    t.start()
    # 启动ROS2节点
    rclpy.init()
    node = CamNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main() 