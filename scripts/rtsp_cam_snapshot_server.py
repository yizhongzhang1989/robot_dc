import cv2
import subprocess
import numpy as np
import json
from flask import Flask, Response, render_template_string, request, jsonify
import base64
import datetime
import pytz

app = Flask(__name__)

RTSP_URLS = {
    'cam100': "rtsp://admin:123456@192.168.1.100/stream0",
    'cam101': "rtsp://admin:123456@192.168.1.101/stream0"
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
    # 只解码一帧，节省资源
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
        # 获取中国时间
        tz = pytz.timezone('Asia/Shanghai')
        now = datetime.datetime.now(tz)
        time_str = now.strftime('%Y-%m-%d %H:%M:%S')
        return jsonify({'status': 'ok', 'img': img_b64, 'time': time_str, 'cam': cam})
    else:
        return jsonify({'status': 'fail', 'error': err, 'cam': cam})

if __name__ == '__main__':
    app.run(host='0.0.0.0', port=8012, threaded=True) 