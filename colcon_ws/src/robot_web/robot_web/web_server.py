from fastapi import FastAPI, Request
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse
import os
from ament_index_python.packages import get_package_share_directory
import cv2
import base64
import datetime
import pytz

from .web_ros_client import WebROSClient

app = FastAPI()
ros_client = None

web_path = os.path.join(get_package_share_directory('robot_web'), 'web')
STATIC_DIR = os.path.abspath(web_path)
print("Serving web from:", STATIC_DIR, flush=True)


@app.on_event("startup")
def initialize():
    global ros_client
    motor_names = os.environ.get("MOTOR_NAMES", "")
    motor_list = motor_names.split(",") if motor_names else []
    ros_client = WebROSClient(motor_list)

@app.get("/")
def serve_index():
    return FileResponse(os.path.join(STATIC_DIR, "index.html"))

@app.get("/motors")
def get_motors():
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    return {"motors": ros_client.motor_list}

@app.post("/api/{motor_id}/cmd")
async def send_motor_command(motor_id: str, request: Request):
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    data = await request.json()
    command = data.get("command")
    value = data.get("value", None)
    return ros_client.send_command(motor_id, command, value)

@app.get("/api/{motor_id}/status")
def get_motor_status(motor_id: str):
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    status = ros_client.get_motor_status(motor_id)
    if status is None:
        return JSONResponse(content={"error": "No status yet"}, status_code=404)
    return status

@app.get("/api/all_status")
def get_all_status():
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    return ros_client.get_all_status()

@app.post("/api/observation/{target}/{action}")
def control_observation(target: str, action: str):
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    if action not in ("start", "stop"):
        return JSONResponse(content={"error": "Invalid action"}, status_code=400)
    result = ros_client.control_observation(target, action)
    if "error" in result:
        return JSONResponse(content=result, status_code=400)
    return result

@app.post("/monitor_control")
async def monitor_control(request: Request):
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    data = await request.json()
    action = data.get("action")
    if action not in ("start", "stop"):
        return JSONResponse(content={"error": "Invalid action"}, status_code=400)
    # 控制所有motor/servo
    targets = ["motor1", "motor2", "servo17", "servo18"]
    results = []
    for target in targets:
        result = ros_client.control_observation(target, action)
        results.append(result)
    return {"result": results}

# 摄像头抓拍接口
RTSP_URLS = {
    'cam100': 'rtsp://admin:123456@192.168.1.100/stream0',
    'cam101': 'rtsp://admin:123456@192.168.1.101/stream0'
}

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

@app.post("/snapshot")
async def snapshot():
    results = {}
    tz = pytz.timezone('Asia/Shanghai')
    now = datetime.datetime.now(tz)
    time_str = now.strftime('%Y-%m-%d %H:%M:%S')
    for cam in ["cam100", "cam101"]:
        img_b64, err = get_rtsp_snapshot(RTSP_URLS[cam])
        if img_b64:
            results[cam] = {"img": img_b64, "time": time_str}
        else:
            results[cam] = {"error": err, "time": time_str}
    return results

app.mount("/web", StaticFiles(directory=STATIC_DIR), name="web")


