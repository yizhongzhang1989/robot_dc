from fastapi import FastAPI, Request
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse
import os
from ament_index_python.packages import get_package_share_directory

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
    # Control all motors/servos
    targets = ["motor1", "motor2", "servo17", "servo18"]
    results = []
    for target in targets:
        result = ros_client.control_observation(target, action)
        results.append(result)
    return {"result": results}

@app.post("/snapshot")
async def take_snapshot():
    """Take a snapshot using the cam_node service."""
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    
    result = ros_client.get_snapshot()
    return JSONResponse(content=result)

@app.post("/restart_camera")
async def restart_camera():
    """Restart the camera node using the restart service."""
    if ros_client is None:
        return JSONResponse(content={"error": "ROS client not initialized"}, status_code=503)
    
    result = ros_client.restart_camera_node()
    return JSONResponse(content=result)

app.mount("/web", StaticFiles(directory=STATIC_DIR), name="web")

if __name__ == "__main__":
    import uvicorn
    uvicorn.run(app, host="0.0.0.0", port=8000)