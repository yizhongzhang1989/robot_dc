# robot_web/web_server.py
from fastapi import FastAPI, Request
from fastapi.staticfiles import StaticFiles
from robot_web.ros_bridge import ROSBridge
import threading
import os
from ament_index_python.packages import get_package_share_directory

app = FastAPI()
ros = ROSBridge()

@app.on_event("startup")
def startup_event():
    threading.Thread(target=ros.spin, daemon=True).start()

# Get absolute path to the web directory relative to this script
web_dir = os.path.join(get_package_share_directory('robot_web'), 'web')
app.mount("/", StaticFiles(directory=web_dir, html=True), name="web")

@app.get("/status")
def get_status():
    return ros.get_status()

@app.post("/motor/{motor_id}/set_vel")
async def set_velocity(motor_id: int, request: Request):
    data = await request.json()
    return ros.set_velocity(motor_id, data.get("velocity", 0))

@app.post("/motor/{motor_id}/jog")
async def jog_motor(motor_id: int, request: Request):
    data = await request.json()
    return ros.jog(motor_id, data.get("direction", "left"))

@app.post("/stop")
async def stop():
    return ros.stop_all()
