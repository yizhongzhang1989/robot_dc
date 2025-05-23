from fastapi import FastAPI, Request
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
import os

from .web_ros_client import WebROSClient

app = FastAPI()
ros_client = None

STATIC_DIR = os.path.join(os.path.dirname(__file__), "web")

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
    return {"motors": ros_client.motor_list}

@app.post("/motor/{motor_id}/command")
async def send_motor_command(motor_id: str, request: Request):
    data = await request.json()
    command = data.get("command")
    value = data.get("value", None)
    return ros_client.send_command(motor_id, command, value)

app.mount("/web", StaticFiles(directory=STATIC_DIR), name="web")
