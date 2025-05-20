import os
import threading

from fastapi import FastAPI, HTTPException
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from pydantic import BaseModel
from ament_index_python.packages import get_package_share_directory

import rclpy
from rclpy.node import Node
from std_msgs.msg import String

# -----------------------------------------------------------------------------
# Configuration
# -----------------------------------------------------------------------------
PACKAGE_NAME = 'robot_web'
WEB_DIR = os.path.join(get_package_share_directory(PACKAGE_NAME), 'web')
MOTOR_TOPICS = {
    1: '/motor1/motor_cmd',
    2: '/motor2/motor_cmd',
}

VALID_DIRECTIONS = {'left': 'jog_left', 'right': 'jog_right'}

# -----------------------------------------------------------------------------
# ROS Node
# -----------------------------------------------------------------------------
class WebROSClient(Node):
    def __init__(self):
        super().__init__('web_ros_client')
        self._motor_publishers = {
            motor_id: self.create_publisher(String, topic, 10)
            for motor_id, topic in MOTOR_TOPICS.items()
        }

    def publish_cmd(self, motor_id: int, cmd: str):
        if motor_id not in self._motor_publishers:
            self.get_logger().error(f"Invalid motor ID: {motor_id}")
            return
        msg = String(data=cmd)
        self._motor_publishers[motor_id].publish(msg)
        self.get_logger().info(f"Published '{cmd}' to motor{motor_id}")

def start_ros_spin(node: Node):
    rclpy.spin(node)

# -----------------------------------------------------------------------------
# FastAPI Initialization
# -----------------------------------------------------------------------------
app = FastAPI()
app.mount("/static", StaticFiles(directory=WEB_DIR, html=True), name="static")

@app.get("/")
async def root():
    return FileResponse(os.path.join(WEB_DIR, "index.html"))

# -----------------------------------------------------------------------------
# API Models
# -----------------------------------------------------------------------------
class MoveRequest(BaseModel):
    motor: int
    direction: str

# -----------------------------------------------------------------------------
# API Routes
# -----------------------------------------------------------------------------
@app.post("/api/move")
async def move_motor(req: MoveRequest):
    motor_id = req.motor
    direction = req.direction.lower()

    print(f"[API] Received move request: motor={motor_id}, direction={direction}")

    if motor_id not in MOTOR_TOPICS:
        raise HTTPException(status_code=400, detail="Invalid motor_id, must be 1 or 2")
    if direction not in VALID_DIRECTIONS:
        raise HTTPException(status_code=400, detail="Invalid direction, must be 'left' or 'right'")

    ros_client.publish_cmd(motor_id, VALID_DIRECTIONS[direction])
    return {"result": "OK", "motor_id": motor_id, "direction": direction}

# -----------------------------------------------------------------------------
# ROS Initialization (Threaded)
# -----------------------------------------------------------------------------
rclpy.init()
ros_client = WebROSClient()
ros_thread = threading.Thread(target=start_ros_spin, args=(ros_client,), daemon=True)
ros_thread.start()
