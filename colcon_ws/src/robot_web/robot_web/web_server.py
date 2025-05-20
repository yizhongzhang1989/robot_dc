# robot_web/web_server.py
import os
from fastapi import FastAPI, HTTPException
from fastapi.staticfiles import StaticFiles
from ament_index_python.packages import get_package_share_directory
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import threading
from pydantic import BaseModel

class MoveRequest(BaseModel):
    motor: int
    direction: str


app = FastAPI()

# Serve static files
web_dir = os.path.join(get_package_share_directory('robot_web'), 'web')
app.mount("/static", StaticFiles(directory=web_dir, html=True), name="static")

# Initialize ROS 2 in a separate thread
rclpy.init()

class WebROSClient(Node):
    def __init__(self):
        super().__init__('web_ros_client')
        self.pub_motor1 = self.create_publisher(String, '/motor1/motor_cmd', 10)
        self.pub_motor2 = self.create_publisher(String, '/motor2/motor_cmd', 10)

    def publish_cmd(self, motor_id, cmd):
        msg = String()
        msg.data = cmd
        if motor_id == 1:
            self.pub_motor1.publish(msg)
        elif motor_id == 2:
            self.pub_motor2.publish(msg)
        self.get_logger().info(f"Published '{cmd}' to motor{motor_id}")

ros_client = WebROSClient()

def ros_spin():
    rclpy.spin(ros_client)

spin_thread = threading.Thread(target=ros_spin, daemon=True)
spin_thread.start()

from fastapi.responses import FileResponse

@app.get("/")
async def root():
    return FileResponse(os.path.join(web_dir, "index.html"))


@app.post("/api/move")
async def move_motor(req: MoveRequest):
    motor_id = req.motor
    direction = req.direction

    print(f"[API] Received move request: motor={motor_id}, direction={direction}")


    if motor_id not in (1, 2):
        raise HTTPException(status_code=400, detail="Invalid motor_id, must be 1 or 2")
    if direction not in ('left', 'right'):
        raise HTTPException(status_code=400, detail="Invalid direction, must be 'left' or 'right'")
    
    cmd = "jog_left" if direction == "left" else "jog_right"
    ros_client.publish_cmd(motor_id, cmd)
    return {"result": "OK", "motor_id": motor_id, "direction": direction}