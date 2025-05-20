# robot_web/ros_bridge.py
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class ROSBridge(Node):
    def __init__(self):
        rclpy.init()
        super().__init__('web_bridge')
        self.pub_motor1 = self.create_publisher(String, '/motor1/motor_cmd', 10)
        self.pub_motor2 = self.create_publisher(String, '/motor2/motor_cmd', 10)

    def spin(self):
        rclpy.spin(self)

    def publish_cmd(self, motor_id, cmd):
        pub = self.pub_motor1 if motor_id == 1 else self.pub_motor2
        msg = String()
        msg.data = cmd
        pub.publish(msg)

    def set_velocity(self, motor_id, velocity):
        self.publish_cmd(motor_id, f"set_vel {int(velocity)}")
        self.publish_cmd(motor_id, "move_vel")
        return {"result": "OK", "velocity": velocity}

    def jog(self, motor_id, direction):
        cmd = "jog_left" if direction == "left" else "jog_right"
        self.publish_cmd(motor_id, cmd)
        return {"result": "OK", "direction": direction}

    def stop_all(self):
        self.publish_cmd(1, "stop")
        self.publish_cmd(2, "stop")
        return {"result": "Stopped"}

    def get_status(self):
        # Add real sensor values or state if needed
        return {"motor_1": {"status": "ok"}, "motor_2": {"status": "ok"}}
