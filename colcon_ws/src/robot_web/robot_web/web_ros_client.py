import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from threading import Thread

class WebROSClient:
    def __init__(self, motor_list):
        self.motor_list = motor_list
        self.node = None
        self.publishers = {}
        self._start_ros_node()

    def _start_ros_node(self):
        def ros_spin():
            rclpy.init()
            self.node = Node("web_ros_client")
            for motor in self.motor_list:
                topic_name = f"/{motor}/cmd"
                self.publishers[motor] = self.node.create_publisher(String, topic_name, 10)
            rclpy.spin(self.node)

        Thread(target=ros_spin, daemon=True).start()

    def send_command(self, motor_id, command, value=None):
        if motor_id not in self.publishers:
            return {"error": f"Unknown motor: {motor_id}"}
        cmd_str = f"{command} {value}" if value is not None else command
        msg = String()
        msg.data = cmd_str
        self.publishers[motor_id].publish(msg)
        return {"motor": motor_id, "command": cmd_str}
