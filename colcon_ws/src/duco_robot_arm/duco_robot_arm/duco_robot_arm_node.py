import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import os
import sys

# Use relative imports for the package
from .DucoCobot import DucoCobot  
from .gen_py.robot.ttypes import Op  
from .lib.thrift import Thrift  


class DucoRobotArmNode(Node):
    def __init__(self):
        super().__init__('duco_robot_arm_node')

        # Declare and read device ID parameter
        self.declare_parameter('device_id', 1)
        self.device_id = self.get_parameter('device_id').value
        self.get_logger().info(f"Device ID from param = {self.device_id}")

        # Declare and read IP address of the robot
        self.declare_parameter('ip', '192.168.1.10')
        self.ip = self.get_parameter('ip').value
        self.get_logger().info(f"IP from param = {self.ip}")

        # Declare and read port number
        self.declare_parameter('port', 7003)
        self.port = self.get_parameter('port').value
        self.get_logger().info(f"Port from param = {self.port}")

        # Set up command subscriber (immediate)
        self.cmd_sub = self.create_subscription(String,  f'/arm{self.device_id}/cmd', self.command_callback, 10)
        self.get_logger().info(f"📡 Subscription to /arm{self.device_id}/cmd created")

        # Initialize the robot connection
        self.robot = None
        self.initialize_robot()


    def initialize_robot(self):
        # Create the DucoCobot instance and open connection
        self.robot = DucoCobot(self.ip, self.port)
        res = self.robot.open()
        self.get_logger().info(f"Open connection: {res}")

        # Set up an Op instance with no triggering events (default)  
        self.op = Op()  
        self.op.time_or_dist_1 = 0  
        self.op.trig_io_1 = 1  
        self.op.trig_value_1 = False  
        self.op.trig_time_1 = 0.0  
        self.op.trig_dist_1 = 0.0  
        self.op.trig_event_1 = ""  
        self.op.time_or_dist_2 = 0  
        self.op.trig_io_2 = 1  
        self.op.trig_value_2 = False  
        self.op.trig_time_2 = 0.0  
        self.op.trig_dist_2 = 0.0  
        self.op.trig_event_2 = ""  

            
    def command_callback(self, msg):
        parts = msg.data.strip().lower().split()
        if not parts:
            return

        cmd = parts[0]
        arg = int(parts[1]) if len(parts) > 1 and parts[1].lstrip('-').isdigit() else None

        if self.robot is None:
            self.get_logger().warn("⏳ Robot not initialized yet. Command ignored.")
            return

        try:
            match cmd:
                case "power_on":
                    res = self.robot.power_on(True)
                    self.get_logger().info(f"Power on: {res}")
                case "power_off":
                    res = self.robot.power_off(True)
                    self.get_logger().info(f"Power off: {res}")
                case "enable":
                    res = self.robot.enable(True)
                    self.get_logger().info(f"Enable: {res}")
                case "disable":
                    res = self.robot.disable(True)
                    self.get_logger().info(f"Disable: {res}")
                case _:
                    self.get_logger().warn(f"Unknown command: {cmd}")
        except Exception as e:
            self.get_logger().error(f"❌ Command '{cmd}' failed: {e}")


def main():
    rclpy.init()
    node = DucoRobotArmNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
