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
        command_str = msg.data.strip()
        if not command_str:
            return

        if self.robot is None:
            self.get_logger().warn("⏳ Robot not initialized yet. Command ignored.")
            return

        try:
            # Parse command and arguments
            parts = command_str.split()
            cmd = parts[0].lower()
            
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
                    
                case "servoj":
                    # Parse servoj command: servoj [j1,j2,j3,j4,j5,j6] v a block kp kd
                    if len(parts) < 6:
                        self.get_logger().error("ServoJ command requires at least 5 parameters")
                        return
                        
                    # Extract joint angles from list format [j1,j2,j3,j4,j5,j6]
                    joints_str = parts[1]
                    if joints_str.startswith('[') and joints_str.endswith(']'):
                        joints_str = joints_str[1:-1]  # Remove brackets
                        joint_values = [float(x.strip()) for x in joints_str.split(',')]
                        
                        if len(joint_values) != 6:
                            self.get_logger().error("ServoJ requires exactly 6 joint values")
                            return
                            
                        v = float(parts[2])  # velocity
                        a = float(parts[3])  # acceleration
                        block_str = parts[4].lower()
                        block = block_str == 'true'
                        kp = int(parts[5]) if len(parts) > 5 else 200
                        kd = int(parts[6]) if len(parts) > 6 else 25
                        
                        res = self.robot.servoj(joint_values, v, a, block, kp, kd)
                        self.get_logger().info(f"ServoJ command executed: {res}")
                    else:
                        self.get_logger().error("Invalid joint list format for ServoJ")
                        
                case "servo_tcp":
                    # Parse servo_tcp command: servo_tcp [x,y,z,rx,ry,rz] v a tool block kp kd
                    if len(parts) < 7:
                        self.get_logger().error("ServoTCP command requires at least 6 parameters")
                        return
                        
                    # Extract pose offset from list format [x,y,z,rx,ry,rz]
                    pose_str = parts[1]
                    if pose_str.startswith('[') and pose_str.endswith(']'):
                        pose_str = pose_str[1:-1]  # Remove brackets
                        pose_values = [float(x.strip()) for x in pose_str.split(',')]
                        
                        if len(pose_values) != 6:
                            self.get_logger().error("ServoTCP requires exactly 6 pose values")
                            return
                            
                        v = float(parts[2])  # velocity
                        a = float(parts[3])  # acceleration
                        tool = parts[4].strip('"\'')  # tool number as string, remove quotes if present
                        block_str = parts[5].lower()
                        block = block_str == 'true'
                        kp = int(parts[6]) if len(parts) > 6 else 200
                        kd = int(parts[7]) if len(parts) > 7 else 25
                        
                        res = self.robot.servo_tcp(pose_values, v, a, tool, block, kp, kd)
                        self.get_logger().info(f"ServoTCP command executed: {res}")
                    else:
                        self.get_logger().error("Invalid pose list format for ServoTCP")
                        
                case "movej2":
                    # Parse movej2 command: movej2 [j1,j2,j3,j4,j5,j6] v a r block
                    if len(parts) < 6:
                        self.get_logger().error("Movej2 command requires at least 5 parameters")
                        return
                        
                    # Extract joint angles from list format [j1,j2,j3,j4,j5,j6]
                    joints_str = parts[1]
                    if joints_str.startswith('[') and joints_str.endswith(']'):
                        joints_str = joints_str[1:-1]  # Remove brackets
                        joint_values = [float(x.strip()) for x in joints_str.split(',')]
                        
                        if len(joint_values) != 6:
                            self.get_logger().error("Movej2 requires exactly 6 joint values")
                            return
                            
                        v = float(parts[2])  # velocity
                        a = float(parts[3])  # acceleration
                        r = float(parts[4])  # blend radius
                        block_str = parts[5].lower()
                        block = block_str == 'true'
                        
                        res = self.robot.movej2(joint_values, v, a, r, block, self.op)
                        self.get_logger().info(f"Movej2 command executed: {res}")
                    else:
                        self.get_logger().error("Invalid joint list format for Movej2")
                        
                case "tcp_move":
                    # Parse tcp_move command: tcp_move [x,y,z,rx,ry,rz] v a r tool block
                    if len(parts) < 7:
                        self.get_logger().error("TCP Move command requires at least 6 parameters")
                        return
                        
                    # Extract pose offset from list format [x,y,z,rx,ry,rz]
                    pose_str = parts[1]
                    if pose_str.startswith('[') and pose_str.endswith(']'):
                        pose_str = pose_str[1:-1]  # Remove brackets
                        pose_values = [float(x.strip()) for x in pose_str.split(',')]
                        
                        if len(pose_values) != 6:
                            self.get_logger().error("TCP Move requires exactly 6 pose values")
                            return
                            
                        v = float(parts[2])  # velocity
                        a = float(parts[3])  # acceleration
                        r = float(parts[4])  # blend radius
                        tool = parts[5].strip('"\'')  # tool number as string, remove quotes if present
                        block_str = parts[6].lower()
                        block = block_str == 'true'
                        
                        res = self.robot.tcp_move(pose_values, v, a, r, tool, block, self.op)
                        self.get_logger().info(f"TCP Move command executed: {res}")
                    else:
                        self.get_logger().error("Invalid pose list format for TCP Move")
                        
                case "run_program":
                    # Parse run_program command: run_program program_name [block]
                    if len(parts) < 2:
                        self.get_logger().error("run_program command requires program name")
                        return
                        
                    program_name = parts[1]
                    block = True  # Default to blocking
                    if len(parts) > 2:
                        block_str = parts[2].lower()
                        block = block_str == 'true'
                    
                    res = self.robot.run_program(program_name, block)
                    self.get_logger().info(f"run_program '{program_name}' executed: {res}")
                        
                case _:
                    self.get_logger().warn(f"Unknown command: {cmd}")
                    
        except Exception as e:
            self.get_logger().error(f"❌ Command '{command_str}' failed: {e}")
            import traceback
            self.get_logger().error(f"Traceback: {traceback.format_exc()}")


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
