#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import threading
import json
from .robot_data import RobotData


class DucoRobotArmStateNode(Node):
    def __init__(self):
        super().__init__('duco_robot_arm_state_node')

        # Declare and read device ID parameter (to match duco_robot_arm interface)
        self.declare_parameter('device_id', 1)
        self.device_id = self.get_parameter('device_id').value
        self.get_logger().info(f"Device ID from param = {self.device_id}")

        # Declare and read IP address of the robot
        self.declare_parameter('ip', '192.168.1.10')
        self.ip = self.get_parameter('ip').value
        self.get_logger().info(f"IP from param = {self.ip}")

        # Declare and read port number for state monitoring (default 2001)
        self.declare_parameter('port', 2001)
        self.port = self.get_parameter('port').value
        self.get_logger().info(f"Port from param = {self.port}")

        # Data frame configuration
        self.declare_parameter('frame_size', 1468)
        self.frame_size = self.get_parameter('frame_size').value
        self.min_data_size = 608  # minimum bytes needed for current data structure

        # Publishers for robot state data
        self.joint_state_pub = self.create_publisher(String, f'/arm{self.device_id}/joint_state', 10)
        self.tcp_state_pub = self.create_publisher(String, f'/arm{self.device_id}/tcp_state', 10)
        self.robot_status_pub = self.create_publisher(String, f'/arm{self.device_id}/robot_status', 10)
        self.raw_data_pub = self.create_publisher(String, f'/arm{self.device_id}/raw_data', 10)

        self.get_logger().info(f"📡 Publishers created for device {self.device_id}")

        # Connection management
        self.socket = None
        self.is_connected = False
        self.is_running = False
        self.data_thread = None

        # Start the data monitoring thread
        self.start_monitoring()

    def start_monitoring(self):
        """Start the robot state monitoring thread."""
        self.is_running = True
        self.data_thread = threading.Thread(target=self.data_monitoring_loop, daemon=True)
        self.data_thread.start()
        self.get_logger().info("🔄 Robot state monitoring thread started")

    def stop_monitoring(self):
        """Stop the robot state monitoring thread."""
        self.is_running = False
        if self.socket:
            self.socket.close()
        if self.data_thread:
            self.data_thread.join()
        self.get_logger().info("⏹️ Robot state monitoring stopped")

    def connect_to_robot(self):
        """Establish TCP connection to the robot."""
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.connect((self.ip, self.port))
            self.is_connected = True
            self.get_logger().info(f"✅ Connected to robot at {self.ip}:{self.port}")
            return True
        except Exception as e:
            self.get_logger().error(f"❌ Failed to connect to robot: {e}")
            self.is_connected = False
            return False

    def data_monitoring_loop(self):
        """Main loop for monitoring robot data."""
        while self.is_running:
            if not self.is_connected:
                if not self.connect_to_robot():
                    self.get_logger().warn("⏳ Retrying connection in 5 seconds...")
                    import time
                    time.sleep(5.0)
                    continue

            try:
                # Read exactly frame_size bytes from the socket
                data = b""
                while len(data) < self.frame_size and self.is_running:
                    packet = self.socket.recv(self.frame_size - len(data))
                    if not packet:
                        self.get_logger().warn("📡 Connection closed by robot")
                        self.is_connected = False
                        break
                    data += packet

                if len(data) == self.frame_size:
                    self.process_robot_data(data)
                else:
                    self.get_logger().warn(f"⚠️ Incomplete frame received: {len(data)} bytes")

            except Exception as e:
                self.get_logger().error(f"❌ Error in data monitoring loop: {e}")
                self.is_connected = False
                if self.socket:
                    self.socket.close()

    def process_robot_data(self, data: bytes):
        """Process the received robot data and publish to ROS topics."""
        try:
            # Parse the robot data
            robot_data = RobotData.from_bytes(data)
            
            # Publish joint state data
            joint_state_msg = self.create_joint_state_message(robot_data)
            self.joint_state_pub.publish(joint_state_msg)
            
            # Publish TCP state data
            tcp_state_msg = self.create_tcp_state_message(robot_data)
            self.tcp_state_pub.publish(tcp_state_msg)
            
            # Publish robot status data
            robot_status_msg = self.create_robot_status_message(robot_data)
            self.robot_status_pub.publish(robot_status_msg)
            
            # Publish raw data (optional, for debugging)
            raw_data_msg = self.create_raw_data_message(robot_data)
            self.raw_data_pub.publish(raw_data_msg)
            
        except ValueError as e:
            self.get_logger().error(f"❌ Error parsing robot data: {e}")

    def create_joint_state_message(self, robot_data: RobotData) -> String:
        """Create a joint state message from robot data."""
        joint_state = {
            "actual": {
                "position": robot_data.jointActualPosition,
                "velocity": robot_data.jointActualVelocity,
                "acceleration": robot_data.jointActualAccelera,
                "torque": robot_data.jointActualTorque,
                "temperature": robot_data.jointActualTemperature,
                "current": robot_data.jointActualCurrent
            },
            "expected": {
                "position": robot_data.jointExpectPosition,
                "velocity": robot_data.jointExpectVelocity,
                "acceleration": robot_data.jointExpectAccelera,
                "torque": robot_data.jointExpectTorque
            }
        }
        
        msg = String()
        msg.data = json.dumps(joint_state)
        return msg

    def create_tcp_state_message(self, robot_data: RobotData) -> String:
        """Create a TCP state message from robot data."""
        tcp_state = {
            "actual": {
                "position": robot_data.TCPActualPosition,
                "velocity": robot_data.TCPActualVelocity,
                "acceleration": robot_data.TCPActualAccelera,
                "torque": robot_data.TCPActualTorque
            },
            "expected": {
                "position": robot_data.TCPExpectPosition,
                "velocity": robot_data.TCPExpectVelocity,
                "acceleration": robot_data.TCPExpectAccelera,
                "torque": robot_data.TCPExpectTorque
            }
        }
        
        msg = String()
        msg.data = json.dumps(tcp_state)
        return msg

    def create_robot_status_message(self, robot_data: RobotData) -> String:
        """Create a robot status message from robot data."""
        robot_status = {
            "driver_error_id": robot_data.driverErrorID,
            "driver_state": robot_data.driverState,
            "base_torque": {
                "actual": robot_data.baseActualTorque,
                "expected": robot_data.baseExpectTorque
            }
        }
        
        msg = String()
        msg.data = json.dumps(robot_status)
        return msg

    def create_raw_data_message(self, robot_data: RobotData) -> String:
        """Create a raw data message (for debugging/monitoring)."""
        # Create a simplified version for logging
        raw_data = {
            "tcp_pose": robot_data.get_tcp_pose(),
            "joint_positions": robot_data.jointActualPosition,
            "timestamp": self.get_clock().now().to_msg()
        }
        
        msg = String()
        msg.data = json.dumps(raw_data, default=str)
        return msg

    def destroy_node(self):
        """Clean up when the node is destroyed."""
        self.stop_monitoring()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DucoRobotArmStateNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("🛑 Node interrupted by user")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
