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
        self.robot_state_pub = self.create_publisher(String, f'/arm{self.device_id}/robot_state', 10)
        
        self.get_logger().info(f"📡 Publisher created for device {self.device_id}")

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
            
            # Create comprehensive robot state message
            robot_state_msg = self.create_robot_state_message(robot_data)
            self.robot_state_pub.publish(robot_state_msg)
            
        except ValueError as e:
            self.get_logger().error(f"❌ Error parsing robot data: {e}")

    def create_robot_state_message(self, robot_data: RobotData) -> String:
        """Create a comprehensive robot state message with all data."""
        robot_state = {
            # Joint data - keeping original field names for extensibility
            "jointActualPosition": robot_data.jointActualPosition,
            "jointActualVelocity": robot_data.jointActualVelocity,
            "jointActualAccelera": robot_data.jointActualAccelera,
            "jointActualTorque": robot_data.jointActualTorque,
            "jointExpectPosition": robot_data.jointExpectPosition,
            "jointExpectVelocity": robot_data.jointExpectVelocity,
            "jointExpectAccelera": robot_data.jointExpectAccelera,
            "jointExpectTorque": robot_data.jointExpectTorque,
            
            # Additional joint data
            "jointActualTemperature": robot_data.jointActualTemperature,
            "jointActualCurrent": robot_data.jointActualCurrent,
            "driverErrorID": robot_data.driverErrorID,
            "driverState": robot_data.driverState,
            
            # TCP data
            "TCPActualPosition": robot_data.TCPActualPosition,
            "TCPActualVelocity": robot_data.TCPActualVelocity,
            "TCPActualAccelera": robot_data.TCPActualAccelera,
            "TCPActualTorque": robot_data.TCPActualTorque,
            "TCPExpectPosition": robot_data.TCPExpectPosition,
            "TCPExpectVelocity": robot_data.TCPExpectVelocity,
            "TCPExpectAccelera": robot_data.TCPExpectAccelera,
            "TCPExpectTorque": robot_data.TCPExpectTorque,
            
            # Base torque data
            "baseActualTorque": robot_data.baseActualTorque,
            "baseExpectTorque": robot_data.baseExpectTorque,
            
            # Extended coordinate system data
            "activeToolCoordSystem": robot_data.activeToolCoordSystem,
            "activeWorkpieceCoordSystem": robot_data.activeWorkpieceCoordSystem,
            
            # Speed and control data
            "blendedSpeed": robot_data.blendedSpeed,
            "globalSpeed": robot_data.globalSpeed,
            "jogSpeed": robot_data.jogSpeed,
            
            # Digital I/O data
            "functionalDigitalIOInput": robot_data.functionalDigitalIOInput,
            "functionalDigitalIOOutput": robot_data.functionalDigitalIOOutput,
            "digitalIOInput": robot_data.digitalIOInput,
            "digitalIOOutput": robot_data.digitalIOOutput,
            
            # Analog I/O data
            "analogInput": robot_data.analogInput,
            "analogOutput": robot_data.analogOutput,
            
            # Register data
            "floatRegisterInput": robot_data.floatRegisterInput,
            "floatRegisterOutput": robot_data.floatRegisterOutput,
            "functionalBoolRegisterInput": robot_data.functionalBoolRegisterInput,
            "functionalBoolRegisterOutput": robot_data.functionalBoolRegisterOutput,
            "boolRegisterInput": robot_data.boolRegisterInput,
            "boolRegisterOutput": robot_data.boolRegisterOutput,
            "wordRegisterInput": robot_data.wordRegisterInput,
            "wordRegisterOutput": robot_data.wordRegisterOutput,
            
            # Tool IO
            "toolIOInput": robot_data.toolIOInput,
            "toolIOOutput": robot_data.toolIOOutput,
            "toolAnalogInput": robot_data.toolAnalogInput,
            "toolAnalogOutput": robot_data.toolAnalogOutput,
            "toolButtonStatus": robot_data.toolButtonStatus,
            
            # Robot status (using correct field names)
            "simulationMode": robot_data.simulationMode,
            "robotOperationMode": robot_data.robotOperationMode,
            "robotStatus": robot_data.robotStatus,
            "robotProgramRunStatus": robot_data.robotProgramRunStatus,
            "safetyMonitorStatus": robot_data.safetyMonitorStatus,
            "collisionDetectionTrigger": robot_data.collisionDetectionTrigger,
            "collisionAxis": robot_data.collisionAxis,
            "robotErrorCode": robot_data.robotErrorCode,
            
            # Timestamp for reference
            "timestamp": self.get_clock().now().to_msg()
        }
        
        msg = String()
        msg.data = json.dumps(robot_state, default=str)
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
