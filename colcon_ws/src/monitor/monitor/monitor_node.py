#!/usr/bin/env python3
"""
Robot Monitor Node

This ROS2 node monitors UDP data from a robot and records it using rosbag2.
It implements the same functionality as dk_test.py but with ROS2 integration
and automatic data recording with timestamps.

The node:
1. Listens for UDP packets on port 5566 (robot data) and 5577 (logging)
2. Publishes received data to ROS2 topics with timestamps
3. Records data to rosbag files automatically
4. Provides terminal output for debugging
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import socket
import json
import threading
from datetime import datetime
import os
import subprocess
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class RobotMonitorNode(Node):
    def __init__(self):
        super().__init__('robot_monitor_node')
        
        # Configuration
        self.host = "0.0.0.0"
        self.port_data = 5566  # Must use same port as robot_arm_web_server (data source fixed)
        self.port_log = 5577   # Port for logging
        
        # Data storage configuration - can be set via environment variable or parameter
        self.data_base_dir = os.environ.get('ROBOT_DATA_DIR', os.path.expanduser('~/robot_data'))
        
        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Create publishers for different data types
        self.robot_data_publisher = self.create_publisher(
            String, 'robot_data', 10, callback_group=self.callback_group)
        self.log_data_publisher = self.create_publisher(
            String, 'log_data', 10, callback_group=self.callback_group)
        
        # Initialize rosbag recording
        self.setup_rosbag_recording()
        
        # Start rosbag recording
        self.start_rosbag_recording()
        
        # Start UDP receivers
        self.start_udp_receivers()
        
        self.get_logger().info('Robot Monitor Node started')
        self.get_logger().info(f'Listening on ports {self.port_data} (data) and {self.port_log} (log)')
        self.get_logger().info('Using SO_REUSEPORT to share port 5566 with robot_arm_web_server')
        self.get_logger().info('Both processes will receive UDP packets via kernel load balancing')
    
    def setup_rosbag_recording(self):
        """Setup rosbag recording directory and parameters"""
        # Organize by date and session
        now = datetime.now()
        date_str = now.strftime('%Y-%m-%d')
        # Use high precision timestamp: HHMMSS.microseconds
        session_time = now.strftime('%H%M%S.%f')
        
        # Create rosbag directory structure
        self.bag_dir = os.path.join(self.data_base_dir, date_str)
        os.makedirs(self.bag_dir, exist_ok=True)
        
        # Generate bag filename
        self.bag_name = f'robot_monitor_{session_time}'
        self.bag_path = os.path.join(self.bag_dir, self.bag_name)
        
        self.get_logger().info(f'Data will be saved to: {self.data_base_dir}')
        self.get_logger().info(f'Rosbag will be saved to: {self.bag_path}')
        
        # Create a simple session info file
        self.create_session_info()
        
        self.rosbag_process = None
    
    def create_session_info(self):
        """Create a simple session information file"""
        now = datetime.now()
        session_info = {
            'session_start': now.strftime('%Y-%m-%d %H:%M:%S.%f'),
            'bag_name': self.bag_name,
            'data_sources': {
                'robot_data_port': 5566,
                'log_data_port': 5577
            },
            'topics': ['/robot_data', '/log_data']
        }
        
        session_file = os.path.join(self.bag_dir, f'{self.bag_name}_info.json')
        with open(session_file, 'w') as f:
            json.dump(session_info, f, indent=2)
        
        self.get_logger().info(f'Session info saved to: {session_file}')

    def start_rosbag_recording(self):
        """Start rosbag recording in a separate process"""
        try:
            # Command to record robot_data and log_data topics
            cmd = [
                'ros2', 'bag', 'record', 
                '/robot_data', '/log_data',
                '-o', self.bag_path,
                '--compression-mode', 'file',
                '--compression-format', 'zstd'
            ]
            
            self.rosbag_process = subprocess.Popen(
                cmd,
                stdout=subprocess.PIPE,
                stderr=subprocess.PIPE
            )
            
            self.get_logger().info(f'Started rosbag recording with PID: {self.rosbag_process.pid}')
            
        except Exception as e:
            self.get_logger().error(f'Failed to start rosbag recording: {e}')
    
    def stop_rosbag_recording(self):
        """Stop rosbag recording"""
        if self.rosbag_process and self.rosbag_process.poll() is None:
            self.get_logger().info('Stopping rosbag recording...')
            self.rosbag_process.terminate()
            try:
                self.rosbag_process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.rosbag_process.kill()
            self.get_logger().info('Rosbag recording stopped')
    
    def start_udp_receivers(self):
        """Start UDP receiver threads"""
        # Start data receiver thread
        data_thread = threading.Thread(
            target=self.udp_data_receiver, 
            daemon=True, 
            name="data_receiver"
        )
        data_thread.start()
        
        # Start log receiver thread  
        log_thread = threading.Thread(
            target=self.udp_log_receiver, 
            daemon=True, 
            name="log_receiver"
        )
        log_thread.start()
    
    def udp_data_receiver(self):
        """UDP receiver for robot data on port 5566"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Use SO_REUSEPORT to share port with robot_arm_web_server
        # This allows multiple processes to bind to the same port
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            self.get_logger().info('SO_REUSEPORT enabled for port sharing')
        except AttributeError:
            self.get_logger().error('SO_REUSEPORT not available on this system')
            self.get_logger().error('Cannot share port 5566 with robot_arm_web_server')
            return
        
        try:
            sock.bind((self.host, self.port_data))
            self.get_logger().info(f'Data receiver bound to {self.host}:{self.port_data}')
            self.get_logger().info('Successfully sharing port 5566 with robot_arm_web_server')
            self.get_logger().info('Waiting for UDP data...')
        except Exception as e:
            self.get_logger().error(f'Failed to bind data socket: {e}')
            self.get_logger().error('Make sure robot_arm_web_server also uses SO_REUSEPORT')
            return
        
        while rclpy.ok():
            try:
                data, addr = sock.recvfrom(4096)
                message = data.decode("utf-8").strip()
                
                self.get_logger().info(f'Received UDP packet from {addr[0]}:{addr[1]}')
                
                # Create minimal timestamped message - only store raw_message with ROS2 timestamp
                timestamp = self.get_clock().now()
                timestamped_data = {
                    'raw_message': message
                }
                
                # Parse JSON for logging (don't store parsed data to avoid duplication)
                try:
                    parsed_data = json.loads(message)
                    
                    # Log parsed data to terminal with readable timestamp
                    readable_time = datetime.fromtimestamp(timestamp.nanoseconds / 1e9).strftime('%H:%M:%S.%f')[:-3]
                    self.get_logger().info(
                        f'[DATA] {readable_time} From {addr[0]}:{addr[1]} - '
                        f'RobotTcpPos: {parsed_data.get("RobotTcpPos", "N/A")}, '
                        f'RobotAxis: {parsed_data.get("RobotAxis", "N/A")}, '
                        f'FTSensorData: {parsed_data.get("FTSensorData", "N/A")}'
                        f'FTSensorData: {parsed_data.get("FTSensorData", "N/A")}'
                    )
                except json.JSONDecodeError:
                    self.get_logger().warning(f'Invalid JSON from {addr[0]}:{addr[1]}: {message[:100]}')
                
                # Publish to ROS topic
                msg = String()
                msg.data = json.dumps(timestamped_data)
                self.robot_data_publisher.publish(msg)
                
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f'Error in data receiver: {e}')
                break
        
        sock.close()
        self.get_logger().info('Data socket closed')
    
    def udp_log_receiver(self):
        """UDP receiver for log data on port 5577"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        try:
            sock.bind((self.host, self.port_log))
            self.get_logger().info(f'Log receiver bound to {self.host}:{self.port_log}')
        except Exception as e:
            self.get_logger().error(f'Failed to bind log socket: {e}')
            return
        
        while rclpy.ok():
            try:
                data, addr = sock.recvfrom(4096)
                message = data.decode("utf-8", errors="ignore").strip()
                
                # Create minimal log message - only store the log message content
                timestamp = self.get_clock().now()
                timestamped_log = {
                    'message': message
                }
                
                # Log to terminal with readable timestamp
                readable_time = datetime.fromtimestamp(timestamp.nanoseconds / 1e9).strftime('%H:%M:%S.%f')[:-3]
                self.get_logger().info(f'[LOG] {readable_time} From {addr[0]}:{addr[1]}: {message}')
                
                # Publish to ROS topic
                msg = String()
                msg.data = json.dumps(timestamped_log)
                self.log_data_publisher.publish(msg)
                
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f'Error in log receiver: {e}')
                break
        
        sock.close()
        self.get_logger().info('Log socket closed')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        monitor_node = RobotMonitorNode()
        
        # Use multi-threaded executor to handle multiple callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(monitor_node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            monitor_node.get_logger().info('Shutting down monitor node...')
        finally:
            # Stop rosbag recording before shutting down
            monitor_node.stop_rosbag_recording()
            executor.shutdown()
            monitor_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
