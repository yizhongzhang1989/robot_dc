#!/usr/bin/env python3
"""
ROS2 Node for Robotiq 2F-140 Gripper Control

Subscribed Topics:
    /gripper/command (robotiq_gripper_msgs/GripperCommand): Command to control the gripper

Published Topics:
    /gripper/status (robotiq_gripper_msgs/GripperStatus): Current gripper status

Services:
    /gripper/activate (std_srvs/Trigger): Activate the gripper
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from std_msgs.msg import Header

from robotiq_gripper_msgs.msg import GripperCommand, GripperStatus
from .robotiq_gripper import Robotiq2f140Gripper


class RobotiqGripperNode(Node):
    """ROS2 Node for controlling Robotiq 2F-140 Gripper."""

    def __init__(self):
        super().__init__('robotiq_gripper_node')
        
        # Declare parameters
        self.declare_parameter('device_id', 9)
        self.declare_parameter('rs485_host', '192.168.1.15')
        self.declare_parameter('rs485_port', 54321)
        self.declare_parameter('status_publish_rate', 10.0)  # Hz
        
        # Get parameters
        device_id = self.get_parameter('device_id').value
        rs485_host = self.get_parameter('rs485_host').value
        rs485_port = self.get_parameter('rs485_port').value
        status_rate = self.get_parameter('status_publish_rate').value
        
        self.get_logger().info('Initializing Robotiq Gripper Node')
        self.get_logger().info(f'  Device ID: {device_id}')
        self.get_logger().info(f'  RS485 Gateway: {rs485_host}:{rs485_port}')
        
        # Initialize gripper controller
        self.gripper = Robotiq2f140Gripper(
            device_id=device_id,
            host=rs485_host,
            port=rs485_port
        )
        
        # Create subscriber for gripper commands
        self.command_sub = self.create_subscription(
            GripperCommand,
            '/gripper/command',
            self.command_callback,
            10
        )
        
        # Create publisher for gripper status
        self.status_pub = self.create_publisher(
            GripperStatus,
            '/gripper/status',
            10
        )
        
        # Create service for gripper activation
        self.activate_srv = self.create_service(
            Trigger,
            '/gripper/activate',
            self.activate_callback
        )
        
        # Create timer for status publishing
        timer_period = 1.0 / status_rate if status_rate > 0 else 0.1
        self.status_timer = self.create_timer(timer_period, self.publish_status)
        
        self.get_logger().info('Robotiq Gripper Node initialized successfully')
        self.get_logger().info('Waiting for activation... Call /gripper/activate service')

    def command_callback(self, msg: GripperCommand):
        """Handle gripper command messages."""
        if msg.emergency_stop:
            self.get_logger().warn('Emergency stop received!')
            self.gripper.stop()
            return
        
        self.get_logger().info(
            f'Gripper command: position={msg.position}, '
            f'speed={msg.speed}, force={msg.force}'
        )
        
        success = self.gripper.move_to_position(
            position=msg.position,
            speed=msg.speed,
            force=msg.force,
            wait=False  # Non-blocking
        )
        
        if not success:
            self.get_logger().error('Failed to send gripper command')
    
    def publish_status(self):
        """Periodically publish gripper status."""
        status_dict = self.gripper.get_status()
        
        if status_dict is None:
            self.get_logger().warn('Failed to read gripper status', throttle_duration_sec=5.0)
            return
        
        # Create and populate status message
        msg = GripperStatus()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'gripper'
        
        msg.is_activated = status_dict['activated']
        msg.is_moving = status_dict['moving']
        msg.object_detected = status_dict['object_detected']
        msg.fault = status_dict['fault']
        msg.position = status_dict['position']
        msg.force = status_dict['force']
        msg.raw_registers = status_dict['raw_registers']
        
        self.status_pub.publish(msg)
    
    def activate_callback(self, request, response):
        """Handle gripper activation service request."""
        self.get_logger().info('Activating gripper...')
        
        success = self.gripper.activate(timeout=5.0)
        
        response.success = success
        if success:
            response.message = 'Gripper activated successfully'
            self.get_logger().info('Gripper activated successfully')
        else:
            response.message = 'Failed to activate gripper'
            self.get_logger().error('Failed to activate gripper')
        
        return response
    
    def destroy_node(self):
        """Clean up resources."""
        self.get_logger().info('Shutting down Robotiq Gripper Node...')
        self.gripper.disconnect()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    node = RobotiqGripperNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
