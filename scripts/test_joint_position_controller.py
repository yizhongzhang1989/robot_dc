#!/usr/bin/env python3
"""
Simple test script for JointPositionController
Sends predefined joint positions to the robot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
import time


class JointPositionCommandNode(Node):
    def __init__(self):
        super().__init__('joint_position_command_node')
        
        self.publisher = self.create_publisher(
            Float64MultiArray,
            '/joint_position_controller/joint_command',
            10
        )
        
        self.get_logger().info('Joint Position Command Node started')
    
    def send_position(self, positions, description=""):
        """Send joint position command"""
        if len(positions) != 6:
            self.get_logger().error('Position must have 6 values!')
            return False
        
        msg = Float64MultiArray()
        msg.data = positions
        
        self.publisher.publish(msg)
        self.get_logger().info(f'{description}: {positions}')
        return True


def main():
    rclpy.init()
    node = JointPositionCommandNode()
    
    # Define some test positions (in radians)
    positions = {
        'home': [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
        'position_1': [0.0, -0.5, 0.5, 0.0, 0.5, 0.0],
        'position_2': [0.5, -0.5, 0.5, -0.5, 0.5, 0.5],
        'position_3': [-0.5, -0.3, 0.3, 0.5, -0.5, -0.5],
    }
    
    try:
        # Move to home position
        node.get_logger().info('=== Moving to HOME position ===')
        node.send_position(positions['home'], 'Home')
        time.sleep(6)  # Wait for motion to complete (5s + 1s buffer)
        
        # Move to position 1
        node.get_logger().info('=== Moving to Position 1 ===')
        node.send_position(positions['position_1'], 'Position 1')
        time.sleep(6)
        
        # Move to position 2
        node.get_logger().info('=== Moving to Position 2 ===')
        node.send_position(positions['position_2'], 'Position 2')
        time.sleep(6)
        
        # Move to position 3
        node.get_logger().info('=== Moving to Position 3 ===')
        node.send_position(positions['position_3'], 'Position 3')
        time.sleep(6)
        
        # Return to home
        node.get_logger().info('=== Returning to HOME ===')
        node.send_position(positions['home'], 'Home')
        time.sleep(6)
        
        node.get_logger().info('=== Test sequence completed! ===')
        
    except KeyboardInterrupt:
        node.get_logger().info('Test interrupted by user')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
