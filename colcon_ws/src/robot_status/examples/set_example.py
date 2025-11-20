#!/usr/bin/env python3
"""
Example: Setting robot status

This example shows how to set status for different robots and shared status.
"""

import rclpy
from rclpy.node import Node
from robot_status.client_utils import RobotStatusClient
import time


def main():
    rclpy.init()
    node = Node('status_setter_example')
    
    try:
        # Create client
        client = RobotStatusClient(node)
        
        node.get_logger().info("Setting status for multiple robots...")
        
        # Set robot1 status
        client.set_status('robot1', 'pose', {'x': 1.5, 'y': 2.3, 'z': 0.0, 'theta': 90})
        client.set_status('robot1', 'battery', 85)
        client.set_status('robot1', 'gripper_state', 'open')
        client.set_status('robot1', 'mode', 'autonomous')
        
        # Set robot2 status
        client.set_status('robot2', 'pose', {'x': -1.0, 'y': 3.5, 'z': 0.0, 'theta': 180})
        client.set_status('robot2', 'battery', 92)
        client.set_status('robot2', 'gripper_state', 'closed')
        client.set_status('robot2', 'mode', 'manual')
        
        # Set shared status
        client.set_status('shared', 'mission_id', 'mission_001')
        client.set_status('shared', 'system_mode', 'production')
        client.set_status('shared', 'warehouse_temp', 22.5)
        client.set_status('shared', 'active_robots', ['robot1', 'robot2'])
        
        node.get_logger().info("Status set successfully!")
        node.get_logger().info("View at: http://localhost:8005")
        
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
