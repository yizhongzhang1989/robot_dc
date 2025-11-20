#!/usr/bin/env python3
"""
Example: Getting robot status

This example shows how to query status from robot_status service.
"""

import rclpy
from rclpy.node import Node
from robot_status.client_utils import RobotStatusClient
import json


def main():
    rclpy.init()
    node = Node('status_getter_example')
    
    try:
        # Create client
        client = RobotStatusClient(node)
        
        node.get_logger().info("Querying robot status...")
        
        # Get specific status
        robot1_pose = client.get_status('robot1', 'pose')
        node.get_logger().info(f"Robot1 pose: {robot1_pose}")
        
        robot1_battery = client.get_status('robot1', 'battery')
        node.get_logger().info(f"Robot1 battery: {robot1_battery}%")
        
        # Get all namespaces
        namespaces = client.get_namespaces()
        node.get_logger().info(f"Available namespaces: {namespaces}")
        
        # List all status
        all_status = client.list_status()
        node.get_logger().info("All status:")
        print(json.dumps(all_status, indent=2))
        
        # List specific namespace
        robot2_status = client.list_status('robot2')
        node.get_logger().info("Robot2 status:")
        print(json.dumps(robot2_status, indent=2))
        
    except Exception as e:
        node.get_logger().error(f"Error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
