#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Test Robot Status Package

Test robot_status package basic functionality using client class.
"""

import rclpy
from rclpy.node import Node
import json
import time

from robot_status.client_utils import RobotStatusClient


class TestRobotStatusNode(Node):
    """Test robot_status functionality using client class"""
    
    def __init__(self):
        super().__init__('test_robot_status_node')
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("Testing robot_status package with RobotStatusClient")
        self.get_logger().info("=" * 60)
        
        # Wait for robot_status service to be available
        time.sleep(1)
        
        # Test with client class
        self.test_with_client()
        
        self.get_logger().info("=" * 60)
        self.get_logger().info("All tests completed!")
        self.get_logger().info("View web dashboard at: http://localhost:8005")
        self.get_logger().info("=" * 60)
    
    def test_with_client(self):
        """Test using RobotStatusClient class"""
        self.get_logger().info("\nTesting RobotStatusClient")
        self.get_logger().info("-" * 60)
        
        try:
            # Create client
            self.get_logger().info("Creating RobotStatusClient...")
            client = RobotStatusClient(self, timeout_sec=5.0)
            
            # Save data for multiple robots
            self.get_logger().info("\nSaving data to multiple namespaces:")
            
            # robot1 data
            client.set_status('robot1', 'name', 'UR15_Beijing')
            client.set_status('robot1', 'status', 'operational')
            client.set_status('robot1', 'battery', 95)
            self.get_logger().info("  - Saved robot1 data")
            
            # robot2 data
            client.set_status('robot2', 'name', 'UR10_Shanghai')
            client.set_status('robot2', 'status', 'charging')
            client.set_status('robot2', 'battery', 30)
            self.get_logger().info("  - Saved robot2 data")
            
            # shared data
            client.set_status('shared', 'mission_id', 'mission_2025_11_20')
            client.set_status('shared', 'system_mode', 'production')
            client.set_status('shared', 'warehouse_temp', 22.5)
            self.get_logger().info("  - Saved shared data")
            
            # List all status
            time.sleep(0.5)
            self.get_logger().info("\nListing all namespaces:")
            all_status = client.list_status()
            if all_status:
                for namespace, data in all_status.items():
                    self.get_logger().info(f"  * {namespace}: {len(data)} keys")
                    for key, value in list(data.items())[:3]:  # Show first 3
                        value_str = value if len(str(value)) < 50 else str(value)[:47] + "..."
                        self.get_logger().info(f"    - {key}: {value_str}")
                    if len(data) > 3:
                        self.get_logger().info(f"    ... and {len(data)-3} more keys")
            
            # Get specific namespace data
            self.get_logger().info("\nGetting robot1 data:")
            robot1_name = client.get_status('robot1', 'name')
            robot1_status = client.get_status('robot1', 'status')
            if robot1_name:
                self.get_logger().info(f"  * Name: {robot1_name}")
            if robot1_status:
                self.get_logger().info(f"  * Status: {robot1_status}")
            
            # Get all namespaces
            namespaces = client.get_namespaces()
            if namespaces:
                self.get_logger().info(f"\nAll namespaces: {', '.join(namespaces)}")
            
            # Delete test
            self.get_logger().info("\nDelete test:")
            if client.delete_status('robot1', 'battery'):
                self.get_logger().info("  - Deleted robot1/battery")
            
            self.get_logger().info("\nNote: Other data is kept for web dashboard viewing")
            
        except Exception as e:
            self.get_logger().error(f"Client test error: {e}")


def main(args=None):
    """Main function"""
    rclpy.init(args=args)
    
    try:
        node = TestRobotStatusNode()
        
        # Keep node alive briefly to complete all operations
        rclpy.spin_once(node, timeout_sec=1.0)
        
    except KeyboardInterrupt:
        print("\nTest interrupted by user")
    except Exception as e:
        print(f"\nTest error: {e}")
    finally:
        try:
            node.destroy_node()
        except:
            pass
        rclpy.shutdown()


if __name__ == '__main__':
    main()
