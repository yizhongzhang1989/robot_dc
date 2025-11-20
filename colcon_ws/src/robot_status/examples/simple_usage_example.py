#!/usr/bin/env python3
"""
Simple example showing how to use robot_status utility functions from any node.

This demonstrates the easiest way to get/set status values without creating
a client instance or dealing with service calls directly.
"""

import rclpy
from rclpy.node import Node
import json
import numpy as np

# Simply import the utility functions
from robot_status import get_from_status, set_to_status


class SimpleExampleNode(Node):
    """Example node demonstrating simple robot_status usage."""
    
    def __init__(self):
        super().__init__('simple_example_node')
        
        # Example 1: Save some calibration data
        self.save_calibration_data()
        
        # Example 2: Load the calibration data back
        self.load_calibration_data()
        
        # Example 3: Save simple values
        self.save_simple_values()
        
        # Example 4: Load simple values
        self.load_simple_values()
    
    def save_calibration_data(self):
        """Example: Save camera calibration matrices."""
        self.get_logger().info("=== Saving calibration data ===")
        
        # Create some example calibration data (normally you'd have real calibration data)
        camera_matrix = np.array([
            [800.0, 0.0, 320.0],
            [0.0, 800.0, 240.0],
            [0.0, 0.0, 1.0]
        ])
        
        distortion_coeffs = np.array([0.1, -0.2, 0.0, 0.0, 0.0])
        
        # Convert to JSON strings
        camera_matrix_json = json.dumps(camera_matrix.tolist())
        distortion_json = json.dumps(distortion_coeffs.tolist())
        
        # Save to robot_status - just one line per parameter!
        if set_to_status(self, 'my_robot', 'camera_matrix', camera_matrix_json):
            self.get_logger().info("✓ Saved camera_matrix")
        
        if set_to_status(self, 'my_robot', 'distortion_coeffs', distortion_json):
            self.get_logger().info("✓ Saved distortion_coeffs")
    
    def load_calibration_data(self):
        """Example: Load camera calibration matrices."""
        self.get_logger().info("=== Loading calibration data ===")
        
        # Load from robot_status - just one line per parameter!
        camera_matrix_json = get_from_status(self, 'my_robot', 'camera_matrix')
        if camera_matrix_json:
            camera_matrix = np.array(json.loads(camera_matrix_json))
            self.get_logger().info(f"✓ Loaded camera_matrix shape: {camera_matrix.shape}")
            self.get_logger().info(f"  Values:\n{camera_matrix}")
        
        distortion_json = get_from_status(self, 'my_robot', 'distortion_coeffs')
        if distortion_json:
            distortion_coeffs = np.array(json.loads(distortion_json))
            self.get_logger().info(f"✓ Loaded distortion_coeffs: {distortion_coeffs}")
    
    def save_simple_values(self):
        """Example: Save simple key-value pairs."""
        self.get_logger().info("=== Saving simple values ===")
        
        # Save different types of values
        set_to_status(self, 'my_robot', 'battery_level', '85')
        set_to_status(self, 'my_robot', 'status', 'operational')
        set_to_status(self, 'my_robot', 'temperature', '42.5')
        
        # Save a dict as JSON
        pose_dict = {'x': 1.5, 'y': 2.3, 'z': 0.0}
        set_to_status(self, 'my_robot', 'current_pose', json.dumps(pose_dict))
        
        self.get_logger().info("✓ Saved multiple simple values")
    
    def load_simple_values(self):
        """Example: Load simple key-value pairs."""
        self.get_logger().info("=== Loading simple values ===")
        
        # Load values
        battery = get_from_status(self, 'my_robot', 'battery_level')
        status = get_from_status(self, 'my_robot', 'status')
        temperature = get_from_status(self, 'my_robot', 'temperature')
        
        if battery:
            self.get_logger().info(f"✓ Battery: {battery}%")
        if status:
            self.get_logger().info(f"✓ Status: {status}")
        if temperature:
            self.get_logger().info(f"✓ Temperature: {temperature}°C")
        
        # Load dict
        pose_json = get_from_status(self, 'my_robot', 'current_pose')
        if pose_json:
            pose = json.loads(pose_json)
            self.get_logger().info(f"✓ Current pose: x={pose['x']}, y={pose['y']}, z={pose['z']}")


def main(args=None):
    rclpy.init(args=args)
    
    node = SimpleExampleNode()
    
    # Keep node alive briefly to see output
    rclpy.spin_once(node, timeout_sec=1.0)
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
