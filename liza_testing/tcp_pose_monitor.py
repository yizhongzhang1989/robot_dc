#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
TCP Pose Monitor - Single Shot

This script subscribes to /tcp_pose_broadcaster/pose topic from UR ROS2 driver,
waits for one message, extracts the pose, converts quaternion to rotation matrix,
builds a 4x4 end2base transformation matrix, and prints it.

Usage:
    python3 tcp_pose_monitor.py
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
import numpy as np
import sys


class TCPPoseMonitor(Node):
    """ROS2 node to monitor TCP pose and compute end2base matrix."""
    
    def __init__(self):
        super().__init__('tcp_pose_monitor')
        
        self.get_logger().info("TCP Pose Monitor started")
        self.get_logger().info("Waiting for TCP pose data from /tcp_pose_broadcaster/pose...")

        
        # Flag to track if we've received data
        self.pose_received = False
        
        # Subscribe to TCP pose topic
        self.subscription = self.create_subscription(
            PoseStamped,
            '/tcp_pose_broadcaster/pose',
            self.tcp_pose_callback,
            10
        )
    
    def tcp_pose_callback(self, msg):
        """Callback function for receiving TCP pose."""
        if self.pose_received:
            return  # Already processed one message
        
        try:
            # Extract position (in meters)
            x = msg.pose.position.x
            y = msg.pose.position.y
            z = msg.pose.position.z
            
            # Extract quaternion orientation
            qx = msg.pose.orientation.x
            qy = msg.pose.orientation.y
            qz = msg.pose.orientation.z
            qw = msg.pose.orientation.w
            
            self.get_logger().info("\n" + "="*60)
            self.get_logger().info("TCP Pose Received!")
            self.get_logger().info("="*60)
            
            # Display message metadata
            self.get_logger().info(f"\nMessage Info:")
            self.get_logger().info(f"  Frame ID: {msg.header.frame_id}")
            self.get_logger().info(f"  Timestamp: {msg.header.stamp.sec}.{msg.header.stamp.nanosec:09d}")
            
            self.get_logger().info(f"\nPosition (meters):")
            self.get_logger().info(f"  X: {x:.6f} m  ({x*1000:.2f} mm)")
            self.get_logger().info(f"  Y: {y:.6f} m  ({y*1000:.2f} mm)")
            self.get_logger().info(f"  Z: {z:.6f} m  ({z*1000:.2f} mm)")
            
            self.get_logger().info(f"\nOrientation (quaternion):")
            self.get_logger().info(f"  qx: {qx:.6f}")
            self.get_logger().info(f"  qy: {qy:.6f}")
            self.get_logger().info(f"  qz: {qz:.6f}")
            self.get_logger().info(f"  qw: {qw:.6f}")
            
            # Validate quaternion normalization
            quat_norm = np.sqrt(qx**2 + qy**2 + qz**2 + qw**2)
            self.get_logger().info(f"  Quaternion norm: {quat_norm:.6f} (should be ~1.0)")
            if abs(quat_norm - 1.0) > 0.01:
                self.get_logger().warning("  WARNING: Quaternion is not properly normalized!")
            
            # Convert quaternion to rotation matrix
            R = np.array([
                [1 - 2*(qy**2 + qz**2), 2*(qx*qy - qz*qw), 2*(qx*qz + qy*qw)],
                [2*(qx*qy + qz*qw), 1 - 2*(qx**2 + qz**2), 2*(qy*qz - qx*qw)],
                [2*(qx*qz - qy*qw), 2*(qy*qz + qx*qw), 1 - 2*(qx**2 + qy**2)]
            ])
            
            self.get_logger().info(f"\nRotation Matrix (3x3):")
            for i, row in enumerate(R):
                self.get_logger().info(f"  [{row[0]:9.6f}, {row[1]:9.6f}, {row[2]:9.6f}]")
            
            # Validate rotation matrix properties
            self.get_logger().info(f"\nRotation Matrix Validation:")
            
            # Check if orthogonal: R * R^T should be identity
            R_times_RT = R @ R.T
            identity_error = np.linalg.norm(R_times_RT - np.eye(3))
            self.get_logger().info(f"  Orthogonality error (R*R^T - I): {identity_error:.9f} (should be ~0)")
            
            # Check determinant (should be +1 for proper rotation)
            det_R = np.linalg.det(R)
            self.get_logger().info(f"  Determinant: {det_R:.6f} (should be ~1.0)")
            
            if abs(det_R - 1.0) > 0.01:
                self.get_logger().warning("  WARNING: Determinant is not 1.0, may indicate reflection!")
            
            # Build 4x4 homogeneous transformation matrix (end2base)
            end2base_matrix = np.eye(4)
            end2base_matrix[:3, :3] = R
            end2base_matrix[:3, 3] = [x, y, z]
            
            self.get_logger().info(f"\nEnd-Effector to Base Transformation Matrix (4x4):")
            for i, row in enumerate(end2base_matrix):
                self.get_logger().info(f"  [{row[0]:9.6f}, {row[1]:9.6f}, {row[2]:9.6f}, {row[3]:9.6f}]")
            
            # Convert quaternion to Euler angles (Roll, Pitch, Yaw) for reference
            self.get_logger().info(f"\nOrientation as Euler Angles (XYZ convention):")
            # Roll (rotation around X-axis)
            roll = np.arctan2(2*(qw*qx + qy*qz), 1 - 2*(qx**2 + qy**2))
            # Pitch (rotation around Y-axis)
            pitch = np.arcsin(2*(qw*qy - qz*qx))
            # Yaw (rotation around Z-axis)
            yaw = np.arctan2(2*(qw*qz + qx*qy), 1 - 2*(qy**2 + qz**2))
            
            self.get_logger().info(f"  Roll:  {np.degrees(roll):8.3f}° ({roll:.6f} rad)")
            self.get_logger().info(f"  Pitch: {np.degrees(pitch):8.3f}° ({pitch:.6f} rad)")
            self.get_logger().info(f"  Yaw:   {np.degrees(yaw):8.3f}° ({yaw:.6f} rad)")
            
            # Test inverse transform
            self.get_logger().info(f"\nTransformation Validation:")
            base2end_matrix = np.linalg.inv(end2base_matrix)
            identity_test = end2base_matrix @ base2end_matrix
            inv_error = np.linalg.norm(identity_test - np.eye(4))
            self.get_logger().info(f"  Inverse error (T*T^-1 - I): {inv_error:.9f} (should be ~0)")
            
            self.get_logger().info("\n" + "="*60)
            self.get_logger().info("Computation complete! Shutting down...")
            self.get_logger().info("="*60 + "\n")
            
            # Mark as received and trigger shutdown
            self.pose_received = True
            
            # Shutdown the node
            self.get_logger().info("Exiting...")
            rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f"Error processing TCP pose: {e}")
            rclpy.shutdown()


def main(args=None):
    """Main function."""
    rclpy.init(args=args)
    
    try:
        node = TCPPoseMonitor()
        
        # Spin until we receive data or user interrupts
        rclpy.spin(node)
        
    except KeyboardInterrupt:
        print("\n\nInterrupted by user. Exiting...")
    except Exception as e:
        print(f"\nError: {e}")
        import traceback
        traceback.print_exc()
    finally:
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
