#!/usr/bin/env python3

"""
UR Camera Data Collection Script
This script collects robot pose and camera images when 'q' key is pressed.
Press 'q' to save current pose and image, 'ESC' to exit.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
import json
import os
from datetime import datetime
from ur15_robot_arm.ur15 import UR15Robot
from cv_bridge import CvBridge
import cv2
import sys
import select
import termios
import tty


class URCameraCollector(Node):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, save_dir="../temp/ur15_handles_location_data"):
        super().__init__('ur_camera_collector')
        
        # UR15 Robot connection for reading actual pose and joints
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.ur_robot = None
        self.save_dir = save_dir
        
        # Create save directory if it doesn't exist
        if not os.path.exists(self.save_dir):
            os.makedirs(self.save_dir)
            self.get_logger().info(f'Created save directory: {self.save_dir}')
        
        # Pose file counter - find the next available number
        self.pose_counter = self._get_next_pose_number()
        
        # Current camera image
        self.latest_image = None
        self.cv_bridge = CvBridge()
        
        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribe to camera image
        self.image_subscriber = self.create_subscription(
            Image,
            '/ur15_camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('UR Camera Collector Node started')
        self.get_logger().info(f'Pose save directory: {self.save_dir}')
        self.get_logger().info(f'Starting counter: {self.pose_counter}')
        self.get_logger().info('Press "q" to save current pose and image')
        self.get_logger().info('Press "ESC" to exit')
        
        # Connect to UR robot for reading actual data
        self._connect_to_robot()
    
    def _connect_to_robot(self):
        """Connect to UR robot for reading actual pose and joint data"""
        try:
            self.get_logger().info(f'Connecting to UR robot at {self.robot_ip}:{self.robot_port}...')
            self.ur_robot = UR15Robot(self.robot_ip, self.robot_port)
            result = self.ur_robot.open()
            if result == 0:
                self.get_logger().info('Successfully connected to UR robot for data reading')
            else:
                self.get_logger().warn('Failed to connect to UR robot. Pose saving will not work.')
                self.ur_robot = None
        except Exception as e:
            self.get_logger().error(f'Error connecting to UR robot: {e}')
            self.ur_robot = None
    
    def _rotvec_to_matrix(self, rx, ry, rz):
        """Convert rotation vector to rotation matrix"""
        import math
        angle = math.sqrt(rx**2 + ry**2 + rz**2)
        if angle < 1e-10:
            return np.eye(3)
        
        # Normalize axis
        kx, ky, kz = rx/angle, ry/angle, rz/angle
        
        # Rodrigues' rotation formula
        c = math.cos(angle)
        s = math.sin(angle)
        v = 1 - c
        
        R = np.array([
            [kx*kx*v + c,    kx*ky*v - kz*s, kx*kz*v + ky*s],
            [ky*kx*v + kz*s, ky*ky*v + c,    ky*kz*v - kx*s],
            [kz*kx*v - ky*s, kz*ky*v + kx*s, kz*kz*v + c]
        ])
        return R
    
    def _pose_to_matrix(self, pose):
        """Convert pose [X,Y,Z,Rx,Ry,Rz] to 4x4 homogeneous transformation matrix"""
        T = np.eye(4)
        T[0:3, 0:3] = self._rotvec_to_matrix(pose[3], pose[4], pose[5])
        T[0:3, 3] = [pose[0], pose[1], pose[2]]
        return T
    
    def _get_next_pose_number(self):
        """Find the next available pose number by checking existing files"""
        if not os.path.exists(self.save_dir):
            return 0
        
        existing_files = [f for f in os.listdir(self.save_dir) if f.endswith('.json')]
        if not existing_files:
            return 0
        
        # Extract numbers from filenames like "0.json", "1.json", etc.
        numbers = []
        for f in existing_files:
            try:
                num = int(f.replace('.json', ''))
                numbers.append(num)
            except ValueError:
                continue
        
        if not numbers:
            return 0
        
        return max(numbers) + 1
    
    def save_current_pose_and_image(self):
        """
        Read actual joint positions and TCP pose from UR robot and save to JSON file
        Also save the current camera image as JPG file
        """
        self.get_logger().info(f'>>> Starting save - Counter: {self.pose_counter}')
        
        if self.ur_robot is None:
            self.get_logger().error('UR robot not connected. Cannot save pose.')
            return False
        
        try:
            # Read actual joint positions
            joint_positions = self.ur_robot.get_actual_joint_positions()
            if joint_positions is None:
                self.get_logger().error('Failed to read joint positions')
                return False
            
            # Read actual TCP pose
            tcp_pose = self.ur_robot.get_actual_tcp_pose()
            if tcp_pose is None:
                self.get_logger().error('Failed to read TCP pose')
                return False
            
            # Calculate end2base transformation matrix
            end2base = self._pose_to_matrix(tcp_pose)
            
            # Prepare data structure
            data = {
                "joint_angles": list(joint_positions),
                "end_xyzrpy": {
                    "x": tcp_pose[0],
                    "y": tcp_pose[1],
                    "z": tcp_pose[2],
                    "rx": tcp_pose[3],
                    "ry": tcp_pose[4],
                    "rz": tcp_pose[5]
                },
                "end2base": end2base.tolist(),
                "timestamp": datetime.now().isoformat()
            }
            
            # Generate filename with sequential number
            json_filename = os.path.join(self.save_dir, f"{self.pose_counter}.json")
            
            # Save to JSON file
            with open(json_filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            self.get_logger().info(f'✓ Saved pose to: {json_filename}')
            self.get_logger().info(f'  Joint angles: {[f"{j:.4f}" for j in joint_positions]}')
            self.get_logger().info(f'  TCP pose: {[f"{p:.4f}" for p in tcp_pose]}')
            
            # Save current camera image if available
            if self.latest_image is not None:
                try:
                    # Convert ROS Image message to OpenCV format
                    cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
                    
                    # Generate image filename with same number
                    image_filename = os.path.join(self.save_dir, f"{self.pose_counter}.jpg")
                    
                    # Save image
                    cv2.imwrite(image_filename, cv_image)
                    self.get_logger().info(f'✓ Saved image to: {image_filename}')
                except Exception as e:
                    self.get_logger().error(f'Error saving camera image: {e}')
            else:
                self.get_logger().warn('No camera image available to save')
            
            # Increment counter for next save
            self.pose_counter += 1
            
            self.get_logger().info(f'<<< Finished save - Next counter: {self.pose_counter}')
            self.get_logger().info('Press "q" to save next pose, "ESC" to exit')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error saving pose: {e}')
            return False
    
    def image_callback(self, msg):
        """
        Update current camera image
        
        Args:
            msg: Image message from camera
        """
        self.latest_image = msg


def get_key_press():
    """
    Check if a key is pressed and return it (non-blocking)
    Returns None if no key is pressed
    """
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main(args=None):
    """
    Main function to initialize and run the camera collector node
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    
    # You can customize these parameters
    robot_ip = "192.168.1.15"  # UR robot IP address
    robot_port = 30002          # UR robot port
    save_dir = "../temp/ur15_handles_location_data"  # Directory to save pose files
    
    # Save terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # Set terminal to raw mode for key detection
        tty.setcbreak(sys.stdin.fileno())
        
        collector_node = URCameraCollector(
            robot_ip=robot_ip,
            robot_port=robot_port,
            save_dir=save_dir
        )
        
        # Use multi-threaded executor to handle callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(collector_node)
        
        # Start executor in a separate thread-like manner
        import threading
        executor_thread = threading.Thread(target=executor.spin, daemon=True)
        executor_thread.start()
        
        collector_node.get_logger().info('Node is running. Waiting for keyboard input...')
        
        # Main keyboard listening loop
        running = True
        while running and rclpy.ok():
            key = get_key_press()
            
            if key == 'q' or key == 'Q':
                collector_node.get_logger().info('=== "q" key pressed, saving pose and image ===')
                collector_node.save_current_pose_and_image()
            elif key == '\x1b':  # ESC key
                collector_node.get_logger().info('ESC pressed. Exiting...')
                running = False
            
            # Small sleep to prevent CPU spinning
            rclpy.spin_once(collector_node, timeout_sec=0.01)
        
    except KeyboardInterrupt:
        print('\nCtrl+C pressed. Shutting down...')
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        # Clean up UR robot connection
        if collector_node.ur_robot is not None:
            collector_node.ur_robot.close()
        
        executor.shutdown()
        collector_node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
