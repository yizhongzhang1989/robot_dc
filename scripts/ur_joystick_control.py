#!/usr/bin/env python3

"""
Joystick Control for Robotic Arm in Cartesian Space
This script allows control of a robotic arm using a joystick in Cartesian space.
Based on the joystick_calibration_node implementation.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import Joy, Image
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from scipy.spatial.transform import Rotation as R
import json
import os
from datetime import datetime
from UR15Robot import UR15Robot
from cv_bridge import CvBridge
import cv2


class JoystickCartesianControl(Node):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, save_dir="../temp/ur15_cam_calibraition_data"):
        super().__init__('joystick_cartesian_control')
        
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
        
        # Button debounce - prevent multiple saves from one button press
        self.last_button_state = False
        self.button_pressed = False
        self.last_save_time = 0.0  # Add timestamp for debouncing
        
        # Current robot pose
        self.latest_pose = None
        
        # Current camera image
        self.latest_image = None
        self.cv_bridge = CvBridge()
        
        # Joystick input values (axes and rotation deltas)
        self.x_diff = 0.0
        self.y_diff = 0.0
        self.z_diff = 0.0
        self.rot_x_diff = 0.0
        self.rot_y_diff = 0.0
        self.rot_z_diff = 0.0
        
        # Motion control gains - adjusted for control frequency
        # These values determine the speed of motion based on joystick input
        self.motion_gain_xy = 0.08  # Linear motion gain for x,y axes (m/time)
        self.motion_gain_z = 0.08   # Linear motion gain for z axis (m/time)
        self.rotation_gain = 0.3    # Rotation gain (rad/time)
        
        # Joystick deadzone - ignore small joystick movements
        self.joy_deadzone = 0.05
        
        # Motion command publish rate limiting
        self.motion_publish_rate = 50.0  # Hz - target publish rate
        self.last_publish_time = self.get_clock().now()
        self.min_publish_interval = 1.0 / self.motion_publish_rate  # seconds
        
        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Subscribe to joystick input
        self.joy_sub = self.create_subscription(
            Joy, 
            '/joy', 
            self.joy_callback, 
            10,
            callback_group=self.callback_group
        )
        
        # Subscribe to current pose from cartesian motion controller
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/cartesian_motion_controller/current_pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Subscribe to camera image
        self.image_subscriber = self.create_subscription(
            Image,
            '/ur15_camera/image_raw',
            self.image_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Publisher for target pose commands
        self.target_frame_publisher = self.create_publisher(
            PoseStamped, 
            'target_frame', 
            10, 
            callback_group=self.callback_group
        )
        
        # Create timer for periodic motion command publishing
        self.timer = self.create_timer(
            1.0 / self.motion_publish_rate,
            self.timer_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('Joystick Cartesian Control Node started')
        self.get_logger().info('Joystick axes mapping:')
        self.get_logger().info('  Axis 0: Y-direction (left/right)')
        self.get_logger().info('  Axis 1: X-direction (forward/backward)')
        self.get_logger().info('  Axis 2: Roll rotation')
        self.get_logger().info('  Axis 3: Pitch rotation')
        self.get_logger().info('  Button 4: Z-up')
        self.get_logger().info('  Button 6: Z-down')
        self.get_logger().info('  Button 5: Yaw-positive')
        self.get_logger().info('  Button 7: Yaw-negative')
        self.get_logger().info('  Button B (button 2): Save current pose')
        self.get_logger().info(f'Motion gains - XY: {self.motion_gain_xy}, Z: {self.motion_gain_z}, Rotation: {self.rotation_gain}')
        self.get_logger().info(f'Publish rate: {self.motion_publish_rate} Hz')
        self.get_logger().info(f'Pose save directory: {self.save_dir}')
        
        # Connect to UR robot for reading actual data
        self._connect_to_robot()
    
    def apply_deadzone(self, value):
        """
        Apply deadzone to joystick input to filter out noise
        
        Args:
            value: Raw joystick axis value
            
        Returns:
            Filtered value (0.0 if within deadzone)
        """
        if abs(value) < self.joy_deadzone:
            return 0.0
        return value
    
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
    
    def save_current_pose(self):
        """
        Read actual joint positions and TCP pose from UR robot and save to JSON file
        Also save the current camera image as JPG file
        """
        self.get_logger().info(f'>>> Starting save_current_pose() - Counter: {self.pose_counter}')
        
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
                "end2base": end2base.tolist()
            }
            
            # Generate filename with sequential number
            json_filename = os.path.join(self.save_dir, f"{self.pose_counter}.json")
            
            # Save to JSON file
            with open(json_filename, 'w') as f:
                json.dump(data, f, indent=2)
            
            self.get_logger().info(f'Saved current pose to: {json_filename}')
            self.get_logger().info(f'Joint angles: {joint_positions}')
            self.get_logger().info(f'TCP pose: {tcp_pose}')
            
            # Save current camera image if available
            if self.latest_image is not None:
                try:
                    # Convert ROS Image message to OpenCV format
                    cv_image = self.cv_bridge.imgmsg_to_cv2(self.latest_image, desired_encoding='bgr8')
                    
                    # Generate image filename with same number
                    image_filename = os.path.join(self.save_dir, f"{self.pose_counter}.jpg")
                    
                    # Save image
                    cv2.imwrite(image_filename, cv_image)
                    self.get_logger().info(f'Saved camera image to: {image_filename}')
                except Exception as e:
                    self.get_logger().error(f'Error saving camera image: {e}')
            else:
                self.get_logger().warn('No camera image available to save')
            
            # Increment counter for next save
            self.pose_counter += 1
            
            self.get_logger().info(f'<<< Finished save_current_pose() - Next counter: {self.pose_counter}')
            
            return True
            
        except Exception as e:
            self.get_logger().error(f'Error saving pose: {e}')
            return False
    
    def joy_callback(self, msg):
        """
        Process joystick input with deadzone filtering
        
        Joystick mapping:
        - Axes 0,1: X,Y translation
        - Axes 2,3: Roll, Pitch rotation
        - Buttons 4,6: Z translation (up/down)
        - Buttons 5,7: Yaw rotation
        - Button 2 (B button): Save current pose
        
        Args:
            msg: Joy message from joystick
        """
        # Linear motion from analog axes
        self.y_diff = self.apply_deadzone(msg.axes[0])  # Left/Right
        self.x_diff = self.apply_deadzone(msg.axes[1])  # Forward/Backward
        
        # Rotation from analog axes
        self.rot_x_diff = self.apply_deadzone(msg.axes[2])  # Roll
        self.rot_y_diff = self.apply_deadzone(msg.axes[3])  # Pitch
        
        # Z-axis motion from buttons
        if msg.buttons[4]:  # Button 4: Move up
            self.z_diff = 1.0
        elif msg.buttons[6]:  # Button 6: Move down
            self.z_diff = -1.0
        else:
            self.z_diff = 0.0
        
        # Yaw rotation from buttons
        if msg.buttons[5]:  # Button 5: Yaw positive
            self.rot_z_diff = 1.0
        elif msg.buttons[7]:  # Button 7: Yaw negative
            self.rot_z_diff = -1.0
        else:
            self.rot_z_diff = 0.0
        
        # Save pose when button 2 (B button) is pressed
        # Debounce: only trigger on rising edge (button pressed, not held)
        # Also add time-based debounce to prevent rapid repeated triggers
        if len(msg.buttons) > 2:
            current_button_state = msg.buttons[2]
            current_time = self.get_clock().now().nanoseconds / 1e9
            
            # Only trigger if button just pressed AND at least 0.5 seconds since last save
            if current_button_state and not self.last_button_state:
                if current_time - self.last_save_time > 0.5:  # 500ms debounce
                    self.get_logger().info('=== B button pressed, saving pose and image ===')
                    # Update time BEFORE calling save to prevent race condition
                    self.last_save_time = current_time
                    self.save_current_pose()
                else:
                    self.get_logger().warn(f'Button press ignored (too soon: {current_time - self.last_save_time:.3f}s since last save)')
            
            self.last_button_state = current_button_state
    
    def pose_callback(self, msg):
        """
        Update current pose from the robot
        
        Args:
            msg: PoseStamped message with current robot pose
        """
        self.latest_pose = msg
    
    def image_callback(self, msg):
        """
        Update current camera image
        
        Args:
            msg: Image message from camera
        """
        self.latest_image = msg
    
    def publish_motion_command(self):
        """
        Compute and publish target pose based on current pose and joystick input
        
        This method:
        1. Takes current pose
        2. Applies linear motion based on joystick axes
        3. Applies rotational motion based on joystick axes/buttons
        4. Publishes the new target pose
        """
        if self.latest_pose is None:
            return
        
        # Create target pose message
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = self.latest_pose.header.frame_id
        
        # Apply linear motion gains to current position
        # Note: x_diff is negated to match typical joystick conventions
        target_pose.pose.position.x = (self.latest_pose.pose.position.x - 
                                       self.x_diff * self.motion_gain_xy)
        target_pose.pose.position.y = (self.latest_pose.pose.position.y - 
                                       self.y_diff * self.motion_gain_xy)
        target_pose.pose.position.z = (self.latest_pose.pose.position.z + 
                                       self.z_diff * self.motion_gain_z)
        
        # Convert current quaternion to scipy Rotation object
        current_quat = [
            self.latest_pose.pose.orientation.x,
            self.latest_pose.pose.orientation.y,
            self.latest_pose.pose.orientation.z,
            self.latest_pose.pose.orientation.w
        ]
        current_rotation = R.from_quat(current_quat)
        
        # Create rotation changes from joystick input
        delta_rx = self.rot_x_diff * self.rotation_gain
        delta_ry = self.rot_y_diff * self.rotation_gain
        delta_rz = self.rot_z_diff * self.rotation_gain
        
        # Create delta rotation from Euler angles (XYZ order)
        delta_rotation = R.from_euler('xyz', [delta_rx, delta_ry, delta_rz])
        
        # Apply rotation: R_new = delta_rotation * current_rotation
        target_rotation = delta_rotation * current_rotation
        
        # Convert back to quaternion
        target_quat = target_rotation.as_quat()
        
        # Set target orientation
        target_pose.pose.orientation.x = target_quat[0]
        target_pose.pose.orientation.y = target_quat[1]
        target_pose.pose.orientation.z = target_quat[2]
        target_pose.pose.orientation.w = target_quat[3]
        
        # Publish target pose
        self.target_frame_publisher.publish(target_pose)
        
        # Log motion commands (only if there's actual motion)
        if (abs(self.x_diff) > 0 or abs(self.y_diff) > 0 or abs(self.z_diff) > 0 or
            abs(self.rot_x_diff) > 0 or abs(self.rot_y_diff) > 0 or abs(self.rot_z_diff) > 0):
            self.get_logger().debug(
                f'Motion cmd - X:{self.x_diff:.3f} Y:{self.y_diff:.3f} Z:{self.z_diff:.3f} '
                f'Rx:{self.rot_x_diff:.3f} Ry:{self.rot_y_diff:.3f} Rz:{self.rot_z_diff:.3f}'
            )
    
    def timer_callback(self):
        """
        Timer callback to publish motion commands at fixed rate
        
        This ensures smooth motion even with varying joystick input rates
        """
        current_time = self.get_clock().now()
        time_since_last_publish = (current_time - self.last_publish_time).nanoseconds / 1e9
        
        if time_since_last_publish >= self.min_publish_interval:
            self.publish_motion_command()
            self.last_publish_time = current_time


def main(args=None):
    """
    Main function to initialize and run the joystick control node
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    
    # You can customize these parameters
    robot_ip = "192.168.1.15"  # UR robot IP address
    robot_port = 30002          # UR robot port
    save_dir = "../temp/ur15_cam_calibraition_data"  # Directory to save pose files
    
    try:
        joystick_control_node = JoystickCartesianControl(
            robot_ip=robot_ip,
            robot_port=robot_port,
            save_dir=save_dir
        )
        
        # Use multi-threaded executor to handle multiple callbacks concurrently
        executor = MultiThreadedExecutor()
        executor.add_node(joystick_control_node)
        
        try:
            joystick_control_node.get_logger().info('Node is running. Use joystick to control the robot.')
            joystick_control_node.get_logger().info('Press Ctrl+C to stop.')
            executor.spin()
        except KeyboardInterrupt:
            joystick_control_node.get_logger().info('Shutting down joystick control node...')
        finally:
            # Clean up UR robot connection
            if joystick_control_node.ur_robot is not None:
                joystick_control_node.ur_robot.close()
            executor.shutdown()
            joystick_control_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
