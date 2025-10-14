#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
import socket
from sensor_msgs.msg import Joy
import json
import threading
import numpy as np
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from .utils import quat_to_rot_matrix
from scipy.spatial.transform import Rotation as R


class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        
        # Configuration
        self.host = "0.0.0.0"
        self.port_data = 5566  # Must use same port as robot_arm_web_server (data source fixed)
                
        # Current robot orientation for gravity compensation
        self.latest_pose = None
                
        # Joystick input values
        self.x_diff = 0.0
        self.y_diff = 0.0
        self.z_diff = 0.0
        self.rot_x_diff = 0.0
        self.rot_y_diff = 0.0
        self.rot_z_diff = 0.0

        # motion control gains
        self.motion_gain_xy = 1  # motion mode gain for x,y
        self.motion_gain_z = 1   # motion mode gain for z
        
        # Multi-stage filter parameters and state
        self.filter_alpha = 0.2  # LPF coefficient (0 < alpha < 1, smaller = more filtering)
        self.filtered_force = np.zeros(3)  # Previous filtered force values
        self.filtered_torque = np.zeros(3)  # Previous filtered torque values
        self.filter_initialized = True  # Flag to check if filter is initialized
        
        # Rate limiter parameters
        self.max_force_rate = 100.0  # N/s
        self.max_torque_rate = 10.0  # N⋅m/s
        self.previous_force = np.zeros(3)
        self.previous_torque = np.zeros(3)
        self.dt = 0.01  # Assumed sampling time (adjust based on actual rate)
        
        # Median filter (3-tap for spike removal)
        self.median_window_size = 3
        self.force_history = np.zeros((self.median_window_size, 3))
        self.torque_history = np.zeros((self.median_window_size, 3))
        self.history_index = 0
        
        # Deadband filter parameters
        self.force_deadband = 2.0  # N
        self.torque_deadband = 0.1  # N⋅m

        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()

        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        # Create publishers for force/torque data
        self.ft_sensor_publisher = self.create_publisher(
            WrenchStamped, 'ft_sensor_wrench', 10, callback_group=self.callback_group)
        self.target_wrench_publisher = self.create_publisher(
            WrenchStamped, 'target_wrench', 10, callback_group=self.callback_group)
        self.target_frame_publisher = self.create_publisher(
            PoseStamped, 'target_frame', 10, callback_group=self.callback_group)

        self.pose_subscriber_motion = self.create_subscription(
            PoseStamped,
            '/cartesian_motion_controller/current_pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )

        self.start_udp_receiver()

    def publish_motion_command(self):
        """Publish pose command for motion control"""
        if self.latest_pose is None:
            return
        
        target_pose = PoseStamped()
        target_pose.header.stamp = self.get_clock().now().to_msg()
        target_pose.header.frame_id = self.latest_pose.header.frame_id
        
        # Apply motion gains and add to current pose
        target_pose.pose.position.x = (self.latest_pose.pose.position.x - 
                                     self.x_diff * self.motion_gain_xy)
        target_pose.pose.position.y = (self.latest_pose.pose.position.y - 
                                     self.y_diff * self.motion_gain_xy)
        target_pose.pose.position.z = (self.latest_pose.pose.position.z + 
                                     self.z_diff * self.motion_gain_z)
        
        # Convert current quaternion to scipy Rotation object
        current_quat = [self.latest_pose.pose.orientation.x,
                   self.latest_pose.pose.orientation.y, 
                   self.latest_pose.pose.orientation.z,
                   self.latest_pose.pose.orientation.w]
        current_rotation = R.from_quat(current_quat)
        
        # Create rotation changes from joystick input
        gain_rot = 10  # Rotation gain
        delta_rx = self.rot_x_diff * gain_rot
        delta_ry = self.rot_y_diff * gain_rot
        delta_rz = self.rot_z_diff * gain_rot
        
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

        self.target_frame_publisher.publish(target_pose)

    def joy_callback(self, msg):
        """Process joystick input"""
        self.y_diff = msg.axes[0]
        self.x_diff = msg.axes[1]
        self.rot_x_diff = msg.axes[2]
        self.rot_y_diff = msg.axes[3]
        if msg.buttons[4]:  # button 4 pressed
            self.z_diff = 1.0
        elif msg.buttons[6]:  # button 6 pressed
            self.z_diff = -1.0
        else:
            self.z_diff = 0.0
        if msg.buttons[5]:  # button 5 pressed
            self.rot_z_diff = 1.0
        elif msg.buttons[7]:  # button 7 pressed
            self.rot_z_diff = -1.0
        else:
            self.rot_z_diff = 0.0

    def pose_callback(self, msg):
        self.latest_pose = msg

    def predict_gravity_wrench(self, qx, qy, qz, qw):
        """Predict gravity-induced forces and torques in sensor frame"""
        
        # Convert quaternion to rotation matrix
        R_b_s = quat_to_rot_matrix(qx, qy, qz, qw)
        
        # Transform gravity vector to sensor frame
        h = R_b_s.T @ self.gravity_vector  # h = R^T * g_b
        
        # Calculate gravity-induced force and torque
        f_g = self.mass * h
        tau_g = np.cross(self.p_vector, h)  # p x h
        
        return f_g, tau_g
        
    def start_udp_receiver(self):
        """Start UDP receiver thread"""
        data_thread = threading.Thread(
            target=self.udp_data_receiver, 
            daemon=True, 
            name="ft_data_receiver"
        )
        data_thread.start()
    
    
    def create_sensor_wrench_msg(self, ft_data):
        """Create WrenchStamped message from 6D FT sensor data with multi-stage filtering"""
        msg = WrenchStamped()
        
        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ft_sensor_frame"
        
        # FTSensorData is 6D: [Fx, Fy, Fz, Tx, Ty, Tz]
        if len(ft_data) >= 6:
            # Store raw force and torque data
            f_comp = np.array([float(ft_data[0]), float(ft_data[1]), float(ft_data[2])])
            t_comp = np.array([float(ft_data[3]), float(ft_data[4]), float(ft_data[5])])
            
            self.latest_compensated_force = f_comp
            self.latest_compensated_torque = t_comp
            
            self.filtered_force = f_comp.copy()
            self.filtered_torque = t_comp.copy()
            self.previous_force = f_comp.copy()
            self.previous_torque = f_comp.copy()
            # Initialize median filter history
            for i in range(self.median_window_size):
                self.force_history[i] = f_comp
                self.torque_history[i] = t_comp
            final_force = f_comp
            final_torque = t_comp

            # Set final filtered values to message
            msg.wrench.force.x = final_force[0]
            msg.wrench.force.y = final_force[1]
            msg.wrench.force.z = final_force[2]
            msg.wrench.torque.x = final_torque[0]
            msg.wrench.torque.y = final_torque[1]
            msg.wrench.torque.z = final_torque[2]

        
        return msg
    
    def udp_data_receiver(self):
        """UDP receiver for robot data on port 5566"""
        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        
        # Use SO_REUSEPORT to share port with robot_arm_web_server
        try:
            sock.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEPORT, 1)
            self.get_logger().info('SO_REUSEPORT enabled for port sharing')
        except AttributeError:
            self.get_logger().error('SO_REUSEPORT not available on this system')
            self.get_logger().error('Cannot share port 5566 with robot_arm_web_server')
            return
        
        try:
            sock.bind((self.host, self.port_data))
            self.get_logger().info(f'FT data receiver bound to {self.host}:{self.port_data}')
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
                
                # Parse JSON to extract FTSensorData
                try:
                    parsed_data = json.loads(message)
                    ft_sensor_data = parsed_data.get("FTSensorData", None)
                    
                    if ft_sensor_data is not None:
                        # Create sensor wrench message with calibration compensation
                        ft_wrench_msg = self.create_sensor_wrench_msg(ft_sensor_data)
                        self.ft_sensor_publisher.publish(ft_wrench_msg)
                        self.publish_motion_command()
                    else:
                        self.get_logger().debug(f'No FTSensorData found in message from {addr[0]}:{addr[1]}')
                        
                except json.JSONDecodeError:
                    self.get_logger().warning(f'Invalid JSON from {addr[0]}:{addr[1]}: {message[:100]}')
                except (IndexError, ValueError) as e:
                    self.get_logger().warning(f'Error processing FTSensorData: {e}')
                
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f'Error in FT data receiver: {e}')
                break
        
        sock.close()
        self.get_logger().info('FT data socket closed')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        joystick_control_node = JoystickControlNode()
        
        # Use multi-threaded executor to handle multiple callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(joystick_control_node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            joystick_control_node.get_logger().info('Shutting down joystick control node...')
        finally:
            executor.shutdown()
            joystick_control_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
