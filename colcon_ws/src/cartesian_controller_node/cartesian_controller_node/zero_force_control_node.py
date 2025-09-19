#!/usr/bin/env python3
"""
Zero Force Control Node

This ROS2 node monitors UDP data from a robot and publishes force/torque data
in WrenchStamped format for force control applications.

The node:
1. Listens for UDP packets on port 5566 (robot data) 
2. Extracts FTSensorData from the UDP messages
3. Applies FT calibration compensation using ./ft_calib_result.json
4. Publishes /ft_sensor_wrench and /target_wrench topics
5. Uses geometry_msgs/WrenchStamped format
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState
import socket
import json
import threading
from datetime import datetime
import os
import numpy as np
import math
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


def quat_to_rot_matrix(qx, qy, qz, qw) -> np.ndarray:
    """Convert quaternion to rotation matrix"""
    # Normalize
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = qx/n, qy/n, qz/n, qw/n
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array([
        [1 - 2*(yy + zz), 2*(xy - wz),   2*(xz + wy)],
        [2*(xy + wz),     1 - 2*(xx+zz), 2*(yz - wx)],
        [2*(xz - wy),     2*(yz + wx),   1 - 2*(xx+yy)]
    ], dtype=float)
    return R


class ZeroForceControlNode(Node):
    def __init__(self):
        super().__init__('zero_force_control_node')
        
        # Configuration
        self.host = "0.0.0.0"
        self.port_data = 5566  # Must use same port as robot_arm_web_server (data source fixed)
        
        # Load FT calibration data
        self.load_ft_calibration()
        
        # Current robot orientation for gravity compensation
        self.latest_pose = None
        
        # Store latest FT data for logging
        self.latest_raw_force = np.zeros(3)
        self.latest_raw_torque = np.zeros(3)
        self.latest_compensated_force = np.zeros(3)
        self.latest_compensated_torque = np.zeros(3)
        self.target_force = np.array([0.0, 0.0, 0.0])  # All zeros for target
        self.target_torque = np.zeros(3)  # All zeros for target
        
        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Create publishers for force/torque data
        self.ft_sensor_publisher = self.create_publisher(
            WrenchStamped, 'ft_sensor_wrench', 10, callback_group=self.callback_group)
        self.target_wrench_publisher = self.create_publisher(
            WrenchStamped, 'target_wrench', 10, callback_group=self.callback_group)
        
        self.pose_subscriber_force = self.create_subscription(
            PoseStamped,
            '/cartesian_force_controller/current_pose',
            self.force_con_pose_callback,
            10,
            callback_group=self.callback_group
        )

        self.pose_subscriber_compliance = self.create_subscription(
            PoseStamped,
            '/cartesian_compliance_controller/current_pose',
            self.compliance_con_pose_callback,
            10,
            callback_group=self.callback_group
        )

        # Create timer for 1-second logging
        self.log_timer = self.create_timer(
            1.0,  # 1 second
            self.log_ft_data,
            callback_group=self.callback_group
        )
        
        # Start UDP receiver
        self.start_udp_receiver()
        
        self.get_logger().info('Zero Force Control Node started')
        self.get_logger().info(f'Listening on port {self.port_data} for robot data')
        self.get_logger().info('Using SO_REUSEPORT to share port 5566 with robot_arm_web_server')
        self.get_logger().info('Publishing /ft_sensor_wrench and /target_wrench topics')
        self.get_logger().info(f'Force bias: {self.force_bias}')
        self.get_logger().info(f'Torque bias: {self.torque_bias}')
        self.get_logger().info(f'Mass: {self.mass:.4f} kg')
    
    def load_ft_calibration(self):
        """Load FT calibration data from JSON file"""
        calib_file = './ft_calib_result.json'
        
        try:
            if os.path.exists(calib_file):
                with open(calib_file, 'r') as f:
                    calib_data = json.load(f)
                
                # Extract calibration parameters
                calib = calib_data['calibration']
                self.force_bias = np.array(calib['force_bias'])
                self.torque_bias = np.array(calib['torque_bias'])
                self.mass = float(calib['mass'])
                self.center_of_mass = np.array(calib['center_of_mass'])
                self.p_vector = np.array(calib['p_vector'])
                self.gravity_vector = np.array(calib['gravity_vector'])
                
                self.get_logger().info(f'Loaded FT calibration from: {calib_file}')
                
            else:
                self.get_logger().warning(f'Calibration file not found: {calib_file}')
                self.get_logger().warning('Using fallback baseline values')
                raise()                
        except Exception as e:
            self.get_logger().error(f'Failed to load calibration: {e}')
            self.get_logger().warning('Using fallback baseline values')
    
    def force_con_pose_callback(self, msg):
        self.latest_pose = msg

    def compliance_con_pose_callback(self, msg):
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
    
    def compensate_ft_data(self, fx, fy, fz, tx, ty, tz):
        """Apply FT calibration compensation"""
        f_meas = np.array([fx, fy, fz])
        t_meas = np.array([tx, ty, tz])
        
        if self.latest_pose is not None:
            # Use full calibration with gravity compensation
            qx = self.latest_pose.pose.orientation.x
            qy = self.latest_pose.pose.orientation.y
            qz = self.latest_pose.pose.orientation.z
            qw = self.latest_pose.pose.orientation.w
            
            f_g, tau_g = self.predict_gravity_wrench(qx, qy, qz, qw)
            f_comp = f_meas - self.force_bias - f_g
            t_comp = t_meas - self.torque_bias - tau_g
            
        else:
            # Use calibration without gravity compensation (no pose available)
            f_comp = f_meas - self.force_bias
            t_comp = t_meas - self.torque_bias
            
        return f_comp, t_comp
    
    def start_udp_receiver(self):
        """Start UDP receiver thread"""
        data_thread = threading.Thread(
            target=self.udp_data_receiver, 
            daemon=True, 
            name="ft_data_receiver"
        )
        data_thread.start()
    
    def log_ft_data(self):
        """Timer callback to log FT data every second"""
        timestamp = datetime.now().strftime('%H:%M:%S.%f')[:-3]
        
        compensation_type = "Full" if (self.latest_pose is not None) else \
                          "Bias-only"
        
        self.get_logger().info(
            f'[{timestamp}] '
            f'Raw Force: [{self.latest_raw_force[0]:.3f}, {self.latest_raw_force[1]:.3f}, {self.latest_raw_force[2]:.3f}], '
            f'Compensated Force: [{self.latest_compensated_force[0]:.3f}, {self.latest_compensated_force[1]:.3f}, {self.latest_compensated_force[2]:.3f}], '
            f'Target Force: [{self.target_force[0]:.3f}, {self.target_force[1]:.3f}, {self.target_force[2]:.3f}], '
            f'Raw Torque: [{self.latest_raw_torque[0]:.3f}, {self.latest_raw_torque[1]:.3f}, {self.latest_raw_torque[2]:.3f}], '
            f'Compensated Torque: [{self.latest_compensated_torque[0]:.3f}, {self.latest_compensated_torque[1]:.3f}, {self.latest_compensated_torque[2]:.3f}], '
            f'Target Torque: [{self.target_torque[0]:.3f}, {self.target_torque[1]:.3f}, {self.target_torque[2]:.3f}], '
            f'Comp: {compensation_type}'
        )
    
    def create_sensor_wrench_msg(self, ft_data):
        """Create WrenchStamped message from 6D FT sensor data with calibration compensation"""
        msg = WrenchStamped()
        
        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ft_sensor_frame"
        
        # FTSensorData is 6D: [Fx, Fy, Fz, Tx, Ty, Tz]
        if len(ft_data) >= 6:
            # Store raw force and torque data
            self.latest_raw_force = np.array([float(ft_data[0]), float(ft_data[1]), float(ft_data[2])])
            self.latest_raw_torque = np.array([float(ft_data[3]), float(ft_data[4]), float(ft_data[5])])
            
            # Apply calibration compensation
            f_comp, t_comp = self.compensate_ft_data(
                float(ft_data[0]), float(ft_data[1]), float(ft_data[2]),
                float(ft_data[3]), float(ft_data[4]), float(ft_data[5])
            )
            
            # Store compensated force and torque data
            self.latest_compensated_force = f_comp
            self.latest_compensated_torque = t_comp
            
            # Set compensated values
            msg.wrench.force.x = f_comp[0]
            msg.wrench.force.y = f_comp[1]
            msg.wrench.force.z = f_comp[2]
            msg.wrench.torque.x = t_comp[0]
            msg.wrench.torque.y = t_comp[1]
            msg.wrench.torque.z = t_comp[2]
            
        else:
            self.get_logger().warning(f'FTSensorData has insufficient elements: {len(ft_data)}, expected 6')
            # Set all to zero if insufficient data
            msg.wrench.force.x = 0.0
            msg.wrench.force.y = 0.0
            msg.wrench.force.z = 0.0
            msg.wrench.torque.x = 0.0
            msg.wrench.torque.y = 0.0
            msg.wrench.torque.z = 0.0
            
            # Reset stored data
            self.latest_raw_force = np.zeros(3)
            self.latest_raw_torque = np.zeros(3)
            self.latest_compensated_force = np.zeros(3)
            self.latest_compensated_torque = np.zeros(3)
        
        return msg
    
    def create_target_wrench_msg(self):
        """Create target WrenchStamped message with all zeros except z force = 10"""
        msg = WrenchStamped()
        
        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ft_sensor_frame"
        
        # All zeros except z force = 10
        msg.wrench.force.x = 0.0
        msg.wrench.force.y = 0.0
        msg.wrench.force.z = 0.0
        msg.wrench.torque.x = 0.0
        msg.wrench.torque.y = 0.0
        msg.wrench.torque.z = 0.0
        
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
                        
                        # Create target wrench message (all zeros except z=10)
                        target_wrench_msg = self.create_target_wrench_msg()
                        
                        # Publish both topics
                        self.ft_sensor_publisher.publish(ft_wrench_msg)
                        self.target_wrench_publisher.publish(target_wrench_msg)
                        
                        # Removed per-message logging - now using timer-based logging
                        
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
        zero_force_node = ZeroForceControlNode()
        
        # Use multi-threaded executor to handle multiple callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(zero_force_node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            zero_force_node.get_logger().info('Shutting down zero force control node...')
        finally:
            executor.shutdown()
            zero_force_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
