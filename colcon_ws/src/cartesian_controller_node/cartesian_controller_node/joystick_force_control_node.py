#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
import socket
from sensor_msgs.msg import Joy
import json
import threading
from datetime import datetime
import os
import numpy as np
from scipy.spatial.transform import Rotation as R
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class JoystickControlNode(Node):
    def __init__(self):
        super().__init__('joystick_control_node')
        
        # Configuration
        self.host = "0.0.0.0"
        self.port_data = 5566  # Must use same port as robot_arm_web_server (data source fixed)
        
        # Load FT calibration data
        self.load_ft_calibration()
        
        # Current robot orientation for gravity compensation
        self.latest_pose = None
        self.controller = 'force'  # default mode
                
        # Joystick input values
        self.x_diff = 0.0
        self.y_diff = 0.0
        self.z_diff = 0.0
        self.lock_ampl = 50.0  # Amplification factor when z is locked
        # Force limits
        self.MAX_FORCE = 100.0  # Maximum force limit in N
        self.force_gain_z = 50  # Force mode gain for z
        self.force_gain_xy = self.MAX_FORCE  # Force mode gain for x,y

        # Store latest FT data for logging
        self.latest_raw_force = np.zeros(3)
        self.latest_raw_torque = np.zeros(3)
        self.latest_compensated_force = np.zeros(3)
        self.latest_compensated_torque = np.zeros(3)
        self.target_force = np.array([0.0, 0.0, 0.0])  # All zeros for target
        self.target_torque = np.zeros(3)  # All zeros for target
        
        # Setup log file
        self.setup_log_file()
        
        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()

        self.joy_sub = self.create_subscription(
            Joy, '/joy', self.joy_callback, 10)

        # Create publishers for force/torque data
        self.ft_sensor_publisher = self.create_publisher(
            WrenchStamped, 'ft_sensor_wrench', 10, callback_group=self.callback_group)
        self.target_wrench_publisher = self.create_publisher(
            WrenchStamped, 'target_wrench', 10, callback_group=self.callback_group)
        self.pose_subscriber_force = self.create_subscription(
            PoseStamped,
            '/cartesian_force_controller/current_pose',
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )
        # Create timer for 1-second logging
        self.log_timer = self.create_timer(
            0.05,  # 1 second
            self.log_ft_data,
            callback_group=self.callback_group
        )
        
        # Start UDP receiver
        self.start_udp_receiver()
        self.lock_z = 0

    def setup_log_file(self):
        """Setup log file for real-time data logging"""
        # Create logs directory if it doesn't exist
        log_dir = "/home/a/ws_heecheol/logs"
        os.makedirs(log_dir, exist_ok=True)
        
        # Create log filename with timestamp
        timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
        self.log_filename = os.path.join(log_dir, f"joystick_force_control_{timestamp}.txt")
        
        # Write header to log file
        with open(self.log_filename, 'w') as f:
            f.write("timestamp,")
            f.write("raw_fx,raw_fy,raw_fz,raw_tx,raw_ty,raw_tz,")
            f.write("comp_fx,comp_fy,comp_fz,comp_tx,comp_ty,comp_tz,")
            f.write("target_fx,target_fy,target_fz,target_tx,target_ty,target_tz,")
            f.write("pos_x,pos_y,pos_z,quat_x,quat_y,quat_z,quat_w,")
            f.write("joy_x,joy_y,joy_z,lock_z,compensation_type\n")
        
        self.get_logger().info(f'Log file created: {self.log_filename}')

    def publish_force_command(self):
        """Publish wrench command for force control"""
        wrench_msg = WrenchStamped()
        wrench_msg.header.stamp = self.get_clock().now().to_msg()
        wrench_msg.header.frame_id = 'base_link'
        
        # Apply force gains
        self.force_x = -self.x_diff * self.force_gain_xy
        self.force_y = self.y_diff * self.force_gain_xy
        self.force_z = -self.z_diff * self.force_gain_z
        
        # Limit individual forces to MAX_FORCE
        self.force_x = np.clip(self.force_x, -self.MAX_FORCE, self.MAX_FORCE)
        self.force_y = np.clip(self.force_y, -self.MAX_FORCE, self.MAX_FORCE)
        if self.lock_z > 0:
            self.force_z = self.lock_ampl * (self.latest_pose.pose.position.z - self.lock_z)
        else:
            self.force_z = np.clip(self.force_z, -self.MAX_FORCE, self.MAX_FORCE)
        
        wrench_msg.wrench.force.x = self.force_x
        wrench_msg.wrench.force.y = self.force_y
        wrench_msg.wrench.force.z = self.force_z
        
        # No torque commands from joystick
        wrench_msg.wrench.torque.x = 0.0
        wrench_msg.wrench.torque.y = 0.0
        wrench_msg.wrench.torque.z = 0.0
        self.target_force = [self.force_x, self.force_y, self.force_z]
        self.target_torque = [0.0, 0.0, 0.0]
        self.target_wrench_publisher.publish(wrench_msg)

    def load_ft_calibration(self):
        """Load FT calibration data from JSON file"""
        calib_file = './ft_calib_result.json'
        
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

    def joy_callback(self, msg):
        """Process joystick input"""
        # axes[0] -> y_diff, axes[1] -> x_diff
        if len(msg.axes) >= 2:
            self.y_diff = msg.axes[0]
            self.x_diff = msg.axes[1]
        
        # buttons[4] -> z_diff (+), buttons[6] -> z_diff (-)
        self.z_diff = 0.0
        if msg.buttons[4]:  # button 4 pressed
            self.z_diff = 1.0
        elif msg.buttons[6]:  # button 6 pressed
            self.z_diff = -1.0
        if msg.buttons[0]:
            self.lock_z = self.latest_pose.pose.position.z
        else:
            self.lock_z = 0

    def pose_callback(self, msg):
        self.latest_pose = msg

    def predict_gravity_wrench(self, qx, qy, qz, qw):
        """Predict gravity-induced forces and torques in sensor frame"""
        
        # Convert quaternion to rotation matrix using scipy
        rotation = R.from_quat([qx, qy, qz, qw])  # scipy uses [x, y, z, w] format
        R_b_s = rotation.as_matrix()
        
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
        
        compensation_type = "Full" if (self.latest_pose is not None) else "Bias-only"
        
        # Get pose data if available
        if self.latest_pose is not None:
            pos_x = self.latest_pose.pose.position.x
            pos_y = self.latest_pose.pose.position.y
            pos_z = self.latest_pose.pose.position.z
            quat_x = self.latest_pose.pose.orientation.x
            quat_y = self.latest_pose.pose.orientation.y
            quat_z = self.latest_pose.pose.orientation.z
            quat_w = self.latest_pose.pose.orientation.w
        else:
            pos_x = pos_y = pos_z = 0.0
            quat_x = quat_y = quat_z = quat_w = 0.0
        
        # Console logging with pose data
        self.get_logger().info(
            f'[{timestamp}] '
            f'Raw Force: [{self.latest_raw_force[0]:.3f}, {self.latest_raw_force[1]:.3f}, {self.latest_raw_force[2]:.3f}], '
            f'Compensated Force: [{self.latest_compensated_force[0]:.3f}, {self.latest_compensated_force[1]:.3f}, {self.latest_compensated_force[2]:.3f}], '
            f'Target Force: [{self.target_force[0]:.3f}, {self.target_force[1]:.3f}, {self.target_force[2]:.3f}], '
            f'Raw Torque: [{self.latest_raw_torque[0]:.3f}, {self.latest_raw_torque[1]:.3f}, {self.latest_raw_torque[2]:.3f}], '
            f'Compensated Torque: [{self.latest_compensated_torque[0]:.3f}, {self.latest_compensated_torque[1]:.3f}, {self.latest_compensated_torque[2]:.3f}], '
            f'Target Torque: [{self.target_torque[0]:.3f}, {self.target_torque[1]:.3f}, {self.target_torque[2]:.3f}], '
            f'Position: [{pos_x:.3f}, {pos_y:.3f}, {pos_z:.3f}], '
            f'Orientation: [{quat_x:.3f}, {quat_y:.3f}, {quat_z:.3f}, {quat_w:.3f}], '
            f'Comp: {compensation_type}'
        )
        
        # Write to log file
        try:
            with open(self.log_filename, 'a') as f:
                log_line = (
                    f"{timestamp},"
                    f"{self.latest_raw_force[0]:.6f},{self.latest_raw_force[1]:.6f},{self.latest_raw_force[2]:.6f},"
                    f"{self.latest_raw_torque[0]:.6f},{self.latest_raw_torque[1]:.6f},{self.latest_raw_torque[2]:.6f},"
                    f"{self.latest_compensated_force[0]:.6f},{self.latest_compensated_force[1]:.6f},{self.latest_compensated_force[2]:.6f},"
                    f"{self.latest_compensated_torque[0]:.6f},{self.latest_compensated_torque[1]:.6f},{self.latest_compensated_torque[2]:.6f},"
                    f"{self.target_force[0]:.6f},{self.target_force[1]:.6f},{self.target_force[2]:.6f},"
                    f"{self.target_torque[0]:.6f},{self.target_torque[1]:.6f},{self.target_torque[2]:.6f},"
                    f"{pos_x:.6f},{pos_y:.6f},{pos_z:.6f},"
                    f"{quat_x:.6f},{quat_y:.6f},{quat_z:.6f},{quat_w:.6f},"
                    f"{self.x_diff:.6f},{self.y_diff:.6f},{self.z_diff:.6f},{self.lock_z:.6f},"
                    f"{compensation_type}\n"
                )
                f.write(log_line)
        except Exception as e:
            self.get_logger().error(f'Failed to write to log file: {e}')

    def create_sensor_wrench_msg(self, ft_data):
        """Create WrenchStamped message from 6D FT sensor data with multi-stage filtering"""
        msg = WrenchStamped()
        
        # Set timestamp
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "ft_sensor_frame"        
        self.latest_raw_force = np.array([float(ft_data[0]), float(ft_data[1]), float(ft_data[2])])
        self.latest_raw_torque = np.array([float(ft_data[3]), float(ft_data[4]), float(ft_data[5])])
        
        # Step 1: Apply calibration compensation
        f_comp, t_comp = self.compensate_ft_data(
            float(ft_data[0]), float(ft_data[1]), float(ft_data[2]),
            float(ft_data[3]), float(ft_data[4]), float(ft_data[5])
        )
        
        # Store compensated force and torque data
        self.latest_compensated_force = f_comp
        self.latest_compensated_torque = t_comp

        # Set final filtered values to message
        msg.wrench.force.x = f_comp[0]
        msg.wrench.force.y = f_comp[1]
        msg.wrench.force.z = f_comp[2]
        msg.wrench.torque.x = t_comp[0]
        msg.wrench.torque.y = t_comp[1]
        msg.wrench.torque.z = t_comp[2]
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
                        self.publish_force_command()
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
