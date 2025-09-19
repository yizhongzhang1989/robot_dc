#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import WrenchStamped, PoseStamped
from sensor_msgs.msg import JointState
import json
import threading
import os
from datetime import datetime
import time
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class CalibrationDataRecorder(Node):
    def __init__(self):
        super().__init__('calibration_data_recorder')
        
        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Data storage
        self.latest_pose = None
        self.latest_ft_data = None
        self.latest_joint_states = None
        
        # Timer settings
        self.save_interval = 5  # seconds
        self.countdown_interval = 1  # seconds
        
        # Create ./calib directory if it doesn't exist
        self.calib_dir = "./calib"
        os.makedirs(self.calib_dir, exist_ok=True)
        
        # Subscribers
        self.pose_subscriber = self.create_subscription(
            PoseStamped,
            '/cartesian_motion_controller/current_pose',  # Adjust topic name as needed
            self.pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        self.ft_subscriber = self.create_subscription(
            WrenchStamped,
            '/ft_sensor_wrench',
            self.ft_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Also subscribe to joint states for additional context
        self.joint_subscriber = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Start auto-save timer thread
        self.start_auto_save_timer()
        
        self.get_logger().info('Calibration Data Recorder started')
        self.get_logger().info(f'Saving calibration data to: {self.calib_dir}')
        self.get_logger().info(f'Auto-saving every {self.save_interval} seconds')
        self.get_logger().info('Press Ctrl+C to exit')
    
    def pose_callback(self, msg):
        """Store latest pose data"""
        self.latest_pose = msg
        
    def ft_callback(self, msg):
        """Store latest FT sensor data"""
        self.latest_ft_data = msg
        
    def joint_callback(self, msg):
        """Store latest joint states"""
        self.latest_joint_states = msg
    
    def start_auto_save_timer(self):
        """Start auto-save timer thread"""
        timer_thread = threading.Thread(
            target=self.auto_save_timer,
            daemon=True,
            name="auto_save_timer"
        )
        timer_thread.start()
    
    def auto_save_timer(self):
        """Auto-save timer with countdown display"""
        self.get_logger().info('Auto-save timer started')
        
        while rclpy.ok():
            try:
                # Countdown from save_interval to 0
                for remaining in range(self.save_interval, 0, -1):
                    if not rclpy.ok():
                        break
                    self.get_logger().info(f'Next calibration save in {remaining} seconds...')
                    time.sleep(self.countdown_interval)
                
                if rclpy.ok():
                    self.record_calibration_data()
            except Exception as e:
                if rclpy.ok():
                    self.get_logger().error(f'Error in auto-save timer: {e}')
                break
    
    def record_calibration_data(self):
        """Record current pose and FT sensor data to JSON file"""
        if self.latest_pose is None:
            self.get_logger().warning('No pose data available yet - skipping save')
            return
            
        if self.latest_ft_data is None:
            self.get_logger().warning('No FT sensor data available yet - skipping save')
            return
        
        # Create timestamp for filename
        timestamp = datetime.now()
        filename = f"calib_data_{timestamp.strftime('%Y%m%d_%H%M%S_%f')[:-3]}.json"
        filepath = os.path.join(self.calib_dir, filename)
        
        # Prepare data structure
        calibration_data = {
            "timestamp": timestamp.isoformat(),
            "pose": {
                "header": {
                    "stamp": {
                        "sec": self.latest_pose.header.stamp.sec,
                        "nanosec": self.latest_pose.header.stamp.nanosec
                    },
                    "frame_id": self.latest_pose.header.frame_id
                },
                "position": {
                    "x": self.latest_pose.pose.position.x,
                    "y": self.latest_pose.pose.position.y,
                    "z": self.latest_pose.pose.position.z
                },
                "orientation": {
                    "x": self.latest_pose.pose.orientation.x,
                    "y": self.latest_pose.pose.orientation.y,
                    "z": self.latest_pose.pose.orientation.z,
                    "w": self.latest_pose.pose.orientation.w
                }
            },
            "ft_sensor": {
                "header": {
                    "stamp": {
                        "sec": self.latest_ft_data.header.stamp.sec,
                        "nanosec": self.latest_ft_data.header.stamp.nanosec
                    },
                    "frame_id": self.latest_ft_data.header.frame_id
                },
                "force": {
                    "x": self.latest_ft_data.wrench.force.x,
                    "y": self.latest_ft_data.wrench.force.y,
                    "z": self.latest_ft_data.wrench.force.z
                },
                "torque": {
                    "x": self.latest_ft_data.wrench.torque.x,
                    "y": self.latest_ft_data.wrench.torque.y,
                    "z": self.latest_ft_data.wrench.torque.z
                }
            }
        }
        
        # Add joint states if available
        if self.latest_joint_states is not None:
            calibration_data["joint_states"] = {
                "header": {
                    "stamp": {
                        "sec": self.latest_joint_states.header.stamp.sec,
                        "nanosec": self.latest_joint_states.header.stamp.nanosec
                    },
                    "frame_id": self.latest_joint_states.header.frame_id
                },
                "name": self.latest_joint_states.name,
                "position": list(self.latest_joint_states.position),
                "velocity": list(self.latest_joint_states.velocity) if self.latest_joint_states.velocity else [],
                "effort": list(self.latest_joint_states.effort) if self.latest_joint_states.effort else []
            }
        
        # Save to JSON file
        try:
            with open(filepath, 'w') as f:
                json.dump(calibration_data, f, indent=2)
            
            self.get_logger().info(f'*** CALIBRATION DATA SAVED: {filename} ***')
            self.get_logger().info(f'Pose: [{self.latest_pose.pose.position.x:.3f}, {self.latest_pose.pose.position.y:.3f}, {self.latest_pose.pose.position.z:.3f}]')
            self.get_logger().info(f'FT Force: [{self.latest_ft_data.wrench.force.x:.3f}, {self.latest_ft_data.wrench.force.y:.3f}, {self.latest_ft_data.wrench.force.z:.3f}]')
            
        except Exception as e:
            self.get_logger().error(f'Failed to save calibration data: {e}')


def main(args=None):
    rclpy.init(args=args)
    
    try:
        recorder_node = CalibrationDataRecorder()
        
        # Use multi-threaded executor to handle multiple callbacks
        executor = MultiThreadedExecutor()
        executor.add_node(recorder_node)
        
        try:
            executor.spin()
        except KeyboardInterrupt:
            recorder_node.get_logger().info('Shutting down calibration data recorder...')
        finally:
            executor.shutdown()
            recorder_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
