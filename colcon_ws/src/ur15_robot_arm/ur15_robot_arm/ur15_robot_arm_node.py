#!/usr/bin/env python3

"""
UR15 Robot Arm Node
This node provides control interface for UR15 robot arm.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup


class UR15RobotArmNode(Node):
    def __init__(self):
        super().__init__('ur15_robot_arm_node')
        
        self.get_logger().info('UR15 Robot Arm Node initializing...')
        
        # Create callback group for thread safety
        self.callback_group = ReentrantCallbackGroup()
        
        # Robot state
        self.current_joint_states = None
        self.current_tcp_pose = None
        
        # Subscribe to joint states from UR driver
        self.joint_state_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.joint_state_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Subscribe to TCP pose (if available from ur_robot_driver)
        self.tcp_pose_sub = self.create_subscription(
            PoseStamped,
            '/cartesian_motion_controller/current_pose',
            self.tcp_pose_callback,
            10,
            callback_group=self.callback_group
        )
        
        # Publisher for target pose commands
        self.target_pose_pub = self.create_publisher(
            PoseStamped,
            '/target_frame',
            10,
            callback_group=self.callback_group
        )
        
        # Publisher for robot status
        self.status_pub = self.create_publisher(
            String,
            '/ur15_robot_arm/status',
            10,
            callback_group=self.callback_group
        )
        
        # Create timer for periodic status updates
        self.timer = self.create_timer(
            1.0,  # 1 Hz
            self.timer_callback,
            callback_group=self.callback_group
        )
        
        self.get_logger().info('UR15 Robot Arm Node started successfully')
        self.get_logger().info('Subscribed to: /joint_states, /cartesian_motion_controller/current_pose')
        self.get_logger().info('Publishing to: /target_frame, /ur15_robot_arm/status')
    
    def joint_state_callback(self, msg):
        """
        Callback for joint state updates from UR driver
        
        Args:
            msg: JointState message containing current joint positions, velocities, efforts
        """
        self.current_joint_states = msg
        # self.get_logger().debug(f'Received joint states: {msg.position}')
    
    def tcp_pose_callback(self, msg):
        """
        Callback for TCP pose updates
        
        Args:
            msg: PoseStamped message containing current TCP pose
        """
        self.current_tcp_pose = msg
        # self.get_logger().debug(f'Received TCP pose: {msg.pose.position}')
    
    def timer_callback(self):
        """
        Timer callback for periodic status updates
        """
        status_msg = String()
        
        if self.current_joint_states is not None:
            status_msg.data = f'Robot connected - Joint positions: {len(self.current_joint_states.position)} joints'
        else:
            status_msg.data = 'Waiting for robot connection...'
        
        self.status_pub.publish(status_msg)
    
    def publish_target_pose(self, pose_stamped):
        """
        Publish target pose command to robot controller
        
        Args:
            pose_stamped: PoseStamped message with target pose
        """
        self.target_pose_pub.publish(pose_stamped)
        self.get_logger().info(f'Published target pose: [{pose_stamped.pose.position.x:.3f}, '
                               f'{pose_stamped.pose.position.y:.3f}, '
                               f'{pose_stamped.pose.position.z:.3f}]')


def main(args=None):
    """
    Main function to initialize and run the UR15 robot arm node
    
    Args:
        args: Command line arguments
    """
    rclpy.init(args=args)
    
    try:
        ur15_node = UR15RobotArmNode()
        
        # Use multi-threaded executor to handle multiple callbacks concurrently
        executor = MultiThreadedExecutor()
        executor.add_node(ur15_node)
        
        try:
            ur15_node.get_logger().info('UR15 Robot Arm Node is running...')
            ur15_node.get_logger().info('Press Ctrl+C to stop.')
            executor.spin()
        except KeyboardInterrupt:
            ur15_node.get_logger().info('Shutting down UR15 Robot Arm Node...')
        finally:
            executor.shutdown()
            ur15_node.destroy_node()
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
