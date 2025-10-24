#!/usr/bin/env python3
"""
UR15 Camera Calibration Validation Launch File

This launch file starts:
1. UR15 robot driver (ur_control.launch.py)
2. UR15 camera node (ur15_cam_launch.py)
3. UR15 camera validation node (ur15_cam_validate_node)
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare arguments
    ur15_ip_arg = DeclareLaunchArgument(
        'ur15_ip',
        default_value='192.168.1.15',
        description='IP address of the UR15 robot'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8030',
        description='Port for the web interface'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/ur15_camera/image_raw',
        description='Camera topic name'
    )
    
    # Get launch configurations
    ur15_ip = LaunchConfiguration('ur15_ip')
    web_port = LaunchConfiguration('web_port')
    camera_topic = LaunchConfiguration('camera_topic')
    
    # UR robot driver launch
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': 'ur15',
            'robot_ip': ur15_ip,
            'launch_rviz': 'false'
        }.items()
    )
    
    # UR15 camera launch
    ur15_cam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('camera_node'),
                'launch',
                'ur15_cam_launch.py'
            ])
        ])
    )
    
    # UR15 camera validation node
    ur15_cam_validate_node = Node(
        package='camera_node',
        executable='ur15_cam_validate_node',
        name='ur15_cam_validate_node',
        output='screen',
        parameters=[{
            'camera_topic': camera_topic,
            'web_port': web_port,
            'ur15_ip': ur15_ip,
            'ur15_port': 30002
        }]
    )
    
    return LaunchDescription([
        # Arguments
        ur15_ip_arg,
        web_port_arg,
        camera_topic_arg,
        
        # Nodes and launch files
        ur_control_launch,
        ur15_cam_launch,
        ur15_cam_validate_node
    ])
