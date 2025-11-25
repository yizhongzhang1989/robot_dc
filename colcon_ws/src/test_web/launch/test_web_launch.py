#!/usr/bin/env python3
"""
Launch file for Test Web Service

Usage:
    ros2 launch test_web test_web_launch.py
    ros2 launch test_web test_web_launch.py port:=8080
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for test_web service."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8006',
        description='Port for the web service'
    )
    
    host_arg = DeclareLaunchArgument(
        'host',
        default_value='0.0.0.0',
        description='Host address for the web service'
    )
    
    # Get launch configurations
    port = LaunchConfiguration('port')
    host = LaunchConfiguration('host')
    
    # Define the test_web service node
    test_web_node = Node(
        package='test_web',
        executable='test_web_node',
        name='test_web_node',
        output='screen',
        parameters=[{
            'port': port,
            'host': host
        }],
        emulate_tty=True
    )
    
    # Define the image_labeling service node
    image_labeling_node = Node(
        package='image_labeling_service',
        executable='image_labeling_node',
        name='image_labeling_service_node',
        output='screen',
        parameters=[{
            'port': 8007,
            'host': '0.0.0.0'
        }],
        emulate_tty=True
    )
    
    return LaunchDescription([
        # Arguments
        port_arg,
        host_arg,
        
        # Nodes
        test_web_node,
        image_labeling_node
    ])
