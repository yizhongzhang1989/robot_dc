#!/usr/bin/env python3
"""
Launch file for Image Labeling Service

This launch file starts the image labeling web service with configurable parameters.
The service provides a web interface for labeling images with keypoints.

Usage:
    ros2 launch image_labeling_service image_labeling_launch.py
    ros2 launch image_labeling_service image_labeling_launch.py port:=8003
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for image_labeling service."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8007',
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
    
    # Define the image_labeling service node
    image_labeling_node = Node(
        package='image_labeling_service',
        executable='image_labeling_node',
        name='image_labeling_service_node',
        output='screen',
        parameters=[{
            'port': port,
            'host': host
        }],
        emulate_tty=True
    )
    
    return LaunchDescription([
        # Arguments
        port_arg,
        host_arg,
        
        # Nodes
        image_labeling_node
    ])
