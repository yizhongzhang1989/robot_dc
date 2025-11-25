#!/usr/bin/env python3
"""
Launch file for Camera Calibration Web Service

This launch file starts the camera calibration web service with configurable parameters.
The service provides a web interface for camera calibration.

Usage:
    ros2 launch camcalib_web_service camcalib_web_launch.py
    ros2 launch camcalib_web_service camcalib_web_launch.py port:=8006
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from common.config_manager import ConfigManager


def generate_launch_description():
    """Generate launch description for camcalib_web service."""
    
    # Load configuration
    config = ConfigManager()
    service_config = config.get('services.camcalib_web')
    
    # Declare launch arguments with defaults from config
    port_arg = DeclareLaunchArgument(
        'port',
        default_value=str(service_config.get('port', 8006)),
        description='Port for the web service'
    )
    
    host_arg = DeclareLaunchArgument(
        'host',
        default_value=service_config.get('host', '0.0.0.0'),
        description='Host address for the web service'
    )
    
    # Get launch configurations
    port = LaunchConfiguration('port')
    host = LaunchConfiguration('host')
    
    # Define the camcalib_web service node with proper signal handling
    camcalib_web_node = Node(
        package='camcalib_web_service',
        executable='camcalib_web_node',
        name='camcalib_web_service_node',
        output='screen',
        parameters=[{
            'port': port,
            'host': host
        }],
        emulate_tty=True,
        # Important: this ensures the node receives SIGINT/SIGTERM properly
        sigterm_timeout='5',
        sigkill_timeout='10'
    )
    
    return LaunchDescription([
        # Arguments
        port_arg,
        host_arg,
        
        # Nodes
        camcalib_web_node
    ])
