#!/usr/bin/env python3

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    # Declare arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Web server port'
    )
    
    urdf_file_arg = DeclareLaunchArgument(
        'urdf_file',
        default_value='',  # Empty string will trigger auto-discovery in the node
        description='Full path to the URDF file (leave empty for auto-discovery)'
    )
    
    # URDF Web Server Node
    urdf_web_server = Node(
        package='urdf_web_viewer',
        executable='urdf_web_server',
        name='urdf_web_server',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'urdf_file': LaunchConfiguration('urdf_file')
        }],
        output='screen'
    )
    
    return LaunchDescription([
        port_arg,
        urdf_file_arg,
        urdf_web_server
    ])
