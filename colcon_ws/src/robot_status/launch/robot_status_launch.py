#!/usr/bin/env python3
"""
Launch file for Robot Status Management System

This launches the robot_status_node and optionally the web dashboard.

Usage:
    ros2 launch robot_status robot_status_launch.py
    ros2 launch robot_status robot_status_launch.py web_enabled:=true web_port:=8005
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from pathlib import Path
import os


def generate_launch_description():
    """Generate launch description for robot status system."""
    
    # Determine default auto-save path (temp/robot_status_auto_save.json in robot_dc root)
    try:
        current_dir = Path.cwd()
        robot_dc_root = None
        
        # Find robot_dc root directory
        for parent in [current_dir] + list(current_dir.parents):
            if parent.name == 'robot_dc':
                robot_dc_root = parent
                break
        
        if robot_dc_root is not None:
            default_save_path = str(robot_dc_root / 'temp' / 'robot_status_auto_save.json')
        else:
            default_save_path = 'robot_status_auto_save.json'
    except Exception:
        default_save_path = 'robot_status_auto_save.json'
    
    # Declare launch arguments
    auto_save_file_path_arg = DeclareLaunchArgument(
        'auto_save_file_path',
        default_value=default_save_path,
        description='Path to JSON file for auto-saving status'
    )
    
    web_enabled_arg = DeclareLaunchArgument(
        'web_enabled',
        default_value='true',
        description='Whether to start the web dashboard'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8005',
        description='Port for the web dashboard'
    )
    
    web_host_arg = DeclareLaunchArgument(
        'web_host',
        default_value='0.0.0.0',
        description='Host address for the web dashboard'
    )
    
    # Get launch configurations
    auto_save_file_path = LaunchConfiguration('auto_save_file_path')
    web_enabled = LaunchConfiguration('web_enabled')
    web_port = LaunchConfiguration('web_port')
    web_host = LaunchConfiguration('web_host')
    
    # Robot status node (always runs)
    robot_status_node = Node(
        package='robot_status',
        executable='robot_status_node.py',
        name='robot_status_node',
        output='screen',
        parameters=[{
            'auto_save_file_path': auto_save_file_path
        }],
        emulate_tty=True
    )
    
    # Web dashboard node (optional)
    web_dashboard_node = Node(
        package='robot_status',
        executable='web_dashboard.py',
        name='robot_status_web',
        output='screen',
        parameters=[{
            'port': web_port,
            'host': web_host
        }],
        condition=IfCondition(web_enabled),
        emulate_tty=True
    )
    
    return LaunchDescription([
        # Arguments
        auto_save_file_path_arg,
        web_enabled_arg,
        web_port_arg,
        web_host_arg,
        
        # Nodes
        robot_status_node,
        web_dashboard_node
    ])
