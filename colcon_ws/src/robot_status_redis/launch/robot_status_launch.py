#!/usr/bin/env python3
"""
Launch file for Robot Status Management System

This launches the robot_status_redis node and optionally the web dashboard.

Usage:
    ros2 launch robot_status_redis robot_status_launch.py
    ros2 launch robot_status_redis robot_status_launch.py web_enabled:=true web_port:=8005
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node
from common.config_manager import ConfigManager
from common.workspace_utils import get_workspace_root
from pathlib import Path


def generate_launch_description():
    """Generate launch description for robot status system."""
    
    # Load configuration
    config = ConfigManager()
    service_config = config.get('services.robot_status_redis')
    
    # Resolve auto_save_file_path (handle both absolute and relative paths)
    auto_save_path = service_config['auto_save_file_path']
    auto_save_path_obj = Path(auto_save_path)
    if not auto_save_path_obj.is_absolute():
        # Treat as relative path from workspace root
        workspace_root = Path(get_workspace_root())
        auto_save_path = str(workspace_root / auto_save_path)
    
    # Declare launch arguments with defaults from config
    auto_save_file_path_arg = DeclareLaunchArgument(
        'auto_save_file_path',
        default_value=auto_save_path,
        description='Path to JSON file for auto-saving status (absolute or relative to repo root)'
    )
    
    web_enabled_arg = DeclareLaunchArgument(
        'web_enabled',
        default_value=str(service_config['web']['enabled']).lower(),
        description='Whether to start the web dashboard'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value=str(service_config['web']['port']),
        description='Port for the web dashboard'
    )
    
    web_host_arg = DeclareLaunchArgument(
        'web_host',
        default_value=service_config['web']['host'],
        description='Host address for the web dashboard'
    )
    
    # Get launch configurations
    auto_save_file_path = LaunchConfiguration('auto_save_file_path')
    web_enabled = LaunchConfiguration('web_enabled')
    web_port = LaunchConfiguration('web_port')
    web_host = LaunchConfiguration('web_host')
    
    # Robot status node (always runs)
    robot_status_node = Node(
        package='robot_status_redis',
        executable='robot_status_node.py',
        name='robot_status_node',
        output='screen',
        parameters=[{
            'auto_save_file_path': auto_save_file_path,
            'redis_host': service_config['redis']['host'],
            'redis_port': service_config['redis']['port'],
            'redis_db': service_config['redis']['db'],
            'redis_password': service_config['redis']['password'] if service_config['redis']['password'] else ''
        }],
        emulate_tty=True
    )
    
    # Web dashboard node (optional)
    web_dashboard_node = Node(
        package='robot_status_redis',
        executable='web_dashboard_node.py',
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
