#!/usr/bin/env python3
"""
Launch file for Robot Status Management System

This launches the robot_status_redis node and optionally the web dashboard.

Usage:
    ros2 launch robot_status_redis robot_status_launch.py
    ros2 launch robot_status_redis robot_status_launch.py web_enabled:=true web_port:=8005
"""

import os
import sys

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from ament_index_python.packages import get_package_prefix
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
    
    # Robot status node — launched as a Python module via the interpreter
    # rather than `launch_ros.actions.Node(executable='robot_status_node.py')`.
    # The Node action performs a libexec lookup that requires the script to
    # be marked +x; under `colcon build --symlink-install` the installed
    # path is a symlink to source, so its effective mode is whatever git
    # tracks for the source file. By invoking `python3 <abs_path>` we let
    # the Python interpreter open the script as a regular file — no +x
    # required — matching the pattern used by the other web services
    # (positioning_3d_service, image_labeling_service, camcalib_web_service).
    node_script = os.path.join(
        get_package_prefix('robot_status_redis'),
        'lib', 'robot_status_redis', 'robot_status_node.py',
    )

    node_cmd = [
        sys.executable,
        node_script,
        '--ros-args',
        '-r', '__node:=robot_status_node',
        '-p', ['auto_save_file_path:=', auto_save_file_path],
        '-p', f'redis_host:={service_config["redis"]["host"]}',
        '-p', f'redis_port:={int(service_config["redis"]["port"])}',
        '-p', f'redis_db:={int(service_config["redis"]["db"])}',
    ]
    # Only forward redis_password when actually set; the node declares an
    # empty-string default, and `key:=` with an empty value parses as YAML
    # null which would surprise the param consumer.
    if service_config['redis'].get('password'):
        node_cmd += ['-p', f'redis_password:={service_config["redis"]["password"]}']

    robot_status_node = ExecuteProcess(
        cmd=node_cmd,
        name='robot_status_node',
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='2',
        emulate_tty=True,
    )

    # Web dashboard process (optional)
    web_dashboard_process = ExecuteProcess(
        cmd=[
            sys.executable,
            '-m', 'robot_status_redis.web_dashboard',
            '--host', web_host,
            '--port', web_port
        ],
        output='screen',
        sigterm_timeout='2',
        sigkill_timeout='2',
        condition=IfCondition(web_enabled)
    )
    
    return LaunchDescription([
        # Arguments
        auto_save_file_path_arg,
        web_enabled_arg,
        web_port_arg,
        web_host_arg,
        
        # Nodes
        robot_status_node,
        web_dashboard_process
    ])
