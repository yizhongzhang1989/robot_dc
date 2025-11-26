#!/usr/bin/env python3
"""
Launch file for 3D Positioning Triangulation Service

This launch file starts the positioning_3d web service with configurable parameters.
The service provides multi-view triangulation using FlowFormer++ for keypoint tracking.

Usage:
    ros2 launch positioning_3d_service positioning_3d_launch.py
    ros2 launch positioning_3d_service positioning_3d_launch.py ffpp_url:=http://192.168.1.100:8001
    ros2 launch positioning_3d_service positioning_3d_launch.py dataset_path:=/path/to/dataset
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from common.config_manager import ConfigManager
from common.workspace_utils import get_workspace_root
from pathlib import Path
import os


def generate_launch_description():
    """Generate launch description for positioning_3d service."""
    
    # Load configuration
    config = ConfigManager()
    service_config = config.get('services.positioning_3d')
    
    # Get paths
    workspace_root = get_workspace_root()
    app_path = os.path.join(workspace_root, 'scripts', 'ThirdParty', 'robot_vision',
                            'web', 'positioning_3d', 'app.py')
    
    # Resolve dataset path (handle both absolute and relative paths)
    dataset_path = service_config['dataset_path']
    dataset_path_obj = Path(dataset_path)
    if not dataset_path_obj.is_absolute():
        # Treat as relative path from workspace root
        workspace_root_path = Path(workspace_root)
        dataset_path = str(workspace_root_path / dataset_path)
    
    # Declare launch arguments with defaults from config
    ffpp_url_arg = DeclareLaunchArgument(
        'ffpp_url',
        default_value=service_config['ffpp_url'],
        description='FlowFormer++ server URL for keypoint tracking'
    )
    
    dataset_path_arg = DeclareLaunchArgument(
        'dataset_path',
        default_value=dataset_path,
        description='Path to dataset directory containing reference images (absolute or relative to repo root)'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value=str(service_config['port']),
        description='Port for the web service'
    )
    
    host_arg = DeclareLaunchArgument(
        'host',
        default_value=service_config['host'],
        description='Host address for the web service'
    )
    
    # Get launch configurations
    ffpp_url = LaunchConfiguration('ffpp_url')
    dataset_path_cfg = LaunchConfiguration('dataset_path')
    port = LaunchConfiguration('port')
    host = LaunchConfiguration('host')
    
    # Define the web service process (direct Python launch, no ROS node wrapper)
    web_process = ExecuteProcess(
        cmd=[
            'python3', app_path,
            '--ffpp-url', ffpp_url,
            '--dataset-path', dataset_path_cfg,
            '--host', host,
            '--port', port
        ],
        output='screen',
        name='positioning_3d_web',
        # Proper signal handling for clean shutdown
        sigterm_timeout='2',
        sigkill_timeout='2',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # Arguments
        ffpp_url_arg,
        dataset_path_arg,
        port_arg,
        host_arg,
        
        # Process
        web_process
    ])
