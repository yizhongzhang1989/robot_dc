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
from launch.actions import DeclareLaunchArgument, ExecuteProcess, RegisterEventHandler
from launch.substitutions import LaunchConfiguration, TextSubstitution
from launch.event_handlers import OnProcessExit, OnShutdown
from launch.events import Shutdown
from common.config_manager import ConfigManager
from common.workspace_utils import get_workspace_root
import os
import signal


def generate_launch_description():
    """Generate launch description for image_labeling service."""
    
    # Load configuration
    config = ConfigManager()
    service_config = config.get('services.image_labeling')
    
    # Get paths
    workspace_root = get_workspace_root()
    launch_script = os.path.join(workspace_root, 'scripts', 'ThirdParty', 'robot_vision', 
                                  'ThirdParty', 'ImageLabelingWeb', 'launch_server.py')
    
    # Declare launch arguments with defaults from config
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
    port = LaunchConfiguration('port')
    host = LaunchConfiguration('host')
    
    # Define the web service process (direct Python launch, no ROS node wrapper)
    web_process = ExecuteProcess(
        cmd=[
            'python3', launch_script,
            '--host', host,
            '--port', port,
            '--no-browser'  # Don't open browser automatically
        ],
        output='screen',
        name='image_labeling_web',
        # Use process group for proper signal handling
        sigterm_timeout='2',
        sigkill_timeout='2',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # Arguments
        port_arg,
        host_arg,
        
        # Process
        web_process
    ])
