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
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from common.config_manager import ConfigManager
from common.workspace_utils import get_workspace_root
import os


def generate_launch_description():
    """Generate launch description for camcalib_web service."""
    
    # Load configuration
    config = ConfigManager()
    service_config = config.get('services.camcalib_web')
    
    # Get paths
    workspace_root = get_workspace_root()
    app_path = os.path.join(workspace_root, 'scripts', 'ThirdParty', 
                            'camera_calibration_toolkit', 'web', 'app.py')
    
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
    
    # Define the web service process (direct Flask launch, no ROS node wrapper)
    web_process = ExecuteProcess(
        cmd=[
            'flask', '--app', app_path, 'run',
            '--host', host,
            '--port', port
        ],
        output='screen',
        name='camcalib_web',
        # Proper signal handling for clean shutdown
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
