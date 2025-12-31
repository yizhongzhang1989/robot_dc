#!/usr/bin/env python3
"""
Launch file for Workflow Config Center

This launch file starts the workflow configuration center web service.
The service provides a web interface for managing workflow configuration files.

Usage:
    ros2 launch ur15_workflow workflow_config_center_launch.py
    ros2 launch ur15_workflow workflow_config_center_launch.py port:=8009
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from common.config_manager import ConfigManager
from common.workspace_utils import get_workspace_root
import os


def generate_launch_description():
    """Generate launch description for workflow config center service."""
    
    # Load configuration
    config = ConfigManager()
    service_config = config.get('services.workflow_config_center')
    ur15_config = config.get_robot('ur15')
    
    # Get paths
    workspace_root = get_workspace_root()
    launch_script = os.path.join(workspace_root, 'colcon_ws', 'src', 'ur15_workflow', 
                                  'web', 'workflow_api.py')
    
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
    
    ur15_ip_arg = DeclareLaunchArgument(
        'ur15_ip',
        default_value=ur15_config.get('robot.ip'),
        description='UR15 robot IP address'
    )
    
    ur15_port_arg = DeclareLaunchArgument(
        'ur15_port',
        default_value=str(ur15_config.get('robot.ports.control')),
        description='UR15 robot control port'
    )
    
    # Get launch configurations
    port = LaunchConfiguration('port')
    host = LaunchConfiguration('host')
    ur15_ip = LaunchConfiguration('ur15_ip')
    ur15_port = LaunchConfiguration('ur15_port')
    
    # Define the web service process (direct Python/Flask launch)
    web_process = ExecuteProcess(
        cmd=[
            'python3', launch_script,
            '--host', host,
            '--port', port,
            '--ur15-ip', ur15_ip,
            '--ur15-port', ur15_port
        ],
        output='screen',
        name='workflow_config_center_web',
        # Use process group for proper signal handling
        sigterm_timeout='2',
        sigkill_timeout='2',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        # Arguments
        port_arg,
        host_arg,
        ur15_ip_arg,
        ur15_port_arg,
        
        # Process
        web_process
    ])
