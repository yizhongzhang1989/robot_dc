#!/usr/bin/env python3
"""
UR15 Beijing Bringup Launch File

This launch file dynamically loads and launches modules based on the
ur15.launch_modules configuration. Modules are launched sequentially
with specified delays and can be enabled/disabled via config.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from common.config_manager import ConfigManager


def generate_launch_description():
    # Load configuration
    config = ConfigManager()
    ur15_config = config.get_robot('ur15')
    launch_modules = ur15_config.get('launch_modules')
    
    # Build launch actions
    launch_actions = []
    
    for module in launch_modules:
        # Skip disabled modules
        if not module.get('enabled', True):
            continue
        
        package = module['package']
        launch_file = module['launch_file']
        delay = module.get('delay', 0.0)
        
        # Prepare launch arguments if this is ur15_web
        launch_arguments = {}
        if package == 'ur15_web' and launch_file == 'ur15_web_launch.py':
            # Explicitly pass web_port argument to ensure correct port is used
            launch_arguments = {
                'web_port': str(ur15_config.get('web.port')),
            }.items()
        
        # Create launch action for this module
        module_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource([
                PathJoinSubstitution([
                    FindPackageShare(package),
                    'launch',
                    launch_file
                ])
            ]),
            launch_arguments=launch_arguments
        )
        
        # Add delay if specified
        if delay > 0:
            module_launch = TimerAction(
                period=delay,
                actions=[module_launch]
            )
        
        launch_actions.append(module_launch)
    
    return LaunchDescription(launch_actions)
