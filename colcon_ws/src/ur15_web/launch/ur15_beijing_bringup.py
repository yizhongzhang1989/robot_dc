#!/usr/bin/env python3
"""
UR15 Beijing Bringup Launch File

This launch file dynamically loads and launches modules based on the
ur15.launch_modules configuration. Modules are launched sequentially
with specified delays and can be enabled/disabled via config.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction
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
                
        # Create launch action using ExecuteProcess to launch as if run directly
        # This ensures the launch file evaluates its own DeclareLaunchArgument defaults
        module_launch = ExecuteProcess(
            cmd=['ros2', 'launch', package, launch_file],
            output='screen',
            shell=False
        )
        
        # Add delay if specified
        if delay > 0:
            module_launch = TimerAction(
                period=delay,
                actions=[module_launch]
            )
        
        launch_actions.append(module_launch)
    
    return LaunchDescription(launch_actions)
