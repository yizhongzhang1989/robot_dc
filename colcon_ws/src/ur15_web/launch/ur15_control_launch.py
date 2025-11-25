#!/usr/bin/env python3
"""
UR15 Control Launch File

This launch file starts only the UR15 robot driver (ur_control.launch.py).
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare
from common.config_manager import ConfigManager


def generate_launch_description():
    # Load configuration
    config = ConfigManager()
    ur15_config = config.get_robot('ur15')
    
    # Declare arguments with defaults from config
    ur15_ip_arg = DeclareLaunchArgument(
        'ur15_ip',
        default_value=ur15_config.get('robot.ip'),
        description='IP address of the UR15 robot'
    )
    
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value=ur15_config.get('robot.type'),
        description='Type of UR robot'
    )
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value=str(ur15_config.get('robot.launch_rviz')).lower(),
        description='Launch RViz for visualization'
    )
    
    # Get launch configurations
    ur15_ip = LaunchConfiguration('ur15_ip')
    ur_type = LaunchConfiguration('ur_type')
    launch_rviz = LaunchConfiguration('launch_rviz')
    
    # UR robot driver launch
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_driver'),
                'launch',
                'ur_control.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': ur15_ip,
            'launch_rviz': launch_rviz
        }.items()
    )
    
    return LaunchDescription([
        # Arguments
        ur15_ip_arg,
        ur_type_arg,
        launch_rviz_arg,
        
        # Launch robot driver
        ur_control_launch
    ])
