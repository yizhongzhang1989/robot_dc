#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Get the package directory
    pkg_dir = get_package_share_directory('duco_ros_driver')
    
    # Path to the configuration file
    config_file = os.path.join(pkg_dir, 'config', 'duco_robot_control.yaml')
    
    # Declare launch arguments
    arm_num_arg = DeclareLaunchArgument(
        'arm_num',
        default_value='1',
        description='Number of robot arms'
    )
    
    arm_dof_arg = DeclareLaunchArgument(
        'arm_dof',
        default_value='6',
        description='Degrees of freedom per arm'
    )
    
    server_host_arg = DeclareLaunchArgument(
        'server_host_1',
        default_value='127.0.0.1',
        description='IP address of robot controller'
    )
    
    # DucoRobotControl node
    duco_robot_control_node = Node(
        package='duco_ros_driver',
        executable='DucoRobotControl',
        name='DucoRobotControl',
        output='screen',
        parameters=[config_file],
    )
    
    return LaunchDescription([
        arm_num_arg,
        arm_dof_arg,
        server_host_arg,
        duco_robot_control_node,
    ])
