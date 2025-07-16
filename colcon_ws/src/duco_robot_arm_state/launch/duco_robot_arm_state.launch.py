#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='1',
        description='Device ID for the robot arm'
    )
    
    ip_arg = DeclareLaunchArgument(
        'ip',
        default_value='192.168.1.10',
        description='IP address of the robot'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='2001',
        description='Port for robot state monitoring'
    )

    # Robot state monitoring node
    robot_state_node = Node(
        package='duco_robot_arm_state',
        executable='duco_robot_arm_state_node',
        name='duco_robot_arm_state_node',
        parameters=[{
            'device_id': LaunchConfiguration('device_id'),
            'ip': LaunchConfiguration('ip'),
            'port': LaunchConfiguration('port'),
        }],
        output='screen'
    )

    return LaunchDescription([
        device_id_arg,
        ip_arg,
        port_arg,
        robot_state_node,
    ])
