#!/usr/bin/env python3

"""
Launch file for UR15 Robot Arm
This launch file starts both the UR robot driver and the UR15 robot arm node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Generate launch description for UR15 robot arm with driver
    """
    
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.15',
        description='IP address of the UR15 robot'
    )
    
    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value='ur15',
        description='Type of UR robot (ur3, ur5, ur10, ur15, ur20)'
    )
    
    # Get launch configurations
    robot_ip = LaunchConfiguration('robot_ip')
    ur_type = LaunchConfiguration('ur_type')
    
    # Include ur_robot_driver launch file
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
            'robot_ip': robot_ip,
        }.items()
    )
    
    # Create UR15 robot arm node
    ur15_robot_arm_node = Node(
        package='ur15_robot_arm',
        executable='ur15_robot_arm_node',
        name='ur15_robot_arm_node',
        output='screen',
        parameters=[{
            'robot_ip': robot_ip,
        }],
        emulate_tty=True
    )
    
    # Log info
    log_info = LogInfo(
        msg=['Starting UR15 Robot with ur_type: ', ur_type, ' and robot_ip: ', robot_ip]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        ur_type_arg,
        log_info,
        ur_control_launch,
        ur15_robot_arm_node,
    ])
