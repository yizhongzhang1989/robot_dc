#!/usr/bin/env python3

"""
Launch file for UR15 Robot Arm with Joystick
This launch file starts the UR robot driver, the UR15 robot arm node, and the joy node for joystick control.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    """
    Generate launch description for UR15 robot arm with driver and joystick
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
    
    joy_dev_arg = DeclareLaunchArgument(
        'joy_dev',
        default_value='/dev/input/js0',
        description='Joystick device path'
    )
    
    # Get launch configurations
    robot_ip = LaunchConfiguration('robot_ip')
    ur_type = LaunchConfiguration('ur_type')
    joy_dev = LaunchConfiguration('joy_dev')
    
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
    
    # Create joy node for joystick input
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[{
            'dev': joy_dev,
            'deadzone': 0.05,
            'autorepeat_rate': 20.0,
        }],
        emulate_tty=True
    )
    
    # Log info
    log_info = LogInfo(
        msg=['Starting UR15 Robot with joystick control - ur_type: ', ur_type, 
             ', robot_ip: ', robot_ip, ', joy_dev: ', joy_dev]
    )
    
    return LaunchDescription([
        robot_ip_arg,
        ur_type_arg,
        joy_dev_arg,
        log_info,
        ur_control_launch,
        ur15_robot_arm_node,
        joy_node,
    ])
