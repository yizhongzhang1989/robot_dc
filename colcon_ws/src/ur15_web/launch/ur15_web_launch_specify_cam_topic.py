#!/usr/bin/env python3
"""
UR15 Web Launch File (Without Camera)

This launch file starts:
1. UR15 robot driver (ur_control.launch.py)
2. UR15 web node (ur15_web_node) - waits 5 seconds after robot driver

Note: This launch file does not start the camera node.
You should launch a camera separately if needed.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    # Declare arguments
    ur15_ip_arg = DeclareLaunchArgument(
        'ur15_ip',
        default_value='192.168.1.15',
        description='IP address of the UR15 robot'
    )
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value='/ur15_camera/image_raw',
        description='UR15 Camera topic name (should be published by external camera node)'
    )
    
    dataset_dir_arg = DeclareLaunchArgument(
        'dataset_dir',
        default_value=os.path.join(os.path.dirname(os.getcwd()), 'dataset'),
        description='Directory for storing dataset files'
    )
    
    calib_data_dir_arg = DeclareLaunchArgument(
        'calib_data_dir',
        default_value=os.path.join(os.path.dirname(os.getcwd()), 'temp', 'ur15_cam_calibration_data'),
        description='Directory for camera calibration data'
    )
    
    chessboard_config_arg = DeclareLaunchArgument(
        'chessboard_config',
        default_value=os.path.join(os.path.dirname(os.getcwd()), 'temp', 'ur15_cam_calibration_data', 'chessboard_config.json'),
        description='JSON file containing chessboard pattern configuration'
    )
    
    # Get launch configurations
    ur15_ip = LaunchConfiguration('ur15_ip')
    camera_topic = LaunchConfiguration('camera_topic')
    dataset_dir = LaunchConfiguration('dataset_dir')
    calib_data_dir = LaunchConfiguration('calib_data_dir')
    chessboard_config = LaunchConfiguration('chessboard_config')
    
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
            'ur_type': 'ur15',
            'robot_ip': ur15_ip,
            'launch_rviz': 'false'
        }.items()
    )
    
    # UR15 web node
    ur15_web_node = Node(
        package='ur15_web',
        executable='ur15_web_node',
        name='ur15_web_node',
        output='screen',
        parameters=[{
            'camera_topic': camera_topic,
            'web_port': 8030,
            'ur15_ip': ur15_ip,
            'ur15_port': 30002,
            'dataset_dir': dataset_dir,
            'calib_data_dir': calib_data_dir,
            'chessboard_config': chessboard_config
        }]
    )
    
    # Delay web node by 5 seconds after robot driver
    delayed_web_node = TimerAction(
        period=5.0,
        actions=[ur15_web_node]
    )
    
    return LaunchDescription([
        # Arguments
        ur15_ip_arg,
        camera_topic_arg,
        dataset_dir_arg,
        calib_data_dir_arg,
        chessboard_config_arg,
        
        # Sequential launch: robot -> web (no camera)
        ur_control_launch,          # Start immediately
        delayed_web_node            # Start after 5 seconds
    ])
