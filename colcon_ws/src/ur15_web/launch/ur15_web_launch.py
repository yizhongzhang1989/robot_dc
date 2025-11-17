#!/usr/bin/env python3
"""
UR15 Web Launch File

This launch file starts sequentially:
1. UR15 robot driver (ur_control.launch.py)
2. UR15 camera node (ur15_cam_launch.py) - waits 3 seconds after robot driver
3. UR15 web node (ur15_web_node) - waits 2 seconds after camera
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction, RegisterEventHandler
from launch.event_handlers import OnProcessStart
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
        description='UR15 Camera topic name'
    )
    
    rtsp_url_arg = DeclareLaunchArgument(
        'rtsp_url',
        default_value='rtsp://admin:123456@192.168.1.101/stream0',
        description='RTSP URL for camera stream'
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
    rtsp_url = LaunchConfiguration('rtsp_url')
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
    
    # UR15 camera node
    camera_node = Node(
        package='camera_node',
        executable='camera_node',
        name='ur15_camera_node',
        output='screen',
        parameters=[{
            'camera_name': 'UR15Camera',
            'rtsp_url_main': rtsp_url,
            'camera_ip': '192.168.1.101',
            'server_port': 8019,
            'stream_fps': 25,
            'jpeg_quality': 75,
            'max_width': 800,
            'publish_ros_image': True,
            'ros_topic_name': camera_topic
        }]
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
    
    # Delay camera node by 5 seconds after robot driver
    delayed_camera_node = TimerAction(
        period=5.0,
        actions=[camera_node]
    )
    
    # Delay web node by 8 seconds total (5s for robot + 3s for camera to initialize)
    delayed_web_node = TimerAction(
        period=8.0,
        actions=[ur15_web_node]
    )
    
    return LaunchDescription([
        # Arguments
        ur15_ip_arg,
        camera_topic_arg,
        rtsp_url_arg,
        dataset_dir_arg,
        calib_data_dir_arg,
        chessboard_config_arg,
        
        # Sequential launch: robot -> camera -> web
        ur_control_launch,          # Start immediately
        delayed_camera_node,         # Start after 3 seconds
        delayed_web_node            # Start after 5 seconds
    ])