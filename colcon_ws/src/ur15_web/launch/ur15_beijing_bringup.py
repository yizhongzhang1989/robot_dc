#!/usr/bin/env python3
"""
UR15 Beijing Bringup Launch File

This launch file starts the complete UR15 system sequentially:
0. Robot Status service - starts immediately (first)
1. UR15 robot driver (ur_control) - starts immediately
2. UR15 camera node - waits 5 seconds after robot driver
3. UR15 web node - waits 8 seconds total (5s for robot + 3s for camera)

This is the main launch file for bringing up the entire UR15 system in Beijing.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
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
    
    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value='false',
        description='Launch RViz for visualization'
    )
    
    # Get launch configurations
    ur15_ip = LaunchConfiguration('ur15_ip')
    camera_topic = LaunchConfiguration('camera_topic')
    rtsp_url = LaunchConfiguration('rtsp_url')
    dataset_dir = LaunchConfiguration('dataset_dir')
    calib_data_dir = LaunchConfiguration('calib_data_dir')
    chessboard_config = LaunchConfiguration('chessboard_config')
    launch_rviz = LaunchConfiguration('launch_rviz')
    
    # 0. Robot Status launch (first)
    robot_status_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('robot_status_redis'),
                'launch',
                'robot_status_launch.py'
            ])
        ])
    )
    
    # 1. UR15 robot control launch
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur15_web'),
                'launch',
                'ur15_control_launch.py'
            ])
        ]),
        launch_arguments={
            'ur15_ip': ur15_ip,
            'launch_rviz': launch_rviz
        }.items()
    )
    
    # 2. UR15 camera launch
    camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('camera_node'),
                'launch',
                'ur15_cam_launch.py'
            ])
        ]),
        launch_arguments={
            'ros_topic_name': camera_topic,
            'rtsp_url_main': rtsp_url,
            'camera_name': 'UR15Camera',
            'server_port': '8019'
        }.items()
    )
    
    # 3. UR15 web launch
    web_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur15_web'),
                'launch',
                'ur15_web_launch.py'
            ])
        ]),
        launch_arguments={
            'ur15_ip': ur15_ip,
            'camera_topic': camera_topic,
            'web_port': '8030',
            'dataset_dir': dataset_dir,
            'calib_data_dir': calib_data_dir,
            'chessboard_config': chessboard_config
        }.items()
    )
    
    # Delay camera launch by 5 seconds after robot driver
    delayed_camera_launch = TimerAction(
        period=5.0,
        actions=[camera_launch]
    )
    
    # Delay web launch by 8 seconds total (5s for robot + 3s for camera to initialize)
    delayed_web_launch = TimerAction(
        period=8.0,
        actions=[web_launch]
    )
    
    # 4. Positioning 3D service launch
    positioning_3d_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('positioning_3d_service'),
                'launch',
                'positioning_3d_launch.py'
            ])
        ]),
        launch_arguments={
            'port': '8004'
        }.items()
    )
    
    # Delay positioning service by 10 seconds (after web is ready)
    delayed_positioning_launch = TimerAction(
        period=10.0,
        actions=[positioning_3d_launch]
    )
    
    return LaunchDescription([
        # Arguments
        ur15_ip_arg,
        camera_topic_arg,
        rtsp_url_arg,
        dataset_dir_arg,
        calib_data_dir_arg,
        chessboard_config_arg,
        launch_rviz_arg,
        
        # Sequential launch: robot_status -> control -> camera -> web -> positioning
        robot_status_launch,        # Start first (immediately)
        ur_control_launch,          # Start immediately
        delayed_camera_launch,      # Start after 5 seconds
        delayed_web_launch,         # Start after 8 seconds
        delayed_positioning_launch  # Start after 10 seconds
    ])
