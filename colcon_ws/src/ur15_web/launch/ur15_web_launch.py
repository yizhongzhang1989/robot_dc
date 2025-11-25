#!/usr/bin/env python3
"""
UR15 Web Launch File

This launch file starts only the UR15 web node.
Assumes ur_control and camera are already running.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from common.config_manager import ConfigManager
from pathlib import Path


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
    
    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value=ur15_config.get('web.camera_topic'),
        description='UR15 Camera topic name'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value=str(ur15_config.get('web.port')),
        description='Web server port'
    )
    
    ur15_port_arg = DeclareLaunchArgument(
        'ur15_port',
        default_value=str(ur15_config.get('robot.ports.control')),
        description='UR15 robot port'
    )
    
    dataset_dir_arg = DeclareLaunchArgument(
        'dataset_dir',
        default_value=ur15_config.get('paths.dataset'),
        description='Directory for storing dataset files'
    )
    
    calib_data_dir_arg = DeclareLaunchArgument(
        'calib_data_dir',
        default_value=ur15_config.get('paths.calibration_data'),
        description='Directory for camera calibration data'
    )
    
    chessboard_config_path = Path(ur15_config.get('paths.calibration_data_raw')) / 'chessboard_config.json'
    chessboard_config_arg = DeclareLaunchArgument(
        'chessboard_config',
        default_value=str(chessboard_config_path),
        description='JSON file containing chessboard pattern configuration'
    )
    
    # Get launch configurations
    ur15_ip = LaunchConfiguration('ur15_ip')
    camera_topic = LaunchConfiguration('camera_topic')
    web_port = LaunchConfiguration('web_port')
    ur15_port = LaunchConfiguration('ur15_port')
    dataset_dir = LaunchConfiguration('dataset_dir')
    calib_data_dir = LaunchConfiguration('calib_data_dir')
    chessboard_config = LaunchConfiguration('chessboard_config')
    
    # UR15 web node
    ur15_web_node = Node(
        package='ur15_web',
        executable='ur15_web_node',
        name='ur15_web_node',
        output='screen',
        parameters=[{
            'camera_topic': camera_topic,
            'web_port': web_port,
            'ur15_ip': ur15_ip,
            'ur15_port': ur15_port,
            'dataset_dir': dataset_dir,
            'calib_data_dir': calib_data_dir,
            'chessboard_config': chessboard_config
        }]
    )
    
    return LaunchDescription([
        # Arguments
        ur15_ip_arg,
        camera_topic_arg,
        web_port_arg,
        ur15_port_arg,
        dataset_dir_arg,
        calib_data_dir_arg,
        chessboard_config_arg,
        
        # Launch nodes
        ur15_web_node
    ])