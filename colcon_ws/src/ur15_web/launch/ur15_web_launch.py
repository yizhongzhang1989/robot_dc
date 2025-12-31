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
from common.workspace_utils import get_workspace_root
from pathlib import Path


def generate_launch_description():
    # Load configuration
    config = ConfigManager()
    ur15_config = config.get_robot('ur15')
    
    # Get workspace root for resolving relative paths
    workspace_root = Path(get_workspace_root())
    
    # Resolve paths (convert relative to absolute)
    dataset_path = ur15_config.get('web.dataset_path')
    dataset_path_obj = Path(dataset_path)
    if not dataset_path_obj.is_absolute():
        dataset_path = str(workspace_root / dataset_path)
    
    calib_data_path = ur15_config.get('web.calibration_data_path')
    calib_data_path_obj = Path(calib_data_path)
    if not calib_data_path_obj.is_absolute():
        calib_data_path = str(workspace_root / calib_data_path)
    
    calib_result_path = ur15_config.get('web.calibration_result_path')
    calib_result_path_obj = Path(calib_result_path)
    if not calib_result_path_obj.is_absolute():
        calib_result_path = str(workspace_root / calib_result_path)
    
    chessboard_config_path = ur15_config.get('web.chessboard_config_path')
    chessboard_config_path_obj = Path(chessboard_config_path)
    if not chessboard_config_path_obj.is_absolute():
        chessboard_config_path = str(workspace_root / chessboard_config_path)
    
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
        default_value=dataset_path,
        description='Directory for storing dataset files'
    )
    
    calib_data_dir_arg = DeclareLaunchArgument(
        'calib_data_dir',
        default_value=calib_data_path,
        description='Directory for camera calibration data'
    )
    
    calib_result_dir_arg = DeclareLaunchArgument(
        'calib_result_dir',
        default_value=calib_result_path,
        description='Directory for camera calibration results'
    )
    
    chessboard_config_arg = DeclareLaunchArgument(
        'chessboard_config',
        default_value=chessboard_config_path,
        description='JSON file containing chessboard pattern configuration'
    )
    
    # Get service ports from config
    all_config = config.get_all()
    services_config = all_config.get('services', {})
    image_labeling_port = services_config.get('image_labeling', {}).get('port', 8007)
    workflow_config_center_port = services_config.get('workflow_config_center', {}).get('port', 8008)
    
    image_labeling_port_arg = DeclareLaunchArgument(
        'image_labeling_port',
        default_value=str(image_labeling_port),
        description='Port for image labeling service'
    )
    
    workflow_config_center_port_arg = DeclareLaunchArgument(
        'workflow_config_center_port',
        default_value=str(workflow_config_center_port),
        description='Port for workflow config center service'
    )
    
    # Get launch configurations
    ur15_ip = LaunchConfiguration('ur15_ip')
    camera_topic = LaunchConfiguration('camera_topic')
    web_port = LaunchConfiguration('web_port')
    ur15_port = LaunchConfiguration('ur15_port')
    dataset_dir = LaunchConfiguration('dataset_dir')
    calib_data_dir = LaunchConfiguration('calib_data_dir')
    calib_result_dir = LaunchConfiguration('calib_result_dir')
    chessboard_config = LaunchConfiguration('chessboard_config')
    image_labeling_port = LaunchConfiguration('image_labeling_port')
    workflow_config_center_port = LaunchConfiguration('workflow_config_center_port')
    
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
            'calib_result_dir': calib_result_dir,
            'chessboard_config': chessboard_config,
            'image_labeling_port': image_labeling_port,
            'workflow_config_center_port': workflow_config_center_port
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
        calib_result_dir_arg,
        chessboard_config_arg,
        image_labeling_port_arg,
        workflow_config_center_port_arg,
        
        # Launch nodes
        ur15_web_node
    ])