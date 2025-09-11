#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from common import get_temp_directory


def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/robot_arm_camera/image_raw',
        description='Input image topic to subscribe to'
    )
    
    output_topic_arg = DeclareLaunchArgument(
        'output_topic', 
        default_value='/robot_arm_camera/image_undistorted',
        description='Output topic for undistorted images'
    )
    
    output_compressed_topic_arg = DeclareLaunchArgument(
        'output_compressed_topic',
        default_value='/robot_arm_camera/image_undistorted_jpeg',
        description='Output topic for compressed undistorted images, with format of JPEG'
    )
    
    output_resized_compressed_topic_arg = DeclareLaunchArgument(
        'output_resized_compressed_topic',
        default_value='/robot_arm_camera/image_undistorted_resize_jpeg',
        description='Output topic for resized and compressed undistorted images'
    )
    
    resize_width_arg = DeclareLaunchArgument(
        'resize_width',
        default_value='640',
        description='Width for resized image'
    )
    
    resize_height_arg = DeclareLaunchArgument(
        'resize_height',
        default_value='480',
        description='Height for resized image'
    )
    
    calibration_file_arg = DeclareLaunchArgument(
        'calibration_file',
        default_value=os.path.join(get_temp_directory(), 'calibration_result.json'),
        description='Path to camera calibration file'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8085',
        description='Port for web interface'
    )
    
    # Create the image process node
    image_process_node = Node(
        package='image_process',
        executable='image_process_node',
        name='image_process_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_topic': LaunchConfiguration('output_topic'),
            'output_compressed_topic': LaunchConfiguration('output_compressed_topic'),
            'output_resized_compressed_topic': LaunchConfiguration('output_resized_compressed_topic'),
            'resize_width': LaunchConfiguration('resize_width'),
            'resize_height': LaunchConfiguration('resize_height'),
            'calibration_file': LaunchConfiguration('calibration_file'),
        }],
        emulate_tty=True
    )
    
    # Create the web viewer node
    web_viewer_node = Node(
        package='image_process',
        executable='image_web_viewer_node',
        name='image_web_viewer_node',
        output='screen',
        parameters=[{
            'raw_topic': LaunchConfiguration('input_topic'),
            'processed_topic': LaunchConfiguration('output_topic'),
            'resized_compressed_topic': LaunchConfiguration('output_resized_compressed_topic'),
            'web_port': LaunchConfiguration('web_port'),
        }],
        emulate_tty=True
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_topic_arg,
        output_compressed_topic_arg,
        output_resized_compressed_topic_arg,
        resize_width_arg,
        resize_height_arg,
        calibration_file_arg,
        web_port_arg,
        image_process_node,
        web_viewer_node
    ])
