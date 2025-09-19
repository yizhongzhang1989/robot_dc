#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    input_topic_arg = DeclareLaunchArgument(
        'input_topic',
        default_value='/robot_arm_camera/image_raw',
        description='Input image topic to subscribe to'
    )
    
    output_resized_topic_arg = DeclareLaunchArgument(
        'output_resized_topic', 
        default_value='/robot_arm_camera/image_resized',
        description='Output topic for resized images'
    )
    
    resize_width_arg = DeclareLaunchArgument(
        'resize_width',
        default_value='640',
        description='Width for resized image'
    )
    
    resize_height_arg = DeclareLaunchArgument(
        'resize_height',
        default_value='0',
        description='Height for resized image (0 means keep aspect ratio)'
    )
    
    # Create the image process temp node
    image_process_temp_node = Node(
        package='image_process_temp',
        executable='image_process_temp_node',
        name='image_process_temp_node',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('input_topic'),
            'output_resized_topic': LaunchConfiguration('output_resized_topic'),
            'resize_width': LaunchConfiguration('resize_width'),
            'resize_height': LaunchConfiguration('resize_height'),
        }],
        emulate_tty=True
    )
    
    return LaunchDescription([
        input_topic_arg,
        output_resized_topic_arg,
        resize_width_arg,
        resize_height_arg,
        image_process_temp_node
    ])
