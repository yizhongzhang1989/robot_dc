#!/usr/bin/env python3
"""
UR15 Camera Launch File

This launch file starts only the UR15 camera node.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    # Declare arguments
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
    
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.1.101',
        description='IP address of the camera'
    )
    
    camera_name_arg = DeclareLaunchArgument(
        'camera_name',
        default_value='UR15Camera',
        description='Name of the camera'
    )
    
    server_port_arg = DeclareLaunchArgument(
        'server_port',
        default_value='8019',
        description='HTTP server port for camera streaming'
    )
    
    stream_fps_arg = DeclareLaunchArgument(
        'stream_fps',
        default_value='25',
        description='Camera stream FPS'
    )
    
    jpeg_quality_arg = DeclareLaunchArgument(
        'jpeg_quality',
        default_value='75',
        description='JPEG compression quality (0-100)'
    )
    
    max_width_arg = DeclareLaunchArgument(
        'max_width',
        default_value='800',
        description='Maximum width for camera image'
    )
    
    publish_ros_image_arg = DeclareLaunchArgument(
        'publish_ros_image',
        default_value='true',
        description='Whether to publish ROS image topic'
    )
    
    # Get launch configurations
    camera_topic = LaunchConfiguration('camera_topic')
    rtsp_url = LaunchConfiguration('rtsp_url')
    camera_ip = LaunchConfiguration('camera_ip')
    camera_name = LaunchConfiguration('camera_name')
    server_port = LaunchConfiguration('server_port')
    stream_fps = LaunchConfiguration('stream_fps')
    jpeg_quality = LaunchConfiguration('jpeg_quality')
    max_width = LaunchConfiguration('max_width')
    publish_ros_image = LaunchConfiguration('publish_ros_image')
    
    # UR15 camera node
    camera_node = Node(
        package='camera_node',
        executable='camera_node',
        name='ur15_camera_node',
        output='screen',
        parameters=[{
            'camera_name': camera_name,
            'rtsp_url_main': rtsp_url,
            'camera_ip': camera_ip,
            'server_port': server_port,
            'stream_fps': stream_fps,
            'jpeg_quality': jpeg_quality,
            'max_width': max_width,
            'publish_ros_image': publish_ros_image,
            'ros_topic_name': camera_topic
        }]
    )
    
    return LaunchDescription([
        # Arguments
        camera_topic_arg,
        rtsp_url_arg,
        camera_ip_arg,
        camera_name_arg,
        server_port_arg,
        stream_fps_arg,
        jpeg_quality_arg,
        max_width_arg,
        publish_ros_image_arg,
        
        # Launch camera node
        camera_node
    ])
