from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Single Camera Stream with Web Visualization
        Node(
            package='cam_node',
            executable='cam_node',
            name='cam_node_stream1',
            output='screen',
            parameters=[{
                'camera_name': 'stream1',
                'rtsp_url': 'rtsp://admin:123456@192.168.1.100/stream0',
                'camera_ip': '192.168.1.100',
                'stream_port': 8080,
                'stream_fps': 30,
                'jpeg_quality': 60,
                'max_width': 640
            }]
        ),
    ])
