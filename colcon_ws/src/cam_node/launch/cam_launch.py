from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera 100
        Node(
            package='cam_node',
            executable='cam_node',
            name='cam_node_100',
            output='screen',
            parameters=[{
                'camera_name': 'cam100',
                'rtsp_url': 'rtsp://admin:123456@192.168.1.100/stream0',
                'camera_ip': '192.168.1.100'
            }]
        ),
        # Camera 101
        Node(
            package='cam_node',
            executable='cam_node',
            name='cam_node_101',
            output='screen',
            parameters=[{
                'camera_name': 'cam101',
                'rtsp_url': 'rtsp://admin:123456@192.168.1.101/stream0',
                'camera_ip': '192.168.1.101'
            }]
        ),
    ])
