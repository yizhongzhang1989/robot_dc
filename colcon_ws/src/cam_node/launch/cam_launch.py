from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Camera 100 - Low latency settings
        Node(
            package='cam_node',
            executable='cam_node',
            name='cam_node_100',
            output='screen',
            parameters=[{
                'camera_name': 'cam100',
                'rtsp_url': 'rtsp://admin:123456@192.168.1.100/stream0',
                'camera_ip': '192.168.1.100',
                'stream_port': 8010,
                'stream_fps': 10,  # Lower FPS for less latency
                'jpeg_quality': 40,  # Lower quality for faster encoding
                'max_width': 480  # Smaller resolution for minimal latency
            }]
        ),
        # Camera 101 - Even lower settings for stability
        Node(
            package='cam_node',
            executable='cam_node',
            name='cam_node_101',
            output='screen',
            parameters=[{
                'camera_name': 'cam101',
                'rtsp_url': 'rtsp://admin:123456@192.168.1.101/stream0',
                'camera_ip': '192.168.1.101',
                'stream_port': 8011,
                'stream_fps': 10,  # Even lower FPS
                'jpeg_quality': 40,  # Lower quality
                'max_width': 480  # Even smaller for minimal latency
            }]
        ),
        # Camera 102 - Similar settings for consistency
        Node(
            package='cam_node',
            executable='cam_node',
            name='cam_node_102',
            output='screen',
            parameters=[{
                'camera_name': 'cam102',
                'rtsp_url': 'rtsp://admin:123456@192.168.1.102/stream0',
                'camera_ip': '192.168.1.102',
                'stream_port': 8012,
                'stream_fps': 10,  # Lower FPS for less latency
                'jpeg_quality': 40,  # Lower quality for faster encoding
                'max_width': 480  # Smaller resolution for minimal latency
            }]
        ),
    ])
