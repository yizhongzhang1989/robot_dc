from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='platform_controller',
            executable='platform_node',
            name='platform',
            output='screen',
            emulate_tty=True,
            parameters=[{'device_id': 32, 'use_ack_patch': 1}],
        ),
    ])
