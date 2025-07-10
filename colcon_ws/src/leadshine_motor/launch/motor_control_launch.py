from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leadshine_motor',
            executable='motor_node',
            name='motor1',
            output='screen',
            emulate_tty=True,
            parameters=[{'device_id': 1, 'use_ack_patch': 0}],
        ),
        Node(
            package='leadshine_motor',
            executable='motor_node',
            name='motor2',
            output='screen',
            emulate_tty=True,
            parameters=[{'device_id': 2, 'use_ack_patch': 0}],
        ),
    ])
