from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='feetech_servo',
            executable='servo_node',
            name='motor17',
            output='screen',
            emulate_tty=True,
            parameters=[{'device_id': 17, 'use_ack_patch': 0}],
        ),
        Node(
            package='feetech_servo',
            executable='servo_node',
            name='motor18',
            output='screen',
            emulate_tty=True,
            parameters=[{'device_id': 18, 'use_ack_patch': 0}],
        ),
    ])
