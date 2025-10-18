from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lift_robot_pushrod',
            executable='pushrod_node',
            name='lift_robot_pushrod',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': 53,
                'use_ack_patch': True
            }],
        ),
    ])
