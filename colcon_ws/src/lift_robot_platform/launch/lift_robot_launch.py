from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lift_robot_platform',
            executable='lift_robot_node_action',  # Changed to Action-based node
            name='lift_robot_platform',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': 50,
                'use_ack_patch': True
            }],
        ),
    ])
