from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lift_robot_force_sensor',
            executable='force_sensor_node',
            name='lift_robot_force_sensor',
            output='screen',
            emulate_tty=True,
            parameters=[{'device_id': 52, 'use_ack_patch': True, 'read_interval': 0.1}],  # 10Hz
        ),
    ])
