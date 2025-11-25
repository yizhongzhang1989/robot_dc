from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lift_robot_force_sensor_4channel',
            executable='force_sensor_node_4channel',
            name='lift_robot_force_sensor_4channel',
            output='screen',
            emulate_tty=True,
            parameters=[{'device_id': 60, 'use_ack_patch': True, 'read_interval': 0.02}],  # 50Hz, CH2 only
        ),
    ])