from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lift_robot_cable_sensor',
            executable='cable_sensor_node',
            name='cable_sensor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': 51,
                'use_ack_patch': True,
                'read_interval': 1.0  # 每秒读取一次
            }],
        ),
    ])
