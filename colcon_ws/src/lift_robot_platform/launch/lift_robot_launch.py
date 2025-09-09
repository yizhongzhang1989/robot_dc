from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lift_robot_platform',
            executable='lift_robot_node',
            name='lift_robot_platform',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': 50,
                'use_ack_patch': True,
                'baudrate': 115200,
                'serial_port': '/dev/ttyLIFT_ROBOT'  # 使用固定的udev符号链接
            }],
        ),
    ])
