from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_arm_web',
            executable='robot_arm_web_server',
            name='robot_arm_web_server',
            parameters=[
                {'device_id': 1},
                {'port': 8080}
            ],
            output='screen',
            emulate_tty=True,
        )
    ])
