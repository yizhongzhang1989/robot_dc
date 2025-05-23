from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leadshine_motor',
            executable='motor_node',
            name='motor_1',
            output='screen',
            emulate_tty=True,
            parameters=[{'motor_id': 1}],
        ),
        Node(
            package='leadshine_motor',
            executable='motor_node',
            name='motor_2',
            output='screen',
            emulate_tty=True,
            parameters=[{'motor_id': 2}],
        ),
    ])
