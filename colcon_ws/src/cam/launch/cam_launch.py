from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='cam',
            executable='cam_node',
            name='cam_node',
            output='screen',
            emulate_tty=True,
        ),
    ]) 