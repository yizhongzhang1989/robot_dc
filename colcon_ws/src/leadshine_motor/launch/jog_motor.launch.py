from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='leadshine_motor',
            executable='jog_node',  # ✅ Not jog_node.py!
            name='leadshine_jogger',
            output='screen',
            parameters=['config/motor1.yaml']
        )
    ])
