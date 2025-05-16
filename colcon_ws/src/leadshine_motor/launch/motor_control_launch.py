from launch import LaunchDescription
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import Node
import os

def generate_launch_description():
    config_file = os.path.join(
        get_package_share_directory('leadshine_motor'),
        'config',
        'motor1.yaml'
    )

    return LaunchDescription([
        Node(
            package='leadshine_motor',
            executable='motor_node',
            name='leadshine_motor_node',
            output='screen',
            parameters=[config_file],
        )
    ])
