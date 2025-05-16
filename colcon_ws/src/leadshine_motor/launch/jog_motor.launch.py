from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="leadshine_motor",
            executable="jog_node",
            name="leadshine_motor_1",
            parameters=["config/motor1.yaml"]
        )
    ])
