from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lift_robot_web',
            executable='server',
            name='lift_robot_web_server',
            parameters=[{'port': 8090}, {'sensor_topic': '/cable_sensor/data'}],
            output='screen'
        )
    ])
