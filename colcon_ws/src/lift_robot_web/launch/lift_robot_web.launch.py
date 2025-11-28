from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # Web server node (handles web interface, publishes commands to queue)
        Node(
            package='lift_robot_web',
            executable='server',
            name='lift_robot_web_server',
            parameters=[{'port': 8090}, {'sensor_topic': '/draw_wire_sensor/data'}],
            output='screen'
        ),
        # Command processor node (polls queue at 50Hz, executes Actions)
        Node(
            package='lift_robot_web',
            executable='cmd_processor',
            name='lift_robot_cmd_processor',
            output='screen'
        )
    ])
