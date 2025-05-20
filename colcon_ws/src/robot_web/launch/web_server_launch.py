# launch/robot_web_launch.py
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='robot_web',
            executable='ros_bridge',
            output='screen',
            name='ros_bridge_node'
        ),
        ExecuteProcess(
            cmd=['uvicorn', 'robot_web.web_server:app', '--host', '0.0.0.0', '--port', '8000'],
            output='screen'
        )
    ])
