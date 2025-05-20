from launch import LaunchDescription
from launch.actions import ExecuteProcess

def generate_launch_description():
    return LaunchDescription([
        ExecuteProcess(
            cmd=['uvicorn', 'robot_web.web_server:app', '--host', '0.0.0.0', '--port', '8000'],
            output='screen'
        )
    ])
