from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    motor_names = "motor1,motor2,motor17,motor18"

    return LaunchDescription([
        SetEnvironmentVariable(name='MOTOR_NAMES', value=motor_names),

        Node(
            package='robot_web',
            executable='ros_bridge',
            output='screen',
            name='ros_bridge_node',
            parameters=[{"motor_names": motor_names}]
        ),

        ExecuteProcess(
            cmd=[
                'uvicorn', 'robot_web.web_server:app',
                '--host', '0.0.0.0',
                '--port', '8000',
                '--log-level=warning'
            ],
            output='screen'
        )
    ])
