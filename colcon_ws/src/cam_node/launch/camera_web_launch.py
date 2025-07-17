from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, SetEnvironmentVariable

def generate_launch_description():
    motor_names = "motor1,motor2,motor17,motor18,platform"

    return LaunchDescription([
        # Set environment variable for motor names
        SetEnvironmentVariable(name='MOTOR_NAMES', value=motor_names),

        # Launch cam_node
        Node(
            package='cam_node',
            executable='cam_node',
            name='cam_node',
            output='screen',
            parameters=[]
        ),

        # Launch robot_web ROS bridge
        Node(
            package='robot_web',
            executable='ros_bridge',
            output='screen',
            name='ros_bridge_node',
            parameters=[{"motor_names": motor_names}]
        ),

        # Launch web server
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
