from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='modbus_driver',
            executable='modbus_manager',
            name='modbus_manager',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 38400}
            ],
            output='screen'
        )
    ])
