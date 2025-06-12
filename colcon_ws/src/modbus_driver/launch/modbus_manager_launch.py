from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='modbus_driver',
            executable='modbus_manager_node',
            name='modbus_manager_node',
            parameters=[
                {'port': '/dev/ttyUSB0'},
                {'baudrate': 115200}
            ],
            output='screen'
        )
    ])
