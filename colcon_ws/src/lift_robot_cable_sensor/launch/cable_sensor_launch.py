from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    read_interval_arg = DeclareLaunchArgument(
        'read_interval', default_value='0.1', description='Sensor read interval (seconds)')
    return LaunchDescription([
        read_interval_arg,
        Node(
            package='lift_robot_cable_sensor',
            executable='cable_sensor_node',
            name='cable_sensor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': 51,
                'use_ack_patch': True,
                'read_interval': LaunchConfiguration('read_interval')
            }],
        ),
    ])
