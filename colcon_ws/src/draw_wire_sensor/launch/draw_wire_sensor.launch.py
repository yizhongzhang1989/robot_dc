from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

def generate_launch_description():
    device_id_arg = DeclareLaunchArgument('device_id', default_value='51', description='Modbus device ID')
    # Default changed to 0.02s (50Hz) for system-wide consistency
    read_interval_arg = DeclareLaunchArgument('read_interval', default_value='0.02', description='Sensor read interval (s, 0.02=50Hz)')
    return LaunchDescription([
        device_id_arg,
        read_interval_arg,
        Node(
            package='draw_wire_sensor',
            executable='draw_wire_sensor_node',
            name='draw_wire_sensor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'use_ack_patch': True,
                'read_interval': LaunchConfiguration('read_interval')
            }],
        )
    ])
