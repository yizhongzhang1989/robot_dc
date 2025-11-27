from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Launch arguments
    start_web_arg = DeclareLaunchArgument(
        'start_web', default_value='true', description='Start lift_robot_web server')
    web_port_arg = DeclareLaunchArgument(
        'web_port', default_value='8090', description='Port for lift_robot_web server')
    
    # Paths to launch files
    modbus_path = os.path.join(
        get_package_share_directory('modbus_driver'),
        'launch',
        'modbus_manager_launch.py'
    )
    
    lift_robot_path = os.path.join(
        get_package_share_directory('lift_robot_platform'),
        'launch',
        'lift_robot_launch.py'
    )

    cable_sensor_path = os.path.join(
        get_package_share_directory('draw_wire_sensor'),
        'launch',
        'draw_wire_sensor.launch.py'
    )

    force_sensor_path = os.path.join(
        get_package_share_directory('lift_robot_force_sensor'),
        'launch',
        'lift_robot_force_sensor_launch.py'
    )

    web_path = os.path.join(
        get_package_share_directory('lift_robot_web'),
        'launch',
        'lift_robot_web.launch.py'
    )

    # Launch descriptions
    modbus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(modbus_path)
    )

    # Start lift robot platform after modbus driver is ready
    lift_robot_launch = TimerAction(
        period=3.0,  # Wait 3 seconds for modbus driver to initialize
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(lift_robot_path))]
    )

    # Start cable sensor after modbus driver is ready
    cable_sensor_launch = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(cable_sensor_path), launch_arguments={'device_id': '51', 'read_interval': '0.02'}.items())]
    )

    # Start force sensor (device_id=52, topic=/force_sensor) after cable sensor
    force_sensor_launch = TimerAction(
        period=4.5,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(force_sensor_path),
            launch_arguments={
                'device_id': '52',
                'topic_name': '/force_sensor',
                'node_name_suffix': 'right'
            }.items()
        )]
    )

    # Start web after sensors (optional)
    web_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_path),
            launch_arguments={
                'port': LaunchConfiguration('web_port'),
                'sensor_topic': '/draw_wire_sensor/data'
            }.items()
        )],
        condition=IfCondition(LaunchConfiguration('start_web'))
    )

    return LaunchDescription([
        start_web_arg,
        web_port_arg,
        modbus_launch,
        lift_robot_launch,
        cable_sensor_launch,
        force_sensor_launch,
        web_launch,
    ])
