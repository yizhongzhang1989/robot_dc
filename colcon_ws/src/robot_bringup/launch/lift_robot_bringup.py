from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import sys

# Add common package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', 'src', 'common'))


def generate_launch_description():
    # Try to load defaults from config file
    default_start_web = 'true'
    default_web_port = '8090'
    default_draw_wire_interval = '0.06'
    default_force_sensor_interval = '0.06'
    default_draw_wire_id = '51'
    default_force_right_id = '52'
    default_force_left_id = '53'
    timing = {
        'platform': 3.0,
        'draw_wire': 4.0,
        'force_sensor': 4.5,
        'force_sensor_2': 5.0,
        'web': 5.5
    }
    
    try:
        from common.config_manager import ConfigManager
        config = ConfigManager()
        
        default_start_web = 'true' if config.get('lift_robot.launch_flags.start_web', True) else 'false'
        default_web_port = str(config.get('lift_robot.web.port', 8090))
        default_draw_wire_interval = str(config.get('lift_robot.sensors.draw_wire.read_interval', 0.06))
        default_force_sensor_interval = str(config.get('lift_robot.sensors.force_sensor.read_interval', 0.06))
        default_draw_wire_id = str(config.get('lift_robot.device_ids.draw_wire_sensor', 51))
        default_force_right_id = str(config.get('lift_robot.device_ids.force_sensor_right', 52))
        default_force_left_id = str(config.get('lift_robot.device_ids.force_sensor_left', 53))
        
        # Load timing configuration
        if config.has('lift_robot.launch_timing'):
            timing['platform'] = config.get('lift_robot.launch_timing.platform_delay', 3.0)
            timing['draw_wire'] = config.get('lift_robot.launch_timing.draw_wire_delay', 4.0)
            timing['force_sensor'] = config.get('lift_robot.launch_timing.force_sensor_delay', 4.5)
            timing['force_sensor_2'] = config.get('lift_robot.launch_timing.force_sensor_2_delay', 5.0)
            timing['web'] = config.get('lift_robot.launch_timing.web_delay', 5.5)
        
        print(f"[lift_robot_bringup] Loaded config successfully")
    except Exception as e:
        print(f"[lift_robot_bringup] Could not load config: {e}")
        print(f"[lift_robot_bringup] Using default values")
    
    # Launch arguments
    start_web_arg = DeclareLaunchArgument(
        'start_web', default_value=default_start_web, description='Start lift_robot_web server')
    web_port_arg = DeclareLaunchArgument(
        'web_port', default_value=default_web_port, description='Port for lift_robot_web server')
    
    # Sensor read interval arguments (frequency configuration)
    draw_wire_interval_arg = DeclareLaunchArgument(
        'draw_wire_interval', default_value=default_draw_wire_interval, description='Draw wire sensor read interval (s, 0.06=~17Hz)')
    force_sensor_interval_arg = DeclareLaunchArgument(
        'force_sensor_interval', default_value=default_force_sensor_interval, description='Force sensor read interval (s, 0.06=~17Hz)')
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
        period=timing['platform'],
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(lift_robot_path))]
    )

    # Start cable sensor after modbus driver is ready
    cable_sensor_launch = TimerAction(
        period=timing['draw_wire'],
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(cable_sensor_path), launch_arguments={
            'device_id': default_draw_wire_id,
            'read_interval': LaunchConfiguration('draw_wire_interval')
        }.items())]
    )

    # Start force sensor (device_id=52, topic=/force_sensor) after cable sensor
    force_sensor_launch = TimerAction(
        period=timing['force_sensor'],
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(force_sensor_path),
            launch_arguments={
                'device_id': default_force_right_id,
                'topic_name': '/force_sensor',
                'node_name_suffix': 'right',
                'read_interval': LaunchConfiguration('force_sensor_interval')
            }.items()
        )]
    )

    # Start second force sensor (device_id=53, topic=/force_sensor_2) after first sensor
    force_sensor_2_launch = TimerAction(
        period=timing['force_sensor_2'],
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(force_sensor_path),
            launch_arguments={
                'device_id': default_force_left_id,
                'topic_name': '/force_sensor_2',
                'node_name_suffix': 'left',
                'read_interval': LaunchConfiguration('force_sensor_interval')
            }.items()
        )]
    )

    # Start web after sensors (optional)
    web_launch = TimerAction(
        period=timing['web'],
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
        draw_wire_interval_arg,
        force_sensor_interval_arg,
        modbus_launch,
        lift_robot_launch,
        cable_sensor_launch,
        force_sensor_launch,
        force_sensor_2_launch,
        web_launch,
    ])
