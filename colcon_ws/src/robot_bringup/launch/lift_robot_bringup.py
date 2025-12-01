from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os
import sys

# Add common package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src', 'common'))


def generate_launch_description():
    # Load defaults from config file
    force_right_device_id_default = '52'
    force_left_device_id_default = '53'
    try:
        from common.config_manager import ConfigManager
        config = ConfigManager()
        if config.has('lift_robot.force_sensor_right.device_id'):
            force_right_device_id_default = str(config.get('lift_robot.force_sensor_right.device_id'))
        if config.has('lift_robot.force_sensor_left.device_id'):
            force_left_device_id_default = str(config.get('lift_robot.force_sensor_left.device_id'))
        print(f"[lift_robot_bringup] Loaded device_id from config: right={force_right_device_id_default}, left={force_left_device_id_default}")
    except Exception as e:
        print(f"[lift_robot_bringup] Could not load device_id from config: {e}")
        print(f"[lift_robot_bringup] Using defaults: right={force_right_device_id_default}, left={force_left_device_id_default}")
    
    # Launch arguments - Modbus Driver
    modbus_port_arg = DeclareLaunchArgument(
        'modbus_port', default_value='auto', description='Modbus serial port (auto or /dev/ttyUSB0)')
    modbus_baudrate_arg = DeclareLaunchArgument(
        'modbus_baudrate', default_value='115200', description='Modbus baudrate')
    modbus_ready_delay_arg = DeclareLaunchArgument(
        'modbus_ready_delay', default_value='3.0', description='Wait time for modbus driver initialization (s)')
    enable_dashboard_arg = DeclareLaunchArgument(
        'enable_dashboard', default_value='false', description='Enable Modbus dashboard web interface')
    dashboard_host_arg = DeclareLaunchArgument(
        'dashboard_host', default_value='0.0.0.0', description='Dashboard host address')
    dashboard_port_arg = DeclareLaunchArgument(
        'dashboard_port', default_value='5000', description='Dashboard port')
    
    # Launch arguments - Platform
    platform_device_id_arg = DeclareLaunchArgument(
        'platform_device_id', default_value='50', description='Platform controller Modbus slave ID')
    platform_delay_arg = DeclareLaunchArgument(
        'platform_delay', default_value='3.0', description='Platform node launch delay (s)')
    
    # Launch arguments - Draw Wire Sensor
    draw_wire_device_id_arg = DeclareLaunchArgument(
        'draw_wire_device_id', default_value='51', description='Draw wire sensor Modbus slave ID')
    draw_wire_interval_arg = DeclareLaunchArgument(
        'draw_wire_interval', default_value='0.06', description='Draw wire sensor read interval (s, 0.06=~17Hz)')
    draw_wire_topic_arg = DeclareLaunchArgument(
        'draw_wire_topic', default_value='/draw_wire_sensor/data', description='Draw wire sensor topic')
    draw_wire_delay_arg = DeclareLaunchArgument(
        'draw_wire_delay', default_value='4.0', description='Draw wire sensor launch delay (s)')
    
    # Launch arguments - Force Sensor Right
    force_right_device_id_arg = DeclareLaunchArgument(
        'force_right_device_id', default_value=force_right_device_id_default, description='Right force sensor Modbus slave ID')
    force_right_interval_arg = DeclareLaunchArgument(
        'force_right_interval', default_value='0.06', description='Right force sensor read interval (s, 0.06=~17Hz)')
    force_right_topic_arg = DeclareLaunchArgument(
        'force_right_topic', default_value='/force_sensor_right', description='Right force sensor topic')
    force_right_delay_arg = DeclareLaunchArgument(
        'force_right_delay', default_value='4.5', description='Right force sensor launch delay (s)')
    
    # Launch arguments - Force Sensor Left
    force_left_device_id_arg = DeclareLaunchArgument(
        'force_left_device_id', default_value=force_left_device_id_default, description='Left force sensor Modbus slave ID')
    force_left_interval_arg = DeclareLaunchArgument(
        'force_left_interval', default_value='0.06', description='Left force sensor read interval (s, 0.06=~17Hz)')
    force_left_topic_arg = DeclareLaunchArgument(
        'force_left_topic', default_value='/force_sensor_left', description='Left force sensor topic')
    force_left_delay_arg = DeclareLaunchArgument(
        'force_left_delay', default_value='5.0', description='Left force sensor launch delay (s)')
    
    # Launch arguments - Web Interface
    web_enabled_arg = DeclareLaunchArgument(
        'web_enabled', default_value='true', description='Enable web interface')
    web_port_arg = DeclareLaunchArgument(
        'web_port', default_value='8090', description='Web server port')
    web_host_arg = DeclareLaunchArgument(
        'web_host', default_value='localhost', description='Web server host')
    web_listen_host_arg = DeclareLaunchArgument(
        'web_listen_host', default_value='0.0.0.0', description='Web server listen host')
    web_sensor_topic_arg = DeclareLaunchArgument(
        'web_sensor_topic', default_value='/draw_wire_sensor/data', description='Web sensor topic')
    web_delay_arg = DeclareLaunchArgument(
        'web_delay', default_value='5.5', description='Web interface launch delay (s)')
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
        PythonLaunchDescriptionSource(modbus_path),
        launch_arguments={
            'modbus_port': LaunchConfiguration('modbus_port'),
            'baudrate': LaunchConfiguration('modbus_baudrate'),
            'enable_dashboard': LaunchConfiguration('enable_dashboard'),
            'dashboard_host': LaunchConfiguration('dashboard_host'),
            'dashboard_port': LaunchConfiguration('dashboard_port')
        }.items()
    )

    # Start lift robot platform after modbus driver is ready
    lift_robot_launch = TimerAction(
        period=LaunchConfiguration('platform_delay'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(lift_robot_path),
            launch_arguments={
                'device_id': LaunchConfiguration('platform_device_id')
            }.items()
        )]
    )

    # Start draw wire sensor after modbus driver is ready
    draw_wire_sensor_launch = TimerAction(
        period=LaunchConfiguration('draw_wire_delay'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cable_sensor_path),
            launch_arguments={
                'device_id': LaunchConfiguration('draw_wire_device_id'),
                'read_interval': LaunchConfiguration('draw_wire_interval'),
                'topic': LaunchConfiguration('draw_wire_topic')
            }.items()
        )]
    )

    # Start right force sensor
    force_sensor_right_launch = TimerAction(
        period=LaunchConfiguration('force_right_delay'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(force_sensor_path),
            launch_arguments={
                'device_id': LaunchConfiguration('force_right_device_id'),
                'topic_name': LaunchConfiguration('force_right_topic'),
                'node_name_suffix': 'right',
                'read_interval': LaunchConfiguration('force_right_interval')
            }.items()
        )]
    )

    # Start left force sensor
    force_sensor_left_launch = TimerAction(
        period=LaunchConfiguration('force_left_delay'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(force_sensor_path),
            launch_arguments={
                'device_id': LaunchConfiguration('force_left_device_id'),
                'topic_name': LaunchConfiguration('force_left_topic'),
                'node_name_suffix': 'left',
                'read_interval': LaunchConfiguration('force_left_interval')
            }.items()
        )]
    )

    # Start web interface (optional)
    web_launch = TimerAction(
        period=LaunchConfiguration('web_delay'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_path),
            launch_arguments={
                'port': LaunchConfiguration('web_port'),
                'host': LaunchConfiguration('web_host'),
                'listen_host': LaunchConfiguration('web_listen_host'),
                'sensor_topic': LaunchConfiguration('web_sensor_topic')
            }.items()
        )],
        condition=IfCondition(LaunchConfiguration('web_enabled'))
    )

    return LaunchDescription([
        # Modbus driver arguments
        modbus_port_arg,
        modbus_baudrate_arg,
        modbus_ready_delay_arg,
        enable_dashboard_arg,
        dashboard_host_arg,
        dashboard_port_arg,
        # Platform arguments
        platform_device_id_arg,
        platform_delay_arg,
        # Draw wire sensor arguments
        draw_wire_device_id_arg,
        draw_wire_interval_arg,
        draw_wire_topic_arg,
        draw_wire_delay_arg,
        # Force sensor right arguments
        force_right_device_id_arg,
        force_right_interval_arg,
        force_right_topic_arg,
        force_right_delay_arg,
        # Force sensor left arguments
        force_left_device_id_arg,
        force_left_interval_arg,
        force_left_topic_arg,
        force_left_delay_arg,
        # Web interface arguments
        web_enabled_arg,
        web_port_arg,
        web_host_arg,
        web_listen_host_arg,
        web_sensor_topic_arg,
        web_delay_arg,
        # Launch actions
        modbus_launch,
        lift_robot_launch,
        draw_wire_sensor_launch,
        force_sensor_right_launch,
        force_sensor_left_launch,
        web_launch,
    ])
