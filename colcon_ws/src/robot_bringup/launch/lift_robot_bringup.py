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
    def get_config_value(key, fallback):
        """Helper to safely get config value with fallback"""
        try:
            from common.config_manager import ConfigManager
            config = ConfigManager()
            if config.has(key):
                return str(config.get(key))
        except:
            pass
        return fallback
    
    # Modbus driver defaults
    modbus_port_default = get_config_value('lift_robot.modbus_driver.port', 'auto')
    modbus_baudrate_default = get_config_value('lift_robot.modbus_driver.baudrate', '115200')
    dashboard_enabled_default = get_config_value('lift_robot.modbus_driver.dashboard.enabled', 'false')
    dashboard_host_default = get_config_value('lift_robot.modbus_driver.dashboard.host', '0.0.0.0')
    dashboard_port_default = get_config_value('lift_robot.modbus_driver.dashboard.port', '5000')
    
    # Platform defaults
    platform_device_id_default = get_config_value('lift_robot.platform.device_id', '50')
    platform_delay_default = get_config_value('lift_robot.platform.launch_delay', '3.0')
    
    # Draw wire sensor defaults
    draw_wire_device_id_default = get_config_value('lift_robot.draw_wire_sensor.device_id', '51')
    draw_wire_interval_default = get_config_value('lift_robot.draw_wire_sensor.read_interval', '0.06')
    draw_wire_topic_default = get_config_value('lift_robot.draw_wire_sensor.topic', '/draw_wire_sensor/data')
    draw_wire_delay_default = get_config_value('lift_robot.draw_wire_sensor.launch_delay', '4.0')
    
    # Force sensor right defaults
    force_right_device_id_default = get_config_value('lift_robot.force_sensor_right.device_id', '52')
    force_right_interval_default = get_config_value('lift_robot.force_sensor_right.read_interval', '0.06')
    force_right_topic_default = get_config_value('lift_robot.force_sensor_right.topic', '/force_sensor_right')
    force_right_delay_default = get_config_value('lift_robot.force_sensor_right.launch_delay', '4.5')
    force_right_zero_drift_default = get_config_value('lift_robot.force_sensor_right.zero_drift_threshold', '65336')
    
    # Force sensor left defaults
    force_left_device_id_default = get_config_value('lift_robot.force_sensor_left.device_id', '53')
    force_left_interval_default = get_config_value('lift_robot.force_sensor_left.read_interval', '0.06')
    force_left_topic_default = get_config_value('lift_robot.force_sensor_left.topic', '/force_sensor_left')
    force_left_delay_default = get_config_value('lift_robot.force_sensor_left.launch_delay', '5.0')
    force_left_zero_drift_default = get_config_value('lift_robot.force_sensor_left.zero_drift_threshold', '65336')
    
    # Web interface defaults
    web_enabled_default = get_config_value('lift_robot.web.enabled', 'true')
    web_port_default = get_config_value('lift_robot.web.port', '8090')
    web_host_default = get_config_value('lift_robot.web.host', 'localhost')
    web_listen_host_default = get_config_value('lift_robot.web.listen_host', '0.0.0.0')
    web_sensor_topic_default = get_config_value('lift_robot.web.sensor_topic', '/draw_wire_sensor/data')
    web_delay_default = get_config_value('lift_robot.web.launch_delay', '5.5')
    
    print(f"[lift_robot_bringup] Loaded config defaults:")
    print(f"  Modbus: port={modbus_port_default}, baudrate={modbus_baudrate_default}")
    print(f"  Platform: device_id={platform_device_id_default}, delay={platform_delay_default}")
    print(f"  DrawWire: device_id={draw_wire_device_id_default}, interval={draw_wire_interval_default}, delay={draw_wire_delay_default}")
    print(f"  Force Right: device_id={force_right_device_id_default}, interval={force_right_interval_default}, delay={force_right_delay_default}")
    print(f"  Force Left: device_id={force_left_device_id_default}, interval={force_left_interval_default}, delay={force_left_delay_default}")
    print(f"  Web: port={web_port_default}, enabled={web_enabled_default}, delay={web_delay_default}")
    
    # Launch arguments - Modbus Driver
    modbus_port_arg = DeclareLaunchArgument(
        'modbus_port', default_value=modbus_port_default, description='Modbus serial port (auto or /dev/ttyUSB0)')
    modbus_baudrate_arg = DeclareLaunchArgument(
        'modbus_baudrate', default_value=modbus_baudrate_default, description='Modbus baudrate')
    enable_dashboard_arg = DeclareLaunchArgument(
        'enable_dashboard', default_value=dashboard_enabled_default, description='Enable Modbus dashboard web interface')
    dashboard_host_arg = DeclareLaunchArgument(
        'dashboard_host', default_value=dashboard_host_default, description='Dashboard host address')
    dashboard_port_arg = DeclareLaunchArgument(
        'dashboard_port', default_value=dashboard_port_default, description='Dashboard port')
    
    # Launch arguments - Platform
    platform_device_id_arg = DeclareLaunchArgument(
        'platform_device_id', default_value=platform_device_id_default, description='Platform controller Modbus slave ID')
    platform_delay_arg = DeclareLaunchArgument(
        'platform_delay', default_value=platform_delay_default, description='Platform node launch delay (s)')
    
    # Launch arguments - Draw Wire Sensor
    draw_wire_device_id_arg = DeclareLaunchArgument(
        'draw_wire_device_id', default_value=draw_wire_device_id_default, description='Draw wire sensor Modbus slave ID')
    draw_wire_interval_arg = DeclareLaunchArgument(
        'draw_wire_interval', default_value=draw_wire_interval_default, description='Draw wire sensor read interval (s, 0.06=~17Hz)')
    draw_wire_topic_arg = DeclareLaunchArgument(
        'draw_wire_topic', default_value=draw_wire_topic_default, description='Draw wire sensor topic')
    draw_wire_delay_arg = DeclareLaunchArgument(
        'draw_wire_delay', default_value=draw_wire_delay_default, description='Draw wire sensor launch delay (s)')
    
    # Launch arguments - Force Sensor Right
    force_right_device_id_arg = DeclareLaunchArgument(
        'force_right_device_id', default_value=force_right_device_id_default, description='Right force sensor Modbus slave ID')
    force_right_interval_arg = DeclareLaunchArgument(
        'force_right_interval', default_value=force_right_interval_default, description='Right force sensor read interval (s, 0.06=~17Hz)')
    force_right_topic_arg = DeclareLaunchArgument(
        'force_right_topic', default_value=force_right_topic_default, description='Right force sensor topic')
    force_right_delay_arg = DeclareLaunchArgument(
        'force_right_delay', default_value=force_right_delay_default, description='Right force sensor launch delay (s)')
    force_right_zero_drift_arg = DeclareLaunchArgument(
        'force_right_zero_drift', default_value=force_right_zero_drift_default, description='Right force sensor zero drift threshold')
    
    # Launch arguments - Force Sensor Left
    force_left_device_id_arg = DeclareLaunchArgument(
        'force_left_device_id', default_value=force_left_device_id_default, description='Left force sensor Modbus slave ID')
    force_left_interval_arg = DeclareLaunchArgument(
        'force_left_interval', default_value=force_left_interval_default, description='Left force sensor read interval (s, 0.06=~17Hz)')
    force_left_topic_arg = DeclareLaunchArgument(
        'force_left_topic', default_value=force_left_topic_default, description='Left force sensor topic')
    force_left_delay_arg = DeclareLaunchArgument(
        'force_left_delay', default_value=force_left_delay_default, description='Left force sensor launch delay (s)')
    force_left_zero_drift_arg = DeclareLaunchArgument(
        'force_left_zero_drift', default_value=force_left_zero_drift_default, description='Left force sensor zero drift threshold')
    
    # Launch arguments - Web Interface
    web_enabled_arg = DeclareLaunchArgument(
        'web_enabled', default_value=web_enabled_default, description='Enable web interface')
    web_port_arg = DeclareLaunchArgument(
        'web_port', default_value=web_port_default, description='Web server port')
    web_host_arg = DeclareLaunchArgument(
        'web_host', default_value=web_host_default, description='Web server host')
    web_listen_host_arg = DeclareLaunchArgument(
        'web_listen_host', default_value=web_listen_host_default, description='Web server listen host')
    web_sensor_topic_arg = DeclareLaunchArgument(
        'web_sensor_topic', default_value=web_sensor_topic_default, description='Web sensor topic')
    web_delay_arg = DeclareLaunchArgument(
        'web_delay', default_value=web_delay_default, description='Web interface launch delay (s)')
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
                'read_interval': LaunchConfiguration('force_right_interval'),
                'zero_drift_threshold': LaunchConfiguration('force_right_zero_drift')
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
                'read_interval': LaunchConfiguration('force_left_interval'),
                'zero_drift_threshold': LaunchConfiguration('force_left_zero_drift')
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
        force_right_zero_drift_arg,
        # Force sensor left arguments
        force_left_device_id_arg,
        force_left_interval_arg,
        force_left_topic_arg,
        force_left_delay_arg,
        force_left_zero_drift_arg,
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
