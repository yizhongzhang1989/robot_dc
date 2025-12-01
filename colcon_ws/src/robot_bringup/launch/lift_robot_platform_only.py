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
    
    modbus_port_default = get_config_value('lift_robot.modbus_driver.port', 'auto')
    modbus_baudrate_default = get_config_value('lift_robot.modbus_driver.baudrate', '115200')
    platform_device_id_default = get_config_value('lift_robot.platform.device_id', '50')
    platform_delay_default = get_config_value('lift_robot.platform.launch_delay', '3.0')
    draw_wire_device_id_default = get_config_value('lift_robot.draw_wire_sensor.device_id', '51')
    draw_wire_interval_default = get_config_value('lift_robot.draw_wire_sensor.read_interval', '0.06')
    draw_wire_delay_default = get_config_value('lift_robot.draw_wire_sensor.launch_delay', '4.0')
    web_enabled_default = get_config_value('lift_robot.web.enabled', 'true')
    web_port_default = get_config_value('lift_robot.web.port', '8090')
    sensor_topic_default = get_config_value('lift_robot.web.sensor_topic', '/draw_wire_sensor/data')
    web_delay_default = get_config_value('lift_robot.web.launch_delay', '5.0')
    
    print(f"[lift_robot_platform_only] Loaded config defaults:")
    print(f"  Modbus: port={modbus_port_default}, baudrate={modbus_baudrate_default}")
    print(f"  Platform: device_id={platform_device_id_default}, delay={platform_delay_default}")
    print(f"  DrawWire: device_id={draw_wire_device_id_default}, interval={draw_wire_interval_default}, delay={draw_wire_delay_default}")
    print(f"  Web: port={web_port_default}, enabled={web_enabled_default}, delay={web_delay_default}")
    
    # Launch arguments - Modbus Driver
    modbus_port_arg = DeclareLaunchArgument(
        'modbus_port', default_value=modbus_port_default, description='Modbus serial port (auto or /dev/ttyUSB0)')
    modbus_baudrate_arg = DeclareLaunchArgument(
        'modbus_baudrate', default_value=modbus_baudrate_default, description='Modbus baudrate')
    
    # Launch arguments - Platform
    platform_device_id_arg = DeclareLaunchArgument(
        'platform_device_id', default_value=platform_device_id_default, description='Platform controller Modbus slave ID')
    platform_delay_arg = DeclareLaunchArgument(
        'platform_delay', default_value=platform_delay_default, description='Platform node launch delay (s)')
    
    # Launch arguments - Draw Wire Sensor
    draw_wire_device_id_arg = DeclareLaunchArgument(
        'draw_wire_device_id', default_value=draw_wire_device_id_default, description='Draw wire sensor Modbus slave ID')
    draw_wire_interval_arg = DeclareLaunchArgument(
        'draw_wire_interval', default_value=draw_wire_interval_default, description='Draw wire sensor read interval (s)')
    draw_wire_delay_arg = DeclareLaunchArgument(
        'draw_wire_delay', default_value=draw_wire_delay_default, description='Draw wire sensor launch delay (s)')
    
    # Launch arguments - Web Interface
    start_web_arg = DeclareLaunchArgument(
        'start_web', default_value=web_enabled_default, description='Start lift_robot_web server')
    web_port_arg = DeclareLaunchArgument(
        'web_port', default_value=web_port_default, description='Port for lift_robot_web server')
    sensor_topic_arg = DeclareLaunchArgument(
        'sensor_topic', default_value=sensor_topic_default, description='Draw wire sensor topic')
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
            'baudrate': LaunchConfiguration('modbus_baudrate')
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
                'topic': LaunchConfiguration('sensor_topic')
            }.items()
        )]
    )
    
    # Start web after sensors (optional)
    web_launch = TimerAction(
        period=LaunchConfiguration('web_delay'),
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(web_path),
            launch_arguments={
                'port': LaunchConfiguration('web_port'),
                'sensor_topic': LaunchConfiguration('sensor_topic')
            }.items()
        )],
        condition=IfCondition(LaunchConfiguration('start_web'))
    )

    return LaunchDescription([
        # Modbus driver arguments
        modbus_port_arg,
        modbus_baudrate_arg,
        # Platform arguments
        platform_device_id_arg,
        platform_delay_arg,
        # Draw wire sensor arguments
        draw_wire_device_id_arg,
        draw_wire_interval_arg,
        draw_wire_delay_arg,
        # Web interface arguments
        start_web_arg,
        web_port_arg,
        sensor_topic_arg,
        web_delay_arg,
        # Launch actions
        modbus_launch,
        lift_robot_launch,
        draw_wire_sensor_launch,
        web_launch,
    ])
