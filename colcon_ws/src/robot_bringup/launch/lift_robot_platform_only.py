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
    modbus_ready_delay_default = '3.0'
    web_port_default = '8090'
    sensor_topic_default = '/draw_wire_sensor/data'
    web_delay_default = '5.0'
    
    try:
        from common.config_manager import ConfigManager
        config = ConfigManager()
        if config.has('lift_robot.modbus_driver.ready_delay'):
            modbus_ready_delay_default = str(config.get('lift_robot.modbus_driver.ready_delay'))
        if config.has('lift_robot.web.port'):
            web_port_default = str(config.get('lift_robot.web.port'))
        if config.has('lift_robot.web.sensor_topic'):
            sensor_topic_default = config.get('lift_robot.web.sensor_topic')
        if config.has('lift_robot.web.launch_delay'):
            web_delay_default = str(config.get('lift_robot.web.launch_delay'))
        print(f"[lift_robot_platform_only] Loaded from config: ready_delay={modbus_ready_delay_default}, web_port={web_port_default}")
    except Exception as e:
        print(f"[lift_robot_platform_only] Could not load config: {e}, using defaults")
    
    # Launch arguments
    start_web_arg = DeclareLaunchArgument(
        'start_web', default_value='true', description='Start lift_robot_web server')
    web_port_arg = DeclareLaunchArgument(
        'web_port', default_value=web_port_default, description='Port for lift_robot_web server')
    sensor_topic_arg = DeclareLaunchArgument(
        'sensor_topic', default_value=sensor_topic_default, description='Draw wire sensor topic')
    modbus_ready_delay_arg = DeclareLaunchArgument(
        'modbus_ready_delay', default_value=modbus_ready_delay_default, description='Modbus driver ready delay (s)')
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
        PythonLaunchDescriptionSource(modbus_path)
    )

    # Start lift robot platform after modbus driver is ready
    lift_robot_launch = TimerAction(
        period=LaunchConfiguration('modbus_ready_delay'),
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(lift_robot_path))]
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
        start_web_arg,
        web_port_arg,
        sensor_topic_arg,
        modbus_ready_delay_arg,
        web_delay_arg,
        modbus_launch,
        lift_robot_launch,
        
        
        web_launch,
    ])
