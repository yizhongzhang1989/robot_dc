from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import sys
import os

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
                return config.get(key)
        except:
            pass
        return fallback
    
    port_default = get_config_value('lift_robot.web.port', 8090)
    sensor_topic_default = get_config_value('lift_robot.web.sensor_topic', '/draw_wire_sensor/data')
    server_id_default = get_config_value('lift_robot.web.server_id', 0)
    hybrid_high_base_default = get_config_value('lift_robot.web.hybrid_params.high_base', 133.7)
    hybrid_middle_base_default = get_config_value('lift_robot.web.hybrid_params.middle_base', 108.7)
    hybrid_low_base_default = get_config_value('lift_robot.web.hybrid_params.low_base', 113.7)
    hybrid_step_default = get_config_value('lift_robot.web.hybrid_params.step', 48.0)
    # Note: 'host' in config is the server address (e.g., localhost, 192.168.1.3)
    # 'listen_host' is for server binding (e.g., 0.0.0.0)
    # The web server node uses port for binding, not the host address
    
    print(f"[lift_robot_web] Config: port={port_default}, sensor_topic={sensor_topic_default}")
    print(f"[lift_robot_web] Hybrid: server_id={server_id_default}, high_base={hybrid_high_base_default},middle_base={hybrid_middle_base_default},low_base={hybrid_low_base_default}, step={hybrid_step_default}")
    
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value=str(port_default), description='Web server port'),
        DeclareLaunchArgument('host', default_value='localhost', description='Web server host'),
        DeclareLaunchArgument('listen_host', default_value='0.0.0.0', description='Web server listen host'),
        DeclareLaunchArgument('sensor_topic', default_value=str(sensor_topic_default), description='Sensor topic'),
        DeclareLaunchArgument('server_id', default_value=str(server_id_default), description='Server ID for hybrid control'),
        DeclareLaunchArgument('hybrid_high_base', default_value=str(hybrid_high_base_default), description='Hybrid high position base (mm)'),
        DeclareLaunchArgument('hybrid_middle_base', default_value=str(hybrid_middle_base_default), description='Hybrid middle position base (mm)'),
        DeclareLaunchArgument('hybrid_low_base', default_value=str(hybrid_low_base_default), description='Hybrid low position base (mm)'),
        DeclareLaunchArgument('hybrid_step', default_value=str(hybrid_step_default), description='Hybrid step per ID (mm)'),
        
        # Web server node (handles web interface, publishes commands to queue)
        Node(
            package='lift_robot_web',
            executable='server',
            name='lift_robot_web_server',
            parameters=[{
                'port': LaunchConfiguration('port'),
                'sensor_topic': LaunchConfiguration('sensor_topic'),
                'server_id': LaunchConfiguration('server_id'),
                'hybrid_high_base': LaunchConfiguration('hybrid_high_base'),
                'hybrid_middle_base': LaunchConfiguration('hybrid_middle_base'),
                'hybrid_low_base': LaunchConfiguration('hybrid_low_base'),
                'hybrid_step': LaunchConfiguration('hybrid_step')
            }],
            output='screen'
        ),
        # Command processor node (polls queue at 50Hz, executes Actions)
        Node(
            package='lift_robot_web',
            executable='cmd_processor',
            name='lift_robot_cmd_processor',
            output='screen'
        )
    ])
