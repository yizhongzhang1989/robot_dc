from launch import LaunchDescription
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
    
    port = get_config_value('lift_robot.web.port', 8090)
    sensor_topic = get_config_value('lift_robot.web.sensor_topic', '/draw_wire_sensor/data')
    # Note: 'host' in config is the server address (e.g., localhost, 192.168.1.3)
    # 'listen_host' is for server binding (e.g., 0.0.0.0)
    # The web server node uses port for binding, not the host address
    
    print(f"[lift_robot_web] Config: port={port}, sensor_topic={sensor_topic}")
    
    return LaunchDescription([
        # Web server node (handles web interface, publishes commands to queue)
        Node(
            package='lift_robot_web',
            executable='server',
            name='lift_robot_web_server',
            parameters=[{'port': port}, {'sensor_topic': sensor_topic}],
            output='screen'
        ),
        # Command processor node (polls queue at 50Hz, executes Actions)
        Node(
            package='lift_robot_web',
            executable='cmd_processor',
            name='lift_robot_cmd_processor',
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
