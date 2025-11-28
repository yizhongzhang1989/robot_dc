from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import os

# Add common package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src', 'common'))

def generate_launch_description():
    # Try to load from config file
    port = 8090
    sensor_topic = '/draw_wire_sensor/data'
    
    try:
        from common.config_manager import ConfigManager
        config = ConfigManager()
        port = config.get('lift_robot.web.port', 8090)
        sensor_topic = config.get('lift_robot.web.sensor_topic', '/draw_wire_sensor/data')
        # Note: 'host' in config is the server address (e.g., localhost, 192.168.1.3)
        # 'listen_host' is for server binding (e.g., 0.0.0.0)
        # The web server node uses port for binding, not the host address
        print(f"[lift_robot_web] Loaded from config: port={port}, sensor_topic={sensor_topic}")
    except Exception as e:
        print(f"[lift_robot_web] Could not load config: {e}")
        print(f"[lift_robot_web] Using defaults: port={port}, sensor_topic={sensor_topic}")
    
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
        )
    ])
