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
    
    device_id = get_config_value('lift_robot.platform.device_id', 50)
    use_ack_patch = get_config_value('lift_robot.platform.use_ack_patch', True)
    
    print(f"[lift_robot_launch] Config: device_id={device_id}, use_ack_patch={use_ack_patch}")
    
    return LaunchDescription([
        Node(
            package='lift_robot_platform',
            executable='lift_robot_node_action',
            name='lift_robot_platform',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': device_id,
                'use_ack_patch': use_ack_patch
            }],
        ),
    ])
