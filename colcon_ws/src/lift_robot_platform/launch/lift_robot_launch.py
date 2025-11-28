from launch import LaunchDescription
from launch_ros.actions import Node
import sys
import os

# Add common package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src', 'common'))

def generate_launch_description():
    # Try to load from config file
    device_id = 50  # Default
    
    try:
        from common.config_manager import ConfigManager
        config = ConfigManager()
        device_id = config.get('lift_robot.device_ids.platform', 50)
        print(f"[lift_robot_launch] Loaded device_id from config: {device_id}")
    except Exception as e:
        print(f"[lift_robot_launch] Could not load config: {e}")
        print(f"[lift_robot_launch] Using default device_id: {device_id}")
    
    return LaunchDescription([
        Node(
            package='lift_robot_platform',
            executable='lift_robot_node_action',
            name='lift_robot_platform',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': device_id,
                'use_ack_patch': True
            }],
        ),
    ])
