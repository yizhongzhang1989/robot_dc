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
    
    device_id_default = str(get_config_value('lift_robot.platform.device_id', 50))
    use_ack_patch_default = str(get_config_value('lift_robot.platform.use_ack_patch', True)).lower()
    overshoot_settle_time_default = str(get_config_value('lift_robot.platform.overshoot_settle_time', 0.8))
    
    print(f"[lift_robot_launch] Config: device_id={device_id_default}, use_ack_patch={use_ack_patch_default}, "
          f"overshoot_settle_time={overshoot_settle_time_default}")
    
    # Declare launch arguments
    device_id_arg = DeclareLaunchArgument(
        'device_id', default_value=device_id_default, description='Platform Modbus device ID')
    use_ack_patch_arg = DeclareLaunchArgument(
        'use_ack_patch', default_value=use_ack_patch_default, description='Enable ACK patch for Modbus')
    overshoot_settle_time_arg = DeclareLaunchArgument(
        'overshoot_settle_time', default_value=overshoot_settle_time_default, 
        description='Settle time after stop command (seconds)')
    
    return LaunchDescription([
        device_id_arg,
        use_ack_patch_arg,
        overshoot_settle_time_arg,
        Node(
            package='lift_robot_platform',
            executable='lift_robot_node_action',
            name='lift_robot_platform',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'use_ack_patch': LaunchConfiguration('use_ack_patch'),
                'overshoot_settle_time': LaunchConfiguration('overshoot_settle_time')
            }],
        ),
    ])
