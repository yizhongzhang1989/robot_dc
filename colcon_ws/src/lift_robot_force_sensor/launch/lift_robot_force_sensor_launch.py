from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import json


def generate_launch_description():
    """
    Force sensor launch file with automatic calibration loading from JSON config.
    
    Usage:
      ros2 launch lift_robot_force_sensor lift_robot_force_sensor_launch.py device_id:=52
      ros2 launch lift_robot_force_sensor lift_robot_force_sensor_launch.py device_id:=53
    
    Config file paths:
      - /home/robot/Documents/robot_dc/colcon_ws/config/force_sensor_calibration_52.json
      - /home/robot/Documents/robot_dc/colcon_ws/config/force_sensor_calibration_53.json
    """
    
    # Get device_id from environment or use default
    # This allows: DEVICE_ID=53 ros2 launch ...
    device_id_env = os.environ.get('DEVICE_ID', '52')
    device_id = int(device_id_env)
    
    # Declare device_id argument
    device_id_arg = DeclareLaunchArgument(
        'device_id', 
        default_value=str(device_id), 
        description='Force sensor device ID (52=/force_sensor, 53=/force_sensor_2)'
    )
    
    # Load calibration from JSON config file based on device_id
    config_path = f'/home/robot/Documents/robot_dc/colcon_ws/config/force_sensor_calibration_{device_id}.json'
    
    # Default calibration values (fallback when config file doesn't exist)
    # These are initial calibration results - update via web interface for better accuracy
    if device_id == 52:
        # Right force sensor (device_id=52, topic=/force_sensor)
        # Default from initial manual calibration
        calib_scale = 0.116125
        calib_offset = 0.0
    elif device_id == 53:
        # Left force sensor (device_id=53, topic=/force_sensor_2)
        # Default from initial manual calibration (same as right sensor initially)
        # TODO: Update after performing separate calibration for left sensor
        calib_scale = 0.116125
        calib_offset = 0.0
    else:
        # Generic fallback for other device IDs
        calib_scale = 0.116125
        calib_offset = 0.0
    
    # Load from JSON if exists (overrides defaults)
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                config_data = json.load(f)
                calib_scale = config_data.get('scale', calib_scale)
                calib_offset = config_data.get('offset', calib_offset)
                print(f"[force_sensor_{device_id}] ✓ Loaded calibration: scale={calib_scale:.6f}, offset={calib_offset:.6f}")
        except Exception as e:
            print(f"[force_sensor_{device_id}] ⚠ Failed to load {config_path}: {e}")
            print(f"[force_sensor_{device_id}] Using default values")
    else:
        print(f"[force_sensor_{device_id}] ℹ No config at {config_path}")
        print(f"[force_sensor_{device_id}] Using defaults: scale={calib_scale:.6f}, offset={calib_offset:.6f}")
    
    # Determine topic name based on device_id
    topic_name = '/force_sensor' if device_id == 52 else '/force_sensor_2'
    
    return LaunchDescription([
        device_id_arg,
        Node(
            package='lift_robot_force_sensor',
            executable='force_sensor_node',
            name=f'lift_robot_force_sensor_{device_id}',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': device_id,  # Use resolved device_id, not LaunchConfiguration
                'use_ack_patch': True,
                'read_interval': 0.02,  # 50Hz for consistency with other sensors
                'calibration_scale': calib_scale,
                'calibration_offset': calib_offset
            }],
            remappings=[
                ('/force_sensor', topic_name)  # Remap to correct topic
            ]
        ),
    ])


