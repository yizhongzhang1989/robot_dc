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
    
        Config file paths (resolved dynamically):
            Base directory priority:
                1. ENV LIFT_ROBOT_CONFIG_DIR
                2. Ancestor 'colcon_ws' (config under it)
                3. CWD/config
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
    env_dir = os.environ.get('LIFT_ROBOT_CONFIG_DIR')
    def resolve_config_dir():
        if env_dir:
            base = os.path.abspath(env_dir)
            if base.endswith('config'):
                return base
            parts = base.split(os.sep)
            if 'colcon_ws' in parts:
                return os.path.join(os.sep.join(parts[:parts.index('colcon_ws')+1]), 'config')
            candidate = os.path.join(base, 'colcon_ws', 'config')
            if os.path.isdir(candidate):
                return candidate
            return os.path.join(base, 'config')
        cur = os.path.abspath(os.path.dirname(__file__))
        while cur and cur != os.sep:
            if os.path.basename(cur) == 'colcon_ws':
                return os.path.join(cur, 'config')
            cur = os.path.dirname(cur)
        return os.path.join(os.getcwd(), 'config')
    config_dir = resolve_config_dir()
    os.makedirs(config_dir, exist_ok=True)
    config_path = os.path.join(config_dir, f'force_sensor_calibration_{device_id}.json')
    
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
        # Create default file for consistency
        try:
            default_data = {
                'device_id': device_id,
                'scale': calib_scale,
                'offset': calib_offset,
                'generated_at': None,
                'generated_at_iso': None
            }
            with open(config_path, 'w') as f:
                json.dump(default_data, f, indent=2)
            print(f"[force_sensor_{device_id}] Created default calibration at {config_path}")
        except Exception as e:
            print(f"[force_sensor_{device_id}] Failed to create default calibration file: {e}")
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


