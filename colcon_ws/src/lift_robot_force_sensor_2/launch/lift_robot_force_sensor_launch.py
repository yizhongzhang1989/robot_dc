from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import json


def generate_launch_description():
    """
    Force sensor 2 (left sensor) launch file with automatic calibration loading from JSON config.
    
        Config file path resolved dynamically (ENV -> colcon_ws -> CWD/config)
    
    Usage:
      ros2 launch lift_robot_force_sensor_2 lift_robot_force_sensor_launch.py
    """
    
    # Load calibration from JSON config file for device_id=53
    device_id = 53
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
    
    # Default calibration values for device_id=53 (left sensor, /force_sensor_2)
    # From initial manual calibration
    calib_scale = 0.023614
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
    
    return LaunchDescription([
        Node(
            package='lift_robot_force_sensor_2',
            executable='force_sensor_node_2',
            name='lift_robot_force_sensor_2',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': device_id,
                'use_ack_patch': True,
                'read_interval': 0.02,  # 50Hz for consistency with other sensors
                'enable_visualization': False,
                'calibration_scale': calib_scale,
                'calibration_offset': calib_offset
            }],
        ),
    ])

