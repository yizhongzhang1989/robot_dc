from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import json
import sys

# Add common package to Python path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', '..', '..', 'src', 'common'))


def launch_setup(context, *args, **kwargs):
    """
    OpaqueFunction to access LaunchConfiguration values at runtime.
    This allows us to load the correct calibration file based on device_id parameter.
    """
    # Get actual parameter values from context
    device_id = int(LaunchConfiguration('device_id').perform(context))
    topic_name = LaunchConfiguration('topic_name').perform(context)
    node_name_suffix = LaunchConfiguration('node_name_suffix').perform(context)
    read_interval = float(LaunchConfiguration('read_interval').perform(context))
    
    # Portable config path resolution (ENV -> colcon_ws -> CWD)
    env_dir = os.environ.get('LIFT_ROBOT_CONFIG_DIR')
    def resolve_config_dir():
        if env_dir:
            base = os.path.abspath(env_dir)
            if base.endswith('config'):
                return base
            parts = base.split(os.sep)
            if 'colcon_ws' in parts:
                return os.path.join(base[:base.index('colcon_ws') + len('colcon_ws')], 'config') if False else os.path.join(os.sep.join(parts[:parts.index('colcon_ws')+1]), 'config')
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
    if device_id == 52:
        calib_scale = 0.116125
        calib_offset = 0.0
    elif device_id == 53:
        calib_scale = 0.023614
        calib_offset = 0.0
    else:
        calib_scale = 0.116125
        calib_offset = 0.0
    
    # Load from JSON if exists (overrides defaults)
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                config_data = json.load(f)
                calib_scale = config_data.get('scale', calib_scale)
                calib_offset = config_data.get('offset', calib_offset)
                print(f"[force_sensor_{device_id}] Loaded calibration: scale={calib_scale:.6f}, offset={calib_offset:.6f}")
        except Exception as e:
            print(f"[force_sensor_{device_id}] Warning: Failed to load calibration config: {e}")
            print(f"[force_sensor_{device_id}] Using default calibration values")
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
            print(f"[force_sensor_{device_id}] Created default calibration file at {config_path}")
        except Exception as e:
            print(f"[force_sensor_{device_id}] Failed to create default calibration file: {e}")
        print(f"[force_sensor_{device_id}] Using default calibration values: scale={calib_scale:.6f}, offset={calib_offset:.6f}")
    
    return [
        Node(
            package='lift_robot_force_sensor',
            executable='force_sensor_node',
            name='lift_robot_force_sensor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': device_id,
                'topic_name': topic_name,
                'node_name_suffix': node_name_suffix,
                'use_ack_patch': True,
                'read_interval': read_interval,
                'calibration_scale': calib_scale,
                'calibration_offset': calib_offset
            }]
        ),
    ]


def generate_launch_description():
    """
    Force sensor launch file with automatic calibration loading from JSON config.
    
    This launch file is designed to be reusable for multiple force sensors.
    It accepts parameters for device_id, topic_name, and node_name_suffix.
    
    Usage:
      # Single instance (default device_id=52, topic=/force_sensor)
      ros2 launch lift_robot_force_sensor lift_robot_force_sensor_launch.py
      
      # Custom instance
      ros2 launch lift_robot_force_sensor lift_robot_force_sensor_launch.py \
          device_id:=53 topic_name:=/force_sensor_2 node_name_suffix:=left
    
    Config file paths (resolved dynamically):
        Base directory priority:
            1. ENV LIFT_ROBOT_CONFIG_DIR
            2. Ancestor 'colcon_ws' (config under it)
            3. CWD/config
    """
    
    # Try to load defaults from config file
    default_device_id = '52'
    default_read_interval = '0.06'
    
    try:
        from common.config_manager import ConfigManager
        config = ConfigManager()
        default_device_id = str(config.get('lift_robot.device_ids.force_sensor_right', 52))
        default_read_interval = str(config.get('lift_robot.sensors.force_sensor.read_interval', 0.06))
        print(f"[force_sensor] Loaded from config: device_id={default_device_id}, interval={default_read_interval}")
    except Exception as e:
        print(f"[force_sensor] Could not load config: {e}")
        print(f"[force_sensor] Using defaults: device_id={default_device_id}, interval={default_read_interval}")
    
    # Declare launch arguments
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value=default_device_id,
        description='Force sensor Modbus device ID (52=right sensor, 53=left sensor)'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value='/force_sensor',
        description='Topic name for publishing force data'
    )
    
    node_name_suffix_arg = DeclareLaunchArgument(
        'node_name_suffix',
        default_value='',
        description='Suffix for node name (e.g., "right", "left")'
    )
    
    read_interval_arg = DeclareLaunchArgument(
        'read_interval',
        default_value=default_read_interval,
        description='Sensor read interval in seconds (0.06 = ~17Hz, 0.02 = 50Hz, 0.01 = 100Hz)'
    )
    
    return LaunchDescription([
        device_id_arg,
        topic_name_arg,
        node_name_suffix_arg,
        read_interval_arg,
        OpaqueFunction(function=launch_setup)
    ])


