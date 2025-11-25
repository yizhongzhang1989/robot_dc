from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import json

def generate_launch_description():
    device_id_arg = DeclareLaunchArgument('device_id', default_value='51', description='Modbus device ID')
    # Default changed to 0.02s (50Hz) for system-wide consistency
    read_interval_arg = DeclareLaunchArgument('read_interval', default_value='0.02', description='Sensor read interval (s, 0.02=50Hz)')
    
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
    config_path = os.path.join(config_dir, 'draw_wire_calibration.json')
    calib_scale = 0.024537  # Default values
    calib_offset = 681.837575
    calib_enable = True
    
    if os.path.exists(config_path):
        try:
            with open(config_path, 'r') as f:
                config_data = json.load(f)
                calib_scale = config_data.get('scale', calib_scale)
                calib_offset = config_data.get('offset', calib_offset)
                calib_enable = config_data.get('enable', calib_enable)
                print(f"[draw_wire_sensor] Loaded calibration: scale={calib_scale}, offset={calib_offset}")
        except Exception as e:
            print(f"[draw_wire_sensor] Warning: Failed to load calibration config: {e}")
            print(f"[draw_wire_sensor] Using default calibration values")
    else:
        # Create default file for consistency
        try:
            default_data = {
                'scale': calib_scale,
                'offset': calib_offset,
                'enable': calib_enable,
                'generated_at': None,
                'generated_at_iso': None
            }
            with open(config_path, 'w') as f:
                json.dump(default_data, f, indent=2)
            print(f"[draw_wire_sensor] Created default calibration file at {config_path}")
        except Exception as e:
            print(f"[draw_wire_sensor] Failed to create default calibration file: {e}")
        print(f"[draw_wire_sensor] Using default calibration values: scale={calib_scale}, offset={calib_offset}")
    
    return LaunchDescription([
        device_id_arg,
        read_interval_arg,
        Node(
            package='draw_wire_sensor',
            executable='draw_wire_sensor_node',
            name='draw_wire_sensor',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': LaunchConfiguration('device_id'),
                'use_ack_patch': True,
                'read_interval': LaunchConfiguration('read_interval'),
                'publish_compact': False,  # Disable compact mode to get register_1 for calibration
                'calibration.scale': calib_scale,
                'calibration.offset': calib_offset,
                'calibration.enable': calib_enable
            }],
        )
    ])
