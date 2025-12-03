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
    dashboard_enabled = LaunchConfiguration('dashboard_enabled').perform(context).lower() == 'true'
    dashboard_host = LaunchConfiguration('dashboard_host').perform(context)
    dashboard_port = int(LaunchConfiguration('dashboard_port').perform(context))
    serial_port = LaunchConfiguration('serial_port').perform(context)
    serial_baudrate = int(LaunchConfiguration('serial_baudrate').perform(context))
    
    # Try to load calibration file name from config
    calib_filename = None
    try:
        from common.config_manager import ConfigManager
        config = ConfigManager()
        # Dynamically find which sensor (right/left) matches this device_id
        for sensor_name in ['force_sensor_right', 'force_sensor_left']:
            config_path = f'lift_robot.{sensor_name}'
            if config.has(f'{config_path}.device_id') and config.get(f'{config_path}.device_id') == device_id:
                if config.has(f'{config_path}.calibration_file'):
                    calib_filename = config.get(f'{config_path}.calibration_file')
                    break
    except Exception:
        pass
    
    # Fallback: Use generic device_id-based naming
    if calib_filename is None:
        calib_filename = f'force_sensor_calibration_{device_id}.json'
    
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
    
    config_path = os.path.join(config_dir, calib_filename)
    
    # Default calibration values (fallback when config file doesn't exist)
    # Use neutral defaults - actual calibration should be done via web interface
    calib_scale = 0.1
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
    
    # Construct node name with suffix
    node_name = 'lift_robot_force_sensor'
    if node_name_suffix:
        node_name = f'{node_name}_{node_name_suffix}'
    
    return [
        Node(
            package='lift_robot_force_sensor',
            executable='force_sensor_node',
            name=node_name,  # Use dynamic node name with suffix
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': device_id,
                'topic_name': topic_name,
                'node_name_suffix': node_name_suffix,
                'use_ack_patch': True,
                'read_interval': read_interval,
                'calibration_scale': calib_scale,
                'calibration_offset': calib_offset,
                'dashboard_enabled': dashboard_enabled,
                'dashboard_host': dashboard_host,
                'dashboard_port': dashboard_port,
                'serial_port': serial_port,
                'serial_baudrate': serial_baudrate
            }]
        ),
    ]


def generate_launch_description():
    """
    Force sensor launch file with automatic calibration loading from JSON config.
    
    This launch file is designed to be reusable for multiple force sensors.
    It accepts parameters for device_id, topic_name, and node_name_suffix.
    
    Usage:
      # Single instance (default device_id=52, topic=/force_sensor_right)
      ros2 launch lift_robot_force_sensor lift_robot_force_sensor_launch.py
      
      # Custom instance
      ros2 launch lift_robot_force_sensor lift_robot_force_sensor_launch.py \
          device_id:=53 topic_name:=/force_sensor_left node_name_suffix:=left
    
    Config file paths (resolved dynamically):
        Base directory priority:
            1. ENV LIFT_ROBOT_CONFIG_DIR
            2. Ancestor 'colcon_ws' (config under it)
            3. CWD/config
    """
    
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
    
    # For right sensor by default
    default_device_id = str(get_config_value('lift_robot.force_sensor_right.device_id', 52))
    default_topic = get_config_value('lift_robot.force_sensor_right.topic', '/force_sensor_right')
    default_read_interval = str(get_config_value('lift_robot.force_sensor_right.read_interval', 0.06))
    default_node_suffix = get_config_value('lift_robot.force_sensor_right.node_name_suffix', '')
    default_dashboard_enabled = str(get_config_value('lift_robot.force_sensor_right.dashboard.enabled', False)).lower()
    default_dashboard_host = get_config_value('lift_robot.force_sensor_right.dashboard.host', '0.0.0.0')
    default_dashboard_port = str(get_config_value('lift_robot.force_sensor_right.dashboard.port', 5001))
    default_serial_port = get_config_value('lift_robot.modbus_driver.port', 'auto')
    default_serial_baudrate = str(get_config_value('lift_robot.modbus_driver.baudrate', 115200))
    
    print(f"[force_sensor_launch] Config: device_id={default_device_id}, topic={default_topic}, interval={default_read_interval}")
    
    # Declare launch arguments
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value=default_device_id,
        description='Force sensor Modbus device ID (52=right, 53=left)'
    )
    
    topic_name_arg = DeclareLaunchArgument(
        'topic_name',
        default_value=default_topic,
        description='Topic name for publishing force data'
    )
    
    node_name_suffix_arg = DeclareLaunchArgument(
        'node_name_suffix',
        default_value=default_node_suffix,
        description='Suffix for node name (e.g., "right", "left")'
    )
    
    read_interval_arg = DeclareLaunchArgument(
        'read_interval',
        default_value=default_read_interval,
        description='Sensor read interval in seconds (0.06 = ~17Hz, 0.02 = 50Hz, 0.01 = 100Hz)'
    )
    
    dashboard_enabled_arg = DeclareLaunchArgument(
        'dashboard_enabled',
        default_value=default_dashboard_enabled,
        description='Enable force sensor configuration dashboard'
    )
    
    dashboard_host_arg = DeclareLaunchArgument(
        'dashboard_host',
        default_value=default_dashboard_host,
        description='Dashboard host address'
    )
    
    dashboard_port_arg = DeclareLaunchArgument(
        'dashboard_port',
        default_value=default_dashboard_port,
        description='Dashboard port'
    )
    
    serial_port_arg = DeclareLaunchArgument(
        'serial_port',
        default_value=default_serial_port,
        description='Serial port for Modbus communication'
    )
    
    serial_baudrate_arg = DeclareLaunchArgument(
        'serial_baudrate',
        default_value=default_serial_baudrate,
        description='Serial baudrate for Modbus communication'
    )
    
    return LaunchDescription([
        device_id_arg,
        topic_name_arg,
        node_name_suffix_arg,
        read_interval_arg,
        dashboard_enabled_arg,
        dashboard_host_arg,
        dashboard_port_arg,
        serial_port_arg,
        serial_baudrate_arg,
        OpaqueFunction(function=launch_setup)
    ])


