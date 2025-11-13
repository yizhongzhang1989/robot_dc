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
    
    # Load calibration from JSON config file
    config_path = '/home/robot/Documents/robot_dc/colcon_ws/config/draw_wire_calibration.json'
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
        print(f"[draw_wire_sensor] No calibration config found at {config_path}")
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
