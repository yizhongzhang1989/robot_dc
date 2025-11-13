from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os
import json


def generate_launch_description():
    """
    Force sensor 2 (left sensor) launch file with automatic calibration loading from JSON config.
    
    Config file path:
      /home/robot/Documents/robot_dc/colcon_ws/config/force_sensor_calibration_53.json
    
    Usage:
      ros2 launch lift_robot_force_sensor_2 lift_robot_force_sensor_launch.py
    """
    
    # Load calibration from JSON config file for device_id=53
    device_id = 53
    config_path = f'/home/robot/Documents/robot_dc/colcon_ws/config/force_sensor_calibration_{device_id}.json'
    
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
        print(f"[force_sensor_{device_id}] ℹ No config at {config_path}")
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

