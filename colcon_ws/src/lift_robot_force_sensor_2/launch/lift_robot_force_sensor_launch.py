from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='lift_robot_force_sensor_2',
            executable='force_sensor_node_2',
            name='lift_robot_force_sensor_2',
            output='screen',
            emulate_tty=True,
            parameters=[{
                'device_id': 53,
                'use_ack_patch': True,
                'read_interval': 0.02,
                'enable_visualization': False,
                'calibration_scale': 0.023614,  # Calibration result for device_id=53
                'calibration_offset': 0.0
            }],
        ),
    ])
