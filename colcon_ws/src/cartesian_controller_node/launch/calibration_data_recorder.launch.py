import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Calibration Data Recorder Launch File
    
    Launches the calibration data recorder node that records pose and FT sensor
    data pairs when Enter key is pressed.
    
    Usage:
    ros2 launch cartesian_controller_node calibration_data_recorder.launch.py
    """
    
    return LaunchDescription([
        Node(
            package='cartesian_controller_node',
            executable='calibration_data_recorder',
            name='calibration_data_recorder',
            output='both',
            parameters=[],
            remappings=[]
        ),
    ])
