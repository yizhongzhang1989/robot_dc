import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Zero Force Control Launch File
    
    Launches the zero force control node that publishes FT sensor data
    and target wrench for force control applications.
    
    Usage:
    ros2 launch cartesian_controller_node zero_force_control.launch.py
    """
    
    return LaunchDescription([
        Node(
            package='cartesian_controller_node',
            executable='zero_force_control_node',
            name='zero_force_control',
            output='both',
            parameters=[],
            remappings=[]
        ),
    ])
