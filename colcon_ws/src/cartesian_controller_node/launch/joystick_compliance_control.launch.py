import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Joystick Control Launch File

    Launches the joystick control node that publishes joystick input data
    and target wrench for force control applications.
    
    Usage:
    ros2 launch cartesian_controller_node joystick_compliance_control.launch.py
    """
    
    return LaunchDescription([
        Node(
            package='cartesian_controller_node',
            executable='joystick_compliance_control_node',
            name='joystick_compliance_control',
            output='both',
            parameters=[],
            remappings=[]
        ),
    ])
