import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration


def generate_launch_description():

    return LaunchDescription([
        Node(
            package='monitor',
            executable='joystick_calibration_node',
            name='joystick_calibration',
            output='both',
            parameters=[],
            remappings=[]
        ),
    ])
