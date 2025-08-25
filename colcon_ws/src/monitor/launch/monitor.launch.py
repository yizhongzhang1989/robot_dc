import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument, SetEnvironmentVariable
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    """
    Robot Monitor Launch File
    
    Usage:
    ros2 launch monitor monitor.launch.py                    # Use default ~/robot_data
    ros2 launch monitor monitor.launch.py data_dir:=/path    # Use custom path
    """
    
    data_dir_arg = DeclareLaunchArgument(
        'data_dir',
        default_value=os.path.expanduser('~/robot_data'),
        description='Directory to store robot data'
    )
    
    set_data_dir = SetEnvironmentVariable(
        'ROBOT_DATA_DIR',
        LaunchConfiguration('data_dir')
    )
    
    return LaunchDescription([
        data_dir_arg,
        set_data_dir,
        Node(
            package='monitor',
            executable='monitor_node',
            name='robot_monitor',
            output='both'
        ),
    ])
