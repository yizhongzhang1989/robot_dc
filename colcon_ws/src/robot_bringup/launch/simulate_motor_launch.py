from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paths to launch files
    motor_path = os.path.join(
        get_package_share_directory('leadshine_motor'),
        'launch',
        'test_motor_simulation_launch.py'
    )
    web_path = os.path.join(
        get_package_share_directory('robot_web'),
        'launch',
        'web_server_launch.py'
    )

    # Launch descriptions
    motor_launch = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(motor_path))]
    )

    web_launch = TimerAction(
        period=1.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(web_path))]
    )

    return LaunchDescription([
        motor_launch,
        web_launch
    ])
