from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Paths to launch files
    modbus_path = os.path.join(
        get_package_share_directory('modbus_driver'),
        'launch',
        'modbus_manager_launch.py'
    )
    
    lift_robot_path = os.path.join(
        get_package_share_directory('lift_robot_platform'),
        'launch',
        'lift_robot_launch.py'
    )

    cable_sensor_path = os.path.join(
        get_package_share_directory('lift_robot_cable_sensor'),
        'launch',
        'cable_sensor_launch.py'
    )

    # Launch descriptions
    modbus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(modbus_path)
    )

    # Start lift robot platform after modbus driver is ready
    lift_robot_launch = TimerAction(
        period=3.0,  # Wait 3 seconds for modbus driver to initialize
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(lift_robot_path))]
    )

    # Start cable sensor after modbus driver is ready
    cable_sensor_launch = TimerAction(
        period=4.0,  # Wait 4 seconds for modbus driver to initialize
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(cable_sensor_path))]
    )

    return LaunchDescription([
        modbus_launch,
        lift_robot_launch,
        cable_sensor_launch,
    ])
