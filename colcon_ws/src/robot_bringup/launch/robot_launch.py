from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    # Path to modbus_driver launch
    modbus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('modbus_driver'),
                'launch',
                'modbus_manager_launch.py'
            )
        )
    )

    # Delay motor node launch by 2 seconds
    motor_launch = TimerAction(
        period=2.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('leadshine_motor'),
                        'launch',
                        'motor_control_launch.py'
                    )
                )
            )
        ]
    )

    # Delay teleop launch by 3 seconds (to ensure motor nodes are ready)
    teleop_launch = TimerAction(
        period=3.0,
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('robot_teleop'),
                        'launch',
                        'joystick_teleop_launch.py'
                    )
                )
            )
        ]
    )

    return LaunchDescription([
        modbus_launch,
        motor_launch,
        teleop_launch
    ])
