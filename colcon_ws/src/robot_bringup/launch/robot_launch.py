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
    motor_path = os.path.join(
        get_package_share_directory('leadshine_motor'),
        'launch',
        'motor_control_launch.py'
    )
    servo_path = os.path.join(
        get_package_share_directory('feetech_servo'),
        'launch',
        'servo_control_launch.py'
    )
    platform_path = os.path.join(
        get_package_share_directory('platform_controller'),
        'launch',
        'platform_control_launch.py'
    )
    teleop_path = os.path.join(
        get_package_share_directory('robot_teleop'),
        'launch',
        'joystick_teleop_launch.py'
    )
    web_path = os.path.join(
        get_package_share_directory('robot_web'),
        'launch',
        'web_server_launch.py'
    )
    motor_status_path = os.path.join(
        get_package_share_directory('motor_status'),
        'launch',
        'motor_status_launch.py'
    )
    cam_node_path = os.path.join(
        get_package_share_directory('camera_node'),
        'launch',
        'double_camera_launch.py'
    )

    # Launch descriptions
    modbus_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(modbus_path)
    )

    motor_launch = TimerAction(
        period=2.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(motor_path))]
    )

    servo_launch = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(servo_path))]
    )

    platform_launch = TimerAction(
        period=4.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(platform_path))]
    )

    teleop_launch = TimerAction(
        period=5.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(teleop_path))]
    )

    web_launch = TimerAction(
        period=6.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(web_path))]
    )

    motor_status_launch = TimerAction(
        period=7.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(motor_status_path))]
    )

    cam_node_launch = TimerAction(
        period=8.0,
        actions=[IncludeLaunchDescription(PythonLaunchDescriptionSource(cam_node_path))]
    )

    return LaunchDescription([
        modbus_launch,
        motor_launch,
        servo_launch,
        platform_launch,
        teleop_launch,
        web_launch,
        motor_status_launch,
        cam_node_launch
    ])
