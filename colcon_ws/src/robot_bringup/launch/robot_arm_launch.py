from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='IP address of the DUCO robot arm'
    )
    
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='7003',
        description='Port number of the DUCO robot arm'
    )
    
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='1',
        description='Device ID for the robot arm'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Port for the web interface'
    )

    # Robot arm node
    robot_arm_node = Node(
        package='duco_robot_arm',
        executable='duco_robot_arm_node',
        name='duco_robot_arm_node',
        parameters=[
            {'ip': LaunchConfiguration('robot_ip')},
            {'port': LaunchConfiguration('robot_port')},
            {'device_id': LaunchConfiguration('device_id')}
        ],
        output='screen',
        emulate_tty=True,
    )

    # Robot arm web interface - launch with a small delay to ensure robot arm node is ready
    robot_arm_web_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_arm_web',
                executable='robot_arm_web_server',
                name='robot_arm_web_server',
                parameters=[
                    {'device_id': LaunchConfiguration('device_id')},
                    {'port': LaunchConfiguration('web_port')}
                ],
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    return LaunchDescription([
        robot_ip_arg,
        robot_port_arg,
        device_id_arg,
        web_port_arg,
        robot_arm_node,
        robot_arm_web_node
    ])
