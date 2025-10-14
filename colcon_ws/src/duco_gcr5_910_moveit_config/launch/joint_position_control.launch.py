"""
Simple launch file for joint position control
Only loads essential components without MoveIt2
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # Declare launch arguments
    declared_arguments = []
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_ip",
            default_value="192.168.1.10",
            description="Robot IP address",
        )
    )
    declared_arguments.append(
        DeclareLaunchArgument(
            "robot_port", 
            default_value="7003",
            description="Robot port number",
        )
    )
    
    # Get package path
    pkg_share = get_package_share_directory('duco_gcr5_910_moveit_config')
    
    # Robot description with network parameters
    robot_description_content = Command([
        "xacro ", os.path.join(pkg_share, "config/gcr5_910.urdf.xacro"),
        " robot_ip:=", LaunchConfiguration("robot_ip"),
        " robot_port:=", LaunchConfiguration("robot_port")
    ])
    
    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    
    # ROS2 Control Node
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_param,
            os.path.join(pkg_share, "config/ros2_controllers.yaml"),
        ],
        output="both",
    )
    
    # Robot State Publisher
    robot_state_pub_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="both",
        parameters=[robot_description_param],
    )
    
    # Joint State Broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    
    # Joint Position Controller
    joint_position_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_position_controller", "--controller-manager", "/controller_manager"],
    )
    
    nodes = [
        ros2_control_node,
        robot_state_pub_node,
        joint_state_broadcaster_spawner,
        joint_position_controller_spawner,
    ]
    
    return LaunchDescription(declared_arguments + nodes)
