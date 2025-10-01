from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import generate_demo_launch

import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue

from srdfdom.srdf import SRDF

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution


from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
)
from launch.substitutions import LaunchConfiguration

from moveit_configs_utils.launch_utils import (
    add_debuggable_node,
    DeclareBooleanLaunchArg,
)

from launch_ros.actions import Node



from moveit_configs_utils import MoveItConfigsBuilder
#from moveit_configs_utils.launches import generate_demo_launch

def generate_demo_launch(moveit_config, launch_package_path=None, declared_arguments=None):
    """
    Launches a self contained demo

    launch_package_path is optional to use different launch and config packages
    declared_arguments is optional list of declared launch arguments

    Includes
     * static_virtual_joint_tfs
     * robot_state_publisher
     * ros2_control_node + controller spawners
    """
    if launch_package_path == None:
        launch_package_path = moveit_config.package_path

    ld = LaunchDescription()
    
    # Add declared arguments if provided
    if declared_arguments:
        for arg in declared_arguments:
            ld.add_action(arg)
    ld.add_action(
        DeclareBooleanLaunchArg(
            "db",
            default_value=False,
            description="By default, we do not start a database (it can be large)",
        )
    )
    ld.add_action(
        DeclareBooleanLaunchArg(
            "debug",
            default_value=False,
            description="By default, we are not in debug mode",
        )
    )
    ld.add_action(DeclareBooleanLaunchArg("use_rviz", default_value=True))
    # If there are virtual joints, broadcast static tf by including virtual_joints launch
    virtual_joints_launch = (
        launch_package_path / "launch/static_virtual_joint_tfs.launch.py"
    )

    if virtual_joints_launch.exists():
        ld.add_action(
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(str(virtual_joints_launch)),
            )
        )

    # Given the published joint states, publish tf for the robot links
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/rsp.launch.py")
            ),
        )
    )


    ld.add_action(
    	DeclareLaunchArgument(
    	'arm_num', 
    	default_value='1',
    	)
    )
    ld.add_action(
    	DeclareLaunchArgument(
    	'arm_dof',  
    	default_value='6',
    	)
    )
    
    # Official ROS2 Control Node with DUCO hardware interface
    # Override robot_description with network parameters
    from launch_ros.parameter_descriptions import ParameterValue
    from launch.substitutions import Command
    
    # (heecheol) use robot descriptions as usual (in the moveit_pkg?)
    robot_description_content = Command([
        "xacro ", str(moveit_config.package_path / "config/gcr5_910.urdf.xacro"),
        " robot_ip:=", LaunchConfiguration("robot_ip"),
        " robot_port:=", LaunchConfiguration("robot_port")
    ])
    
    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    # (heecheol) ros2_control_node must be changed
    robot_pkg = FindPackageShare("duco_gcr5_910_moveit_config") # <- change package name if it is wrong!
    # (heecheol) controllers description for Cartesian Controller
    ros2_controllers = PathJoinSubstitution([robot_pkg, "config", "cartesian_controller_manager.yaml"])

    # (heecheol) ros2_control_node, added remappings for Cartesian controllers
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[robot_description_param, ros2_controllers],
        output="both",
        remappings=[
            ("motion_control_handle/target_frame", "target_frame"),
            ("cartesian_motion_controller/target_frame", "target_frame"),
            ("cartesian_compliance_controller/target_frame", "target_frame"),
            ("cartesian_force_controller/target_wrench", "target_wrench"),
            ("cartesian_compliance_controller/target_wrench", "target_wrench"),
            ("cartesian_force_controller/ft_sensor_wrench", "ft_sensor_wrench"),
            ("cartesian_compliance_controller/ft_sensor_wrench", "ft_sensor_wrench"),
        ],
    )
    ld.add_action(ros2_control_node)

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    ld.add_action(joint_state_broadcaster_spawner)
    
    # (heecheol) Joint trajectory controller is deleted (cartesian_controller controls the arm)
    # arm_controller_spawner = Node(
    #     package="controller_manager",
    #     executable="spawner",
    #     arguments=["arm_1_controller", "--controller-manager", "/controller_manager"],
    # )
    # ld.add_action(arm_controller_spawner)

    # (heecheol) Cartesian controllers are spawned. Only motion controller is activated, others are inactive for now.
    def controller_spawner(name, *extra_args):
        return Node(
            package="controller_manager",
            executable="spawner",
            arguments=[name, "--controller-manager", "/controller_manager", *extra_args],
            output="screen",
        )

    active_list = ["cartesian_compliance_controller"]

    active_spawners = [controller_spawner(c) for c in active_list]

    inactive_list = [
        "cartesian_motion_controller",
        "cartesian_force_controller",
        #"motion_control_handle",
    ]

    inactive_spawners = [controller_spawner(c, "--inactive") for c in inactive_list]
    for spawner in active_spawners + inactive_spawners:
        ld.add_action(spawner)

    return ld


def generate_launch_description():
    # Declare launch arguments for network configuration
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
    
    # Build robot description with network parameters  
    moveit_config = MoveItConfigsBuilder("gcr5_910", package_name="duco_gcr5_910_moveit_config").to_moveit_configs()
    
    launch_description = generate_demo_launch(moveit_config, declared_arguments=declared_arguments)
    
    return launch_description
