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
     * move_group
     * moveit_rviz
     * warehouse_db (optional)
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
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/move_group.launch.py")
            ),
        )
    )

    # Run Rviz and load the default config to see the state of the move_group node
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/moveit_rviz.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("use_rviz")),
        )
    )

    # If database loading was enabled, start mongodb as well
    ld.add_action(
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                str(launch_package_path / "launch/warehouse_db.launch.py")
            ),
            condition=IfCondition(LaunchConfiguration("db")),
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
    
    robot_description_content = Command([
        "xacro ", str(moveit_config.package_path / "config/gcr5_910.urdf.xacro"),
        " robot_ip:=", LaunchConfiguration("robot_ip"),
        " robot_port:=", LaunchConfiguration("robot_port")
    ])
    
    robot_description_param = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }
    
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[
            robot_description_param,
            str(moveit_config.package_path / "config/ros2_controllers.yaml"),
        ],
        output="both",
    )
    ld.add_action(ros2_control_node)

    # Joint state broadcaster
    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster", "--controller-manager", "/controller_manager"],
    )
    ld.add_action(joint_state_broadcaster_spawner)
    
    # Joint trajectory controller  
    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_1_controller", "--controller-manager", "/controller_manager"],
    )
    ld.add_action(arm_controller_spawner)

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
