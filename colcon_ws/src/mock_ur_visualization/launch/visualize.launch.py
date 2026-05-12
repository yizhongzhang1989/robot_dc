#!/usr/bin/env python3
"""
visualize.launch.py — RViz visualization of a real or simulated UR robot,
driven by direct URScript polling (no ur_robot_driver required).

What it starts:
  1. robot_state_publisher    — publishes /tf from the UR URDF + /joint_states
  2. joint_publisher          — polls the robot's URScript port, publishes /joint_states
  3. rviz2                    — with a preset config showing the UR robot model

Args:
  robot_ip                   IP of the robot           (default 192.168.1.16)
  port                       URScript primary port     (default 30002)
  ur_type                    UR model for the URDF     (default ur10e; one of
                                                        ur3/5/10[e], ur7e/12e/16e,
                                                        ur8long, ur15/18/20/30)
  rate_hz                    joint poll/publish rate   (default 30.0)
  rviz                       launch RViz               (default true)
  use_robot_state_publisher  start our own RSP         (default true; set false
                                                        when ur_robot_driver or
                                                        another launch is already
                                                        publishing /robot_description
                                                        and /tf — avoids the
                                                        "two-models flicker")
  use_joint_publisher        start our URScript poller (default true; set false
                                                        when ur_robot_driver is
                                                        already publishing
                                                        /joint_states)

Examples:
  # Standalone (no driver running):
  ros2 launch mock_ur_visualization visualize.launch.py \\
        robot_ip:=192.168.1.16 ur_type:=ur10e

  # Alongside ur15_bringup (driver already provides /joint_states + /tf):
  ros2 launch mock_ur_visualization visualize.launch.py \\
        use_robot_state_publisher:=false use_joint_publisher:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, FindExecutable, LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_UR_MODEL_CHOICES = [
    "ur3", "ur5", "ur10",
    "ur3e", "ur5e", "ur7e", "ur10e", "ur12e", "ur16e",
    "ur8long", "ur15", "ur18", "ur20", "ur30",
]


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.16",
                              description="IP of the UR robot / URSim mock."),
        DeclareLaunchArgument("port", default_value="30002",
                              description="URScript primary port."),
        DeclareLaunchArgument("ur_type", default_value="ur10e",
                              choices=_UR_MODEL_CHOICES,
                              description="UR model to use for the URDF."),
        DeclareLaunchArgument("rate_hz", default_value="30.0",
                              description="JointState publish rate (Hz)."),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz2."),
        DeclareLaunchArgument("joint_prefix", default_value="",
                              description="Optional prefix for joint names."),
        DeclareLaunchArgument(
            "use_robot_state_publisher", default_value="true",
            description="Start our own robot_state_publisher + load the UR URDF. "
                        "Set false when another launch (e.g. ur_robot_driver) "
                        "is already publishing /robot_description and /tf — "
                        "this avoids the duplicate-RSP flicker in RViz."),
        DeclareLaunchArgument(
            "use_joint_publisher", default_value="true",
            description="Start our URScript joint poller. Set false when "
                        "ur_robot_driver / ros2_control is already publishing "
                        "/joint_states."),
    ]

    robot_ip = LaunchConfiguration("robot_ip")
    port = LaunchConfiguration("port")
    ur_type = LaunchConfiguration("ur_type")
    rate_hz = LaunchConfiguration("rate_hz")
    rviz = LaunchConfiguration("rviz")
    joint_prefix = LaunchConfiguration("joint_prefix")
    use_rsp = LaunchConfiguration("use_robot_state_publisher")
    use_jp = LaunchConfiguration("use_joint_publisher")

    # Build the URDF on the fly from ur_description's xacro.
    # ur_description ships a parameterized xacro that selects mesh + limits
    # based on `name:=<ur_type>`.
    description_path = PathJoinSubstitution([
        FindPackageShare("ur_description"), "urdf", "ur.urdf.xacro",
    ])
    robot_description_content = Command([
        FindExecutable(name="xacro"), " ",
        description_path, " ",
        "name:=", ur_type, " ",
        "ur_type:=", ur_type, " ",
        "tf_prefix:=", joint_prefix,
    ])
    robot_description = {
        "robot_description": ParameterValue(robot_description_content, value_type=str)
    }

    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="mock_ur_robot_state_publisher",
        output="screen",
        parameters=[robot_description],
        condition=IfCondition(use_rsp),
    )

    joint_publisher = Node(
        package="mock_ur_visualization",
        executable="joint_publisher",
        name="mock_ur_joint_publisher",
        output="screen",
        parameters=[{
            "robot_ip": robot_ip,
            "port": port,
            "rate_hz": rate_hz,
            "joint_prefix": joint_prefix,
            "frame_id": "base_link",
        }],
        condition=IfCondition(use_jp),
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare("mock_ur_visualization"), "config", "visualize.rviz",
    ])
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="mock_ur_rviz",
        output="log",
        arguments=["-d", rviz_config],
        condition=IfCondition(rviz),
    )

    return LaunchDescription(args + [robot_state_publisher, joint_publisher, rviz_node])
