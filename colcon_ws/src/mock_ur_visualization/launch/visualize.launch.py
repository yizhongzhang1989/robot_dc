#!/usr/bin/env python3
"""
visualize.launch.py — RViz visualization of a real or simulated UR robot,
driven by direct URScript polling (no ur_robot_driver required).

What it starts:
  1. robot_state_publisher    — publishes /tf from the UR URDF + /joint_states
  2. joint_publisher          — polls the robot's URScript port, publishes /joint_states
  3. rviz2                    — with a preset config showing the UR robot model

Args:
  robot_name                 If non-empty, robot_ip/port/ur_type defaults are
                             looked up from config/robot_config.yaml under
                             ``<robot_name>.robot.{ip,ports.control,type}``.
                             Examples: ``robot_name:=ur15`` or
                             ``robot_name:=ur10e``. When empty (default), the
                             explicit args below are used as-is.
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
  # Visualize ur15 (auto-load IP/type from robot_config.yaml):
  ros2 launch mock_ur_visualization visualize.launch.py robot_name:=ur15

  # Visualize ur10e (auto-load IP/type from robot_config.yaml):
  ros2 launch mock_ur_visualization visualize.launch.py robot_name:=ur10e

  # Standalone with explicit args (legacy / no config dependency):
  ros2 launch mock_ur_visualization visualize.launch.py \\
        robot_ip:=192.168.1.16 ur_type:=ur10e

  # Attach mode — alongside a running dual_ur_bringup. The bringup already
  # publishes /tf, /robot_description and /joint_states for BOTH robots
  # (ur15 unprefixed, ur10e with tf_prefix=ur10e_), so neither RSP nor the
  # URScript joint poller is needed here. Works for either robot:
  ros2 launch mock_ur_visualization visualize.launch.py \\
        robot_name:=ur15 \\
        use_robot_state_publisher:=false use_joint_publisher:=false
  ros2 launch mock_ur_visualization visualize.launch.py \\
        robot_name:=ur10e \\
        use_robot_state_publisher:=false use_joint_publisher:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
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


def _launch_setup(context, *args, **kwargs):
    """Resolve LaunchConfigurations and (optionally) override from config.

    Run inside an OpaqueFunction so we can do string-typed lookups against
    robot_config.yaml before building the Node actions.
    """

    robot_name = LaunchConfiguration("robot_name").perform(context)
    robot_ip = LaunchConfiguration("robot_ip").perform(context)
    port = LaunchConfiguration("port").perform(context)
    ur_type = LaunchConfiguration("ur_type").perform(context)

    if robot_name:
        # Pull IP / port / ur_type from robot_config.yaml. Falls back silently
        # to the explicit args if a key is missing.
        try:
            from common.config_manager import ConfigManager
            cfg = ConfigManager().get_robot(robot_name)
            robot_ip = cfg.get("robot.ip", robot_ip)
            port = str(cfg.get("robot.ports.control", port))
            ur_type = cfg.get("robot.type", ur_type)
            print(f"[mock_ur_visualization] robot_name={robot_name!r} → "
                  f"ip={robot_ip}, port={port}, ur_type={ur_type}")
        except Exception as exc:  # noqa: BLE001
            print(f"[mock_ur_visualization] WARNING: failed to resolve "
                  f"robot_name={robot_name!r} from config: {exc}. Falling "
                  f"back to explicit launch args.")

    if ur_type not in _UR_MODEL_CHOICES:
        raise RuntimeError(
            f"ur_type={ur_type!r} (resolved from robot_name={robot_name!r}) is "
            f"not in {_UR_MODEL_CHOICES}. Update robot_config.yaml or pass "
            f"ur_type:=<model> explicitly."
        )

    rate_hz = LaunchConfiguration("rate_hz")
    rviz = LaunchConfiguration("rviz")
    joint_prefix = LaunchConfiguration("joint_prefix")
    use_rsp = LaunchConfiguration("use_robot_state_publisher")
    use_jp = LaunchConfiguration("use_joint_publisher")

    # Build the URDF on the fly from ur_description's xacro.
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
            "port": int(port),
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

    return [robot_state_publisher, joint_publisher, rviz_node]


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument("robot_name", default_value="",
                              description="If set (e.g. 'ur15', 'ur10e'), "
                                          "robot_ip/port/ur_type are auto-loaded "
                                          "from config/robot_config.yaml."),
        DeclareLaunchArgument("robot_ip", default_value="192.168.1.16",
                              description="IP of the UR robot / URSim mock "
                                          "(ignored when robot_name is set)."),
        DeclareLaunchArgument("port", default_value="30002",
                              description="URScript primary port "
                                          "(ignored when robot_name is set)."),
        DeclareLaunchArgument("ur_type", default_value="ur10e",
                              choices=_UR_MODEL_CHOICES,
                              description="UR model to use for the URDF "
                                          "(ignored when robot_name is set)."),
        DeclareLaunchArgument("rate_hz", default_value="30.0",
                              description="JointState publish rate (Hz)."),
        DeclareLaunchArgument("rviz", default_value="true",
                              description="Launch RViz2."),
        DeclareLaunchArgument("joint_prefix", default_value="",
                              description="Optional prefix for joint names."),
        DeclareLaunchArgument(
            "use_robot_state_publisher", default_value="true",
            description="Start our own robot_state_publisher + load the UR URDF. "
                        "Set false when another launch (e.g. ur_robot_driver or "
                        "dual_ur_bringup) is already publishing /robot_description "
                        "and /tf — this avoids the duplicate-RSP flicker in RViz."),
        DeclareLaunchArgument(
            "use_joint_publisher", default_value="true",
            description="Start our URScript joint poller. Set false when "
                        "ur_robot_driver / ros2_control (or dual_ur_bringup) "
                        "is already publishing /joint_states."),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])

