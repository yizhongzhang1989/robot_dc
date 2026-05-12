#!/usr/bin/env python3
"""
visualize.launch.py — RViz visualization of a real or simulated UR robot,
driven by direct URScript polling (no ur_robot_driver required).

Single-arm visualization. This is what you want for hand-eye calibration,
rack calibration debugging, or any time you have a UR sitting on the
network and you want to *see* it in RViz without bringing up ros2_control.

What it starts (all under the ``/mock`` ROS namespace by default so it
never races with a real driver running in the same graph):

  1. robot_state_publisher    — publishes ``/mock/robot_description`` and
                                ``/tf`` from the UR URDF
  2. mock_ur_joint_publisher  — polls the robot's URScript port and
                                publishes ``/mock/joint_states``
  3. rviz2                    — preset config (visualize.rviz) shows the
                                robot model from ``/mock/robot_description``

Args (most common first):
  robot_name     If set (e.g. ``ur15`` / ``ur10e``), robot_ip / port /
                 ur_type defaults are auto-loaded from
                 ``config/robot_config.yaml`` under
                 ``<robot_name>.robot.{ip, ports.control, type}``.
                 Default: ``ur15``.

  rviz           Launch RViz2. Default: ``true``.

  namespace      ROS namespace under which this visualization stack
                 lives. Default: ``mock``. Change only if you need
                 multiple simultaneous mock visualizations (rare).

  rate_hz        URScript poll / JointState publish rate. Default: 30.

  robot_ip       Override the IP looked up via ``robot_name``. Default:
                 192.168.1.16 (used only when robot_name is empty).

  port           Override the URScript primary port. Default: 30002.

  ur_type        Override the URDF model. Default: ur10e.

  joint_prefix   Advanced. Prefix prepended to URDF link/joint names and
                 to the published JointState names. Use only when this
                 stack shares a global TF tree with other robots (e.g.
                 the dual_ur_bringup pipeline). Leave empty (default)
                 for plain single-arm visualization.

Examples:
  # Visualize ur15 (auto-loads IP/type from robot_config.yaml):
  ros2 launch mock_ur_visualization visualize.launch.py robot_name:=ur15

  # Visualize ur10e:
  ros2 launch mock_ur_visualization visualize.launch.py robot_name:=ur10e

  # Visualize a robot not in robot_config.yaml — explicit args:
  ros2 launch mock_ur_visualization visualize.launch.py \\
        robot_name:='' robot_ip:=192.168.1.50 ur_type:=ur10e

  # Headless (joint_states + robot_description only, no RViz window):
  ros2 launch mock_ur_visualization visualize.launch.py robot_name:=ur15 rviz:=false
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import (
    Command,
    FindExecutable,
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from launch_ros.parameter_descriptions import ParameterValue
from launch_ros.substitutions import FindPackageShare


_UR_MODEL_CHOICES = [
    "ur3", "ur5", "ur10",
    "ur3e", "ur5e", "ur7e", "ur10e", "ur12e", "ur16e",
    "ur8long", "ur15", "ur18", "ur20", "ur30",
]


def _launch_setup(context, *args, **kwargs):
    """Resolve LaunchConfigurations (with optional robot_config.yaml lookup).

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
    namespace = LaunchConfiguration("namespace")
    joint_prefix = LaunchConfiguration("joint_prefix").perform(context)

    # Build the URDF on the fly from ur_description's xacro. tf_prefix is
    # set from joint_prefix so when this stack runs inside a shared TF
    # tree (dual_ur_bringup) the link/joint frame names are disjoint from
    # other robots' frames.
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
            "frame_id": f"{joint_prefix}base_link" if joint_prefix else "base_link",
        }],
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

    # Wrap everything (including RViz) in the same namespace push so the
    # RViz config's relative ``robot_description`` topic resolves to
    # ``/<namespace>/robot_description`` — exactly what the RSP publishes.
    # /tf stays global so RViz can render frames either way.
    return [
        GroupAction([
            PushRosNamespace(namespace),
            SetRemap("tf", "/tf"),
            SetRemap("tf_static", "/tf_static"),
            robot_state_publisher,
            joint_publisher,
            rviz_node,
        ]),
    ]


def generate_launch_description() -> LaunchDescription:
    args = [
        DeclareLaunchArgument(
            "robot_name",
            default_value="ur15",
            description="If non-empty (e.g. 'ur15', 'ur10e'), "
                        "robot_ip/port/ur_type are auto-loaded from "
                        "config/robot_config.yaml. Set to '' to use the "
                        "explicit robot_ip/port/ur_type args below.",
        ),
        DeclareLaunchArgument(
            "rviz", default_value="true",
            description="Launch RViz2 with the preset visualize.rviz config.",
        ),
        DeclareLaunchArgument(
            "namespace", default_value="mock",
            description="ROS namespace for the entire visualization stack "
                        "(robot_state_publisher + joint_publisher + rviz2). "
                        "Default 'mock' keeps it out of the way of any real "
                        "driver running in the same ROS graph.",
        ),
        DeclareLaunchArgument(
            "rate_hz", default_value="30.0",
            description="URScript poll / JointState publish rate (Hz).",
        ),
        DeclareLaunchArgument(
            "robot_ip", default_value="192.168.1.16",
            description="IP of the UR robot (used only when robot_name='').",
        ),
        DeclareLaunchArgument(
            "port", default_value="30002",
            description="URScript primary port (used only when robot_name='').",
        ),
        DeclareLaunchArgument(
            "ur_type", default_value="ur10e", choices=_UR_MODEL_CHOICES,
            description="UR model for the URDF (used only when robot_name='').",
        ),
        DeclareLaunchArgument(
            "joint_prefix", default_value="",
            description="Advanced. Prefix for URDF link/joint names and "
                        "JointState names. Used by the dual_ur_bringup "
                        "pipeline; leave empty for plain single-arm "
                        "visualization.",
        ),
    ]

    return LaunchDescription(args + [OpaqueFunction(function=_launch_setup)])
