#!/usr/bin/env python3
"""
Dual UR (ur15 + ur10e) Bringup Launch File

This launch file brings up two UR robots in one ROS graph:
  - ur15  at 192.168.1.15  (under /ur15  ROS namespace; camera 192.168.1.101)
  - ur10e at 192.168.1.16  (under /ur10e ROS namespace; camera 192.168.1.102)

ROS topology
============
Each robot's ros2_control / mock stack lives under its own ROS namespace
(``/ur15``, ``/ur10e``). This is enforced by each robot's *_control_launch.py
and *_web_launch.py wrapping their nodes in PushRosNamespace. Topics that
matter for RViz / control are therefore disjoint:

  /ur15/robot_description         /ur10e/robot_description
  /ur15/joint_states              /ur10e/joint_states
  /ur15/controller_manager/...    /ur10e/controller_manager/...
  /ur15/dashboard_client/...      /ur10e/dashboard_client/...

The TF tree is intentionally kept GLOBAL — each robot's
robot_state_publisher remaps its relative ``tf`` and ``tf_static`` outputs
back to ``/tf`` and ``/tf_static`` from inside the namespace push. Frame
name collisions are avoided via per-robot URDF tf_prefix (ur10e uses
``ur10e_``; ur15 keeps unprefixed link names for backward compat). This
gives one shared world for the cell — hand-eye calibrations, cross-robot
reasoning, and a single RViz view all work without TF bridging.

This launch file also publishes static transforms anchoring each robot
under a ``world`` frame so RViz can show both side by side. EDIT the
``WORLD_ANCHORS`` block below to match the real cell layout.

When running this bringup, the duco robot stack MUST NOT also be running,
because ur10e shares the physical camera at 192.168.1.102 with duco.
"""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    ExecuteProcess,
    GroupAction,
    IncludeLaunchDescription,
    LogInfo,
    TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from common.config_manager import ConfigManager


# ---------------------------------------------------------------------------
# WORLD ANCHORS — static transforms from a shared 'world' frame to each
# robot's base. Edit (x, y, z, yaw, pitch, roll) per arm in YOUR cell.
# The defaults below place the two arms 1.5 m apart along +X (good for
# visualization-only setups).
# ---------------------------------------------------------------------------
WORLD_ANCHORS = [
    # (parent, child,  x,    y,   z,   yaw, pitch, roll)
    ('world', 'base_link',         0.0, 0.0, 0.0, 0.0, 0.0, 0.0),  # ur15
    ('world', 'ur10e_base_link',   1.5, 0.0, 0.0, 0.0, 0.0, 0.0),  # ur10e
]


def _world_anchor_actions():
    """Build static_transform_publisher nodes for every entry in WORLD_ANCHORS."""
    actions = []
    for parent, child, x, y, z, yaw, pitch, roll in WORLD_ANCHORS:
        actions.append(Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name=f'static_tf_{parent}_to_{child}'.replace('/', '_'),
            arguments=[
                str(x), str(y), str(z),
                str(yaw), str(pitch), str(roll),
                parent, child,
            ],
            output='screen',
        ))
    return actions


def _build_module_actions(robot_name, modules):
    """Convert a launch_modules YAML list into LaunchDescription actions."""
    actions = [LogInfo(msg=f'[dual_ur_bringup] === starting modules for {robot_name} ===')]
    for module in modules:
        if not module.get('enabled', True):
            continue

        package = module['package']
        launch_file = module['launch_file']
        delay = module.get('delay', 0.0)

        action = ExecuteProcess(
            cmd=['ros2', 'launch', package, launch_file],
            output='screen',
            shell=False,
        )

        if delay > 0:
            action = TimerAction(period=float(delay), actions=[action])

        actions.append(action)
    return actions


def generate_launch_description():
    config = ConfigManager()

    ur15_modules = config.get_robot('ur15').get('launch_modules', []) or []
    ur10e_modules = config.get_robot('ur10e').get('launch_modules', []) or []

    # CLI flag to enable/disable the dual-robot RViz window.
    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='true',
        description='Launch a single RViz window pre-configured to show '
                    'both robots from their namespaced /robot_description '
                    'topics. Set false for headless bringup.',
    )

    rviz_config = PathJoinSubstitution([
        FindPackageShare('mock_ur_visualization'),
        'config',
        'dual_ur.rviz',
    ])
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='dual_ur_rviz',
        arguments=['-d', rviz_config],
        output='log',
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    launch_actions = []
    launch_actions.append(rviz_arg)

    # Singletons + ur15 per-robot modules (singletons live in
    # ur15.launch_modules by historical convention).
    launch_actions += _build_module_actions('ur15', ur15_modules)
    # ur10e per-robot modules.
    launch_actions += _build_module_actions('ur10e', ur10e_modules)

    # World-frame anchors so both robots share a 'world' frame in /tf.
    # Delay slightly so the per-robot RSPs are running before the static
    # publishers come up (helps RViz pick frames up in order).
    launch_actions.append(
        TimerAction(
            period=3.0,
            actions=[
                LogInfo(msg='[dual_ur_bringup] === publishing world anchors ==='),
                *_world_anchor_actions(),
            ],
        )
    )

    # RViz comes up after a longer delay so the latched /<robot>/robot_description
    # topics have already been published — otherwise RViz may show empty
    # RobotModel until the next URDF refresh.
    launch_actions.append(
        TimerAction(period=8.0, actions=[
            LogInfo(msg='[dual_ur_bringup] === starting dual RViz ==='),
            rviz_node,
        ])
    )

    return LaunchDescription(launch_actions)
