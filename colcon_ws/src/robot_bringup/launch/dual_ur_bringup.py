#!/usr/bin/env python3
"""
Dual UR (ur15 + ur10e) Bringup Launch File

This launch file brings up two UR robots in one ROS graph:
  - ur15  at 192.168.1.15  (default ROS namespace; camera 192.168.1.101)
  - ur10e at 192.168.1.16  (ROS namespace ``/ur10e``; camera 192.168.1.102)

It reuses the existing config-driven pattern: it iterates
``ur15.launch_modules`` (which contains the shared/singleton services *and*
the ur15 per-robot modules) followed by ``ur10e.launch_modules`` (which
contains only the ur10e per-robot modules). The singleton/per-robot split is
therefore encoded entirely in robot_config.yaml.

Modules are spawned as ``ros2 launch <package> <launch_file>`` subprocesses
with an optional per-entry ``delay`` (TimerAction). No launch-arguments are
forwarded; each child launch file reads its own config block.

NOTE: When running this bringup, the duco robot stack MUST NOT also be
running, because ur10e shares the physical camera at 192.168.1.102 with duco.
"""

from launch import LaunchDescription
from launch.actions import ExecuteProcess, TimerAction, LogInfo
from common.config_manager import ConfigManager


def _build_module_actions(robot_name: str, modules):
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

    launch_actions = []
    # ur15 list contains both the shared singleton services and the ur15
    # per-robot modules; start it first so singletons (redis, workflow, etc.)
    # come up before per-robot consumers.
    launch_actions += _build_module_actions('ur15', ur15_modules)
    # ur10e list contains only the per-robot modules (ur_robot_arm, camera,
    # ur_web) configured for ur10e.
    launch_actions += _build_module_actions('ur10e', ur10e_modules)

    return LaunchDescription(launch_actions)
