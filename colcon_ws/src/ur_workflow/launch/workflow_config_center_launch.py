#!/usr/bin/env python3
"""
Launch file for Workflow Config Center

This launch file starts the workflow configuration center web service.
The service provides a web interface for managing workflow configuration files.

Usage:
    ros2 launch ur_workflow workflow_config_center_launch.py
    ros2 launch ur_workflow workflow_config_center_launch.py port:=8009
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from common.config_manager import ConfigManager
from common.workspace_utils import get_workspace_root
import os


# UR robot config keys we want to expose to the workflow editor for live
# joint-pose readback. The set is small and known, so we hard-list it here
# instead of trying to enumerate every top-level key in robot_config.yaml
# (which would also include non-UR robots / lift / cam).
_UR_ROBOT_KEYS = ('ur15', 'ur10e')


def _build_robot_args(context, *args, **kwargs):
    """Append one --robot NAME:IP:PORT flag per UR robot in robot_config.yaml.

    Done inside an OpaqueFunction so we can read the config at launch
    time (the launch arg defaults are also pulled from the config but
    this function is what wires the multi-robot --robot flag).
    """
    config = ConfigManager()
    workspace_root = get_workspace_root()
    launch_script = os.path.join(workspace_root, 'colcon_ws', 'src', 'ur_workflow',
                                 'web', 'workflow_api.py')

    host = LaunchConfiguration('host').perform(context)
    port = LaunchConfiguration('port').perform(context)

    cmd = ['python3', launch_script, '--host', host, '--port', port]

    registered = []
    for name in _UR_ROBOT_KEYS:
        try:
            robot_cfg = config.get_robot(name)
        except Exception:
            continue
        ip = robot_cfg.get('robot.ip')
        ctrl_port = robot_cfg.get('robot.ports.control', 30002)
        if not ip:
            continue
        cmd.extend(['--robot', f'{name}:{ip}:{ctrl_port}'])
        registered.append((name, ip, ctrl_port))

    if not registered:
        print('[workflow_config_center_launch] WARNING: no UR robots found in '
              'robot_config.yaml; the editor "Load Joint Angles" button will '
              'return 503.')
    else:
        for name, ip, p in registered:
            print(f'[workflow_config_center_launch] --robot {name}:{ip}:{p}')

    return [ExecuteProcess(
        cmd=cmd,
        output='screen',
        name='workflow_config_center_web',
        sigterm_timeout='2',
        sigkill_timeout='2',
        emulate_tty=True,
    )]


def generate_launch_description():
    """Generate launch description for workflow config center service."""

    # Load configuration just to compute default args.
    config = ConfigManager()
    service_config = config.get('services.workflow_config_center')

    port_arg = DeclareLaunchArgument(
        'port',
        default_value=str(service_config['port']),
        description='Port for the web service'
    )

    host_arg = DeclareLaunchArgument(
        'host',
        default_value=service_config['host'],
        description='Host address for the web service'
    )

    return LaunchDescription([
        port_arg,
        host_arg,
        OpaqueFunction(function=_build_robot_args),
    ])
