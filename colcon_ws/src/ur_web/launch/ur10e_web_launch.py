#!/usr/bin/env python3
"""
UR10e Web Launch File

Starts a second ``ur_web_node`` instance (the executable shipped by the
``ur_web`` package) configured from ``ur10e.web`` / ``ur10e.robot`` in
robot_config.yaml. Uses a distinct ROS node name (``ur10e_web_node``)
and disjoint web port so it coexists with the ur15 web.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, GroupAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node, PushRosNamespace, SetRemap
from common.config_manager import ConfigManager
from common.workspace_utils import get_workspace_root
from pathlib import Path


def generate_launch_description():
    config = ConfigManager()
    ur10e_config = config.get_robot('ur10e')

    workspace_root = Path(get_workspace_root())

    def _resolve(rel_or_abs: str) -> str:
        p = Path(rel_or_abs)
        return str(p if p.is_absolute() else workspace_root / p)

    dataset_path = _resolve(ur10e_config.get('web.dataset_path'))
    calib_data_path = _resolve(ur10e_config.get('web.calibration_data_path'))
    calib_result_path = _resolve(ur10e_config.get('web.calibration_result_path'))
    chessboard_config_path = _resolve(ur10e_config.get('web.chessboard_config_path'))

    ur_ip_arg = DeclareLaunchArgument(
        'ur10e_ip',
        default_value=ur10e_config.get('robot.ip'),
        description='IP address of the UR10e robot'
    )

    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='ur10e',
        description='Namespace used for this robot in robot_status_redis '
                    '(matches the top-level robot key in robot_config.yaml).'
    )

    robot_type_arg = DeclareLaunchArgument(
        'robot_type',
        default_value=ur10e_config.get('robot.type', 'ur10e'),
        description='Robot model identifier (e.g. ur15, ur10e) published to '
                    'robot_status_redis so workflow consumers can discover it'
    )

    joint_prefix_arg = DeclareLaunchArgument(
        'joint_prefix',
        default_value='ur10e_',
        description='Prefix used on the joint names on /joint_states that '
                    'belong to this robot. The web node uses this to filter '
                    'messages from the other robot on the shared topic and '
                    'to look up J1..J6 by name.'
    )

    camera_topic_arg = DeclareLaunchArgument(
        'camera_topic',
        default_value=ur10e_config.get('web.camera_topic'),
        description='UR10e camera topic name'
    )

    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value=str(ur10e_config.get('web.port')),
        description='Web server port'
    )

    ur_port_arg = DeclareLaunchArgument(
        'ur10e_port',
        default_value=str(ur10e_config.get('robot.ports.control')),
        description='UR10e robot control port'
    )

    dataset_dir_arg = DeclareLaunchArgument(
        'dataset_dir',
        default_value=dataset_path,
        description='Directory for storing dataset files'
    )

    calib_data_dir_arg = DeclareLaunchArgument(
        'calib_data_dir',
        default_value=calib_data_path,
        description='Directory for camera calibration data'
    )

    calib_result_dir_arg = DeclareLaunchArgument(
        'calib_result_dir',
        default_value=calib_result_path,
        description='Directory for camera calibration results'
    )

    chessboard_config_arg = DeclareLaunchArgument(
        'chessboard_config',
        default_value=chessboard_config_path,
        description='JSON file containing chessboard pattern configuration'
    )

    # Shared service ports (read from services.* — same singleton instances as ur15)
    all_config = config.get_all()
    services_config = all_config.get('services', {})
    image_labeling_port = services_config.get('image_labeling', {}).get('port', 8007)
    workflow_config_center_port = services_config.get('workflow_config_center', {}).get('port', 8008)

    image_labeling_port_arg = DeclareLaunchArgument(
        'image_labeling_port',
        default_value=str(image_labeling_port),
        description='Port for image labeling service (shared singleton)'
    )

    workflow_config_center_port_arg = DeclareLaunchArgument(
        'workflow_config_center_port',
        default_value=str(workflow_config_center_port),
        description='Port for workflow config center service (shared singleton)'
    )

    ur_ip = LaunchConfiguration('ur10e_ip')
    camera_topic = LaunchConfiguration('camera_topic')
    web_port = LaunchConfiguration('web_port')
    ur_port = LaunchConfiguration('ur10e_port')
    dataset_dir = LaunchConfiguration('dataset_dir')
    calib_data_dir = LaunchConfiguration('calib_data_dir')
    calib_result_dir = LaunchConfiguration('calib_result_dir')
    chessboard_config = LaunchConfiguration('chessboard_config')
    image_labeling_port_cfg = LaunchConfiguration('image_labeling_port')
    workflow_config_center_port_cfg = LaunchConfiguration('workflow_config_center_port')
    robot_namespace = LaunchConfiguration('robot_namespace')
    robot_type = LaunchConfiguration('robot_type')
    joint_prefix = LaunchConfiguration('joint_prefix')

    # Distinct ROS node name so this instance does not collide with the
    # ur15 web instance (both run the same ``ur_web_node`` executable).
    ur10e_web_node = Node(
        package='ur_web',
        executable='ur_web_node',
        name='ur10e_web_node',
        output='screen',
        parameters=[{
            'camera_topic': camera_topic,
            'web_port': web_port,
            'ur_ip': ur_ip,
            'ur_port': ur_port,
            'dataset_dir': dataset_dir,
            'calib_data_dir': calib_data_dir,
            'calib_result_dir': calib_result_dir,
            'chessboard_config': chessboard_config,
            'image_labeling_port': image_labeling_port_cfg,
            'workflow_config_center_port': workflow_config_center_port_cfg,
            'robot_namespace': robot_namespace,
            'robot_type': robot_type,
            'joint_prefix': joint_prefix,
        }]
    )
    
    # Wrap under /<robot_namespace>/ (matching ur10e_control_launch.py),
    # but keep /tf and /tf_static global so RViz can show both robots
    # from a single TF tree.
    namespaced_web = GroupAction([
        PushRosNamespace(robot_namespace),
        SetRemap('tf', '/tf'),
        SetRemap('tf_static', '/tf_static'),
        ur10e_web_node,
    ])

    return LaunchDescription([
        ur_ip_arg,
        camera_topic_arg,
        web_port_arg,
        ur_port_arg,
        dataset_dir_arg,
        calib_data_dir_arg,
        calib_result_dir_arg,
        chessboard_config_arg,
        image_labeling_port_arg,
        workflow_config_center_port_arg,
        robot_namespace_arg,
        robot_type_arg,
        joint_prefix_arg,
        namespaced_web,
    ])
