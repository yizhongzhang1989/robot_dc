#!/usr/bin/env python3
"""
UR15 Control Launch File

Starts the upstream UR robot driver under the ``/ur15`` ROS namespace so it
can coexist with the ur10e driver in one ROS graph.

What gets namespaced:
  - ros2_control stack:        /ur15/controller_manager, /ur15/joint_state_broadcaster,
                               /ur15/scaled_joint_trajectory_controller, ...
  - state publishers:          /ur15/robot_state_publisher (URDF) →
                               /ur15/robot_description, /ur15/joint_states
  - dashboard / URScript:      /ur15/dashboard_client, /ur15/urscript_interface

What stays at GLOBAL namespace:
  - /tf, /tf_static            (single TF tree for the whole cell; remapped
                                back to global from inside the namespace so
                                RViz can show both robots together)
  - cameras, workflow, redis,  unchanged — these are singletons.
    positioning, etc.

This requires a one-time vendored copy of ur_control.launch.py with a
two-line patch so its controller spawner uses a relative
``--controller-manager`` argument (see _ur_control_namespaced.launch.py).
"""

from launch import LaunchDescription
from launch.actions import (
    IncludeLaunchDescription,
    DeclareLaunchArgument,
    GroupAction,
)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare
from common.config_manager import ConfigManager


def generate_launch_description():
    # Load configuration
    config = ConfigManager()
    ur15_config = config.get_robot('ur15')

    # Declare arguments with defaults from config
    ur15_ip_arg = DeclareLaunchArgument(
        'ur15_ip',
        default_value=ur15_config.get('robot.ip'),
        description='IP address of the UR15 robot'
    )

    ur_type_arg = DeclareLaunchArgument(
        'ur_type',
        default_value=ur15_config.get('robot.type'),
        description='Type of UR robot'
    )

    launch_rviz_arg = DeclareLaunchArgument(
        'launch_rviz',
        default_value=str(ur15_config.get('robot.launch_rviz')).lower(),
        description='Launch RViz for visualization'
    )

    # Namespace under which the entire ur15 driver stack lives. Defaults to
    # the robot's status_namespace so the ROS topic prefix matches the
    # robot_status_redis key prefix (single source of identity).
    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value=ur15_config.get('robot.status_namespace', 'ur15'),
        description='ROS namespace under which the ur15 driver stack lives '
                    '(controller_manager, joint_states, robot_description). '
                    '/tf and /tf_static remain global.'
    )

    # Get launch configurations
    ur15_ip = LaunchConfiguration('ur15_ip')
    ur_type = LaunchConfiguration('ur_type')
    launch_rviz = LaunchConfiguration('launch_rviz')
    robot_namespace = LaunchConfiguration('robot_namespace')

    # Include the vendored, namespace-friendly copy of ur_control.launch.py
    # from this package (relative spawner --controller-manager).
    ur_control_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('ur_robot_arm'),
                'launch',
                '_ur_control_namespaced.launch.py'
            ])
        ]),
        launch_arguments={
            'ur_type': ur_type,
            'robot_ip': ur15_ip,
            'launch_rviz': launch_rviz,
        }.items()
    )

    # Push the entire ros2_control stack under /<robot_namespace>/, but
    # keep /tf and /tf_static at the GLOBAL namespace so the two robots
    # share a single TF tree (RViz multi-RobotModel display, hand-eye
    # calibrations, world-frame reasoning all work without bridging).
    #
    # SetRemap with a relative source rewrites the topic AFTER namespace
    # resolution: inside the namespace push, robot_state_publisher's
    # default ``tf`` relative topic would resolve to ``/<ns>/tf``, but
    # SetRemap('tf', '/tf') sends it to global ``/tf`` instead. Frame name
    # collisions between robots are avoided via ``tf_prefix`` on the URDF
    # side (ur10e uses ``tf_prefix=ur10e_``; ur15 keeps the unprefixed
    # ``base_link`` for backward compatibility).
    namespaced_stack = GroupAction([
        PushRosNamespace(robot_namespace),
        SetRemap('tf', '/tf'),
        SetRemap('tf_static', '/tf_static'),
        ur_control_launch,
    ])

    return LaunchDescription([
        # Arguments
        ur15_ip_arg,
        ur_type_arg,
        launch_rviz_arg,
        robot_namespace_arg,
        # Launch (namespaced)
        namespaced_stack,
    ])
