#!/usr/bin/env python3
"""
UR10e Control Launch File — URScript-only path, namespaced.

Brings up just enough to visualise and operate the ur10e robot inside
``dual_ur_bringup`` under the ``/ur10e`` ROS namespace:

  * robot_state_publisher    URDF with ``tf_prefix=ur10e_``. Publishes to
                             /ur10e/robot_description and /tf (remapped
                             back to global from inside the namespace).
  * mock_ur_joint_publisher  Polls the robot's URScript port 30002 and
                             publishes /ur10e/joint_states. Joint names
                             carry the ``ur10e_`` prefix so the global
                             /tf tree has no frame-name collision with
                             ur15.

Why URScript polling (not ros2_control)?
    ur_robot_driver / ros2_control require the External Control URCap to
    be installed on the controller. We deliberately keep ur10e on the
    URScript-poll path for simplicity — it is functionally sufficient
    for everything ur_web / ur_workflow does (visualisation + URScript
    motion commands via TCP 30002).

Why namespaced (vs older shared-/joint_states design)?
    /robot_description and /joint_states are TRANSIENT_LOCAL latched
    topics. With two unnamespaced publishers (ur15 + ur10e), RViz's
    single-topic RobotModel display sees both latched URDFs in rapid
    succession and flickers. Per-robot namespacing eliminates the
    collision entirely.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument, GroupAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import PushRosNamespace, SetRemap
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='ur10e',
        description='Robot name in robot_config.yaml (used by mock_ur_visualization '
                    'to look up robot_ip / robot.ports.control / robot.type).'
    )

    # tf_prefix is still needed because /tf and /tf_static are kept at
    # GLOBAL namespace (remapped out of the push below) so all frame names
    # share a single tree — ur10e_base_link is disjoint from ur15's
    # base_link, which is what allows RViz to render both robots side by
    # side without coordinating their TF subscriptions.
    joint_prefix_arg = DeclareLaunchArgument(
        'joint_prefix',
        default_value='ur10e_',
        description='Prefix prepended to every joint name in the URDF and on '
                    'the published JointState messages. /tf stays global so '
                    'this is what keeps ur10e frames disjoint from ur15.'
    )

    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='30.0',
        description='URScript poll / JointState publish rate (Hz).'
    )

    robot_namespace_arg = DeclareLaunchArgument(
        'robot_namespace',
        default_value='ur10e',
        description='ROS namespace under which the ur10e stack lives. '
                    'Topics: /<ns>/joint_states, /<ns>/robot_description. '
                    '/tf and /tf_static stay global.'
    )

    robot_name = LaunchConfiguration('robot_name')
    joint_prefix = LaunchConfiguration('joint_prefix')
    rate_hz = LaunchConfiguration('rate_hz')
    robot_namespace = LaunchConfiguration('robot_namespace')

    # Delegate to mock_ur_visualization's visualize.launch.py with RViz
    # disabled (the visualization is provided by dual_ur_bringup's shared
    # RViz config, not per-robot).
    visualize_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('mock_ur_visualization'),
                'launch',
                'visualize.launch.py',
            ])
        ]),
        launch_arguments={
            'robot_name': robot_name,
            'joint_prefix': joint_prefix,
            'rate_hz': rate_hz,
            'rviz': 'false',
            # We push our own namespace below; tell visualize.launch.py to
            # NOT push its default '/mock' or we'd get '/ur10e/mock/...'.
            'namespace': '',
        }.items()
    )

    # Push the entire stack under /<robot_namespace>/. tf and tf_static
    # are remapped back to global so the cell has a single shared TF tree.
    # See ur15_control_launch.py for the full rationale.
    namespaced_stack = GroupAction([
        PushRosNamespace(robot_namespace),
        SetRemap('tf', '/tf'),
        SetRemap('tf_static', '/tf_static'),
        visualize_launch,
    ])

    return LaunchDescription([
        robot_name_arg,
        joint_prefix_arg,
        rate_hz_arg,
        robot_namespace_arg,
        namespaced_stack,
    ])
