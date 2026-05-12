#!/usr/bin/env python3
"""
UR10e Control Launch File — URScript-only path

Brings up just enough to visualise and operate the ur10e robot inside
``dual_ur_bringup``:

  * robot_state_publisher  with the ur10e URDF and ``tf_prefix=ur10e_`` so
    every link/joint frame in /tf is prefixed (no collision with ur15).
  * joint_publisher        from ``mock_ur_visualization`` — polls the robot's
    URScript primary port at 30002 and publishes ``sensor_msgs/JointState``
    onto the shared ``/joint_states`` topic. Joint names are prefixed with
    ``ur10e_`` so they don't collide with ur15's joint_state_broadcaster
    output.

Why we DO NOT include ``ur_robot_driver/ur_control.launch.py``:
    Upstream ``ur_control.launch.py`` hard-codes ``--controller-manager
    /controller_manager`` as an *absolute* path for its controller spawner
    nodes. Wrapping the launch in ``PushRosNamespace('ur10e')`` moves the
    controller_manager to ``/ur10e/controller_manager`` but the spawners
    still target ``/controller_manager`` (absolute), so controller spawn
    silently fails and no joint_state_broadcaster ever publishes. Multi-robot
    operation via ros2_control therefore requires patching the upstream
    launch file, which we avoid here.

    The rest of this codebase drives the arm via direct URScript (see
    ``ur_robot_arm.ur15.UR15Robot``) — ros2_control is not actually required
    for ur10e. The URScript-polling path used in this launch is functionally
    equivalent for visualization and the URScript-based controllers that
    ur_web / ur_workflow use.

This launch file is intended to be invoked from ``dual_ur_bringup.py``; it can
also be launched standalone for ur10e-only testing.
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    robot_name_arg = DeclareLaunchArgument(
        'robot_name',
        default_value='ur10e',
        description='Robot name in robot_config.yaml (used by mock_ur_visualization '
                    'to look up robot_ip / robot.ports.control / robot.type).'
    )

    joint_prefix_arg = DeclareLaunchArgument(
        'joint_prefix',
        default_value='ur10e_',
        description='Prefix prepended to every joint name in the URDF and on '
                    'the published JointState messages. Must be non-empty so '
                    'ur10e joints do not collide with ur15 joint names on the '
                    'shared /joint_states topic.'
    )

    rate_hz_arg = DeclareLaunchArgument(
        'rate_hz',
        default_value='30.0',
        description='URScript poll / JointState publish rate (Hz).'
    )

    robot_name = LaunchConfiguration('robot_name')
    joint_prefix = LaunchConfiguration('joint_prefix')
    rate_hz = LaunchConfiguration('rate_hz')

    # Delegate to mock_ur_visualization's visualize.launch.py with RViz disabled
    # (we don't want dual_ur_bringup to pop an RViz window). It starts our own
    # robot_state_publisher + URScript-polling joint_publisher.
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
        }.items()
    )

    return LaunchDescription([
        robot_name_arg,
        joint_prefix_arg,
        rate_hz_arg,
        visualize_launch,
    ])

