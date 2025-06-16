from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='feetech_servo',
            executable='servo_node',
            name='servo11',
            output='screen',
            emulate_tty=True,
            parameters=[{'motor_id': 11}],
        ),
        Node(
            package='feetech_servo',
            executable='servo_node',
            name='servo12',
            output='screen',
            emulate_tty=True,
            parameters=[{'motor_id': 12}],
        ),
    ])
