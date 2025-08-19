from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    # 声明启动参数
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='1',
        description='Robot arm device ID'
    )
    
    joint_step_size_arg = DeclareLaunchArgument(
        'joint_step_size',
        default_value='0.5',
        description='Joint angle step size in degrees'
    )
    
    tcp_position_step_arg = DeclareLaunchArgument(
        'tcp_position_step',
        default_value='5.0',
        description='TCP position step size in mm'
    )
    
    tcp_rotation_step_arg = DeclareLaunchArgument(
        'tcp_rotation_step',
        default_value='2.0',
        description='TCP rotation step size in degrees'
    )

    # 获取配置文件路径
    config_dir = os.path.join(
        get_package_share_directory('robot_arm_teleop'),
        'config'
    )
    joy_params_file = os.path.join(config_dir, 'joy_params.yaml')

    return LaunchDescription([
        # 声明参数
        device_id_arg,
        joint_step_size_arg,
        tcp_position_step_arg,
        tcp_rotation_step_arg,
        
        # 启动手柄节点
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            parameters=[joy_params_file],
            output='screen'
        ),
        
        # 启动机械臂遥控节点
        Node(
            package='robot_arm_teleop',
            executable='robot_arm_teleop',
            name='robot_arm_teleop',
            parameters=[
                {
                    'device_id': LaunchConfiguration('device_id'),
                    'joint_step_size': LaunchConfiguration('joint_step_size'),
                    'tcp_position_step': LaunchConfiguration('tcp_position_step'),
                    'tcp_rotation_step': LaunchConfiguration('tcp_rotation_step'),
                    'deadzone': 0.1
                }
            ],
            output='screen'
        ),
    ])
