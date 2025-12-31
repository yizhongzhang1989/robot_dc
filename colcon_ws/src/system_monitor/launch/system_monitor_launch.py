from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for System Monitor"""
    
    system_monitor_node = Node(
        package='system_monitor',
        executable='system_monitor_node',
        name='system_monitor_node',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        system_monitor_node
    ])
