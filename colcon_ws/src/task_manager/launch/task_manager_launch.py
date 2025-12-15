from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    """Generate launch description for Task Manager"""
    
    task_manager_node = Node(
        package='task_manager',
        executable='task_manager_node',
        name='task_manager_node',
        output='screen',
        emulate_tty=True,
    )
    
    return LaunchDescription([
        task_manager_node
    ])
