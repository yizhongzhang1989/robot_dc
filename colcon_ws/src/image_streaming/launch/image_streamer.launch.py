from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for image streaming."""
    
    # Declare launch arguments
    image_topic_arg = DeclareLaunchArgument(
        'image_topic',
        default_value='/camera/image_raw',
        description='ROS 2 image topic to stream'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='HTTP server port'
    )
    
    quality_arg = DeclareLaunchArgument(
        'quality',
        default_value='85',
        description='JPEG compression quality (0-100)'
    )
    
    # Create the image streamer node
    image_streamer_node = Node(
        package='image_streaming',
        executable='image_streamer',
        name='image_streamer',
        output='screen',
        parameters=[{
            'image_topic': LaunchConfiguration('image_topic'),
            'port': LaunchConfiguration('port'),
            'quality': LaunchConfiguration('quality'),
        }]
    )
    
    return LaunchDescription([
        image_topic_arg,
        port_arg,
        quality_arg,
        image_streamer_node,
    ])
