from launch import LaunchDescription
from launch.actions import TimerAction, DeclareLaunchArgument
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    # Declare launch arguments
    robot_ip_arg = DeclareLaunchArgument(
        'robot_ip',
        default_value='192.168.1.10',
        description='IP address of the DUCO robot arm'
    )
    
    robot_port_arg = DeclareLaunchArgument(
        'robot_port',
        default_value='7003',
        description='Port number of the DUCO robot arm'
    )
    
    robot_state_port_arg = DeclareLaunchArgument(
        'robot_state_port',
        default_value='2001',
        description='Port number for robot state monitoring'
    )
    
    device_id_arg = DeclareLaunchArgument(
        'device_id',
        default_value='1',
        description='Device ID for the robot arm'
    )
    
    web_port_arg = DeclareLaunchArgument(
        'web_port',
        default_value='8080',
        description='Port for the web interface'
    )
    
    # Camera parameters
    camera_rtsp_1080p_arg = DeclareLaunchArgument(
        'camera_rtsp_1080p',
        default_value='rtsp://admin:123456@192.168.1.102/stream0',
        description='RTSP URL for 1080p camera stream'
    )
    
    camera_rtsp_360p_arg = DeclareLaunchArgument(
        'camera_rtsp_360p',
        default_value='rtsp://admin:123456@192.168.1.102/stream1',
        description='RTSP URL for 360p camera stream'
    )
    
    camera_ip_arg = DeclareLaunchArgument(
        'camera_ip',
        default_value='192.168.1.102',
        description='IP address of the camera'
    )
    
    camera_port_arg = DeclareLaunchArgument(
        'camera_port',
        default_value='8012',
        description='Port for camera web interface'
    )

    # Robot arm node
    robot_arm_node = Node(
        package='duco_robot_arm',
        executable='duco_robot_arm_node',
        name='duco_robot_arm_node',
        parameters=[
            {'ip': LaunchConfiguration('robot_ip')},
            {'port': LaunchConfiguration('robot_port')},
            {'device_id': LaunchConfiguration('device_id')}
        ],
        output='screen',
        emulate_tty=True,
    )

    # Robot state monitoring node
    robot_state_node = Node(
        package='duco_robot_arm_state',
        executable='duco_robot_arm_state_node',
        name='duco_robot_arm_state_node',
        parameters=[
            {'ip': LaunchConfiguration('robot_ip')},
            {'port': LaunchConfiguration('robot_state_port')},
            {'device_id': LaunchConfiguration('device_id')}
        ],
        output='screen',
        emulate_tty=True,
    )

    # Robot arm web interface - launch with a small delay to ensure robot arm node is ready
    robot_arm_web_node = TimerAction(
        period=2.0,
        actions=[
            Node(
                package='robot_arm_web',
                executable='robot_arm_web_server',
                name='robot_arm_web_server',
                parameters=[
                    {'device_id': LaunchConfiguration('device_id')},
                    {'port': LaunchConfiguration('web_port')}
                ],
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    # Robot arm camera node - launch with a small delay to ensure system is ready
    robot_arm_cam_node = TimerAction(
        period=1.0,
        actions=[
            Node(
                package='camera_node',
                executable='camera_node',
                name='robot_arm_cam',
                parameters=[
                    {'camera_name': 'RobotArmCamera'},
                    {'rtsp_url_main': LaunchConfiguration('camera_rtsp_1080p')},
                    {'camera_ip': LaunchConfiguration('camera_ip')},
                    {'server_port': LaunchConfiguration('camera_port')},
                    {'ros_topic_name': '/robot_arm_camera/image_raw'},
                    {'stream_fps': 25},
                    {'jpeg_quality': 75},
                    {'max_width': 800},
                    {'publish_ros_image': True}  # Event-driven publishing (auto-matches camera fps)
                ],
                output='screen',
                emulate_tty=True,
            )
        ]
    )

    # Image process node - launch after camera node is ready
    image_process_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='image_process',
                executable='image_process_node',
                name='image_process_node',
                parameters=[
                    {'input_topic': '/robot_arm_camera/image_raw'},
                    {'output_resized_topic': '/robot_arm_camera/image_resized_raw'},
                    {'resize_width': 640}
                ],
                output='screen',
                emulate_tty=True,
            )
        ]
    )



    return LaunchDescription([
        robot_ip_arg,
        robot_port_arg,
        robot_state_port_arg,
        device_id_arg,
        web_port_arg,
        camera_rtsp_1080p_arg,
        camera_rtsp_360p_arg,
        camera_ip_arg,
        camera_port_arg,
        robot_arm_node,
        robot_state_node,
        robot_arm_web_node,
        robot_arm_cam_node,
        image_process_node
    ])
