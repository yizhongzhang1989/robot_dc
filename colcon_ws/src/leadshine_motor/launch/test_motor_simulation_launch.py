from launch import LaunchDescription  
from launch_ros.actions import Node  
   
def generate_launch_description():  
    return LaunchDescription([  
        Node(  
            package='leadshine_motor',  
            executable='motor_simulation_node',  
            name='sim_motor1',  
            parameters=[{'device_id': 1}],  
            emulate_tty=True,
            output='screen',  
        ),  
        Node(  
            package='leadshine_motor',  
            executable='motor_simulation_node',  
            name='sim_motor2',  
            parameters=[{'device_id': 2}],  
            emulate_tty=True,
            output='screen',  
        ),  
    ])  