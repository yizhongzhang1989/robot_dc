from launch import LaunchDescription  
from launch_ros.actions import Node  
   
def generate_launch_description():  
    return LaunchDescription([  
        Node(  
            package='leadshine_motor',  
            executable='motor_simulation_node',  
            name='motor_sim_1',  
            namespace='motor1',  
            parameters=[{'motor_id': 1}],  
            emulate_tty=True,
            output='screen',  
        ),  
        Node(  
            package='leadshine_motor',  
            executable='motor_simulation_node',  
            name='motor_sim_2',  
            namespace='motor2',  
            parameters=[{'motor_id': 2}],  
            emulate_tty=True,
            output='screen',  
        ),  
    ])  