from launch import LaunchDescription  
from launch_ros.actions import Node  
  
def generate_launch_description():  
    return LaunchDescription([  
  
        # Start the "fake bus":  
        Node(  
            package='modbus_driver',  
            # or wherever you put modbus_simulation_node.py  
            executable='modbus_simulation_node',  
            name='modbus_sim',  
            output='screen',  
            emulate_tty=True,  
            parameters=[{'motor_names': ['motor1','motor2']}]  
        ),  
  
    ])  