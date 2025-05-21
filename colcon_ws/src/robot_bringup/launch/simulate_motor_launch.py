from launch import LaunchDescription  
from launch.actions import IncludeLaunchDescription, TimerAction  
from launch.launch_description_sources import PythonLaunchDescriptionSource  
from ament_index_python.packages import get_package_share_directory  
import os  
   
def generate_launch_description():  
    # Paths to launch files:  
    # 1) The new "modbus_simulation_launch.py" that starts our fake Modbus bus  
    modbus_sim_path = os.path.join(  
        get_package_share_directory('modbus_driver'),  
        'launch',  
        'modbus_simulation_launch.py'  
    )  
    # 2) The motor simulation launch (creates motor_simulation_node for each motor ID)  
    motor_sim_path = os.path.join(  
        get_package_share_directory('leadshine_motor'),  
        'launch',  
        'test_motor_simulation_launch.py'  
    )  
    # 3) The motor control launch (creates motor_node for each motor ID)  
    motor_control_path = os.path.join(  
        get_package_share_directory('leadshine_motor'),  
        'launch',  
        'motor_control_launch.py'  
    )  
    # 4) Web server launch  
    web_path = os.path.join(  
        get_package_share_directory('robot_web'),  
        'launch',  
        'web_server_launch.py'  
    )  
  
    # Optionally stagger startups with TimerAction:  
    # (so we can ensure modbus_sim is running before motors or control nodes)  
    modbus_sim_launch = TimerAction(  
        period=0.0,  
        actions=[IncludeLaunchDescription(  
            PythonLaunchDescriptionSource(modbus_sim_path)  
        )]  
    )  
  
    motor_sim_launch = TimerAction(  
        period=1.0,  
        actions=[IncludeLaunchDescription(  
            PythonLaunchDescriptionSource(motor_sim_path)  
        )]  
    )  
  
    motor_control_launch = TimerAction(  
        period=2.0,  
        actions=[IncludeLaunchDescription(  
            PythonLaunchDescriptionSource(motor_control_path)  
        )]  
    )  
  
    web_launch = TimerAction(  
        period=3.0,  
        actions=[IncludeLaunchDescription(  
            PythonLaunchDescriptionSource(web_path)  
        )]  
    )  
  
    return LaunchDescription([  
        # Start the fake Modbus bus first  
        modbus_sim_launch,  
  
        # Then the motor simulation nodes  
        motor_sim_launch,  
  
        # Then the motor control nodes  
        motor_control_launch,  
  
        # Finally the web server if needed  
        web_launch  
    ])  