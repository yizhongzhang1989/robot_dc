#!/usr/bin/env python3

"""
Duco Robot ROS2 Command Script
Control robot with keyboard keys: q, w, e, r
q - power on
w - power off
e - enable
r - disable

Note that: before run this script, we need to:
cd /home/a/Documents/robot_dc2/colcon_ws
source install/setup.bash
ros2 launch duco_ros_driver duco_robot_control.launch.py
"""

import rclpy
from rclpy.node import Node
from duco_msg.srv import RobotControl
import sys
import termios
import tty
import select


class DucoRobotCommandNode(Node):
    def __init__(self):
        super().__init__('duco_robot_command_node')
        
        # Create service client
        self.client = self.create_client(RobotControl, '/duco_robot/robot_control')
        
        # Wait for service to be available
        while not self.client.wait_for_service(timeout_sec=1.0):
            print('Waiting for robot control service...')
        
        print('Robot control service connected')
        print('=' * 60)
        print('Keyboard Control Instructions:')
        print('  q - Power On')
        print('  w - Power Off')
        print('  e - Enable')
        print('  r - Disable')
        print('  ESC - Exit')
        print('=' * 60)
    
    def log(self, message, level='INFO'):
        """Print message with proper formatting in raw terminal mode"""
        prefix = f'[{level}]'
        print(f'\r{prefix} {message}')
        sys.stdout.flush()
        
    def send_command(self, command, arm_num=0, block=True):
        """
        Send control command to robot
        
        Args:
            command: Command string (poweron, poweroff, enable, disable)
            arm_num: Robot arm number (default: 0)
            block: Block until command completes (default: True)
        """
        # Command descriptions for better user feedback
        command_descriptions = {
            'poweron': 'Powering on robot (this may take 10-20 seconds)',
            'poweroff': 'Powering off robot',
            'enable': 'Enabling robot motors (this may take 5-15 seconds)',
            'disable': 'Disabling robot motors'
        }
        
        request = RobotControl.Request()
        request.command = command
        request.arm_num = arm_num
        request.block = block
        
        self.log('-' * 60)
        self.log(f'📤 Sending command: {command}')
        if command in command_descriptions:
            self.log(f'⏳ {command_descriptions[command]}')
        self.log('⏱  Please wait... (timeout: 30s)')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future, timeout_sec=30.0)
        
        if future.result() is not None:
            response = future.result()
            if response.response == '0':
                self.log(f'✅ Command "{command}" completed successfully!')
            else:
                self.log(f'⚠️  Command "{command}" returned code: {response.response}', 'WARN')
                self.log('    (Non-zero return code may indicate an issue)', 'WARN')
        else:
            self.log(f'❌ Service call timeout for command: {command}', 'ERROR')
            self.log('    The command may still be executing on the robot.', 'ERROR')
            self.log('    Please check robot status manually.', 'ERROR')
        self.log('-' * 60)


def get_key():
    """
    Get single keypress from terminal
    Returns the key pressed as a string
    """
    if select.select([sys.stdin], [], [], 0)[0]:
        return sys.stdin.read(1)
    return None


def main(args=None):
    # Initialize ROS2
    rclpy.init(args=args)
    
    # Create node
    node = DucoRobotCommandNode()
    
    # Save terminal settings
    old_settings = termios.tcgetattr(sys.stdin)
    
    try:
        # Set terminal to cbreak mode (better than raw for output)
        tty.setcbreak(sys.stdin.fileno())
        
        print('\r\nReady for keyboard input...\r\n')
        sys.stdout.flush()
        
        while rclpy.ok():
            # Get key press
            key = get_key()
            
            if key:
                if key == 'q':
                    print('\r\n')
                    node.log('🔑 Key "q" pressed - Powering ON robot')
                    node.send_command('poweron', arm_num=0, block=True)
                    
                elif key == 'w':
                    print('\r\n')
                    node.log('🔑 Key "w" pressed - Powering OFF robot')
                    node.send_command('poweroff', arm_num=0, block=True)
                    
                elif key == 'e':
                    print('\r\n')
                    node.log('🔑 Key "e" pressed - Enabling robot motors')
                    node.send_command('enable', arm_num=0, block=True)
                    
                elif key == 'r':
                    print('\r\n')
                    node.log('🔑 Key "r" pressed - Disabling robot motors')
                    node.send_command('disable', arm_num=0, block=True)
                    
                elif key == '\x1b':  # ESC key
                    print('\r\n')
                    node.log('👋 ESC pressed - Exiting...')
                    break
                    
                elif key == '\x03':  # Ctrl+C
                    print('\r\n')
                    node.log('👋 Ctrl+C pressed - Exiting...')
                    break
            
            # Small delay to prevent CPU overuse
            rclpy.spin_once(node, timeout_sec=0.01)
    
    except KeyboardInterrupt:
        print('\r\n')
        node.log('Keyboard interrupt - Exiting...')
    
    finally:
        # Restore terminal settings
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        # Cleanup
        node.destroy_node()
        rclpy.shutdown()
        print('\r\nProgram terminated\r\n')


if __name__ == '__main__':
    main()
