#!/usr/bin/env python3
"""
Example script to test Robotiq gripper ROS2 node
"""

import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from robotiq_gripper_msgs.msg import GripperCommand, GripperStatus
import time


class GripperTester(Node):
    def __init__(self):
        super().__init__('gripper_tester')
        
        # Create publisher for commands
        self.command_pub = self.create_publisher(
            GripperCommand,
            '/gripper/command',
            10
        )
        
        # Create subscriber for status
        self.status_sub = self.create_subscription(
            GripperStatus,
            '/gripper/status',
            self.status_callback,
            10
        )
        
        # Create service client for activation
        self.activate_client = self.create_client(Trigger, '/gripper/activate')
        
        self.latest_status = None
        
        self.get_logger().info('Gripper Tester initialized')
    
    def status_callback(self, msg: GripperStatus):
        """Store latest status."""
        self.latest_status = msg
    
    def activate(self):
        """Activate the gripper."""
        self.get_logger().info('Activating gripper...')
        
        while not self.activate_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for activation service...')
        
        request = Trigger.Request()
        future = self.activate_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result().success:
            self.get_logger().info('Gripper activated successfully')
            return True
        else:
            self.get_logger().error(f'Activation failed: {future.result().message}')
            return False
    
    def send_command(self, position: int, speed: int = 255, force: int = 255):
        """Send a command to the gripper."""
        msg = GripperCommand()
        msg.position = position
        msg.speed = speed
        msg.force = force
        msg.emergency_stop = False
        
        self.command_pub.publish(msg)
        self.get_logger().info(f'Sent command: position={position}, speed={speed}, force={force}')
    
    def close_gripper(self, speed: int = 255, force: int = 255):
        """Close the gripper."""
        self.get_logger().info('Closing gripper...')
        self.send_command(255, speed, force)
    
    def open_gripper(self, speed: int = 255):
        """Open the gripper."""
        self.get_logger().info('Opening gripper...')
        self.send_command(0, speed, 0)
    
    def get_status(self):
        """Get the latest gripper status."""
        return self.latest_status
    
    def wait_for_motion_complete(self, timeout: float = 5.0):
        """Wait for gripper to stop moving."""
        start_time = time.time()
        while time.time() - start_time < timeout:
            rclpy.spin_once(self, timeout_sec=0.1)
            if self.latest_status and not self.latest_status.is_moving:
                self.get_logger().info('Motion complete')
                return True
        self.get_logger().warn('Motion timeout')
        return False


def main():
    rclpy.init()
    
    tester = GripperTester()
    
    try:
        # Wait for status messages
        print('\nWaiting for gripper status...')
        for _ in range(10):
            rclpy.spin_once(tester, timeout_sec=0.5)
            if tester.latest_status:
                break
        
        if not tester.latest_status:
            print('ERROR: No status messages received. Is the gripper node running?')
            print('Start it with: ros2 launch robotiq2f140gripper robotiq_gripper.launch.py')
            return
        
        # Activate gripper
        print('\n1. Activating gripper...')
        if not tester.activate():
            print('ERROR: Failed to activate gripper')
            return
        
        time.sleep(1)
        
        # Open gripper
        print('\n2. Opening gripper...')
        tester.open_gripper(speed=255)
        tester.wait_for_motion_complete()
        
        time.sleep(1)
        
        # Close gripper
        print('\n3. Closing gripper...')
        tester.close_gripper(speed=255, force=255)
        tester.wait_for_motion_complete()
        
        # Check status
        print('\n4. Checking status...')
        rclpy.spin_once(tester, timeout_sec=0.5)
        status = tester.get_status()
        if status:
            print(f'   Activated: {status.is_activated}')
            print(f'   Moving: {status.is_moving}')
            print(f'   Position: {status.position}')
            print(f'   Object detected: {status.object_detected}')
        
        time.sleep(1)
        
        # Move to 50%
        print('\n5. Moving to 50% position...')
        tester.send_command(128, speed=200, force=150)
        tester.wait_for_motion_complete()
        
        time.sleep(1)
        
        # Final open
        print('\n6. Final open...')
        tester.open_gripper()
        tester.wait_for_motion_complete()
        
        print('\n=== Test Complete ===')
        
    except KeyboardInterrupt:
        pass
    finally:
        tester.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
