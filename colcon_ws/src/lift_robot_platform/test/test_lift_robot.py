#!/usr/bin/env python3
"""
Lift platform controller test.
Sends various commands to validate functionality.
"""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class LiftRobotTester(Node):
    def __init__(self):
        super().__init__('lift_robot_tester')

        # Create command publisher
        self.command_publisher = self.create_publisher(
            String,
            'lift_robot_platform/command',
            10
        )

        # Subscribe to status
        self.status_subscription = self.create_subscription(
            String,
            'lift_robot_platform/status',
            self.status_callback,
            10
        )

        self.get_logger().info("Lift platform tester started")

    def status_callback(self, msg):
        """Status callback"""
        try:
            status = json.loads(msg.data)
            self.get_logger().info(f"Status: {status}")
        except Exception:
            pass

    def send_command(self, command, **kwargs):
        """Send a command"""
        cmd_data = {'command': command}
        cmd_data.update(kwargs)
        msg = String()
        msg.data = json.dumps(cmd_data)
        self.command_publisher.publish(msg)
        self.get_logger().info(f"Send command: {cmd_data}")

    def run_test(self):
        """Run test sequence"""
        self.get_logger().info("Start test sequence ...")

        # Wait node startup
        time.sleep(2)

        # Basic commands
        self.get_logger().info("=== BASIC COMMANDS ===")
        self.send_command('stop')
        time.sleep(1)
        self.send_command('up')
        time.sleep(2)
        self.send_command('stop')
        time.sleep(1)
        self.send_command('down')
        time.sleep(2)
        self.send_command('stop')
        time.sleep(1)

        # Timed commands
        self.get_logger().info("=== TIMED COMMANDS ===")
        self.send_command('timed_up', duration=3.0)
        time.sleep(5)  # wait for auto stop
        self.send_command('timed_down', duration=2.0)
        time.sleep(4)  # wait for auto stop

        # Manual stop of timed motion
        self.get_logger().info("=== MANUAL STOP TIMED ===")
        self.send_command('timed_up', duration=10.0)
        time.sleep(2)
        self.send_command('stop_timed')  # early stop

        self.get_logger().info("Test completed!")


def main():
    rclpy.init()
    try:
        tester = LiftRobotTester()
        # Run test
        tester.run_test()
        # Continue listening for status
        rclpy.spin(tester)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
