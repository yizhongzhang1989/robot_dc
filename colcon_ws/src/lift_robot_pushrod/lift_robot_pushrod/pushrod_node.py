#!/usr/bin/env python3
"""
Lift Robot Pushrod ROS2 Node

Controls the pushrod using discrete relay pulses (relay3 stop, relay4 up, relay5 down).
Each action sends a single-coil ON (0xFF00) followed ~100ms later by OFF (0x0000),
mirroring the lift platform controller style. Timed commands issue a direction pulse
then schedule a stop pulse after the specified duration.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .pushrod_controller import PushrodController
import json
import uuid
import logging

logging.basicConfig(level=logging.INFO)

class PushrodNode(Node):
    def __init__(self):
        super().__init__('lift_robot_pushrod')

        # Parameters
        # Default device_id corrected to 50 (0x32) per hardware mapping
        self.declare_parameter('device_id', 50)
        self.declare_parameter('use_ack_patch', True)

        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value

        self.get_logger().info(
            f"Initialize pushrod controller - device_id: {self.device_id} (serial handled by modbus_driver)"
        )

        # Controller
        self.controller = PushrodController(
            device_id=self.device_id,
            node=self,
            use_ack_patch=self.use_ack_patch
        )

        # Command subscription
        self.subscription = self.create_subscription(
            String,
            'lift_robot_pushrod/command',
            self.command_callback,
            10
        )

        # Status publisher
        self.status_publisher = self.create_publisher(
            String,
            'lift_robot_pushrod/status',
            10
        )

        # Timer for status
        self.status_timer = self.create_timer(1.0, self.publish_status)

        # Initialize
        self.controller.initialize()

        self.get_logger().info("Pushrod control node started")

    def command_callback(self, msg):
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '').lower()
            seq_id_str = command_data.get('seq_id', str(uuid.uuid4())[:8])
            seq_id = abs(hash(seq_id_str)) % 65536

            self.get_logger().info(f"Received pushrod command: {command} [SEQ {seq_id_str}]")

            if command == 'stop':
                self.controller.stop(seq_id=seq_id)
            elif command == 'up':
                self.controller.up(seq_id=seq_id)
            elif command == 'down':
                self.controller.down(seq_id=seq_id)
            elif command == 'timed_up':
                duration = command_data.get('duration', 1.0)
                self.controller.timed_up(duration, seq_id=seq_id)
            elif command == 'timed_down':
                duration = command_data.get('duration', 1.0)
                self.controller.timed_down(duration, seq_id=seq_id)
            elif command == 'stop_timed':
                self.controller.stop_timed(seq_id=seq_id)
            else:
                self.get_logger().warning(f"Unknown pushrod command: {command}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Cannot parse command JSON: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error handling pushrod command: {e}")

    def publish_status(self):
        status = {
            'node': 'lift_robot_pushrod',
            'device_id': self.device_id,
            'has_stop_timer': bool(self.controller.stop_timer),
            'status': 'online'
        }
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_publisher.publish(status_msg)

    def destroy_node(self):
        self.get_logger().info("Stopping pushrod control node ...")
        self.controller.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = PushrodNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Pushrod node runtime error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
