#!/usr/bin/env python3
"""
Lift Robot Platform ROS2 Node
Controls the lift using relay pulse (flash) commands.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .lift_robot_controller import LiftRobotController
import json
import uuid
import logging

# Configure root logging level
logging.basicConfig(level=logging.INFO)


class LiftRobotNode(Node):
    def __init__(self):
        super().__init__('lift_robot_platform')
        
        # Declare parameters
        self.declare_parameter('device_id', 1)
        self.declare_parameter('use_ack_patch', True)
        
        # Retrieve parameters
        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value
        
        # NOTE: Serial port and baudrate are now centrally managed by the modbus_driver node.
        # This node no longer opens the serial device directly; parameters were removed to avoid confusion.
        self.get_logger().info(
            f"Initialize lift platform controller - device_id: {self.device_id} (serial handled by modbus_driver)"
        )
        
        # Create controller
        self.controller = LiftRobotController(
            device_id=self.device_id,
            node=self,
            use_ack_patch=self.use_ack_patch
        )
        
        # Subscribe to command topic
        self.subscription = self.create_subscription(
            String,
            'lift_robot_platform/command',
            self.command_callback,
            10
        )
        
        # Publish status topic
        self.status_publisher = self.create_publisher(
            String,
            'lift_robot_platform/status',
            10
        )
        
        # Status publish timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # Initialize lift platform
        self.controller.initialize()
        
        self.get_logger().info("Lift platform control node started")

    def command_callback(self, msg):
        """Handle command message"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '').lower()
            seq_id_str = command_data.get('seq_id', str(uuid.uuid4())[:8])
            # Convert seq_id string -> bounded int using hash for uniqueness
            seq_id = abs(hash(seq_id_str)) % 65536  # constrain to 0-65535
            
            self.get_logger().info(f"Received command: {command} [SEQ {seq_id_str}]")
            
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
                self.get_logger().warning(f"Unknown command: {command}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Cannot parse command JSON: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error handling command: {e}")

    def publish_status(self):
        """Publish periodic status info"""
        status = {
            'node': 'lift_robot_platform',
            'device_id': self.device_id,
            'active_timers': len(self.controller.active_timers),
            'status': 'online'
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_publisher.publish(status_msg)

    def destroy_node(self):
        """Cleanup resources"""
        self.get_logger().info("Stopping lift platform control node ...")
        
        # Stop platform & cleanup timers
        self.controller.cleanup()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LiftRobotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node runtime error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
