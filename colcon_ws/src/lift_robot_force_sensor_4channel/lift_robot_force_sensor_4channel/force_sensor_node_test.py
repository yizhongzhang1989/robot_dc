#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .force_sensor_controller_test import ForceSensorController
import logging
import uuid

logging.basicConfig(level=logging.INFO)

class LiftRobotForceSensorTestNode(Node):
    def __init__(self):
        super().__init__('lift_robot_force_sensor_4channel')
        # Parameters (device_id default changed to 60, single channel CH2)
        self.declare_parameter('device_id', 60)  # Modbus slave ID (0x3C)
        self.declare_parameter('use_ack_patch', True)
        self.declare_parameter('read_interval', 0.02)  # seconds (50 Hz)

        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value
        self.read_interval = float(self.get_parameter('read_interval').value)
        self.get_logger().info(f"Start force sensor TEST node (CH2 only): device_id={self.device_id}, interval={self.read_interval}s (~{1.0/self.read_interval:.1f}Hz)")

        self.controller = ForceSensorController(self.device_id, self, self.use_ack_patch)
        self.controller.initialize()

        # Sequence id generator
        self.seq_id = 0

        # Publisher for single channel force value (CH2). Keep topic name generic.
        from std_msgs.msg import Float32
        self.force_pub = self.create_publisher(Float32, '/force_sensor_test', 10)

        # Periodic read timer
        self.timer = self.create_timer(self.read_interval, self.periodic_read)
        self.get_logger().info(f"Force sensor reading timer started at {1.0/self.read_interval:.1f} Hz")

    def next_seq(self):
        self.seq_id += 1
        return self.seq_id

    def periodic_read(self):
        """50Hz polling cycle: issue async read, publish previous result.
        
        Pattern explanation:
        1. Issue async Modbus read request for current cycle
        2. Publish the result from PREVIOUS cycle (stored in controller.last_value)
        3. When async callback completes, it updates controller.last_value for NEXT cycle
        
        This approach ensures:
        - Consistent 50Hz publishing rate (no waiting for Modbus response)
        - One cycle latency between hardware read and publish (acceptable for control)
        - No blocking in timer callback (async pattern)
        """
        try:
            seq = self.next_seq()
            # Issue async read request
            self.controller.read_ch2(seq_id=seq)
            # Publish last known value (from previous cycle's callback update)
            # This matches the production node pattern: poll current, publish previous
            last = self.controller.get_last()
            from std_msgs.msg import Float32
            if last['value'] is not None:
                msg = Float32()
                msg.data = float(last['value'])
                self.force_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Force sensor periodic read error: {e}")
            # Continue with next cycle

    def destroy_node(self):
        self.get_logger().info("Shutting down force sensor node...")
        self.controller.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiftRobotForceSensorTestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()