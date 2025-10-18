#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .force_sensor_controller import ForceSensorController
import logging
import uuid

logging.basicConfig(level=logging.INFO)

class LiftRobotForceSensorNode(Node):
    def __init__(self):
        super().__init__('lift_robot_force_sensor')
        # Parameters
        self.declare_parameter('device_id', 52)  # Modbus slave ID
        self.declare_parameter('use_ack_patch', True)
        # For 10Hz we use 0.1s interval
        self.declare_parameter('read_interval', 0.1)  # seconds (10 Hz)

        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value
        self.read_interval = float(self.get_parameter('read_interval').value)

        self.get_logger().info(f"Start force sensor node: device_id={self.device_id}, interval={self.read_interval}s")

        self.controller = ForceSensorController(self.device_id, self, self.use_ack_patch)
        self.controller.initialize()

        # Sequence id generator
        self.seq_id = 0

    # Publisher for force value (32-bit raw)
    from std_msgs.msg import Int64
    self.force_pub = self.create_publisher(Int64, '/lift_robot_force_sensor/value', 10)

    # Periodic read timer (10Hz if read_interval=0.1)
    self.timer = self.create_timer(self.read_interval, self.periodic_read)
    self.get_logger().info(f"Force sensor reading timer started at {1.0/self.read_interval:.1f} Hz")

    def next_seq(self):
        self.seq_id += 1
        return self.seq_id

    def periodic_read(self):
        seq = self.next_seq()
        self.controller.read_once(seq_id=seq)
        # Publish last value after read (will be updated in callback asynchronously; minor race acceptable for simple case)
        val, regs, ts = self.controller.get_last()
        if val is not None:
            from std_msgs.msg import Int64
            msg = Int64()
            msg.data = val
            self.force_pub.publish(msg)
            # Debug log at lower verbosity to avoid spam
            self.get_logger().debug(f"Publish force value: {val} (reg0=0x{regs[0]:04X} reg1=0x{regs[1]:04X})")

    def destroy_node(self):
        self.get_logger().info("Shutting down force sensor node...")
        self.controller.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = LiftRobotForceSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
