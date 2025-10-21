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

        self.get_logger().info(f"Start force sensor node (CH2/CH3 floats): device_id={self.device_id}, interval={self.read_interval}s")

        self.controller = ForceSensorController(self.device_id, self, self.use_ack_patch)
        self.controller.initialize()

        # Sequence id generator
        self.seq_id = 0

        # Publishers for right (CH2) and left (CH3) force values (Float32)
        from std_msgs.msg import Float32
        self.right_pub = self.create_publisher(Float32, '/right_force_sensor', 10)
        self.left_pub = self.create_publisher(Float32, '/left_force_sensor', 10)

        # Periodic read timer
        self.timer = self.create_timer(self.read_interval, self.periodic_read)
        self.get_logger().info(f"Force sensor reading timer started at {1.0/self.read_interval:.1f} Hz")

    def next_seq(self):
        self.seq_id += 1
        return self.seq_id

    def periodic_read(self):
        seq = self.next_seq()
        self.controller.read_ch2_ch3(seq_id=seq)
        # After asynchronous callbacks complete, publish last known values (race acceptable for simple UI display)
        last = self.controller.get_last()
        from std_msgs.msg import Float32
        if last['right_value'] is not None:
            m = Float32(); m.data = float(last['right_value'])
            self.right_pub.publish(m)
        if last['left_value'] is not None:
            m2 = Float32(); m2.data = float(last['left_value'])
            self.left_pub.publish(m2)

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
