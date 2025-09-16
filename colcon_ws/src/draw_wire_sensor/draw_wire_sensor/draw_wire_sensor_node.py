#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, time, threading
from .draw_wire_sensor_controller import DrawWireSensorController

class DrawWireSensorNode(Node):
    def __init__(self):
        super().__init__('draw_wire_sensor_node')
        self.declare_parameter('device_id', 51)
        self.declare_parameter('use_ack_patch', True)
        self.declare_parameter('read_interval', 0.1)

        self.device_id = self.get_parameter('device_id').value
        use_ack_patch = self.get_parameter('use_ack_patch').value
        self.read_interval = float(self.get_parameter('read_interval').value)

        self.get_logger().info(f"Start draw-wire sensor node: device_id={self.device_id}, interval={self.read_interval}s")

        self.controller = DrawWireSensorController(self.device_id, self, use_ack_patch)
        self.controller.initialize()

        self.pub = self.create_publisher(String, '/draw_wire_sensor/data', 10)
        self.cmd_sub = self.create_subscription(String, '/draw_wire_sensor/command', self.command_callback, 10)

        self.seq_id = 0
        self.timer = self.create_timer(self.read_interval, self.periodic_read_callback)

    def next_seq(self):
        self.seq_id += 1
        return self.seq_id

    def command_callback(self, msg):
        try:
            data = json.loads(msg.data)
            command = data.get('command','')
            seq = self.next_seq()
            if command == 'read':
                self.controller.read_sensor_data(seq_id=seq)
            elif command == 'get_data':
                self.publish_sensor_data(seq_id=seq)
            else:
                self.get_logger().warn(f"[SEQ {seq}] Unknown command: {command}")
        except Exception as e:
            self.get_logger().error(f"Command handling error: {e}")

    def periodic_read_callback(self):
        seq = self.next_seq()
        self.controller.read_sensor_data(seq_id=seq)
        def delayed():
            time.sleep(0.05)
            self.publish_sensor_data(seq_id=seq)
        threading.Timer(0.05, delayed).start()

    def publish_sensor_data(self, seq_id=None):
        reg0, reg1, ts = self.controller.get_sensor_data()
        msg_obj = {
            'timestamp': ts,
            'register_0': reg0,
            'register_1': reg1,
            'device_id': self.device_id,
            'seq_id': seq_id,
            'read_interval': self.read_interval
        }
        m = String(); m.data = json.dumps(msg_obj)
        self.pub.publish(m)
        self.get_logger().debug(f"[SEQ {seq_id}] Publish: reg0={reg0} reg1={reg1}")

    def destroy_node(self):
        self.get_logger().info("Shutting down draw-wire sensor node...")
        if hasattr(self, 'controller'):
            self.controller.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = DrawWireSensorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
