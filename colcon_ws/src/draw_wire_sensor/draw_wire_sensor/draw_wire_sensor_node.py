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
        # Calibration parameters (linear): height = register_1 * scale + offset
        self.declare_parameter('calibration.scale', 0.024537)
        self.declare_parameter('calibration.offset', 681.837575)
        self.declare_parameter('calibration.enable', True)

        self.device_id = self.get_parameter('device_id').value
        use_ack_patch = self.get_parameter('use_ack_patch').value
        self.read_interval = float(self.get_parameter('read_interval').value)
        self.cal_scale = float(self.get_parameter('calibration.scale').value)
        self.cal_offset = float(self.get_parameter('calibration.offset').value)
        self.cal_enable = bool(self.get_parameter('calibration.enable').value)

        # Pushrod offset tracking
        self.pushrod_point = None
        self.pushrod_position_seconds = None
        # Offsets in millimeters for each point
        self.pushrod_offsets_mm = {
            'base': 0.0,
            'only forward': 5.5,
            'safe mode': 1.0,
            'fwd&back': 9.8,
            'all direction': 20.1,
        }
        self.pushrod_offset_mm = 0.0

        self.get_logger().info(f"Start draw-wire sensor node: device_id={self.device_id}, interval={self.read_interval}s")

        self.controller = DrawWireSensorController(self.device_id, self, use_ack_patch)
        self.controller.initialize()

        self.pub = self.create_publisher(String, '/draw_wire_sensor/data', 10)
        self.cmd_sub = self.create_subscription(String, '/draw_wire_sensor/command', self.command_callback, 10)
        
        # Subscribe to pushrod status for offset calculation
        self.pushrod_status_sub = self.create_subscription(
            String, 
            '/lift_robot_pushrod/status', 
            self.pushrod_status_callback, 
            10
        )

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

    def pushrod_status_callback(self, msg):
        """Handle pushrod status updates to track offset."""
        try:
            data = json.loads(msg.data)
            self.pushrod_point = data.get('current_point')
            self.pushrod_position_seconds = data.get('current_position_seconds')
            if self.pushrod_point in self.pushrod_offsets_mm:
                self.pushrod_offset_mm = self.pushrod_offsets_mm[self.pushrod_point]
                self.get_logger().debug(f"Pushrod offset updated: {self.pushrod_point} = {self.pushrod_offset_mm}mm")
            else:
                # If not a known point, keep last offset
                pass
        except Exception as e:
            self.get_logger().warn(f'Failed to parse pushrod status JSON: {e}')

    def periodic_read_callback(self):
        try:
            seq = self.next_seq()
            self.controller.read_sensor_data(seq_id=seq)
            def delayed():
                try:
                    time.sleep(0.05)
                    self.publish_sensor_data(seq_id=seq)
                except Exception as e:
                    self.get_logger().error(f"Delayed publish error: {e}")
            threading.Timer(0.05, delayed).start()
        except Exception as e:
            self.get_logger().error(f"Periodic read callback error: {e}")
            # Continue with next cycle

    def publish_sensor_data(self, seq_id=None):
        try:
            reg0, reg1, ts = self.controller.get_sensor_data()
            
            # Calculate raw height from calibration
            raw_height_val = None
            if self.cal_enable and reg1 is not None:
                try:
                    raw_height_val = reg1 * self.cal_scale + self.cal_offset
                except Exception:
                    raw_height_val = None
            
            # Calculate adjusted height with pushrod offset
            adjusted_height_val = None
            if raw_height_val is not None:
                try:
                    adjusted_height_val = raw_height_val + self.pushrod_offset_mm
                except Exception:
                    adjusted_height_val = None
            
            msg_obj = {
                'timestamp': ts,
                'register_0': reg0,
                'register_1': reg1,
                'device_id': self.device_id,
                'seq_id': seq_id,
                'read_interval': self.read_interval,
                'raw_height': raw_height_val,
                'pushrod_offset_mm': self.pushrod_offset_mm,
                'pushrod_point': self.pushrod_point,
                'pushrod_position_seconds': self.pushrod_position_seconds,
                'height': adjusted_height_val
            }
            m = String(); m.data = json.dumps(msg_obj)
            self.pub.publish(m)
            self.get_logger().debug(f"[SEQ {seq_id}] Publish: reg0={reg0} reg1={reg1} raw_height={raw_height_val} adjusted_height={adjusted_height_val}")
        except Exception as e:
            self.get_logger().error(f"[SEQ {seq_id}] Sensor data publish error: {e}")
            # Continue operation, publish what we can or skip this cycle

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
