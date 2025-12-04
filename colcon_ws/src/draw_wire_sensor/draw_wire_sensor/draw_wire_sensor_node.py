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
        # Default to 50Hz (0.02s) to match system-wide sensor update rate
        self.declare_parameter('read_interval', 0.02)
        # Compact publish mode: only output height, seq_id, freq_hz, latency_ms (user request)
        # Set to True to enable minimal JSON; False keeps legacy full fields for backward compatibility.
        self.declare_parameter('publish_compact', True)
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
        self.publish_compact = bool(self.get_parameter('publish_compact').value)

        self.get_logger().info(f"Start draw-wire sensor node: device_id={self.device_id}, interval={self.read_interval}s")

        self.controller = DrawWireSensorController(self.device_id, self, use_ack_patch)
        self.controller.initialize()

        self.pub = self.create_publisher(String, '/draw_wire_sensor/data', 10)
        self.cmd_sub = self.create_subscription(String, '/draw_wire_sensor/command', self.command_callback, 10)
        
        # Emergency reset publisher (for hardware failure detection)
        self.emergency_pub = self.create_publisher(String, '/emergency_reset', 10)
        
        # Sensor disable state tracking
        self.sensor_disabled_triggered = False

        self.seq_id = 0
        self.timer = self.create_timer(self.read_interval, self.periodic_read_callback)
        # For frequency estimation
        self.last_publish_time = None
        self.publish_intervals = []  # store recent dt
        self.max_interval_samples = 100

    def next_seq(self):
        self.seq_id += 1
        if self.seq_id > 65535:
            self.seq_id = 1  # Reset to 1 to avoid 0 (which might have special meaning)
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
        """Periodic sensor poll.

        Changed to immediate publish (no artificial 2×50ms delay) to reduce perceived latency
        and align behavior with `force_sensor_node`. We accept a small race: published data
        may be from the previous successful response if the current asynchronous Modbus reply
        hasn't arrived yet. This mirrors the force sensor approach; typical Modbus turnaround
        is a few milliseconds so data is effectively fresh at 50Hz.

        Added read_start_ts (poll start) into published JSON for downstream latency analysis.
        """
        try:
            seq = self.next_seq()
            read_start_ts = time.time()
            
            # Check if sensor is disabled - if so, stop sending Modbus commands
            if self.controller.sensor_disabled:
                if not self.sensor_disabled_triggered:
                    self.sensor_disabled_triggered = True
                    self.get_logger().error(
                        f"[SEQ {seq}] ❌ Sensor disabled (consecutive {self.controller.disable_threshold} Modbus recv failures) - "
                        f"stopping Modbus commands, triggering emergency reset"
                    )
                    # Trigger emergency reset
                    emergency_msg = String()
                    emergency_msg.data = json.dumps({
                        'reason': f'draw_wire_sensor_{self.device_id}_recv_timeout',
                        'device_id': self.device_id,
                        'consecutive_errors': self.controller.consecutive_errors
                    })
                    self.emergency_pub.publish(emergency_msg)
                # Keep publishing last known values but don't send new Modbus requests
            else:
                # Normal operation - send Modbus read request
                self.controller.read_sensor_data(seq_id=seq)
            
            # Immediate publish using last known registers (race acceptable).
            self.publish_sensor_data(seq_id=seq, read_start_ts=read_start_ts)
        except Exception as e:
            self.get_logger().error(f"Periodic read callback error: {e}")
            # Continue with next cycle

    def publish_sensor_data(self, seq_id=None, read_start_ts=None):
        try:
            reg0, reg1, ts = self.controller.get_sensor_data()
            
            # Get error status
            error_status = self.controller.get_error_status()
            has_error = error_status.get('consecutive_errors', 0) > 0
            error_msg = error_status.get('last_error') if has_error else None
            
            # Calculate height from calibration (this is the real height)
            height_val = None
            if self.cal_enable and reg1 is not None:
                try:
                    height_val = reg1 * self.cal_scale + self.cal_offset
                except Exception:
                    height_val = None
            
            # Frequency estimation
            now = time.time()
            if self.last_publish_time is not None:
                dt = now - self.last_publish_time
                if 0 < dt < 2.0:  # sanity range
                    self.publish_intervals.append(dt)
                    if len(self.publish_intervals) > self.max_interval_samples:
                        self.publish_intervals.pop(0)
            self.last_publish_time = now
            freq_hz = 0.0
            if self.publish_intervals:
                avg_dt = sum(self.publish_intervals) / len(self.publish_intervals)
                if avg_dt > 0:
                    freq_hz = 1.0 / avg_dt
            # Latency estimation: time since sensor timestamp (if available)
            latency_ms = None
            if ts is not None:
                try:
                    latency_ms = (now - ts) * 1000.0
                except Exception:
                    latency_ms = None

            if self.publish_compact:
                # Minimal JSON as requested
                msg_obj = {
                    'height': height_val,
                    'seq_id': seq_id,
                    'freq_hz': freq_hz,
                    'latency_ms': latency_ms,
                    'error': has_error,
                    'error_message': error_msg,
                    'error_count': error_status.get('error_count', 0)
                }
            else:
                # Legacy full payload (retain for other subscribers needing raw registers)
                msg_obj = {
                    'timestamp': ts,
                    'register_0': reg0,
                    'register_1': reg1,
                    'device_id': self.device_id,
                    'seq_id': seq_id,
                    'read_interval': self.read_interval,
                    'height': height_val,
                    'read_start_ts': read_start_ts,
                    'freq_hz': freq_hz,
                    'latency_ms': latency_ms,
                    'error': has_error,
                    'error_message': error_msg,
                    'error_count': error_status.get('error_count', 0)
                }
            m = String(); m.data = json.dumps(msg_obj)
            self.pub.publish(m)
            if has_error:
                self.get_logger().warn(f"[SEQ {seq_id}] Publish with ERROR: {error_msg}")
            else:
                self.get_logger().debug(f"[SEQ {seq_id}] Publish(height={height_val}, freq={freq_hz:.2f}Hz, latency_ms={latency_ms}) compact={self.publish_compact}")
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
