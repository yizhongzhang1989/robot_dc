#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .force_sensor_controller import ForceSensorController
import logging
import uuid
import time
import json
from collections import deque
try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None

logging.basicConfig(level=logging.INFO)

class LiftRobotForceSensorNode(Node):
    def __init__(self):
        # Create node with base name (actual name set by launch file)
        super().__init__('lift_robot_force_sensor')
        
        # Parameters
        self.declare_parameter('device_id', 52)  # Modbus slave ID
        self.declare_parameter('use_ack_patch', True)
        # For 50Hz we use 0.02s interval (was 0.1s for 10Hz previously)
        # NOTE: Increasing frequency increases Modbus bus load. If bus saturation
        # or latency occurs, raise this back toward 0.05 (20Hz) or 0.033 (30Hz).
        self.declare_parameter('read_interval', 0.02)  # seconds (50 Hz default)
        # Visualization enable flag (can also be overridden via CLI args)
        self.declare_parameter('enable_visualization', False)
        # Topic name parameter (allows running multiple instances with different topics)
        self.declare_parameter('topic_name', '/force_sensor_right')
        # Node name suffix (for distinguishing multiple instances in logs)
        self.declare_parameter('node_name_suffix', '')
        # Calibration parameters (from calibration: actual_force = sensor_reading × scale)
        # Default values used when launch file doesn't load from JSON config
        # Device-specific defaults:
        #   - device_id=52 (right sensor, /force_sensor_right): scale=0.116125
        #   - device_id=53 (left sensor, /force_sensor_left): scale=0.116125
        # Note: These are initial calibration results. For best accuracy, perform
        # web-based calibration and save to JSON config file.
        self.declare_parameter('calibration_scale', 0.116125)
        self.declare_parameter('calibration_offset', 0.0)  # Always 0.0 after tare
        
        # Dashboard parameters
        self.declare_parameter('dashboard_enabled', False)
        self.declare_parameter('dashboard_host', '0.0.0.0')
        self.declare_parameter('dashboard_port', 5001)
        self.declare_parameter('serial_port', '')  # Empty = auto-detect
        self.declare_parameter('serial_baudrate', 115200)

        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value
        self.read_interval = float(self.get_parameter('read_interval').value)
        self.enable_visualization = bool(self.get_parameter('enable_visualization').value)
        self.topic_name = self.get_parameter('topic_name').value
        self.node_name_suffix = self.get_parameter('node_name_suffix').value
        self.calibration_scale = float(self.get_parameter('calibration_scale').value)
        self.calibration_offset = float(self.get_parameter('calibration_offset').value)
        
        # Dashboard configuration
        self.dashboard_enabled = bool(self.get_parameter('dashboard_enabled').value)
        self.dashboard_host = self.get_parameter('dashboard_host').value
        self.dashboard_port = int(self.get_parameter('dashboard_port').value)
        self.serial_port = self.get_parameter('serial_port').value or None
        self.serial_baudrate = int(self.get_parameter('serial_baudrate').value)
        self.dashboard = None

        # Log node info with suffix in message (node name itself stays as base name)
        node_identifier = f"{self.get_name()}_{self.node_name_suffix}" if self.node_name_suffix else self.get_name()
        self.get_logger().info(
            f"[{node_identifier}] Start force sensor: device_id={self.device_id}, topic={self.topic_name}, interval={self.read_interval}s (~{(1.0/self.read_interval) if self.read_interval>0 else '∞'} Hz)"
        )
        self.get_logger().info(
            f"[{node_identifier}] Calibration: actual_force = sensor_reading × {self.calibration_scale:.6f} + {self.calibration_offset:.6f}"
        )
        
        # Start dashboard if enabled
        if self.dashboard_enabled:
            self._start_dashboard()

        self.controller = ForceSensorController(self.device_id, self, self.use_ack_patch)
        self.controller.initialize()

        # Sequence id generator
        self.seq_id = 0
        
        # Frequency tracking (like draw_wire_sensor)
        self.last_publish_time = None
        self.publish_intervals = deque(maxlen=10)

        # Single channel force value publishing (JSON String with freq_hz)
        from std_msgs.msg import String
        # Calibrated force (for control) - now JSON with freq_hz
        self.force_pub = self.create_publisher(String, self.topic_name, 10)
        # Raw force (for calibration and debugging) - also JSON
        self.force_raw_pub = self.create_publisher(String, f"{self.topic_name}/raw", 10)

        # Periodic read timer
        self.timer = self.create_timer(self.read_interval, self.periodic_read)
        self.get_logger().info(f"Force sensor reading timer started at {1.0/self.read_interval:.1f} Hz (target 50Hz)")

        # Visualization state
        self.vis_initialized = False
        self.vis_last_draw = 0.0
        # Set to 50FPS to match sampling frequency 50Hz, reducing perception latency
        self.vis_interval = 1.0 / 50.0  # target ~50 FPS
        self.force_history = []  # store tuples (t, right, left)
        self.max_history_seconds = 20.0
        if self.enable_visualization:
            if cv2 is None:
                self.get_logger().warning("OpenCV not available, enable_visualization ignored")
                self.enable_visualization = False
            else:
                self.get_logger().info("Force visualization enabled (window: ForceSensorLive)")
                # Separate timer for drawing to avoid blocking read loop
                self.vis_timer = self.create_timer(self.vis_interval, self._draw_visualization)

    def next_seq(self):
        self.seq_id = (self.seq_id + 1) % 65536  # Wrap around at 65536
        return self.seq_id

    def periodic_read(self):
        try:
            seq = self.next_seq()
            # Single channel read - wrap in try to prevent Modbus errors from crashing
            try:
                self.controller.read_force(seq_id=seq)
            except Exception as e:
                self.get_logger().warn(f"[SEQ {seq}] Force sensor read_force error: {e}")
                # Continue to next cycle without crashing
                return
            
            # Calculate frequency (same logic as draw_wire_sensor)
            now = time.time()
            if self.last_publish_time is not None:
                dt = now - self.last_publish_time
                if 0 < dt < 2.0:  # sanity range
                    self.publish_intervals.append(dt)
            self.last_publish_time = now
            
            freq_hz = 0.0
            if self.publish_intervals:
                avg_dt = sum(self.publish_intervals) / len(self.publish_intervals)
                if avg_dt > 0:
                    freq_hz = 1.0 / avg_dt
            
            # After asynchronous callbacks complete, publish last known values (race acceptable for simple UI display)
            try:
                last = self.controller.get_last()
                from std_msgs.msg import String
                if last['right_value'] is not None:  # Compatible with controller return structure
                    # Apply calibration: actual_force = sensor_reading × scale + offset
                    raw_value = float(last['right_value'])
                    calibrated_force = raw_value * self.calibration_scale + self.calibration_offset
                    
                    # Publish raw value JSON (for calibration) - wrap to prevent publish errors
                    try:
                        msg_raw_obj = {
                            'force': raw_value,
                            'seq_id': seq,
                            'freq_hz': freq_hz,
                            'timestamp': now
                        }
                        msg_raw = String()
                        msg_raw.data = json.dumps(msg_raw_obj)
                        self.force_raw_pub.publish(msg_raw)
                    except Exception as e:
                        self.get_logger().warn(f"[SEQ {seq}] Failed to publish raw force: {e}")
                    
                    # Publish calibrated value JSON (for control) - wrap to prevent publish errors
                    try:
                        msg_force_obj = {
                            'force': calibrated_force,
                            'seq_id': seq,
                            'freq_hz': freq_hz,
                            'timestamp': now
                        }
                        msg_force = String()
                        msg_force.data = json.dumps(msg_force_obj)
                        self.force_pub.publish(msg_force)
                    except Exception as e:
                        self.get_logger().warn(f"[SEQ {seq}] Failed to publish calibrated force: {e}")
            except Exception as e:
                self.get_logger().warn(f"[SEQ {seq}] Force sensor data processing error: {e}")
            
            # Append to history for visualization (use calibrated values)
            try:
                if self.enable_visualization and last['right_value'] is not None and last['left_value'] is not None:
                    # Store calibrated values for visualization
                    raw_right = float(last['right_value'])
                    raw_left = float(last['left_value'])
                    cal_right = raw_right * self.calibration_scale + self.calibration_offset
                    cal_left = raw_left * self.calibration_scale + self.calibration_offset
                    self.force_history.append((now, cal_right, cal_left))
                    # Trim history older than max_history_seconds
                    cutoff = now - self.max_history_seconds
                    while self.force_history and self.force_history[0][0] < cutoff:
                        self.force_history.pop(0)
            except Exception as e:
                self.get_logger().debug(f"Visualization history update error: {e}")
        except Exception as e:
            self.get_logger().error(f"Force sensor periodic read error: {e}")
            # Continue with next cycle - don't crash the node

    # --------------- Visualization ---------------
    def _draw_visualization(self):
        if not self.enable_visualization or cv2 is None or np is None:
            return
        # Ensure history has data
        if len(self.force_history) < 2:
            return
        # Prepare canvas
        w, h = 800, 300
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:] = (20, 20, 20)
        # Extract arrays
        times = np.array([t for t, _, _ in self.force_history])
        right = np.array([r for _, r, _ in self.force_history])
        left = np.array([l for _, _, l in self.force_history])
        total = right + left
        # Normalize time to last seconds window
        t0 = times[0]
        t_rel = times - t0
        duration = t_rel[-1]
        if duration <= 0:
            return
        # Scale X to width
        x = (t_rel / duration) * (w - 1)
        # Determine Y scaling based on max force
        max_force = max(total.max(), right.max(), left.max(), 1.0)
        # Provide some headroom
        max_force *= 1.05
        def to_y(vals):
            return (h - 30) - (vals / max_force) * (h - 60)
        y_right = to_y(right)
        y_left = to_y(left)
        y_total = to_y(total)
        # Draw lines
        for i in range(1, len(x)):
            cv2.line(img, (int(x[i-1]), int(y_right[i-1])), (int(x[i]), int(y_right[i])), (0, 140, 255), 2)
            cv2.line(img, (int(x[i-1]), int(y_left[i-1])), (int(x[i]), int(y_left[i])), (0, 255, 0), 2)
            cv2.line(img, (int(x[i-1]), int(y_total[i-1])), (int(x[i]), int(y_total[i])), (255, 255, 0), 2)
        # Axes & labels
        cv2.putText(img, f"Right (CH2): {right[-1]:.2f} N", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,140,255), 1, cv2.LINE_AA)
        cv2.putText(img, f"Left (CH3): {left[-1]:.2f} N", (10, 45), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,0), 1, cv2.LINE_AA)
        cv2.putText(img, f"Total: {total[-1]:.2f} N", (10, 70), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 1, cv2.LINE_AA)
        cv2.putText(img, f"Window: {duration:.1f}s  MaxForceScale: {max_force:.1f} N", (10, h-15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200,200,200), 1, cv2.LINE_AA)
        cv2.rectangle(img, (0,0), (w-1,h-1), (80,80,80), 1)
        cv2.imshow('ForceSensorLive', img)
        # waitKey must be called to refresh window; use 1ms non-blocking
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("Received 'q' key, closing visualization window")
            cv2.destroyWindow('ForceSensorLive')
            self.enable_visualization = False

    def _start_dashboard(self):
        """Start force sensor configuration dashboard"""
        try:
            from .force_sensor_dashboard import ForceSensorDashboard
            
            node_identifier = f"{self.get_name()}_{self.node_name_suffix}" if self.node_name_suffix else self.get_name()
            
            # Pass ROS2 node to dashboard for Modbus service access
            self.dashboard = ForceSensorDashboard(
                device_id=self.device_id,
                ros_node=self,  # Pass ROS2 node for Modbus service calls
                host=self.dashboard_host,
                web_port=self.dashboard_port
            )
            
            self.dashboard.start()
            
            self.get_logger().info(
                f"[{node_identifier}] Force Sensor Dashboard started at http://{self.dashboard_host}:{self.dashboard_port}"
            )
        except Exception as e:
            self.get_logger().error(f"Failed to start dashboard: {e}")
            import traceback
            traceback.print_exc()

    def destroy_node(self):
        self.get_logger().info("Shutting down force sensor node...")
        
        # Stop dashboard if running
        if self.dashboard is not None:
            try:
                self.dashboard.stop()
                self.get_logger().info("Dashboard stopped")
            except Exception as e:
                self.get_logger().warning(f"Error stopping dashboard: {e}")
        
        # Stop visualization if running
        if cv2 is not None and self.enable_visualization:
            try:
                cv2.destroyWindow('ForceSensorLive')
            except Exception:
                pass
        
        self.controller.cleanup()
        super().destroy_node()


def main(args=None):
    # 支持命令行开启可视化: python3 force_sensor_node.py --visualize
    import sys
    visualize_flag = '--visualize' in sys.argv or '--viz' in sys.argv
    rclpy.init(args=args)
    node = LiftRobotForceSensorNode()
    if visualize_flag and not node.enable_visualization:
        if cv2 is None:
            node.get_logger().warning("命令行要求可视化但 OpenCV 不可用")
        else:
            node.get_logger().info("命令行开启可视化")
            node.enable_visualization = True
            # 创建绘图定时器（如果之前未创建）
            if not hasattr(node, 'vis_timer'):
                node.vis_timer = node.create_timer(node.vis_interval, node._draw_visualization)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
