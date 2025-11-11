#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .force_sensor_controller import ForceSensorController
import logging
import time
try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None

logging.basicConfig(level=logging.INFO)

class LiftRobotForceSensor2Node(Node):
    def __init__(self):
        super().__init__('lift_robot_force_sensor_2')
        # Parameters (device_id default 53 for second sensor)
        self.declare_parameter('device_id', 53)
        self.declare_parameter('use_ack_patch', True)
        self.declare_parameter('read_interval', 0.02)  # 50Hz default
        self.declare_parameter('enable_visualization', False)
        # Calibration parameters (from calibration: actual_force = sensor_reading × scale)
        self.declare_parameter('calibration_scale', 0.023614)  # device_id=53 calibration result
        self.declare_parameter('calibration_offset', 0.0)  # Force zero offset (always 0 after tare)

        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value
        self.read_interval = float(self.get_parameter('read_interval').value)
        self.enable_visualization = bool(self.get_parameter('enable_visualization').value)
        self.calibration_scale = float(self.get_parameter('calibration_scale').value)
        self.calibration_offset = float(self.get_parameter('calibration_offset').value)

        self.get_logger().info(
            f"[Sensor2] Start node: device_id={self.device_id}, interval={self.read_interval}s (~{(1.0/self.read_interval) if self.read_interval>0 else '∞'} Hz)"
        )
        self.get_logger().info(
            f"[Sensor2] Calibration: actual_force = sensor_reading × {self.calibration_scale:.6f} + {self.calibration_offset:.6f}"
        )

        self.controller = ForceSensorController(self.device_id, self, self.use_ack_patch)
        self.controller.initialize()

        self.seq_id = 0

        from std_msgs.msg import Float32
        # Publish to distinct topic to avoid collision
        self.force_pub = self.create_publisher(Float32, '/force_sensor_2', 10)

        self.timer = self.create_timer(self.read_interval, self.periodic_read)
        self.get_logger().info(f"[Sensor2] Reading timer started at {1.0/self.read_interval:.1f} Hz")

        # Visualization state
        self.vis_interval = 1.0 / 50.0
        self.force_history = []
        self.max_history_seconds = 20.0
        if self.enable_visualization and cv2 is not None:
            self.vis_timer = self.create_timer(self.vis_interval, self._draw_visualization)

    def next_seq(self):
        self.seq_id += 1
        return self.seq_id

    def periodic_read(self):
        try:
            seq = self.next_seq()
            self.controller.read_force(seq_id=seq)
            last = self.controller.get_last()
            from std_msgs.msg import Float32
            if last['right_value'] is not None:
                # Apply calibration: actual_force = sensor_reading × scale + offset
                raw_value = float(last['right_value'])
                calibrated_force = raw_value * self.calibration_scale + self.calibration_offset
                msg = Float32()
                msg.data = calibrated_force
                self.force_pub.publish(msg)
            # Append to history for visualization (use calibrated value)
            if self.enable_visualization and last['right_value'] is not None:
                now = time.time()
                raw_value = float(last['right_value'])
                calibrated_force = raw_value * self.calibration_scale + self.calibration_offset
                self.force_history.append((now, calibrated_force))
                cutoff = now - self.max_history_seconds
                while self.force_history and self.force_history[0][0] < cutoff:
                    self.force_history.pop(0)
        except Exception as e:
            self.get_logger().error(f"[Sensor2] periodic read error: {e}")

    def _draw_visualization(self):
        if not self.enable_visualization or cv2 is None or np is None:
            return
        if len(self.force_history) < 2:
            return
        w, h = 600, 250
        img = np.zeros((h, w, 3), dtype=np.uint8)
        img[:] = (25, 25, 25)
        times = np.array([t for t, _ in self.force_history])
        vals = np.array([v for _, v in self.force_history])
        t_rel = times - times[0]
        duration = t_rel[-1]
        if duration <= 0:
            return
        x = (t_rel / duration) * (w - 1)
        max_force = max(vals.max(), 1.0) * 1.05
        y = (h - 30) - (vals / max_force) * (h - 60)
        for i in range(1, len(x)):
            cv2.line(img, (int(x[i-1]), int(y[i-1])), (int(x[i]), int(y[i])), (0, 180, 255), 2)
        cv2.putText(img, f"Force2: {vals[-1]:.2f} N", (10, 20), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,180,255), 1, cv2.LINE_AA)
        cv2.putText(img, f"Window: {duration:.1f}s MaxScale: {max_force:.1f} N", (10, h-15), cv2.FONT_HERSHEY_SIMPLEX, 0.45, (200,200,200), 1, cv2.LINE_AA)
        cv2.rectangle(img, (0,0), (w-1,h-1), (70,70,70), 1)
        cv2.imshow('ForceSensor2Live', img)
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("[Sensor2] q pressed, closing window")
            cv2.destroyWindow('ForceSensor2Live')
            self.enable_visualization = False

    def destroy_node(self):
        self.get_logger().info("[Sensor2] Shutting down node...")
        if cv2 is not None and self.enable_visualization:
            try:
                cv2.destroyWindow('ForceSensor2Live')
            except Exception:
                pass
        self.controller.cleanup()
        super().destroy_node()


def main(args=None):
    import sys
    visualize_flag = '--visualize' in sys.argv or '--viz' in sys.argv
    rclpy.init(args=args)
    node = LiftRobotForceSensor2Node()
    if visualize_flag and not node.enable_visualization:
        if cv2 is None:
            node.get_logger().warning("[Sensor2] Visualization requested but OpenCV not available")
        else:
            node.get_logger().info("[Sensor2] Visualization enabled by CLI")
            node.enable_visualization = True
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
