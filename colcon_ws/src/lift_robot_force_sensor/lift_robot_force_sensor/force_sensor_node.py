#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from .force_sensor_controller import ForceSensorController
import logging
import uuid
import time
try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None

logging.basicConfig(level=logging.INFO)

class LiftRobotForceSensorNode(Node):
    def __init__(self):
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
        # Calibration parameters (from calibration: actual_force = sensor_reading × scale)
        self.declare_parameter('calibration_scale', 0.116125)  # device_id=52 calibration result
        self.declare_parameter('calibration_offset', 0.0)  # Force zero offset (always 0 after tare)

        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value
        self.read_interval = float(self.get_parameter('read_interval').value)
        self.enable_visualization = bool(self.get_parameter('enable_visualization').value)
        self.calibration_scale = float(self.get_parameter('calibration_scale').value)
        self.calibration_offset = float(self.get_parameter('calibration_offset').value)

        self.get_logger().info(
            f"Start single force sensor node: device_id={self.device_id}, interval={self.read_interval}s (~{(1.0/self.read_interval) if self.read_interval>0 else '∞'} Hz)"
        )
        self.get_logger().info(
            f"Calibration: actual_force = sensor_reading × {self.calibration_scale:.6f} + {self.calibration_offset:.6f}"
        )

        self.controller = ForceSensorController(self.device_id, self, self.use_ack_patch)
        self.controller.initialize()

        # Sequence id generator
        self.seq_id = 0

        # 单通道力值发布 (Float32) 统一话题 /force_sensor
        from std_msgs.msg import Float32
        self.force_pub = self.create_publisher(Float32, '/force_sensor', 10)

        # Periodic read timer
        self.timer = self.create_timer(self.read_interval, self.periodic_read)
        self.get_logger().info(f"Force sensor reading timer started at {1.0/self.read_interval:.1f} Hz (target 50Hz)")

        # Visualization state
        self.vis_initialized = False
        self.vis_last_draw = 0.0
        # 修改为 50FPS 以与采样频率 50Hz 保持一致，减少感知延迟
        self.vis_interval = 1.0 / 50.0  # target ~50 FPS
        self.force_history = []  # store tuples (t, right, left)
        self.max_history_seconds = 20.0
        if self.enable_visualization:
            if cv2 is None:
                self.get_logger().warning("OpenCV 不可用，已忽略 enable_visualization")
                self.enable_visualization = False
            else:
                self.get_logger().info("Force visualization 已启用 (窗口: ForceSensorLive)")
                # Separate timer for drawing to avoid blocking read loop
                self.vis_timer = self.create_timer(self.vis_interval, self._draw_visualization)

    def next_seq(self):
        self.seq_id = (self.seq_id + 1) % 65536  # Wrap around at 65536
        return self.seq_id

    def periodic_read(self):
        try:
            seq = self.next_seq()
            # 单通道读取
            self.controller.read_force(seq_id=seq)
            # After asynchronous callbacks complete, publish last known values (race acceptable for simple UI display)
            last = self.controller.get_last()
            from std_msgs.msg import Float32
            if last['right_value'] is not None:  # 兼容 controller 返回结构
                # Apply calibration: actual_force = sensor_reading × scale + offset
                raw_value = float(last['right_value'])
                calibrated_force = raw_value * self.calibration_scale + self.calibration_offset
                msg_force = Float32()
                msg_force.data = calibrated_force
                self.force_pub.publish(msg_force)
            # Append to history for visualization (use calibrated values)
            if self.enable_visualization and last['right_value'] is not None and last['left_value'] is not None:
                now = time.time()
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
            self.get_logger().error(f"Force sensor periodic read error: {e}")
            # Continue with next cycle

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
        # waitKey 必须调用以刷新窗口; 使用 1ms 非阻塞
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            self.get_logger().info("收到 q 键，关闭可视化窗口")
            cv2.destroyWindow('ForceSensorLive')
            self.enable_visualization = False

    def destroy_node(self):
        self.get_logger().info("Shutting down force sensor node...")
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
