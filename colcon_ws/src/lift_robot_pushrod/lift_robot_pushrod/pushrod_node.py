#!/usr/bin/env python3
"""Lift Robot Pushrod ROS2 Node

Controls pushrod via relay pulses (stop/up/down) and supports height & force closed-loop control.
Force control uses summed left+right force with min/max filtering and optional settle cycles.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from .pushrod_controller import PushrodController
import json
import uuid
import logging

logging.basicConfig(level=logging.INFO)

CONTROL_RATE = 0.02             # 50 Hz loop
POSITION_TOLERANCE = 0.5        # mm tolerance for height
COMMAND_INTERVAL = 0.3          # Base seconds between relay pulses (height / coarse force)
FORCE_NEAR_INTERVAL = 0.12      # Faster interval when接近目标力
HISTORY_SIZE = 4                # filtering window size
MIN_FORCE_SAMPLES = 3           # min samples before filtered force valid

class PushrodNode(Node):
    def __init__(self):
        super().__init__('lift_robot_pushrod')
        # Parameters
        self.declare_parameter('device_id', 50)
        self.declare_parameter('use_ack_patch', True)
        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value

        self.get_logger().info(f"Initialize pushrod controller - device_id: {self.device_id}")

        # Height control state
        self.current_height = 0.0
        self.target_height = 0.0
        self.height_history = []
        self.last_command_time = self.get_clock().now()
        self.control_enabled = False
        self.control_mode = 'manual'  # manual | auto | force
        self.movement_state = 'stop'

        # Offset tracking
        self.pushrod_offset = 0.0
        self.height_before_movement = None
        self.is_tracking_offset = False

        # Force control state
        self.force_value = 0.0
        self.current_force = 0.0
        self.force_history = []
        self.target_force = 0.0
        self.force_threshold = 0.0
        self.force_data_valid = False
        self.last_force_update_time = self.get_clock().now()
        self.force_settle_cycles = 5
        self.force_in_band_consecutive = 0
        self.increase_on_up = True  # if True: UP increases force
        # 新增: 力控保持相关
        self.force_settle_hold_seconds = 5.0  # 进入稳定区后继续保持时间
        self.force_hold_start_time = None     # 首次满足稳定循环的时间戳
        # 采样速率估计
        self.force_sample_intervals = []
        self.force_sample_rate_hz = 0.0

        # Controller
        self.controller = PushrodController(
            device_id=self.device_id,
            node=self,
            use_ack_patch=self.use_ack_patch
        )
        self.controller.on_auto_stop_callback = self._on_auto_stop_complete

        # Subscriptions
        self.command_sub = self.create_subscription(String, 'lift_robot_pushrod/command', self.command_callback, 10)
        self.sensor_sub = self.create_subscription(String, '/draw_wire_sensor/data', self.sensor_callback, 10)
        # 单通道力传感器订阅 (/force_sensor)
        self.force_sub = self.create_subscription(Float32, '/force_sensor', self.force_callback, 10)

        # Publishers
        self.status_pub = self.create_publisher(String, 'lift_robot_pushrod/status', 10)
        self.force_debug_pub = self.create_publisher(String, 'lift_robot_pushrod/force_debug', 10)

        # Timers
        self.status_timer = self.create_timer(1.0, self.publish_status)
        self.control_timer = self.create_timer(CONTROL_RATE, self.control_loop)

        self.controller.initialize()
        self.get_logger().info("Pushrod control node started")

    # ------------------------------------------------------------------
    # Command Handling
    # ------------------------------------------------------------------
    def command_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            command = data.get('command', '').lower()
            seq_id_str = data.get('seq_id', str(uuid.uuid4())[:8])
            seq_id = abs(hash(seq_id_str)) % 65536
            self.get_logger().info(f"Received pushrod command: {command} [SEQ {seq_id_str}]")

            if command == 'stop':
                self.controller.stop(seq_id=seq_id)
                if self.is_tracking_offset and self.height_before_movement is not None:
                    delta = self.current_height - self.height_before_movement
                    self.pushrod_offset += delta
                    self.get_logger().info(f"[SEQ {seq_id_str}] Offset updated: {delta:.2f}mm (total {self.pushrod_offset:.2f}mm)")
                    self.is_tracking_offset = False
                    self.height_before_movement = None
                if self.control_enabled:
                    self.control_enabled = False
                    self.control_mode = 'manual'
                self.movement_state = 'stop'

            elif command == 'up':
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                self.controller.up(seq_id=seq_id)
                self.movement_state = 'up'

            elif command == 'down':
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                self.controller.down(seq_id=seq_id)
                self.movement_state = 'down'

            elif command == 'timed_up':
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                dur = float(data.get('duration', 1.0))
                self.controller.timed_up(dur, seq_id=seq_id)

            elif command == 'timed_down':
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                dur = float(data.get('duration', 1.0))
                self.controller.timed_down(dur, seq_id=seq_id)

            elif command == 'stop_timed':
                self.controller.stop_timed(seq_id=seq_id)

            elif command == 'goto_point':
                point = data.get('point')
                if not point:
                    self.get_logger().warning("goto_point requires 'point'")
                else:
                    if point == 'base':
                        self.pushrod_offset = 0.0
                        self.is_tracking_offset = False
                        self.height_before_movement = None
                        self.get_logger().info(f"[SEQ {seq_id_str}] Offset reset (base)")
                    else:
                        self.height_before_movement = self.current_height
                        self.is_tracking_offset = True
                    self.controller.goto_point(point, seq_id=seq_id)

            elif command == 'goto_height':
                target = data.get('target_height')
                if target is None:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] goto_height requires target_height")
                else:
                    self.height_before_movement = self.current_height
                    self.is_tracking_offset = True
                    self.target_height = float(target)
                    self.control_mode = 'auto'
                    self.control_enabled = True
                    self.height_history.clear()
                    self.movement_state = 'stop'
                    self.last_command_time = self.get_clock().now() - rclpy.duration.Duration(seconds=COMMAND_INTERVAL)
                    self.get_logger().info(f"[SEQ {seq_id_str}] Auto height target={self.target_height:.2f}mm")

            elif command == 'enable_force_control':
                self.target_force = float(data.get('target_force', 135.0))
                self.force_threshold = float(data.get('force_threshold', 5.0))
                self.force_settle_cycles = int(data.get('force_settle_cycles', self.force_settle_cycles))
                self.increase_on_up = bool(data.get('increase_on_up', self.increase_on_up))
                self.force_history.clear()
                initial_sum = self.left_force + self.right_force
                self.force_history.append(initial_sum)
                self.current_force = initial_sum
                self.force_in_band_consecutive = 0
                self.control_enabled = True
                self.control_mode = 'force'
                self.movement_state = 'stop'
                self.last_command_time = self.get_clock().now() - rclpy.duration.Duration(seconds=COMMAND_INTERVAL)
                self.get_logger().info(
                    f"[SEQ {seq_id_str}] Force control enabled target={self.target_force:.2f} ±{self.force_threshold:.2f} settle={self.force_settle_cycles}"
                )

            elif command == 'disable_force_control':
                if self.control_mode == 'force' and self.control_enabled:
                    self.controller.stop(seq_id=seq_id)
                    self.control_enabled = False
                    self.control_mode = 'manual'
                    self.movement_state = 'stop'
                    self.get_logger().info(f"[SEQ {seq_id_str}] Force control disabled")
            else:
                self.get_logger().warning(f"Unknown pushrod command: {command}")
        except json.JSONDecodeError:
            self.get_logger().error(f"Cannot parse command JSON: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Command handling error: {e}")

    # ------------------------------------------------------------------
    # Sensor callbacks
    # ------------------------------------------------------------------
    def sensor_callback(self, msg: String):
        try:
            data = json.loads(msg.data)
            height = data.get('height')
            if height is None:
                return
            raw = float(height)
            self.height_history.append(raw)
            if len(self.height_history) > HISTORY_SIZE:
                self.height_history.pop(0)
            self.current_height = self._calculate_filtered_height()
        except Exception:
            pass

    def force_callback(self, msg: Float32):
        try:
            val = float(msg.data)
            # 单通道力值更新
            self.force_value = val
            self.last_force_update_time = self.get_clock().now()
            self._update_force_history()
        except Exception:
            pass

    def _update_force_history(self):
        # 单通道：直接使用 left_force (与 right_force 相同)
        summed = self.force_value
        self.force_history.append(summed)
        if len(self.force_history) > HISTORY_SIZE:
            self.force_history.pop(0)
        self.current_force = self._calculate_filtered_force()
        # 估算采样频率
        now = self.get_clock().now()
        if hasattr(self, 'last_force_update_time') and self.last_force_update_time is not None:
            dt = (now - self.last_force_update_time).nanoseconds * 1e-9
            if 0.0 < dt < 1.0:  # 合理区间
                self.force_sample_intervals.append(dt)
                if len(self.force_sample_intervals) > 50:
                    self.force_sample_intervals.pop(0)
                avg_dt = sum(self.force_sample_intervals) / len(self.force_sample_intervals)
                if avg_dt > 0:
                    self.force_sample_rate_hz = 1.0 / avg_dt
        self.last_force_update_time = now

    def _calculate_filtered_height(self):
        if len(self.height_history) == 0:
            return self.current_height
        if len(self.height_history) < 4:
            return sum(self.height_history) / len(self.height_history)
        s = sorted(self.height_history)
        mid = s[1:-1]
        return sum(mid) / len(mid)

    def _calculate_filtered_force(self):
        if len(self.force_history) == 0:
            return self.current_force
        if len(self.force_history) < 4:
            return sum(self.force_history) / len(self.force_history)
        s = sorted(self.force_history)
        mid = s[1:-1]
        return sum(mid) / len(mid)

    def _on_auto_stop_complete(self):
        if self.is_tracking_offset and self.height_before_movement is not None:
            delta = self.current_height - self.height_before_movement
            self.pushrod_offset += delta
            self.is_tracking_offset = False
            self.height_before_movement = None
            self.get_logger().info(f"Auto-stop offset update: {delta:.2f}mm total={self.pushrod_offset:.2f}mm")

    # ------------------------------------------------------------------
    # Control Loop
    # ------------------------------------------------------------------
    def control_loop(self):
        if not self.control_enabled:
            return
        now = self.get_clock().now()
        dt = (now - self.last_command_time).nanoseconds * 1e-9

        # Height mode
        if self.control_mode == 'auto':
            if len(self.height_history) == 0:
                return
            error = self.target_height - self.current_height
            if abs(error) <= POSITION_TOLERANCE:
                self.controller.stop()
                if self.is_tracking_offset and self.height_before_movement is not None:
                    delta = self.current_height - self.height_before_movement
                    self.pushrod_offset += delta
                    self.is_tracking_offset = False
                    self.height_before_movement = None
                self.control_enabled = False
                self.movement_state = 'stop'
                self.get_logger().info(
                    f"[Pushrod Control] ✅ Height target reached current={self.current_height:.2f} target={self.target_height:.2f}"
                )
                self.last_command_time = now
                return
            if dt < COMMAND_INTERVAL:
                return
            if error > POSITION_TOLERANCE and self.movement_state != 'up':
                self.controller.up()
                self.movement_state = 'up'
                self.last_command_time = now
                self.get_logger().info(f"[Pushrod Control] UP current={self.current_height:.2f} target={self.target_height:.2f} err={error:.2f}")
            elif error < -POSITION_TOLERANCE and self.movement_state != 'down':
                self.controller.down()
                self.movement_state = 'down'
                self.last_command_time = now
                self.get_logger().info(f"[Pushrod Control] DOWN current={self.current_height:.2f} target={self.target_height:.2f} err={error:.2f}")
            return

        # Force mode
        if self.control_mode == 'force':
            sample_count = len(self.force_history)
            if sample_count < MIN_FORCE_SAMPLES:
                filtered_force = self.force_value
                self.force_data_valid = False
            else:
                filtered_force = self.current_force
                self.force_data_valid = True
            lower = self.target_force - self.force_threshold
            upper = self.target_force + self.force_threshold
            # 是否在目标区间
            in_band = lower <= filtered_force <= upper

            if in_band:
                # 停止运动（如果还在动）
                if self.movement_state != 'stop':
                    self.controller.stop()
                    self.movement_state = 'stop'
                    self.last_command_time = now
                # 计数连续 in-band 循环
                self.force_in_band_consecutive += 1
                # 达到最少稳定循环后，开始保持计时
                if self.force_in_band_consecutive >= self.force_settle_cycles:
                    if self.force_hold_start_time is None:
                        self.force_hold_start_time = now
                        self.get_logger().info(
                            f"[Pushrod Force Control] 🕒 已进入稳定区，开始保持 {self.force_settle_hold_seconds:.1f}s force={filtered_force:.2f} target={self.target_force:.2f}"
                        )
                    else:
                        hold_dt = (now - self.force_hold_start_time).nanoseconds * 1e-9
                        if hold_dt >= self.force_settle_hold_seconds:
                            # 保持时间满足 -> 退出力控
                            self.control_enabled = False
                            self.control_mode = 'manual'
                            self.get_logger().info(
                                f"[Pushrod Force Control] ✅ 保持完成 force={filtered_force:.2f} target={self.target_force:.2f} 总稳定循环={self.force_in_band_consecutive}"
                            )
                            return
                return  # 仍在观察期
            else:
                # 一旦离开区间，重置稳定计数和保持计时
                if self.force_in_band_consecutive or self.force_hold_start_time is not None:
                    self.get_logger().debug("[Pushrod Force Control] 离开稳定区，重置保持计时")
                self.force_in_band_consecutive = 0
                self.force_hold_start_time = None

            # 动态指令间隔: 接近目标时加快（精细调整）
            interval = COMMAND_INTERVAL
            if abs(filtered_force - self.target_force) < (2 * self.force_threshold):
                interval = FORCE_NEAR_INTERVAL
            if dt < interval:
                return

            if filtered_force < lower:
                desired = 'up' if self.increase_on_up else 'down'
                if self.movement_state != desired:
                    (self.controller.up() if desired == 'up' else self.controller.down())
                    self.movement_state = desired
                    self.last_command_time = now
                    self.get_logger().info(
                        f"[Pushrod Force Control] MOVE {desired.upper()} force={filtered_force:.2f} < {lower:.2f} target={self.target_force:.2f}"
                    )
            elif filtered_force > upper:
                desired = 'down' if self.increase_on_up else 'up'
                if self.movement_state != desired:
                    (self.controller.up() if desired == 'up' else self.controller.down())
                    self.movement_state = desired
                    self.last_command_time = now
                    self.get_logger().info(
                        f"[Pushrod Force Control] MOVE {desired.upper()} force={filtered_force:.2f} > {upper:.2f} target={self.target_force:.2f}"
                    )
            return
        # 发布调试数据（仅在 force 模式活跃或控制启用时也可发布）
        self._publish_force_debug()

    def _publish_force_debug(self):
        try:
            debug = {
                'time': self.get_clock().now().nanoseconds,
                'mode': self.control_mode,
                'enabled': self.control_enabled,
                'force_instant': self.force_value,
                'filtered_force': self.current_force,
                'target_force': self.target_force,
                'force_threshold': self.force_threshold,
                'lower': self.target_force - self.force_threshold,
                'upper': self.target_force + self.force_threshold,
                'in_band_consecutive': self.force_in_band_consecutive,
                'hold_active': self.force_hold_start_time is not None,
                'movement_state': self.movement_state,
                'sample_count': len(self.force_history),
                'sample_rate_hz': round(self.force_sample_rate_hz, 2),
            }
            msg = String()
            msg.data = json.dumps(debug, ensure_ascii=False)
            self.force_debug_pub.publish(msg)
        except Exception as e:
            self.get_logger().debug(f"force debug publish error: {e}")

    # ------------------------------------------------------------------
    # Status
    # ------------------------------------------------------------------
    def publish_status(self):
        try:
            status = {
                'node': 'lift_robot_pushrod',
                'device_id': self.device_id,
                'control_enabled': self.control_enabled,
                'control_mode': self.control_mode,
                'movement_state': self.movement_state,
                'current_height': self.current_height,
                'target_height': self.target_height,
                'pushrod_offset': self.pushrod_offset,
                'current_force': self.current_force,
                'target_force': self.target_force,
                'force_threshold': self.force_threshold,
                'force_sample_count': len(self.force_history),
                'force_data_valid': self.force_data_valid,
                'force_settle_cycles': self.force_settle_cycles,
                'force_in_band_consecutive': self.force_in_band_consecutive,
                'force_settle_hold_seconds': self.force_settle_hold_seconds,
                'force_hold_active': self.force_hold_start_time is not None,
                'force_sample_rate_hz': round(self.force_sample_rate_hz, 2),
                'increase_on_up': self.increase_on_up,
                'status': 'online'
            }
            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Status publish error: {e}")

    def destroy_node(self):
        self.get_logger().info("Stopping pushrod control node ...")
        self.controller.cleanup()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = PushrodNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Pushrod node runtime error: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
