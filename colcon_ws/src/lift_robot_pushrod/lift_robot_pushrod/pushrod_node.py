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
import time

logging.basicConfig(level=logging.INFO)

CONTROL_RATE = 0.02             # 50 Hz loop
POSITION_TOLERANCE = 0.5        # mm tolerance for height

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
        self.current_height = 0.0       # Raw height from sensor (no filtering)
        self.target_height = 0.0
        self.control_enabled = False
        self.control_mode = 'manual'    # manual | auto | force
        self.movement_state = 'stop'

        # Offset tracking
        self.pushrod_offset = 0.0
        self.height_before_movement = None
        self.is_tracking_offset = False

        # Force control state
        self.force_value = 0.0          # Raw force from sensor (no filtering)
        self.target_force = 0.0
        self.force_threshold = 0.0
        self.last_force_update_time = self.get_clock().now()
        self.increase_on_up = True      # if True: UP increases force
        # 采样速率估计
        self.force_sample_intervals = []
        self.force_sample_rate_hz = 0.0
        
        # ═══════════════════════════════════════════════════════════════
        # Task State Tracking (for web monitoring)
        # ═══════════════════════════════════════════════════════════════
        self.task_state = 'idle'           # 'idle' | 'running' | 'completed'
        self.task_type = None              # None | 'goto_height' | 'force_control' | 'manual_up' | 'manual_down'
        self.task_start_time = None        # Unix timestamp (seconds)
        self.task_end_time = None          # Unix timestamp (seconds)
        self.completion_reason = None      # None | 'target_reached' | 'force_reached' | 'manual_stop'

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
                # Mark task as manually stopped
                if self.task_state == 'running':
                    self._complete_task('manual_stop')

            elif command == 'up':
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                self.controller.up(seq_id=seq_id)
                self.movement_state = 'up'
                # Start manual up task
                self._start_task('manual_up')

            elif command == 'down':
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                self.controller.down(seq_id=seq_id)
                self.movement_state = 'down'
                # Start manual down task
                self._start_task('manual_down')

            elif command == 'timed_up':
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                dur = float(data.get('duration', 1.0))
                self.controller.timed_up(dur, seq_id=seq_id)
                # Start timed_up task
                self._start_task('timed_up')

            elif command == 'timed_down':
                self.height_before_movement = self.current_height
                self.is_tracking_offset = True
                dur = float(data.get('duration', 1.0))
                self.controller.timed_down(dur, seq_id=seq_id)
                # Start timed_down task
                self._start_task('timed_down')

            elif command == 'stop_timed':
                self.controller.stop_timed(seq_id=seq_id)
                # Mark timed task as manually stopped
                if self.task_state == 'running':
                    self._complete_task('manual_stop')

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
                    # Start goto_point task
                    self._start_task('goto_point')

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
                    self.movement_state = 'stop'
                    # Start goto_height task
                    self._start_task('goto_height')
                    self.get_logger().info(f"[SEQ {seq_id_str}] Auto height target={self.target_height:.2f}mm")

            elif command == 'enable_force_control':
                self.target_force = float(data.get('target_force', 750.0))
                self.force_threshold = float(data.get('force_threshold', 10.0))
                self.increase_on_up = bool(data.get('increase_on_up', True))
                self.control_enabled = True
                self.control_mode = 'force'
                self.movement_state = 'stop'
                # Start force control task
                self._start_task('force_control')
                self.get_logger().info(
                    f"[SEQ {seq_id_str}] Force control enabled: target={self.target_force:.2f}N ±{self.force_threshold:.2f}N, increase_on_up={self.increase_on_up}"
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
        """Update height from sensor (no filtering, use raw value)"""
        try:
            data = json.loads(msg.data)
            height = data.get('height')
            if height is not None:
                self.current_height = float(height)
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
        """Update force value and estimate sampling rate (no filtering, use raw value)"""
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

        # Height mode - simplified real-time control (50Hz, no interval limit, no filtering)
        if self.control_mode == 'auto':
            # Use raw height value directly (no filtering)
            error = self.target_height - self.current_height
            
            # In tolerance band - STOP and exit auto mode
            if abs(error) <= POSITION_TOLERANCE:
                self.controller.stop()
                if self.is_tracking_offset and self.height_before_movement is not None:
                    delta = self.current_height - self.height_before_movement
                    self.pushrod_offset += delta
                    self.is_tracking_offset = False
                    self.height_before_movement = None
                self.control_enabled = False
                self.movement_state = 'stop'
                # Mark goto_height task as completed
                self._complete_task('target_reached')
                self.get_logger().info(
                    f"[Pushrod Height] ✅ TARGET REACHED current={self.current_height:.2f}mm target={self.target_height:.2f}mm"
                )
                return
            
            # Need to move UP
            if error > POSITION_TOLERANCE:
                if self.movement_state != 'up':
                    self.controller.up()
                    self.movement_state = 'up'
                    self.get_logger().info(
                        f"[Pushrod Height] ⬆️ UP current={self.current_height:.2f}mm target={self.target_height:.2f}mm err={error:.2f}mm"
                    )
                return
            
            # Need to move DOWN
            if error < -POSITION_TOLERANCE:
                if self.movement_state != 'down':
                    self.controller.down()
                    self.movement_state = 'down'
                    self.get_logger().info(
                        f"[Pushrod Height] ⬇️ DOWN current={self.current_height:.2f}mm target={self.target_height:.2f}mm err={error:.2f}mm"
                    )
                return

        # Force mode - simplified real-time control (50Hz, no interval limit, no filtering)
        if self.control_mode == 'force':
            # Use raw force value directly (no filtering)
            current_force = self.force_value
            lower = self.target_force - self.force_threshold
            upper = self.target_force + self.force_threshold
            
            # In target band - STOP
            if lower <= current_force <= upper:
                if self.movement_state != 'stop':
                    self.controller.stop()
                    self.movement_state = 'stop'
                    # Mark force control task as completed
                    self._complete_task('force_reached')
                    self.get_logger().info(
                        f"[Pushrod Force] ✅ IN BAND force={current_force:.2f}N target={self.target_force:.2f}N ±{self.force_threshold:.2f}N -> STOP"
                    )
                return
            
            # Below target - need to increase force
            if current_force < lower:
                desired = 'up' if self.increase_on_up else 'down'
                if self.movement_state != desired:
                    if desired == 'up':
                        self.controller.up()
                    else:
                        self.controller.down()
                    self.movement_state = desired
                    self.get_logger().info(
                        f"[Pushrod Force] ⬆️ INCREASE force={current_force:.2f}N < {lower:.2f}N -> {desired.upper()}"
                    )
                return
            
            # Above target - need to decrease force
            if current_force > upper:
                desired = 'down' if self.increase_on_up else 'up'
                if self.movement_state != desired:
                    if desired == 'up':
                        self.controller.up()
                    else:
                        self.controller.down()
                    self.movement_state = desired
                    self.get_logger().info(
                        f"[Pushrod Force] ⬇️ DECREASE force={current_force:.2f}N > {upper:.2f}N -> {desired.upper()}"
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
                'force_raw': self.force_value,
                'target_force': self.target_force,
                'force_threshold': self.force_threshold,
                'lower': self.target_force - self.force_threshold,
                'upper': self.target_force + self.force_threshold,
                'movement_state': self.movement_state,
                'sample_rate_hz': round(self.force_sample_rate_hz, 2),
                'increase_on_up': self.increase_on_up,
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
                'force_value': self.force_value,
                'target_force': self.target_force,
                'force_threshold': self.force_threshold,
                'force_sample_rate_hz': round(self.force_sample_rate_hz, 2),
                'increase_on_up': self.increase_on_up,
                'status': 'online'
            }
            
            # Add task state tracking
            status['task_state'] = self._get_task_state()
            status['task_type'] = self.task_type
            if self.task_start_time is not None:
                status['task_start_time'] = self.task_start_time
            if self.task_end_time is not None:
                status['task_end_time'] = self.task_end_time
                status['task_duration'] = self.task_end_time - self.task_start_time
            if self.completion_reason is not None:
                status['completion_reason'] = self.completion_reason
            
            msg = String()
            msg.data = json.dumps(status)
            self.status_pub.publish(msg)
        except Exception as e:
            self.get_logger().error(f"Status publish error: {e}")
    
    # ═══════════════════════════════════════════════════════════════
    # Task State Management Methods
    # ═══════════════════════════════════════════════════════════════
    def _start_task(self, task_type):
        """Start a new task and record timestamp"""
        self.task_state = 'running'
        self.task_type = task_type
        self.task_start_time = time.time()
        self.task_end_time = None
        self.completion_reason = None
        self.get_logger().debug(f"[Task] Started: {task_type}")
    
    def _complete_task(self, reason):
        """Mark task as completed with reason and timestamp"""
        if self.task_state == 'running':
            self.task_state = 'completed'
            self.completion_reason = reason
            self.task_end_time = time.time()
            duration = self.task_end_time - self.task_start_time if self.task_start_time else 0
            self.get_logger().info(
                f"[Task Complete] type={self.task_type} reason={reason} duration={duration:.2f}s"
            )
    
    def _get_task_state(self):
        """Get current task state based on control variables (ensures consistency)"""
        # Running if any control is active
        if self.control_enabled or self.movement_state != 'stop':
            return 'running'
        # Completed state persists for 5 seconds after task end
        elif self.task_end_time is not None and (time.time() - self.task_end_time) < 5.0:
            return 'completed'
        else:
            return 'idle'

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
