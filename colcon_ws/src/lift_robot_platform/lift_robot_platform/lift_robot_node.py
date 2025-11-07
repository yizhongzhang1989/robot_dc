#!/usr/bin/env python3
"""
Lift Robot Platform ROS2 Node
Controls the lift using relay pulse (flash) commands.
Includes high-frequency closed-loop control for smooth height tracking.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .lift_robot_controller import LiftRobotController
import json
import uuid
import logging
import threading

# Configure root logging level
logging.basicConfig(level=logging.INFO)

# ═══════════════════════════════════════════════════════════════
# Control Loop Parameters
# ═══════════════════════════════════════════════════════════════
CONTROL_RATE = 0.02             # 控制循环 50 Hz（每 0.02s）
POSITION_TOLERANCE = 0.05       # 调低误差带：目标高度允许误差 ±0.05 mm，减少过早判定完成
# 提升频率：取消原 0.3s 节流，改为仅在"需要改变方向或停止"时发送继电器脉冲。
# 不再使用 COMMAND_INTERVAL（保留变量以兼容旧逻辑但设为 0）。
COMMAND_INTERVAL = 0.0          # 设为 0 表示不做时间节流，仅靠 movement_state 去重
# 预测性提前停参数（用于减少超调）
OVERSHOOT_INIT_UP = 1.178      # 初始向上超调估计 (mm) 使用实验推荐 median (cv=0.111)
OVERSHOOT_INIT_DOWN = 1.300    # 初始向下超调估计 (mm) 使用实验推荐 median (cv=0.097)
OVERSHOOT_ALPHA = 0.25         # 指数平均权重 (新值占比)
OVERSHOOT_SETTLE_DELAY = 0.25  # (s) 停止后等待稳定再测量超调
OVERSHOOT_MIN_MARGIN = 0.3     # (mm) 低于该值不使用提前停，避免过早停止导致未达目标


class LiftRobotNode(Node):
    def __init__(self):
        super().__init__('lift_robot_platform')
        
        # Declare parameters
        self.declare_parameter('device_id', 1)
        self.declare_parameter('use_ack_patch', True)
        
        # Retrieve parameters
        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value
        
        # NOTE: Serial port and baudrate are now centrally managed by the modbus_driver node.
        # This node no longer opens the serial device directly; parameters were removed to avoid confusion.
        self.get_logger().info(
            f"Initialize lift platform controller - device_id: {self.device_id} (serial handled by modbus_driver)"
        )
        
        # ═══════════════════════════════════════════════════════════════
        # Control Loop State Variables
        # ═══════════════════════════════════════════════════════════════
        self.current_height = 0.0           # Current height from cable sensor (mm) - no filtering needed for digital signal
        self.target_height = 0.0            # Target height setpoint (mm)
        self.last_command_time = self.get_clock().now()  # 兼容旧逻辑（当前不再用于节流）
        self.control_enabled = False        # Enable/disable closed-loop control
        self.control_mode = 'manual'        # 'manual' or 'auto' (height control)
        self.movement_state = 'stop'        # Current movement state: 'up', 'down', or 'stop'
        # Overshoot tracking state (must be instance attributes)
        self.avg_overshoot_up = OVERSHOOT_INIT_UP
        self.avg_overshoot_down = OVERSHOOT_INIT_DOWN
        self.height_at_stop = None
        self.last_stop_direction = None      # 'up' or 'down'
        self.last_stop_time = None
        self.overshoot_timer = None
        # Bootstrap & recent raw samples for recommendation
        self.overshoot_bootstrap_samples_up = []   # 初始引导阶段向上超调原始值
        self.overshoot_bootstrap_samples_down = [] # 初始引导阶段向下超调原始值
        self.OVERSHOOT_BOOTSTRAP_COUNT = 3         # 收集多少原始样本后确定初始 EMA 基准（降低使重置更早）
        self.recent_raw_overshoot_up = []          # 最近若干次向上 raw 超调
        self.recent_raw_overshoot_down = []        # 最近若干次向下 raw 超调
        self.RECENT_RAW_LIMIT = 8                  # 最近样本保留数量
        self.RECOMMEND_MIN_STABLE_COUNT = 3        # 至少多少次非零样本后才给出稳定推荐
        self.OVERSHOOT_VARIANCE_THRESHOLD = 0.18   # 变异系数 (std/mean) 低于该值认为稳定
        
        # Create controller
        self.controller = LiftRobotController(
            device_id=self.device_id,
            node=self,
            use_ack_patch=self.use_ack_patch
        )
        
        # Subscribe to command topic
        self.subscription = self.create_subscription(
            String,
            'lift_robot_platform/command',
            self.command_callback,
            10
        )
        
        # Subscribe to draw-wire sensor for closed-loop height control
        self.sensor_subscription = self.create_subscription(
            String,
            '/draw_wire_sensor/data',
            self.sensor_callback,
            10
        )
        
        # Publish status topic
        self.status_publisher = self.create_publisher(
            String,
            'lift_robot_platform/status',
            10
        )
        
        # Status publish timer
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # High-frequency control loop timer
        self.control_timer = self.create_timer(CONTROL_RATE, self.control_loop)
        
        # Initialize lift platform
        self.controller.initialize()
        
        self.get_logger().info("Lift platform control node started")

    def command_callback(self, msg):
        """Handle command message"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '').lower()
            seq_id_str = command_data.get('seq_id', str(uuid.uuid4())[:8])
            # Convert seq_id string -> bounded int using hash for uniqueness
            seq_id = abs(hash(seq_id_str)) % 65536  # constrain to 0-65535
            
            self.get_logger().info(f"Received command: {command} [SEQ {seq_id_str}]")
            
            if command == 'stop':
                self.controller.stop(seq_id=seq_id)
                # Also disable auto control if active
                if self.control_enabled:
                    self.control_enabled = False
                    self.control_mode = 'manual'
                    self.get_logger().info(f"[SEQ {seq_id_str}] Manual stop - auto control disabled")
                self.movement_state = 'stop'
                
            elif command == 'up':
                self.controller.up(seq_id=seq_id)
                self.movement_state = 'up'
                
            elif command == 'down':
                self.controller.down(seq_id=seq_id)
                self.movement_state = 'down'
                
            elif command == 'timed_up':
                duration = command_data.get('duration', 1.0)
                self.controller.timed_up(duration, seq_id=seq_id)
                
            elif command == 'timed_down':
                duration = command_data.get('duration', 1.0)
                self.controller.timed_down(duration, seq_id=seq_id)
                
            elif command == 'stop_timed':
                self.controller.stop_timed(seq_id=seq_id)
                
            elif command == 'goto_height':
                # New command: go to specific height with closed-loop control
                target = command_data.get('target_height')
                if target is not None:
                    self.target_height = float(target)
                    self.control_mode = 'auto'
                    self.control_enabled = True
                    # Reset tracking to allow immediate first command
                    self.movement_state = 'stop'  # Reset movement state
                    # 旧逻辑通过回退 last_command_time 触发首条指令；现在不再依赖时间节流
                    self.last_command_time = self.get_clock().now()
                    self.get_logger().info(f"[SEQ {seq_id_str}] Auto mode: target height = {self.target_height:.2f} mm")
                else:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] goto_height requires target_height field")
                
            else:
                self.get_logger().warning(f"Unknown command: {command}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"Cannot parse command JSON: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"Error handling command: {e}")

    def sensor_callback(self, msg):
        """Handle draw-wire sensor feedback for closed-loop control"""
        try:
            sensor_data = json.loads(msg.data)
            # Use adjusted height (includes pushrod offset)
            # Cable sensor is digital signal, use raw value directly without filtering
            height = sensor_data.get('height')
            if height is not None:
                self.current_height = float(height)
                
        except (json.JSONDecodeError, ValueError, KeyError) as e:
            self.get_logger().debug(f"Failed to parse sensor data: {e}")

    def control_loop(self):
        """
        Closed-loop control using direct cable sensor reading.
        
        Control strategy:
        1. Use raw cable sensor height (digital signal, no filtering needed)
        2. Calculate error = target - current
        3. Send up/down/stop command based on error
        4. Commands only sent when direction changes (no time throttling)
        """
        try:
            if not self.control_enabled or self.control_mode != 'auto':
                return
                
            # Calculate position error
            error = self.target_height - self.current_height
            abs_error = abs(error)
            
            # 时间戳仍保留用于调试或未来扩展（比如统计命令频率）
            now = self.get_clock().now()
            
        except Exception as e:
            self.get_logger().error(f"[Control] Loop error (calculation): {e}")
            return
        
        # Priority 1: Check if target reached
        if abs_error <= POSITION_TOLERANCE:
            if self.control_enabled:
                # 达到目标误差带：立即终止自动控制，不再发送任何相反方向修正；仅继续超调测量
                self._issue_stop(direction=self.movement_state, reason="target_band", disable_control=True)
            return
        
        # Priority 2: 预测提前停（基于当前方向和平均超调）
        if self.control_enabled:
            # 计算基于方向的提前停阈值
            if self.movement_state == 'up' and self.avg_overshoot_up > OVERSHOOT_MIN_MARGIN:
                threshold_height = self.target_height - self.avg_overshoot_up
                if self.current_height >= threshold_height:
                    # 预测提前停：终止控制，剩余惯性与超调仅记录不纠正
                    self._issue_stop(direction='up', reason=f"early_stop_up(th={threshold_height:.2f})", disable_control=True)
                    return
            elif self.movement_state == 'down' and self.avg_overshoot_down > OVERSHOOT_MIN_MARGIN:
                threshold_height = self.target_height + self.avg_overshoot_down
                if self.current_height <= threshold_height:
                    self._issue_stop(direction='down', reason=f"early_stop_down(th={threshold_height:.2f})", disable_control=True)
                    return

        # Priority 3: 移除时间节流逻辑：只在方向需要变化或到达目标时发送命令。
        # （依靠 movement_state 防止重复脉冲）
        
        # Priority 3: Send movement command based on error direction
        try:
            if error > POSITION_TOLERANCE:
                # Need to move up - only send command if not already moving up
                if self.movement_state != 'up':
                    # 仅在方向变化时发送一次脉冲
                    self.controller.up()
                    self.movement_state = 'up'
                    self.get_logger().info(
                        f"[Control] ⬆️  DIR->UP current={self.current_height:.2f} target={self.target_height:.2f} err={error:.2f}"
                    )
                    self.last_command_time = now
                # 已经在向上运动则不重复发指令（避免 50Hz 重复脉冲）
                
            elif error < -POSITION_TOLERANCE:
                if self.movement_state != 'down':
                    self.controller.down()
                    self.movement_state = 'down'
                    self.get_logger().info(
                        f"[Control] ⬇️  DIR->DOWN current={self.current_height:.2f} target={self.target_height:.2f} err={error:.2f}"
                    )
                    self.last_command_time = now
                
                
        except Exception as e:
            self.get_logger().error(f"[Control] Command execution error: {e}")
            # Don't disable control on command errors, just skip this cycle

    def publish_status(self):
        """Publish periodic status info"""
        try:
            status = {
                'node': 'lift_robot_platform',
                'device_id': self.device_id,
                'active_timers': len(self.controller.active_timers) if hasattr(self.controller, 'active_timers') else 0,
                'control_enabled': self.control_enabled,
                'control_mode': self.control_mode,
                'current_height': self.current_height,
                'target_height': self.target_height,
                'movement_state': self.movement_state,
                'avg_overshoot_up': round(self.avg_overshoot_up, 3),
                'avg_overshoot_down': round(self.avg_overshoot_down, 3),
                'status': 'online'
            }
            
            status_msg = String()
            status_msg.data = json.dumps(status)
            self.status_publisher.publish(status_msg)
        except Exception as e:
            self.get_logger().error(f"Status publish error: {e}")
            # Continue operation, status is not critical

    def destroy_node(self):
        """Cleanup resources"""
        self.get_logger().info("Stopping lift platform control node ...")
        
        # Stop platform & cleanup timers
        self.controller.cleanup()
        
        super().destroy_node()

    # ─────────────────────────────────────────────────────────────
    # Overshoot helper methods
    # ─────────────────────────────────────────────────────────────
    def _issue_stop(self, direction, reason="stop", disable_control=False):
        """发送停止脉冲并安排超调测量。
        当前策略：所有停止（目标带或提前停）一律 disable_control=True 防止后续相反方向补偿；
        但超调测量与 EMA 更新仍继续，以供人工分析和后续手动调整初始参数。
        """
        try:
            self.controller.stop()
        except Exception as e:
            self.get_logger().error(f"[Control] Stop command error: {e}")
        if disable_control:
            self.control_enabled = False
        prev_state = self.movement_state
        self.movement_state = 'stop'
        self.height_at_stop = self.current_height
        self.last_stop_direction = direction if direction in ('up','down') else prev_state
        self.last_stop_time = self.get_clock().now()
        # 取消旧的 overshoot timer
        if self.overshoot_timer and self.overshoot_timer.is_alive():
            self.overshoot_timer.cancel()
        # 计划测量稳定高度
        self.overshoot_timer = threading.Timer(OVERSHOOT_SETTLE_DELAY, self._measure_overshoot)
        self.overshoot_timer.start()
        self.get_logger().info(
            f"[Control] 🛑 STOP ({reason}) height_at_stop={self.height_at_stop:.2f} dir={self.last_stop_direction} disable_control={disable_control}"
        )

    def _measure_overshoot(self):
        """Measure overshoot after settle delay and update EMA."""
        try:
            stable_height = self.current_height
            if self.height_at_stop is None or self.last_stop_direction is None:
                return
            if self.last_stop_direction == 'up':
                raw_overshoot = max(0.0, stable_height - self.height_at_stop)
                # Bootstrap 引导阶段：优先收集原始样本，达到个数后用中位数重置 EMA
                if len(self.overshoot_bootstrap_samples_up) < self.OVERSHOOT_BOOTSTRAP_COUNT:
                    self.overshoot_bootstrap_samples_up.append(raw_overshoot)
                    self.get_logger().info(
                        f"[Overshoot-Bootstrap] UP sample={raw_overshoot:.3f} collected={len(self.overshoot_bootstrap_samples_up)}/{self.OVERSHOOT_BOOTSTRAP_COUNT}"
                    )
                    if len(self.overshoot_bootstrap_samples_up) == self.OVERSHOOT_BOOTSTRAP_COUNT:
                        median_val = sorted(self.overshoot_bootstrap_samples_up)[len(self.overshoot_bootstrap_samples_up)//2]
                        self.avg_overshoot_up = median_val
                        self.get_logger().info(
                            f"[Overshoot-Bootstrap] UP median={median_val:.3f} -> init EMA reset"
                        )
                else:
                    # 正常 EMA 更新
                    self.avg_overshoot_up = (1 - OVERSHOOT_ALPHA) * self.avg_overshoot_up + OVERSHOOT_ALPHA * raw_overshoot
                # 维护最近原始样本列表
                self.recent_raw_overshoot_up.append(raw_overshoot)
                if len(self.recent_raw_overshoot_up) > self.RECENT_RAW_LIMIT:
                    self.recent_raw_overshoot_up.pop(0)
                self.get_logger().info(
                    f"[Overshoot] UP measured={raw_overshoot:.3f} avg={self.avg_overshoot_up:.3f} stable={stable_height:.2f} stop={self.height_at_stop:.2f}"
                )
                # 推荐初始参数输出
                self._recommend_overshoot_init(direction='up')
            elif self.last_stop_direction == 'down':
                raw_overshoot = max(0.0, self.height_at_stop - stable_height)
                if len(self.overshoot_bootstrap_samples_down) < self.OVERSHOOT_BOOTSTRAP_COUNT:
                    self.overshoot_bootstrap_samples_down.append(raw_overshoot)
                    self.get_logger().info(
                        f"[Overshoot-Bootstrap] DOWN sample={raw_overshoot:.3f} collected={len(self.overshoot_bootstrap_samples_down)}/{self.OVERSHOOT_BOOTSTRAP_COUNT}"
                    )
                    if len(self.overshoot_bootstrap_samples_down) == self.OVERSHOOT_BOOTSTRAP_COUNT:
                        median_val = sorted(self.overshoot_bootstrap_samples_down)[len(self.overshoot_bootstrap_samples_down)//2]
                        self.avg_overshoot_down = median_val
                        self.get_logger().info(
                            f"[Overshoot-Bootstrap] DOWN median={median_val:.3f} -> init EMA reset"
                        )
                else:
                    self.avg_overshoot_down = (1 - OVERSHOOT_ALPHA) * self.avg_overshoot_down + OVERSHOOT_ALPHA * raw_overshoot
                self.recent_raw_overshoot_down.append(raw_overshoot)
                if len(self.recent_raw_overshoot_down) > self.RECENT_RAW_LIMIT:
                    self.recent_raw_overshoot_down.pop(0)
                self.get_logger().info(
                    f"[Overshoot] DOWN measured={raw_overshoot:.3f} avg={self.avg_overshoot_down:.3f} stable={stable_height:.2f} stop={self.height_at_stop:.2f}"
                )
                self._recommend_overshoot_init(direction='down')
            # 复位高度参考
            self.height_at_stop = None
            self.last_stop_direction = None
        except Exception as e:
            self.get_logger().error(f"Overshoot measurement error: {e}")

    def _recommend_overshoot_init(self, direction: str):
        """基于最近原始超调样本给出下一次运行的初始参数推荐。
        策略：
        1. 样本数量不足 → 不给推荐。
        2. 计算 mean, std, median。
        3. 若变异系数 (std/mean) < 阈值且样本数≥RECOMMEND_MIN_STABLE_COUNT，推荐使用 median（更抗离群）。
        4. 若波动尚大，仅提示使用当前 EMA。
        5. 最终打印统一格式方便人工复制到配置。
        """
        samples = self.recent_raw_overshoot_up if direction == 'up' else self.recent_raw_overshoot_down
        if len(samples) < self.RECOMMEND_MIN_STABLE_COUNT:
            self.get_logger().info(f"[Overshoot-Recommend] {direction.upper()} insufficient samples ({len(samples)}/{self.RECOMMEND_MIN_STABLE_COUNT})")
            return
        mean_val = sum(samples) / len(samples)
        # 计算标准差
        var = sum((x - mean_val) ** 2 for x in samples) / len(samples)
        std_val = var ** 0.5
        median_val = sorted(samples)[len(samples)//2]
        cv = std_val / mean_val if mean_val > 1e-6 else 0.0
        if cv < self.OVERSHOOT_VARIANCE_THRESHOLD:
            recommended = median_val
            reason = f"stable cv={cv:.3f}<thr use median"
        else:
            # 波动较大：用当前 EMA 作为参考，但加上说明
            ema_val = self.avg_overshoot_up if direction == 'up' else self.avg_overshoot_down
            recommended = ema_val
            reason = f"unstable cv={cv:.3f} use EMA"
        self.get_logger().info(
            f"[Overshoot-Recommend] {direction.upper()} samples={len(samples)} mean={mean_val:.3f} std={std_val:.3f} median={median_val:.3f} cv={cv:.3f} -> next_init={recommended:.3f} ({reason})"
        )


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LiftRobotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"Node runtime error: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
