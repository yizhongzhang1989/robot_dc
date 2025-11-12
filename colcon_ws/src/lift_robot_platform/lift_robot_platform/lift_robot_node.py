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
import time

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

# ═══════════════════════════════════════════════════════════════
# Overshoot Learning Configuration
# ═══════════════════════════════════════════════════════════════
# 超调学习开关：控制是否启用超调自适应学习功能
# - True: 启用学习模式
#   * 每次 goto_height 后测量实际超调值
#   * 使用 EMA 算法持续更新 avg_overshoot_up/down
#   * 打印 Bootstrap 引导和推荐日志
#   * 累积样本跨 goto_height 会话保留
#   * 日志会输出推荐的 OVERSHOOT_INIT 值供人工调整
# - False: 使用固定模式（默认）
#   * 每次 goto_height 启动时重置为下方的 OVERSHOOT_INIT 固定值
#   * 不进行超调测量、不更新 EMA、不打印学习日志
#   * 适合生产环境，避免日志干扰
OVERSHOOT_LEARNING_ENABLED = False  

# 预测性提前停参数（用于减少超调）
# 这些是固定初始值，当 OVERSHOOT_LEARNING_ENABLED=False 时每次都使用这些值
# 当学习模式开启后，可根据日志中的推荐值手动更新这里的常量
OVERSHOOT_INIT_UP = 2.067      # 初始向上超调估计 (mm) 使用实验推荐 median (cv=0.111)
OVERSHOOT_INIT_DOWN = 2.699    # 初始向下超调估计 (mm) 使用实验推荐 median (cv=0.097)
OVERSHOOT_ALPHA = 0.25         # 指数平均权重 (新值占比) - 仅学习模式使用
OVERSHOOT_SETTLE_DELAY = 0.25  # (s) 停止后等待稳定再测量超调 - 仅学习模式使用
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
        
        # ═══════════════════════════════════════════════════════════════
        # Thread Safety: Control Loop Lock
        # ═══════════════════════════════════════════════════════════════
        self.control_lock = threading.RLock()  # Reentrant lock for control loop safety
        self.reset_in_progress = False          # Flag to signal control loop to stop
        
        # NOTE: Serial port and baudrate are now centrally managed by the modbus_driver node.
        # This node no longer opens the serial device directly; parameters were removed to avoid confusion.
        self.get_logger().info(
            f"Initialize lift platform controller - device_id: {self.device_id} (serial handled by modbus_driver)"
        )
        
        # ═══════════════════════════════════════════════════════════════
        # Control Loop State Variables
        # ═══════════════════════════════════════════════════════════════
        # MUTUAL EXCLUSION: Only ONE control mode can be active at a time
        # - control_enabled = True  → Platform height auto control (goto_height)
        # - force_control_active = True → Platform force control (force_up/down)
        # When starting a new control mode, the other MUST be disabled first
        # Pushrod control is in separate node, no conflict with platform
        self.current_height = 0.0           # Current height from cable sensor (mm) - no filtering needed for digital signal
        self.target_height = 0.0            # Target height setpoint (mm)
        self.last_command_time = self.get_clock().now()  # 兼容旧逻辑（当前不再用于节流）
        self.control_enabled = False        # Enable/disable closed-loop HEIGHT control (PLATFORM ONLY)
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
        
        # ═══════════════════════════════════════════════════════════════
        # Global System Lock (Platform and Pushrod Mutual Exclusion)
        # ═══════════════════════════════════════════════════════════════
        # To ensure only ONE control operation runs at a time (platform OR pushrod)
        # This simplifies web status logic - only one task can be running
        self.system_busy = False           # True when ANY control is active
        self.active_control_owner = None   # 'platform' or 'pushrod' - who owns the lock
        
        # ═══════════════════════════════════════════════════════════════
        # Task State Tracking (for web monitoring)
        # ═══════════════════════════════════════════════════════════════
        self.task_state = 'idle'           # 'idle' | 'running' | 'completed' | 'emergency_stop'
        self.task_type = None              # None | 'goto_height' | 'force_up' | 'force_down' | 'manual_up' | 'manual_down'
        self.task_start_time = None        # Unix timestamp (seconds)
        self.task_end_time = None          # Unix timestamp (seconds)
        self.completion_reason = None      # None | 'target_reached' | 'force_reached' | 'limit_exceeded' | 'manual_stop'
        
        # Create controller
        self.controller = LiftRobotController(
            device_id=self.device_id,
            node=self,
            use_ack_patch=self.use_ack_patch
        )
        
        # Set callback for timed operation auto-stop
        self.controller.on_auto_stop_callback = self._on_auto_stop_complete
        
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
        # ═══════════════════════════════════════════════════════════════
        # Force Control State (PLATFORM ONLY, mutually exclusive with height auto)
        # ═══════════════════════════════════════════════════════════════
        # When force_control_active=True, control_enabled MUST be False
        self.current_force_right = None    # /force_sensor
        self.current_force_left = None     # /force_sensor_2
        self.current_force_combined = None # sum or single available
        self.target_force = None           # target threshold (N)
        self.force_control_direction = None  # 'up'|'down'
        self.force_control_active = False  # Enable/disable FORCE control (PLATFORM ONLY)

        # Subscribe force sensors
        try:
            from std_msgs.msg import Float32
            self.force_sub_right = self.create_subscription(Float32, '/force_sensor', self.force_cb_right, 10)
            self.force_sub_left = self.create_subscription(Float32, '/force_sensor_2', self.force_cb_left, 10)
            self.get_logger().info("Subscribed force sensors /force_sensor & /force_sensor_2")
        except Exception as e:
            self.get_logger().warn(f"Force sensor subscription error: {e}")

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
                # Step 1: Disable all auto control modes FIRST
                if self.control_enabled:
                    self.control_enabled = False
                    self.control_mode = 'manual'
                    self.get_logger().info(f"[SEQ {seq_id_str}] Manual stop - height auto control disabled")
                if self.force_control_active:
                    self.force_control_active = False
                    self.get_logger().info(f"[SEQ {seq_id_str}] Manual stop - force control disabled")
                
                # Step 2: Wait for control loop to detect flag change and stop sending commands
                # Wait 1 control cycle (20ms) to ensure control loop has exited
                time.sleep(CONTROL_RATE)  # 20ms = 1 control cycle
                
                # Step 3: Send hardware STOP pulse (after control loop stopped)
                self.controller.stop(seq_id=seq_id)
                self.movement_state = 'stop'
                
                # Step 4: Mark task as manually stopped and release system lock
                if self.task_state == 'running':
                    self._complete_task('manual_stop')
                
            elif command == 'up':
                # If interrupting a running closed-loop task, complete it first
                if self.task_state == 'running' and self.task_type in ['goto_height', 'force_up', 'force_down']:
                    self._complete_task('manual_stop')
                    self.get_logger().info(f"[SEQ {seq_id_str}] Manual up - interrupted {self.task_type}")
                
                # Disable auto control modes if active (manual control takes priority)
                if self.control_enabled:
                    self.control_enabled = False
                    self.control_mode = 'manual'
                    self.get_logger().info(f"[SEQ {seq_id_str}] Manual up - height auto control disabled")
                if self.force_control_active:
                    self.force_control_active = False
                    self.get_logger().info(f"[SEQ {seq_id_str}] Manual up - force control disabled")
                
                self.controller.up(seq_id=seq_id)
                self.movement_state = 'up'
                # Start manual up task
                self._start_task('manual_up')
                
            elif command == 'down':
                # If interrupting a running closed-loop task, complete it first
                if self.task_state == 'running' and self.task_type in ['goto_height', 'force_up', 'force_down']:
                    self._complete_task('manual_stop')
                    self.get_logger().info(f"[SEQ {seq_id_str}] Manual down - interrupted {self.task_type}")
                
                # Disable auto control modes if active (manual control takes priority)
                if self.control_enabled:
                    self.control_enabled = False
                    self.control_mode = 'manual'
                    self.get_logger().info(f"[SEQ {seq_id_str}] Manual down - height auto control disabled")
                if self.force_control_active:
                    self.force_control_active = False
                    self.get_logger().info(f"[SEQ {seq_id_str}] Manual down - force control disabled")
                
                self.controller.down(seq_id=seq_id)
                self.movement_state = 'down'
                # Start manual down task
                self._start_task('manual_down')
                
            elif command == 'timed_up':
                # If interrupting a running closed-loop task, complete it first
                if self.task_state == 'running' and self.task_type in ['goto_height', 'force_up', 'force_down']:
                    self._complete_task('manual_stop')
                    self.get_logger().info(f"[SEQ {seq_id_str}] Timed up - interrupted {self.task_type}")
                
                # Disable auto control modes if active (manual control takes priority)
                if self.control_enabled:
                    self.control_enabled = False
                    self.control_mode = 'manual'
                    self.get_logger().info(f"[SEQ {seq_id_str}] Timed up - height auto control disabled")
                if self.force_control_active:
                    self.force_control_active = False
                    self.get_logger().info(f"[SEQ {seq_id_str}] Timed up - force control disabled")
                
                duration = command_data.get('duration', 1.0)
                self.controller.timed_up(duration, seq_id=seq_id)
                # Start timed_up task
                self._start_task('timed_up')
                
            elif command == 'timed_down':
                # If interrupting a running closed-loop task, complete it first
                if self.task_state == 'running' and self.task_type in ['goto_height', 'force_up', 'force_down']:
                    self._complete_task('manual_stop')
                    self.get_logger().info(f"[SEQ {seq_id_str}] Timed down - interrupted {self.task_type}")
                
                # Disable auto control modes if active (manual control takes priority)
                if self.control_enabled:
                    self.control_enabled = False
                    self.control_mode = 'manual'
                    self.get_logger().info(f"[SEQ {seq_id_str}] Timed down - height auto control disabled")
                if self.force_control_active:
                    self.force_control_active = False
                    self.get_logger().info(f"[SEQ {seq_id_str}] Timed down - force control disabled")
                
                duration = command_data.get('duration', 1.0)
                self.controller.timed_down(duration, seq_id=seq_id)
                # Start timed_down task
                self._start_task('timed_down')
                
            elif command == 'stop_timed':
                self.controller.stop_timed(seq_id=seq_id)
                # Mark timed task as manually stopped
                if self.task_state == 'running':
                    self._complete_task('manual_stop')
                
            elif command == 'goto_height':
                # Platform height control - check if system is busy
                if self.system_busy and self.active_control_owner != 'platform':
                    self.get_logger().warning(
                        f"[SEQ {seq_id_str}] goto_height REJECTED - {self.active_control_owner} is busy (task={self.task_type})"
                    )
                    return
                
                # New command: go to specific height with closed-loop control
                target = command_data.get('target_height')
                if target is not None:
                    # Auto-initialize: disable force control if active
                    if self.force_control_active:
                        self.force_control_active = False
                        self.get_logger().info(f"[SEQ {seq_id_str}] Disabled force control for goto_height")
                    
                    self.target_height = float(target)
                    self.control_mode = 'auto'
                    self.control_enabled = True
                    
                    # Reset movement tracking state for new control session
                    self.movement_state = 'stop'  # Reset movement state
                    self.last_command_time = self.get_clock().now()
                    
                    # Clear any pending overshoot measurement from previous session
                    if self.overshoot_timer and self.overshoot_timer.is_alive():
                        self.overshoot_timer.cancel()
                        self.overshoot_timer = None
                    self.height_at_stop = None
                    self.last_stop_direction = None
                    self.last_stop_time = None
                    
                    # Overshoot learning control
                    if OVERSHOOT_LEARNING_ENABLED:
                        # Learning mode: avg_overshoot values persist and accumulate across sessions
                        # Bootstrap samples and recent raw samples also persist
                        self.get_logger().info(
                            f"[SEQ {seq_id_str}] Overshoot learning ENABLED - "
                            f"using learned values (up={self.avg_overshoot_up:.3f}mm, down={self.avg_overshoot_down:.3f}mm)"
                        )
                    else:
                        # Fixed mode: reset to initial values from constants every time
                        self.avg_overshoot_up = OVERSHOOT_INIT_UP
                        self.avg_overshoot_down = OVERSHOOT_INIT_DOWN
                        # Clear learning samples (no accumulation)
                        self.overshoot_bootstrap_samples_up = []
                        self.overshoot_bootstrap_samples_down = []
                        self.recent_raw_overshoot_up = []
                        self.recent_raw_overshoot_down = []
                        self.get_logger().info(
                            f"[SEQ {seq_id_str}] Overshoot learning DISABLED - "
                            f"using fixed values (up={OVERSHOOT_INIT_UP:.3f}mm, down={OVERSHOOT_INIT_DOWN:.3f}mm)"
                        )
                    
                    # Start goto_height task (owner=platform)
                    self._start_task('goto_height', owner='platform')
                    self.get_logger().info(f"[SEQ {seq_id_str}] Auto mode: target height = {self.target_height:.2f} mm")
                else:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] goto_height requires target_height field")
            elif command == 'force_up':
                # Platform force control - check if system is busy
                if self.system_busy and self.active_control_owner != 'platform':
                    self.get_logger().warning(
                        f"[SEQ {seq_id_str}] force_up REJECTED - {self.active_control_owner} is busy (task={self.task_type})"
                    )
                    return
                
                tf = command_data.get('target_force')
                if tf is None:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] force_up requires target_force field")
                else:
                    try:
                        # Auto-initialize: disable height auto control to avoid interaction
                        if self.control_enabled:
                            self.control_enabled = False
                            self.get_logger().info(f"[SEQ {seq_id_str}] Disabled height control for force_up")
                        
                        self.target_force = float(tf)
                        self.force_control_direction = 'up'
                        self.force_control_active = True
                        self.control_mode = 'manual'
                        # Start force_up task (owner=platform)
                        self._start_task('force_up', owner='platform')
                        self.controller.force_up_start(seq_id=seq_id)
                        self.get_logger().info(f"[SEQ {seq_id_str}] Force-UP start target_force={self.target_force:.2f} N")
                    except Exception:
                        self.get_logger().warning(f"[SEQ {seq_id_str}] Invalid target_force for force_up: {tf}")
            elif command == 'force_down':
                # Platform force control - check if system is busy
                if self.system_busy and self.active_control_owner != 'platform':
                    self.get_logger().warning(
                        f"[SEQ {seq_id_str}] force_down REJECTED - {self.active_control_owner} is busy (task={self.task_type})"
                    )
                    return
                
                tf = command_data.get('target_force')
                if tf is None:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] force_down requires target_force field")
                else:
                    try:
                        # Auto-initialize: disable height auto control to avoid interaction
                        if self.control_enabled:
                            self.control_enabled = False
                            self.get_logger().info(f"[SEQ {seq_id_str}] Disabled height control for force_down")
                        
                        self.target_force = float(tf)
                        self.force_control_direction = 'down'
                        self.force_control_active = True
                        self.control_mode = 'manual'
                        # Start force_down task (owner=platform)
                        self._start_task('force_down', owner='platform')
                        self.controller.force_down_start(seq_id=seq_id)
                        self.get_logger().info(f"[SEQ {seq_id_str}] Force-DOWN start target_force={self.target_force:.2f} N")
                    except Exception:
                        self.get_logger().warning(f"[SEQ {seq_id_str}] Invalid target_force for force_down: {tf}")
            
            elif command == 'reset':
                # ═══════════════════════════════════════════════════════
                # CRITICAL RESET: Thread-safe PLATFORM reset (6-step process)
                # NOTE: Web server sends reset to BOTH Platform and Pushrod
                #       to ensure complete system reset.
                # ═══════════════════════════════════════════════════════
                self.get_logger().warn(f"[SEQ {seq_id_str}] 🔴 PLATFORM RESET COMMAND - Starting safe shutdown sequence")
                
                # Step 1: Signal control loop to stop (set flag FIRST)
                with self.control_lock:
                    self.reset_in_progress = True
                    self.get_logger().info(f"[SEQ {seq_id_str}] Step 1: Reset flag set, control loop will exit on next cycle")
                
                # Step 2: Wait ONE control cycle for loop to detect flag and exit
                # The control loop checks reset_in_progress at PRIORITY 0 (first thing)
                # Worst case: loop just started → needs max 20ms to complete current cycle
                # After this wait, no new control commands will be sent (loop exits early)
                time.sleep(CONTROL_RATE)  # 20ms = 1 control cycle
                self.get_logger().info(f"[SEQ {seq_id_str}] Step 2: Waited 20ms for control loop to finish current cycle")
                
                # Step 3: Disable all control modes (now safe, control loop is blocked)
                with self.control_lock:
                    self.control_enabled = False
                    self.force_control_active = False
                    self.control_mode = 'manual'
                    self.movement_state = 'stop'
                    self.get_logger().info(f"[SEQ {seq_id_str}] Step 3: All control modes disabled")
                
                # Step 3.5: Cancel all active timers (CRITICAL: prevents future relay activations)
                try:
                    self.controller.cancel_all_timers()
                    self.get_logger().info(f"[SEQ {seq_id_str}] Step 3.5: All timers cancelled (prevents delayed relay closures)")
                except Exception as e:
                    self.get_logger().error(f"[SEQ {seq_id_str}] ❌ Timer cancellation failed: {e}")
                
                # Step 4: Reset all Platform relays to 0 FIRST (relays 0,1,2 all OFF)
                try:
                    self.controller.reset_all_relays(seq_id=seq_id)
                    self.get_logger().info(f"[SEQ {seq_id_str}] Step 4: Platform relays reset (0,1,2 cleared to 0)")
                except Exception as e:
                    self.get_logger().error(f"[SEQ {seq_id_str}] ❌ Relay reset failed: {e}")
                
                # Step 5: Send STOP pulse (relay 0 flash) to physically stop hardware motion
                # Note: reset_all_relays only sets relays to OFF, but doesn't trigger stop action
                # Hardware needs explicit stop pulse (relay 0 ON→OFF) to halt motion
                try:
                    self.controller.stop(seq_id=seq_id)
                    self.get_logger().info(f"[SEQ {seq_id_str}] Step 5: Platform STOP pulse sent (triggers hardware stop)")
                except Exception as e:
                    self.get_logger().error(f"[SEQ {seq_id_str}] ❌ Platform stop pulse failed: {e}")
                
                # Step 6: Mark any running task as manually stopped and release system lock
                with self.control_lock:
                    if self.task_state == 'running':
                        # Mark as emergency_stop (manual reset is still an emergency action)
                        self.task_state = 'emergency_stop'
                        self.completion_reason = 'manual_stop'
                        self.task_end_time = time.time()
                    # Release system busy lock
                    self.system_busy = False
                    self.active_control_owner = None
                    # Clear reset flag to allow control loop to resume (if needed)
                    self.reset_in_progress = False
                    self.get_logger().info(f"[SEQ {seq_id_str}] Step 6: Reset complete, system ready (system_busy=False)")
                
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
        
        Thread safety: Acquires control_lock for all operations
        """
        # ═══════════════════════════════════════════════════════════════
        # PRIORITY 0: Check reset flag (exit immediately if reset in progress)
        # ═══════════════════════════════════════════════════════════════
        with self.control_lock:
            if self.reset_in_progress:
                return  # Exit immediately, do not execute any control logic
        
        # ═══════════════════════════════════════════════════════════════
        # PRIORITY 1: Mutual Exclusion Check
        # Ensure only ONE control mode is active at a time
        # ═══════════════════════════════════════════════════════════════
        with self.control_lock:
            active_controls = sum([
                self.control_enabled,           # Platform height auto control
                self.force_control_active,      # Platform force control
                # Pushrod control is managed by separate node, no conflict
            ])
            
            if active_controls > 1:
                # CRITICAL: Multiple controls active simultaneously - emergency stop
                self.get_logger().error(
                    f"🚨 MUTUAL EXCLUSION VIOLATION: {active_controls} controls active! "
                    f"height_auto={self.control_enabled}, force={self.force_control_active}"
                )
                # Disable all and stop
                self.control_enabled = False
                self.force_control_active = False
                self.movement_state = 'stop'
                try:
                    self.controller.stop()
                except Exception as e:
                    self.get_logger().error(f"Emergency stop failed: {e}")
                return
        
        # ─────────────────────────────────────────────────────────────
        # Manual down task: check if reached bottom (height < 832mm)
        # Hardware auto-stops, so we just mark task as completed
        # ─────────────────────────────────────────────────────────────
        if self.task_state == 'running' and self.task_type == 'manual_down':
            if self.current_height < 832.0:
                # Reached bottom, mark as completed (don't send stop, hardware auto-stops)
                self._complete_task('target_reached')
                self.movement_state = 'stop'  # Update state to reflect hardware stop
                self.get_logger().info(
                    f"[ManualDown] Bottom reached: height={self.current_height:.2f}mm < 832mm (hardware auto-stop)"
                )
        
        # ─────────────────────────────────────────────────────────────
        # Force control branch (independent of height auto control)
        # Runs even if control_enabled is False
        # ─────────────────────────────────────────────────────────────
        if self.force_control_active and self.target_force is not None:
            # Update combined force
            if self.current_force_right is not None and self.current_force_left is not None:
                self.current_force_combined = self.current_force_right + self.current_force_left
            elif self.current_force_right is not None:
                self.current_force_combined = self.current_force_right
            elif self.current_force_left is not None:
                self.current_force_combined = self.current_force_left
            else:
                self.current_force_combined = None

            # If no force reading yet, skip
            if self.current_force_combined is None:
                return

            # ═══════════════════════════════════════════════════════════════
            # FORCE CONTROL SAFETY: Check for excessive overshoot
            # ═══════════════════════════════════════════════════════════════
            force_overshoot_threshold = 150.0  # N
            
            if self.force_control_direction == 'up':
                # Force up: check if exceeded target by more than threshold
                if self.current_force_combined > self.target_force + force_overshoot_threshold:
                    self.get_logger().error(
                        f"🚨 FORCE CONTROL EMERGENCY: Force overshoot detected! "
                        f"force={self.current_force_combined:.2f}N > target+threshold={self.target_force + force_overshoot_threshold:.2f}N "
                        f"(overshoot={self.current_force_combined - self.target_force:.2f}N)"
                    )
                    # Trigger emergency reset (6-step process: flag → wait → disable → reset relays → stop → clear)
                    self._trigger_emergency_reset('force_overshoot')
                    return
                    
            elif self.force_control_direction == 'down':
                # Force down: check if exceeded target by more than threshold (negative direction)
                if self.current_force_combined < self.target_force - force_overshoot_threshold:
                    self.get_logger().error(
                        f"🚨 FORCE CONTROL EMERGENCY: Force undershoot detected! "
                        f"force={self.current_force_combined:.2f}N < target-threshold={self.target_force - force_overshoot_threshold:.2f}N "
                        f"(undershoot={self.target_force - self.current_force_combined:.2f}N)"
                    )
                    # Trigger emergency reset (6-step process)
                    self._trigger_emergency_reset('force_undershoot')
                    return

            # Check if target force reached (different logic for up/down)
            force_reached = False
            if self.force_control_direction == 'up':
                # Force up: stop when force >= target
                force_reached = (self.current_force_combined >= self.target_force)
            elif self.force_control_direction == 'down':
                # Force down: stop when force <= target
                force_reached = (self.current_force_combined <= self.target_force)
            
            if force_reached:
                try:
                    self.controller.stop()
                except Exception as e:
                    self.get_logger().error(f"Force control stop error: {e}")
                
                # CRITICAL: Set all control flags to False before completing task
                # This ensures _get_task_state() will return 'completed' instead of 'running'
                self.force_control_active = False
                self.control_enabled = False  # Ensure height control is also disabled
                self.movement_state = 'stop'
                
                # Mark force task as completed
                self._complete_task('force_reached')
                self.get_logger().info(
                    f"[ForceControl] ✅ Target reached: force={self.current_force_combined:.2f}N, "
                    f"target={self.target_force:.2f}N, direction={self.force_control_direction}, "
                    f"task_state={self.task_state}"
                )
                return

            # Continue movement pulses only when direction differs
            try:
                if self.force_control_direction == 'up' and self.movement_state != 'up':
                    self.controller.up()
                    self.movement_state = 'up'
                elif self.force_control_direction == 'down' and self.movement_state != 'down':
                    self.controller.down()
                    self.movement_state = 'down'
            except Exception as e:
                self.get_logger().error(f"Force control pulse error: {e}")
            return

        # ─────────────────────────────────────────────────────────────
        # Height auto control branch
        # ─────────────────────────────────────────────────────────────
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
        
        # ═══════════════════════════════════════════════════════════════
        # HEIGHT CONTROL SAFETY: Check for excessive overshoot
        # ═══════════════════════════════════════════════════════════════
        height_overshoot_threshold = 10.0  # mm
        
        if self.control_enabled and self.control_mode == 'auto':
            if self.movement_state == 'up':
                # Moving up: check if exceeded target by more than threshold
                if self.current_height > self.target_height + height_overshoot_threshold:
                    self.get_logger().error(
                        f"🚨 HEIGHT CONTROL EMERGENCY: Height overshoot detected! "
                        f"height={self.current_height:.2f}mm > target+threshold={self.target_height + height_overshoot_threshold:.2f}mm "
                        f"(overshoot={self.current_height - self.target_height:.2f}mm)"
                    )
                    # Trigger emergency reset (6-step process)
                    self._trigger_emergency_reset('height_overshoot')
                    return
                    
            elif self.movement_state == 'down':
                # Moving down: check if exceeded target by more than threshold (negative direction)
                if self.current_height < self.target_height - height_overshoot_threshold:
                    self.get_logger().error(
                        f"🚨 HEIGHT CONTROL EMERGENCY: Height undershoot detected! "
                        f"height={self.current_height:.2f}mm < target-threshold={self.target_height - height_overshoot_threshold:.2f}mm "
                        f"(undershoot={self.target_height - self.current_height:.2f}mm)"
                    )
                    # Trigger emergency reset (6-step process)
                    self._trigger_emergency_reset('height_undershoot')
                    return
        
        # Priority 1: Check if target reached
        if abs_error <= POSITION_TOLERANCE:
            if self.control_enabled:
                # 达到目标误差带：立即终止自动控制，不再发送任何相反方向修正；仅继续超调测量
                self._issue_stop(direction=self.movement_state, reason="target_band", disable_control=True)
                # Mark goto_height task as completed
                self._complete_task('target_reached')
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
            if self.force_control_active:
                status['force_control_active'] = True
                status['target_force'] = self.target_force
                status['force_direction'] = self.force_control_direction
            if self.current_force_combined is not None:
                status['force_combined'] = self.current_force_combined
            if self.current_force_right is not None:
                status['force_right'] = self.current_force_right
            if self.current_force_left is not None:
                status['force_left'] = self.current_force_left
            
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
        """Measure overshoot after settle delay and update EMA (only if learning enabled)."""
        try:
            # Skip overshoot measurement if learning is disabled
            if not OVERSHOOT_LEARNING_ENABLED:
                # Just clear temporary tracking state
                self.height_at_stop = None
                self.last_stop_direction = None
                return
            
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

    # ─────────────────────────────────────────────────────────────
    # Force sensor callbacks
    # ─────────────────────────────────────────────────────────────
    def force_cb_right(self, msg):
        try:
            self.current_force_right = msg.data
        except Exception as e:
            self.get_logger().warn(f"Force right parse error: {e}")

    def force_cb_left(self, msg):
        try:
            self.current_force_left = msg.data
        except Exception as e:
            self.get_logger().warn(f"Force left parse error: {e}")
    
    # ═══════════════════════════════════════════════════════════════
    # Task State Management Methods
    # ═══════════════════════════════════════════════════════════════
    def _start_task(self, task_type, owner='platform'):
        """
        Start a new task and acquire system lock
        
        Args:
            task_type: task type identifier
            owner: 'platform' or 'pushrod' - who owns this task
        """
        self.task_state = 'running'
        self.task_type = task_type
        self.task_start_time = time.time()
        self.task_end_time = None
        self.completion_reason = None
        self.system_busy = True
        self.active_control_owner = owner
        self.get_logger().debug(f"[Task] Started: {task_type} (owner={owner}, system_busy=True)")
    
    def _complete_task(self, reason):
        """
        Mark task as completed with reason and timestamp
        Release system lock
        """
        if self.task_state == 'running' or self.task_state == 'emergency_stop':
            self.task_state = 'completed'
            self.completion_reason = reason
            self.task_end_time = time.time()
            duration = self.task_end_time - self.task_start_time if self.task_start_time else 0
            
            # Release system lock
            self.system_busy = False
            self.active_control_owner = None
            
            self.get_logger().info(
                f"[Task Complete] type={self.task_type} reason={reason} duration={duration:.2f}s (system_busy=False)"
            )
    
    def _on_auto_stop_complete(self):
        """
        Callback when timed operation auto-stops (from controller's timer)
        Marks task as completed
        """
        # Mark timed task as completed (target_reached)
        if self.task_state == 'running' and self.task_type in ['timed_up', 'timed_down']:
            self._complete_task('target_reached')
            self.get_logger().info(f"Timed operation completed: {self.task_type}")
    
    def _trigger_emergency_reset(self, emergency_reason):
        """
        Internal emergency reset trigger (called from control loop when safety limit exceeded)
        
        This executes the full 6-step reset process to ensure complete system shutdown.
        
        Args:
            emergency_reason: The reason for emergency reset (e.g., 'force_overshoot', 'height_undershoot')
        """
        self.get_logger().error(f"🚨 EMERGENCY RESET TRIGGERED: {emergency_reason}")
        
        # Step 1: Set reset flag
        with self.control_lock:
            self.reset_in_progress = True
            self.get_logger().warn(f"[EMERGENCY] Step 1: Reset flag set")
        
        # Step 2: Wait for control loop to detect and exit (max 20ms detection)
        time.sleep(CONTROL_RATE * 2)  # 40ms = 2 cycles for safety
        self.get_logger().warn(f"[EMERGENCY] Step 2: Waited for control loop exit")
        
        # Step 3: Disable all control modes
        with self.control_lock:
            self.control_enabled = False
            self.force_control_active = False
            self.control_mode = 'manual'
            self.movement_state = 'stop'
            self.get_logger().warn(f"[EMERGENCY] Step 3: All control modes disabled")
        
        # Step 4: Reset all relays to 0 FIRST (before sending stop pulse)
        try:
            self.controller.reset_all_relays()
            self.get_logger().warn(f"[EMERGENCY] Step 4: Platform relays reset (0,1,2 cleared)")
        except Exception as e:
            self.get_logger().error(f"[EMERGENCY] ❌ Relay reset failed: {e}")
        
        # Step 5: Send STOP pulse
        try:
            self.controller.stop()
            self.get_logger().warn(f"[EMERGENCY] Step 5: Platform STOP pulse sent")
        except Exception as e:
            self.get_logger().error(f"[EMERGENCY] ❌ Stop pulse failed: {e}")
        
        # Step 6: Mark task as emergency stop and clear reset flag
        with self.control_lock:
            self.task_state = 'emergency_stop'
            self.completion_reason = emergency_reason
            self.task_end_time = time.time()
            self.system_busy = False
            self.active_control_owner = None
            self.reset_in_progress = False
            self.get_logger().error(
                f"[EMERGENCY] Step 6: Emergency stop complete - reason={emergency_reason} (system_busy=False)"
            )
    
    def _get_task_state(self):
        """Get current task state based on task_state variable"""
        # Return the actual task state
        # Completed state persists for 5 seconds after task end, then returns to idle
        if self.task_state == 'completed' and self.task_end_time is not None:
            if (time.time() - self.task_end_time) >= 5.0:
                return 'idle'
        return self.task_state


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
