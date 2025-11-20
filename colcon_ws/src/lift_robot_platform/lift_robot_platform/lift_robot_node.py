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
import os

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
# Overshoot Calibration Configuration
# ═══════════════════════════════════════════════════════════════
# Predictive early stop parameters (to reduce overshoot)
# These are default initial values used when no calibration config file exists
# Use web calibration interface to collect samples and update these values
OVERSHOOT_INIT_UP = 2.7999999999999545      # Initial upward overshoot estimate (mm)
OVERSHOOT_INIT_DOWN = 3.0139999999999985    # Initial downward overshoot estimate (mm)
OVERSHOOT_SETTLE_DELAY = 0.5   # (s) Wait time after stop to measure stable position (increased from 0.25s)
OVERSHOOT_MIN_MARGIN = 0.3     # (mm) Minimum margin - don't use early stop if below this


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
        self.movement_command_sent = False  # Flag to prevent duplicate commands during flash verification
        
        # ═══════════════════════════════════════════════════════════════
        # Load Overshoot Calibration from Config File (portable path resolution)
        # Priority: ENV LIFT_ROBOT_CONFIG_DIR -> ancestor colcon_ws -> CWD/config
        # NOTE: Auto-creates config directory and default files if missing (needed for calibration)
        # ═══════════════════════════════════════════════════════════════
        env_dir = os.environ.get('LIFT_ROBOT_CONFIG_DIR')
        if env_dir:
            self.config_dir = os.path.abspath(env_dir)
        else:
            try:
                # Walk up from this file to find colcon_ws
                here = os.path.abspath(os.path.dirname(__file__))
                parts = here.split(os.sep)
                if 'colcon_ws' in parts:
                    idx = parts.index('colcon_ws')
                    colcon_ws_path = os.sep.join(parts[:idx+1])
                    self.config_dir = os.path.join(colcon_ws_path, 'config')
                else:
                    self.config_dir = os.path.join(os.getcwd(), 'config')
            except Exception:
                self.config_dir = os.path.join(os.getcwd(), 'config')
        
        # Auto-create config directory for overshoot calibration (writable config)
        try:
            os.makedirs(self.config_dir, exist_ok=True)
        except Exception as e:
            self.get_logger().warn(f"Cannot create config dir '{self.config_dir}': {e}")
        
        # Helper function to get config file path
        def _cfg(name):
            return os.path.join(self.config_dir, name)
        config_path = _cfg('platform_overshoot_calibration.json')
        self.get_logger().info(f"Overshoot config path resolved: {config_path}")
        overshoot_up_loaded = OVERSHOOT_INIT_UP
        overshoot_down_loaded = OVERSHOOT_INIT_DOWN
        
        # Region-based calibration list (each: {lower, upper, overshoot_up, overshoot_down})
        self.overshoot_regions = []
        # Polynomial fit (optional): dict with type='poly', degree, coeffs_up, coeffs_down, x_min, x_max
        self.overshoot_fit = None
        
        # Create default config file if it doesn't exist
        if not os.path.exists(config_path):
            try:
                config_dir = os.path.dirname(config_path)
                if not os.path.exists(config_dir):
                    os.makedirs(config_dir)
                
                # Default polynomial fit (degree 2, based on empirical data)
                # These coefficients provide reasonable estimates for typical platforms
                # NOTE: All overshoot values MUST be ABSOLUTE VALUES (positive)
                # UP: stop at (target - overshoot_up), DOWN: stop at (target + overshoot_down)
                default_config = {
                    'enable': True,
                    'format_version': 2,
                    'regions': [],
                    'samples': [],
                    'fit': {
                        'type': 'poly',
                        'degree': 2,
                        'coeffs_up': [
                            2.2091788116272794e-06,    # x^2 coefficient
                            -0.008710729042700554,      # x coefficient
                            8.728348317269088           # constant (outputs positive overshoot)
                        ],
                        'coeffs_down': [
                            3.4745013091926766e-06,     # x^2 coefficient (NEGATED to output positive)
                            -0.012317707321611337,      # x coefficient (NEGATED to output positive)
                            11.240744855681426          # constant (NEGATED to output positive)
                        ],
                        'x_min': 800.0,   # Typical min height
                        'x_max': 1400.0,  # Typical max height
                        'generated_at': time.time(),
                        'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S')
                    }
                }
                
                with open(config_path, 'w') as f:
                    json.dump(default_config, f, indent=2)
                
                self.get_logger().info(
                    f"[lift_robot_platform] Created default overshoot config with polynomial fit at {config_path}"
                )
            except Exception as e:
                self.get_logger().warn(
                    f"[lift_robot_platform] Failed to create default config: {e}"
                )
        
        # Load config file
        if os.path.exists(config_path):
            try:
                with open(config_path, 'r') as f:
                    config_data = json.load(f)
                    # New format (v2) stores default + regions
                    if 'default' in config_data:
                        overshoot_up_loaded = config_data['default'].get('overshoot_up', OVERSHOOT_INIT_UP)
                        overshoot_down_loaded = config_data['default'].get('overshoot_down', OVERSHOOT_INIT_DOWN)
                    else:
                        # Backward compatibility (legacy single values)
                        overshoot_up_loaded = config_data.get('overshoot_up', OVERSHOOT_INIT_UP)
                        overshoot_down_loaded = config_data.get('overshoot_down', OVERSHOOT_INIT_DOWN)
                    if 'regions' in config_data and isinstance(config_data['regions'], list):
                        # Validate region entries
                        for r in config_data['regions']:
                            try:
                                lb = float(r.get('lower'))
                                ub = float(r.get('upper'))
                                upv = float(r.get('overshoot_up'))
                                dnv = float(r.get('overshoot_down'))
                                if lb < ub:
                                    self.overshoot_regions.append({
                                        'lower': lb,
                                        'upper': ub,
                                        'overshoot_up': upv,
                                        'overshoot_down': dnv
                                    })
                            except Exception:
                                self.get_logger().warn(f"[lift_robot_platform] Skipping invalid region entry: {r}")
                    self.get_logger().info(
                        f"[lift_robot_platform] Loaded overshoot calibration: default(up={overshoot_up_loaded}, down={overshoot_down_loaded}), regions={len(self.overshoot_regions)}"
                    )
                    # Load fit block if present
                    if 'fit' in config_data and isinstance(config_data['fit'], dict):
                        fb = config_data['fit']
                        if fb.get('type') == 'poly' and 'coeffs_up' in fb and 'coeffs_down' in fb:
                            # Basic validation
                            try:
                                self.overshoot_fit = {
                                    'degree': int(fb.get('degree', len(fb['coeffs_up'])-1)),
                                    'coeffs_up': [float(c) for c in fb['coeffs_up']],
                                    'coeffs_down': [float(c) for c in fb['coeffs_down']],
                                    'x_min': float(fb.get('x_min', 0.0)),
                                    'x_max': float(fb.get('x_max', 0.0))
                                }
                                self.get_logger().info(f"[lift_robot_platform] Loaded overshoot fit: degree={self.overshoot_fit['degree']} (poly)")
                            except Exception as e:
                                self.get_logger().warn(f"[lift_robot_platform] Invalid overshoot fit block: {e}")
            except Exception as e:
                self.get_logger().warn(
                    f"[lift_robot_platform] Warning: Failed to load overshoot config: {e}"
                )
                self.get_logger().warn(
                    f"[lift_robot_platform] Using default values: "
                    f"up={OVERSHOOT_INIT_UP}, down={OVERSHOOT_INIT_DOWN}"
                )
        else:
            self.get_logger().warn(
                f"[lift_robot_platform] No overshoot config found at {config_path}"
            )
            self.get_logger().warn(
                f"[lift_robot_platform] Using default values: "
                f"up={OVERSHOOT_INIT_UP}, down={OVERSHOOT_INIT_DOWN}"
            )
        
        # Overshoot parameters (loaded from config, used for predictive early stop)
        self.avg_overshoot_up = overshoot_up_loaded
        self.avg_overshoot_down = overshoot_down_loaded

        # Sort regions by lower bound for deterministic search
        if self.overshoot_regions:
            self.overshoot_regions.sort(key=lambda r: r['lower'])
        
        # Web calibration tracking state
        self.height_at_stop = None           # Height when stop command issued
        self.last_stop_direction = None      # 'up' or 'down'
        self.last_stop_time = None
        self.overshoot_timer = None
        self.last_goto_target = None         # Target height for last goto_height command
        self.last_goto_actual = None         # Actual stable height after settle delay
        self.last_goto_stop_height = None    # Height when stop command was issued (for residual overshoot calculation)
        self.last_goto_direction = None      # 'up' or 'down'
        self.last_goto_timestamp = None      # When the measurement completed
        
        # Pending task completion (triggered by stop relay OFF verification)
        self.pending_task_completion = None  # {'reason': str} or None
        
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
        
        # ═══════════════════════════════════════════════════════════════
        # Motion Stall Detection (Mechanical Failure Monitoring)
        # Detects extreme mechanical failures (stuck gears, seized motors, etc.)
        # Triggers emergency reset instead of retry (mechanical issues can't be fixed by resend)
        # ═══════════════════════════════════════════════════════════════
        self.stall_check_start_time = None  # Time when movement command was sent
        self.stall_check_start_position = None  # Position when movement command was sent
        self.stall_check_duration = 2.0     # seconds - extended to avoid false positives during slow/uneven movement
        self.position_change_tolerance = 0.01  # mm - positions within this range considered identical

        # ═══════════════════════════════════════════════════════════════
        # Platform Range Limits (loaded from platform_range.json)
        # ═══════════════════════════════════════════════════════════════
        self.platform_range_min = None  # Actual minimum height from config
        self.platform_range_max = None  # Actual maximum height from config
        self.platform_range_enabled = False  # Whether range limits are active
        
        # Load platform range from config file (using same portable path resolution)
        range_config_path = _cfg('platform_range.json')
        if os.path.exists(range_config_path):
            try:
                with open(range_config_path, 'r') as f:
                    range_data = json.load(f)
                    actual_min = range_data.get('actual_min')
                    actual_max = range_data.get('actual_max')
                    
                    # Only enable if both values are valid (not None)
                    if actual_min is not None and actual_max is not None:
                        self.platform_range_min = float(actual_min)
                        self.platform_range_max = float(actual_max)
                        self.platform_range_enabled = True
                        self.get_logger().info(
                            f"[lift_robot_platform] Platform range limits loaded: "
                            f"min={self.platform_range_min:.2f}mm, max={self.platform_range_max:.2f}mm"
                        )
                    else:
                        self.get_logger().info(
                            f"[lift_robot_platform] Platform range config exists but values are null - "
                            f"range limits DISABLED (run range detection to enable)"
                        )
            except Exception as e:
                self.get_logger().warn(
                    f"[lift_robot_platform] Failed to load platform range config: {e} - "
                    f"range limits DISABLED"
                )
        else:
            self.get_logger().info(
                f"[lift_robot_platform] No platform range config found at {range_config_path} - "
                f"range limits DISABLED (will be created automatically)"
            )
        
        # ═══════════════════════════════════════════════════════════════
        # Range Scan (Stall-Based Endpoint Detection)
        # Only used during explicit range detection commands. Old generic
        # stall recovery removed. Logic: while scanning in a direction, if
        # movement_state stays 'down' or 'up' and height does not change beyond
        # tolerance for 5s, declare endpoint.
        # Commands to activate: 'range_scan_down', 'range_scan_up', 'range_scan_cancel'
        # Timer runs at 10Hz.
        # ═══════════════════════════════════════════════════════════════
        self.range_scan_active = False
        self.range_scan_direction = None      # 'down' | 'up'
        self.range_scan_low_reached = False
        self.range_scan_high_reached = False
        self.range_scan_reference_height = None
        self.range_scan_stall_start_time = None
        self.range_scan_duration_required = 5.0  # seconds of no movement
        self.range_scan_height_tolerance = 0.05  # mm change threshold to reset stall timer
        self.range_scan_timer = self.create_timer(0.1, self._range_scan_check)  # 10Hz
        self.range_scan_low_height = None   # Measured low endpoint height
        self.range_scan_high_height = None  # Measured high endpoint height
        
        # Create controller
        self.controller = LiftRobotController(
            device_id=self.device_id,
            node=self,
            use_ack_patch=self.use_ack_patch
        )
        
        # Set callbacks
        self.controller.on_auto_stop_callback = self._on_auto_stop_complete
        self.controller.on_flash_complete_callback = self._on_flash_complete
        
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
        
        # Status publish timer (10Hz for responsive web coordination)
        # NOTE: Higher frequency (10Hz vs 1Hz) is needed for web-node coordinated control
        # where web waits for task_state/movement_state changes between calibration steps.
        # Internal node control (goto_height) is unaffected as it uses 50Hz control loop.
        self.status_timer = self.create_timer(0.1, self.publish_status)
        
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
        
        # ═══════════════════════════════════════════════════════════════
        # Maximum Force Limit Protection (Highest Priority Safety Check)
        # ═══════════════════════════════════════════════════════════════
        # Applied to ALL control modes when force sensor is available
        # Triggers emergency reset if exceeded (500N default)
        self.max_force_limit = 500.0       # Maximum allowed force (N) - emergency threshold
        
        # ═══════════════════════════════════════════════════════════════
        # Height-Force Hybrid Control State (NEW - PLATFORM ONLY)
        # ═══════════════════════════════════════════════════════════════
        # Combines height and force control - stops when EITHER condition is met
        self.hybrid_control_active = False  # Enable/disable HYBRID control
        self.hybrid_target_height = None    # Target height (mm)
        self.hybrid_target_force = None     # Target force (N)
        self.hybrid_direction = None        # 'up' | 'down' - determined at command start
        self.hybrid_last_stop_direction = None  # 'up' | 'down' - for hybrid overshoot detection (independent from goto_height)

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
                
                # Step 3: Send hardware STOP pulse and schedule task completion after relay verification
                # Only complete task if there was a running task to stop
                complete_on_stop = 'manual_stop' if self.task_state == 'running' else None
                self._issue_stop(
                    direction=self.movement_state if self.movement_state in ['up', 'down'] else 'up',
                    reason='manual_stop_command',
                    disable_control=False,  # Already disabled above
                    complete_task_on_stop=complete_on_stop
                )
                
            elif command == 'up':
                # If interrupting a running closed-loop task, complete it first
                if self.task_state == 'running' and self.task_type in ['goto_height', 'force_up', 'force_down', 'height_force_hybrid']:
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
                # 若当前仍有异步闪络脉冲在执行，延迟排队执行，避免被忽略
                if getattr(self.controller, 'flash_active', False):
                    relay_busy = None
                    try:
                        ctx = getattr(self.controller, 'flash_context', None)
                        if ctx:
                            relay_busy = ctx.get('relay')
                    except Exception:
                        pass
                    self.get_logger().info(f"[SEQ {seq_id_str}] Flash busy (relay={relay_busy}), schedule UP after 0.18s")
                    threading.Timer(0.18, lambda: self._queued_direction_pulse('up', seq_id)).start()
                else:
                    # Start flash - movement_state and task will be set in callback after verification
                    self.controller.up(seq_id=seq_id)
                    self.get_logger().debug(f"[SEQ {seq_id_str}] UP flash started, waiting for verification...")
                
            elif command == 'down':
                # If interrupting a running closed-loop task, complete it first
                if self.task_state == 'running' and self.task_type in ['goto_height', 'force_up', 'force_down', 'height_force_hybrid']:
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
                if getattr(self.controller, 'flash_active', False):
                    relay_busy = None
                    try:
                        ctx = getattr(self.controller, 'flash_context', None)
                        if ctx:
                            relay_busy = ctx.get('relay')
                    except Exception:
                        pass
                    self.get_logger().info(f"[SEQ {seq_id_str}] Flash busy (relay={relay_busy}), schedule DOWN after 0.18s")
                    threading.Timer(0.18, lambda: self._queued_direction_pulse('down', seq_id)).start()
                else:
                    # Start flash - movement_state and task will be set in callback after verification
                    self.controller.down(seq_id=seq_id)
                    self.get_logger().debug(f"[SEQ {seq_id_str}] DOWN flash started, waiting for verification...")
                
            elif command == 'timed_up':
                # If interrupting a running closed-loop task, complete it first
                if self.task_state == 'running' and self.task_type in ['goto_height', 'force_up', 'force_down', 'height_force_hybrid']:
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
                if self.task_state == 'running' and self.task_type in ['goto_height', 'force_up', 'force_down', 'height_force_hybrid']:
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
                    target_height = float(target)
                    current_error = abs(target_height - self.current_height)
                    
                    # Check if already at target position
                    if current_error <= POSITION_TOLERANCE:
                        # Already at target - complete immediately without starting control
                        self.get_logger().info(
                            f"[SEQ {seq_id_str}] Already at target height={target_height:.2f}mm "
                            f"(current={self.current_height:.2f}mm, error={current_error:.3f}mm) - completing immediately"
                        )
                        # Auto-initialize: disable force control if active
                        if self.force_control_active:
                            self.force_control_active = False
                        # Start and immediately complete the task
                        self._start_task('goto_height', owner='platform')
                        self._complete_task('target_reached')
                    else:
                        # Need to move - start control loop
                        # Auto-initialize: disable force control if active
                        if self.force_control_active:
                            self.force_control_active = False
                            self.get_logger().info(f"[SEQ {seq_id_str}] Disabled force control for goto_height")
                        
                        self.target_height = target_height
                        self.control_mode = 'auto'
                        self.control_enabled = True

                        # Region-specific overshoot selection
                        try:
                            up_o, down_o = self._get_fitted_or_region_overshoot(self.target_height)
                            self.avg_overshoot_up = up_o
                            self.avg_overshoot_down = down_o
                        except Exception as e:
                            self.get_logger().warn(f"[SEQ {seq_id_str}] Overshoot selection failed: {e}")
                        
                        # Store target for web calibration
                        self.last_goto_target = target_height
                        self.last_goto_actual = None  # Will be measured after settle delay
                        self.last_goto_direction = None  # Will be determined by movement
                        self.last_goto_timestamp = None
                        
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
                        
                        # Start goto_height task (owner=platform)
                        self._start_task('goto_height', owner='platform')
                        self.get_logger().info(
                            f"[SEQ {seq_id_str}] Auto mode: target height={self.target_height:.2f}mm "
                            f"(current={self.current_height:.2f}mm, error={current_error:.2f}mm, "
                            f"overshoot_up={self.avg_overshoot_up:.3f}mm, overshoot_down={self.avg_overshoot_down:.3f}mm, regions={len(self.overshoot_regions)})"
                        )
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
            
            elif command == 'height_force_hybrid':
                # NEW: Height-Force Hybrid Control - stops when EITHER height OR force target is reached
                if self.system_busy and self.active_control_owner != 'platform':
                    self.get_logger().warning(
                        f"[SEQ {seq_id_str}] height_force_hybrid REJECTED - {self.active_control_owner} is busy (task={self.task_type})"
                    )
                    return
                
                target_height = command_data.get('target_height')
                target_force = command_data.get('target_force')
                
                if target_height is None or target_force is None:
                    self.get_logger().warning(
                        f"[SEQ {seq_id_str}] height_force_hybrid requires both target_height and target_force fields"
                    )
                    return
                
                try:
                    target_height = float(target_height)
                    target_force = float(target_force)
                    current_error = abs(target_height - self.current_height)
                    
                    # Force sensor is updated in real-time by callbacks
                    # No need to initialize here, current_force_combined is always up-to-date
                    
                    # Check if already at target position (similar to goto_height)
                    if current_error <= POSITION_TOLERANCE:
                        self.get_logger().info(
                            f"[SEQ {seq_id_str}] Already at target height={target_height:.2f}mm "
                            f"(current={self.current_height:.2f}mm, error={current_error:.3f}mm) - completing immediately"
                        )
                        self._start_task('height_force_hybrid', owner='platform')
                        self._complete_task('target_reached')
                    else:
                        # Disable any existing control modes
                        if self.control_enabled:
                            self.control_enabled = False
                            self.get_logger().info(f"[SEQ {seq_id_str}] Disabled height control for hybrid mode")
                        if self.force_control_active:
                            self.force_control_active = False
                            self.get_logger().info(f"[SEQ {seq_id_str}] Disabled force control for hybrid mode")
                        
                        # CRITICAL: Determine movement direction based on height error at command start
                        height_error = target_height - self.current_height
                        if height_error > 0:
                            self.hybrid_direction = 'up'
                        else:
                            self.hybrid_direction = 'down'
                        
                        # Initialize hybrid control state
                        self.hybrid_target_height = target_height
                        self.hybrid_target_force = target_force
                        self.hybrid_control_active = True
                        self.hybrid_last_stop_direction = None  # CRITICAL: Clear previous hybrid stop direction
                        self.control_mode = 'manual'  # Use manual mode for control loop
                        self.movement_state = 'stop'
                        
                        # Start hybrid task
                        self._start_task('height_force_hybrid', owner='platform')
                        self.get_logger().info(
                            f"[SEQ {seq_id_str}] Hybrid control start: target_height={target_height:.2f}mm, "
                            f"target_force={target_force:.2f}N, direction={self.hybrid_direction}, current_height={self.current_height:.2f}mm, "
                            f"current_force={self.current_force_combined:.2f}N"
                        )
                except Exception as e:
                    self.get_logger().warning(
                        f"[SEQ {seq_id_str}] Invalid parameters for height_force_hybrid: {e}"
                    )
            
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
                
            elif command == 'range_scan_down':
                # Start range scan moving downward to find LOW endpoint
                if self.range_scan_active:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] range_scan_down rejected - scan already active")
                    return
                if self.system_busy and self.active_control_owner not in [None, 'platform']:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] range_scan_down rejected - system busy by {self.active_control_owner}")
                    return
                # Disable other control modes
                if self.control_enabled:
                    self.control_enabled = False
                if self.force_control_active:
                    self.force_control_active = False
                # Initialize scan state
                self.range_scan_active = True
                self.range_scan_direction = 'down'
                self.range_scan_low_reached = False
                self.range_scan_high_reached = False
                self.range_scan_reference_height = None
                self.range_scan_stall_start_time = None
                self.range_scan_low_height = None
                self.range_scan_high_height = None
                self._start_task('range_scan', owner='platform')
                # Send initial downward pulse
                self.controller.down(seq_id=seq_id)
                # movement_state will be set in _on_flash_complete callback
                self.get_logger().info(f"[SEQ {seq_id_str}] ▶️ Range scan DOWN started (tolerance={self.range_scan_height_tolerance}mm, duration={self.range_scan_duration_required}s)")
            elif command == 'range_scan_up':
                # Start range scan moving upward to find HIGH endpoint
                if self.range_scan_active:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] range_scan_up rejected - scan already active")
                    return
                if self.system_busy and self.active_control_owner not in [None, 'platform']:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] range_scan_up rejected - system busy by {self.active_control_owner}")
                    return
                if self.control_enabled:
                    self.control_enabled = False
                if self.force_control_active:
                    self.force_control_active = False
                self.range_scan_active = True
                self.range_scan_direction = 'up'
                self.range_scan_low_reached = False
                self.range_scan_high_reached = False
                self.range_scan_reference_height = None
                self.range_scan_stall_start_time = None
                self.range_scan_low_height = None
                self.range_scan_high_height = None
                self._start_task('range_scan', owner='platform')
                self.controller.up(seq_id=seq_id)
                # movement_state will be set in _on_flash_complete callback
                self.get_logger().info(f"[SEQ {seq_id_str}] ▶️ Range scan UP started (tolerance={self.range_scan_height_tolerance}mm, duration={self.range_scan_duration_required}s)")
            elif command == 'range_scan_cancel':
                # Cancel any active range scan
                if not self.range_scan_active:
                    self.get_logger().info(f"[SEQ {seq_id_str}] range_scan_cancel - no active scan")
                else:
                    self.range_scan_active = False
                    self.range_scan_direction = None
                    self.range_scan_reference_height = None
                    self.range_scan_stall_start_time = None
                    self.get_logger().info(f"[SEQ {seq_id_str}] ⏹️ Range scan cancelled")
                # Stop motion safely
                try:
                    self.controller.stop(seq_id=seq_id)
                except Exception as e:
                    self.get_logger().error(f"[SEQ {seq_id_str}] Range scan cancel stop error: {e}")
                self.movement_state = 'stop'
                if self.task_state == 'running' and self.task_type == 'range_scan':
                    self._complete_task('manual_stop')
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
        # Also skip if already in emergency_stop state to prevent repeated triggers
        # ═══════════════════════════════════════════════════════════════
        with self.control_lock:
            if self.reset_in_progress or self.task_state == 'emergency_stop':
                return  # Exit control loop during reset/emergency
        
        # ═══════════════════════════════════════════════════════════════
        # PRIORITY 0.1: Maximum Force Limit Protection (ALL CONTROLS)
        # ═══════════════════════════════════════════════════════════════
        # HIGHEST PRIORITY: Check max force limit before ANY control logic
        # Only active when force sensor is available (current_force_combined not None)
        # Triggers emergency reset to prevent mechanical damage
        if self.current_force_combined is not None:
            if self.current_force_combined > self.max_force_limit:
                self.get_logger().error(
                    f"🚨 MAXIMUM FORCE LIMIT EXCEEDED! "
                    f"force={self.current_force_combined:.2f}N > max_limit={self.max_force_limit:.2f}N - "
                    f"triggering emergency reset"
                )
                self._trigger_emergency_reset('max_force_exceeded')
                return
        
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
        # Manual down task: check if reached bottom (dynamic range limit)
        # Only active if platform_range is enabled and configured
        # ─────────────────────────────────────────────────────────────
        if self.task_state == 'running' and self.task_type == 'manual_down':
            # Debug: Log check conditions every cycle when moving down
            if self.movement_state == 'down':
                self.get_logger().debug(
                    f"[ManualDown] Check: enabled={self.platform_range_enabled}, "
                    f"min={self.platform_range_min}, current={self.current_height:.2f}mm"
                )
            
            if self.platform_range_enabled and self.platform_range_min is not None:
                # Dynamic bottom limit: actual_min + 1mm safety margin
                bottom_limit = self.platform_range_min + 1.0
                if self.current_height < bottom_limit:
                    # Reached bottom - send stop and schedule task completion after relay verification
                    self.get_logger().info(
                        f"[ManualDown] Bottom limit reached: height={self.current_height:.2f}mm < "
                        f"limit={bottom_limit:.2f}mm (actual_min+1mm) - sending stop"
                    )
                    self._issue_stop(
                        direction='down',
                        reason='manual_down_bottom_limit',
                        disable_control=False,
                        complete_task_on_stop='target_reached'
                    )
        
        # ─────────────────────────────────────────────────────────────
        # Manual up task: check if reached top (dynamic range limit)
        # Only active if platform_range is enabled and configured
        # ─────────────────────────────────────────────────────────────
        if self.task_state == 'running' and self.task_type == 'manual_up':
            if self.platform_range_enabled and self.platform_range_max is not None:
                # Dynamic top limit: actual_max - 1mm safety margin
                top_limit = self.platform_range_max - 1.0
                if self.current_height > top_limit:
                    # Reached top - send stop and schedule task completion after relay verification
                    self.get_logger().info(
                        f"[ManualUp] Top limit reached: height={self.current_height:.2f}mm > "
                        f"limit={top_limit:.2f}mm (actual_max-1mm) - sending stop"
                    )
                    self._issue_stop(
                        direction='up',
                        reason='manual_up_top_limit',
                        disable_control=False,
                        complete_task_on_stop='target_reached'
                    )
        
        # ─────────────────────────────────────────────────────────────
        # Force control branch (independent of height auto control)
        # Runs even if control_enabled is False
        # ─────────────────────────────────────────────────────────────
        if self.force_control_active and self.target_force is not None:
            # Check if force sensor is available (updated in real-time by force callbacks)
            if self.current_force_combined is None:
                return  # Skip if no force reading yet

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
                # CRITICAL: Set all control flags to False before stopping
                # This prevents control loop from sending new commands
                self.force_control_active = False
                self.control_enabled = False  # Ensure height control is also disabled
                
                # Send stop and schedule task completion after relay verification
                self._issue_stop(
                    direction=self.force_control_direction,
                    reason=f'force_control_{self.force_control_direction}_reached',
                    disable_control=False,  # Already disabled above
                    complete_task_on_stop='force_reached'
                )
                self.get_logger().info(
                    f"[ForceControl] ✅ Target reached: force={self.current_force_combined:.2f}N, "
                    f"target={self.target_force:.2f}N, direction={self.force_control_direction}"
                )
                return

            # Continue movement pulses only when direction differs
            # movement_state will be set by flash_complete callback
            try:
                if self.force_control_direction == 'up' and self.movement_state != 'up' and not self.movement_command_sent:
                    self.controller.up()
                    self.movement_command_sent = True  # Prevent duplicate sends during flash verification
                    # movement_state will be updated in _on_flash_complete callback
                elif self.force_control_direction == 'down' and self.movement_state != 'down' and not self.movement_command_sent:
                    self.controller.down()
                    self.movement_command_sent = True  # Prevent duplicate sends during flash verification
                    # movement_state will be updated in _on_flash_complete callback
            except Exception as e:
                self.get_logger().error(f"Force control pulse error: {e}")
            return

        # ─────────────────────────────────────────────────────────────
        # Height-Force Hybrid control branch (NEW)
        # Stops when EITHER height OR force target is reached
        # Architecture matches goto_height + force_control for safety consistency
        # ─────────────────────────────────────────────────────────────
        if self.hybrid_control_active and self.hybrid_target_height is not None and self.hybrid_target_force is not None:
            # current_force_combined is updated in real-time by force sensor callbacks
            # Calculate height error
            height_error = self.hybrid_target_height - self.current_height
            abs_height_error = abs(height_error)
            
            # 时间戳用于周期性日志
            now = self.get_clock().now()
            
            # ═══════════════════════════════════════════════════════════════
            # HYBRID CONTROL SAFETY: Check for excessive overshoot (height dimension)
            # Same logic as goto_height to ensure consistent safety behavior
            # Only check when task is actually running (not at command initialization)
            # ═══════════════════════════════════════════════════════════════
            height_overshoot_threshold = 10.0  # mm
            
            # CRITICAL: Only check overshoot when task is running (prevents false triggers at init)
            if self.task_type == 'height_force_hybrid' and self.task_state == 'running':
                # Determine expected movement direction based on current error
                height_error_for_direction = self.hybrid_target_height - self.current_height
                
                # Only check overshoot in the direction we're moving/moved
                if height_error_for_direction > 0 or self.movement_state == 'up' or self.hybrid_last_stop_direction == 'up':
                    # Moving/moved upward: only check upward overshoot
                    if self.current_height > self.hybrid_target_height + height_overshoot_threshold:
                        self.get_logger().error(
                            f"🚨 HYBRID CONTROL EMERGENCY: Height overshoot detected! "
                            f"height={self.current_height:.2f}mm > target+threshold={self.hybrid_target_height + height_overshoot_threshold:.2f}mm "
                            f"(overshoot={self.current_height - self.hybrid_target_height:.2f}mm)"
                        )
                        # Trigger emergency reset (6-step process)
                        self._trigger_emergency_reset('hybrid_height_overshoot')
                        return
                
                if height_error_for_direction < 0 or self.movement_state == 'down' or self.hybrid_last_stop_direction == 'down':
                    # Moving/moved downward: only check downward undershoot
                    if self.current_height < self.hybrid_target_height - height_overshoot_threshold:
                        self.get_logger().error(
                            f"🚨 HYBRID CONTROL EMERGENCY: Height undershoot detected! "
                            f"height={self.current_height:.2f}mm < target-threshold={self.hybrid_target_height - height_overshoot_threshold:.2f}mm "
                            f"(undershoot={self.hybrid_target_height - self.current_height:.2f}mm)"
                        )
                        # Trigger emergency reset (6-step process)
                        self._trigger_emergency_reset('hybrid_height_undershoot')
                        return
            
            # ═══════════════════════════════════════════════════════════════
            # HYBRID CONTROL SAFETY: Check for excessive force overshoot
            # Same logic as force_control to ensure consistent safety behavior
            # ═══════════════════════════════════════════════════════════════
            force_overshoot_threshold = 150.0  # N
            
            if self.current_force_combined is not None:
                if self.hybrid_direction == 'up':
                    # Force up: check if exceeded target by more than threshold
                    if self.current_force_combined > self.hybrid_target_force + force_overshoot_threshold:
                        self.get_logger().error(
                            f"🚨 HYBRID CONTROL EMERGENCY: Force overshoot detected! "
                            f"force={self.current_force_combined:.2f}N > target+threshold={self.hybrid_target_force + force_overshoot_threshold:.2f}N "
                            f"(overshoot={self.current_force_combined - self.hybrid_target_force:.2f}N)"
                        )
                        # Trigger emergency reset (6-step process)
                        self._trigger_emergency_reset('hybrid_force_overshoot')
                        return
                        
                elif self.hybrid_direction == 'down':
                    # Force down: check if exceeded target by more than threshold (negative direction)
                    if self.current_force_combined < self.hybrid_target_force - force_overshoot_threshold:
                        self.get_logger().error(
                            f"🚨 HYBRID CONTROL EMERGENCY: Force undershoot detected! "
                            f"force={self.current_force_combined:.2f}N < target-threshold={self.hybrid_target_force - force_overshoot_threshold:.2f}N "
                            f"(undershoot={self.hybrid_target_force - self.current_force_combined:.2f}N)"
                        )
                        # Trigger emergency reset (6-step process)
                        self._trigger_emergency_reset('hybrid_force_undershoot')
                        return
            
            # Check if EITHER condition is met (OR logic)
            height_reached = abs_height_error <= POSITION_TOLERANCE
            force_reached = False
            
            if self.current_force_combined is not None and self.hybrid_direction is not None:
                # CRITICAL: Force condition depends on movement direction
                # - UP: stop when force >= target (压力增加)
                # - DOWN: stop when force <= target (压力减小)
                if self.hybrid_direction == 'up':
                    force_reached = (self.current_force_combined >= self.hybrid_target_force)
                elif self.hybrid_direction == 'down':
                    force_reached = (self.current_force_combined <= self.hybrid_target_force)
            
            # Priority 0.5: Platform Range Limits Check (same as manual control)
            # Prevents hybrid control from exceeding physical platform boundaries
            if self.platform_range_enabled:
                if self.platform_range_min is not None:
                    bottom_limit = self.platform_range_min + 1.0
                    if self.current_height < bottom_limit:
                        self.get_logger().warn(
                            f"[HybridControl] Bottom limit reached: height={self.current_height:.2f}mm < "
                            f"limit={bottom_limit:.2f}mm (actual_min+1mm) - stopping"
                        )
                        self.hybrid_control_active = False
                        self._issue_stop(
                            direction='down',
                            reason='hybrid_bottom_limit',
                            disable_control=True,
                            complete_task_on_stop='limit_exceeded'
                        )
                        return
                
                if self.platform_range_max is not None:
                    top_limit = self.platform_range_max - 1.0
                    if self.current_height > top_limit:
                        self.get_logger().warn(
                            f"[HybridControl] Top limit reached: height={self.current_height:.2f}mm > "
                            f"limit={top_limit:.2f}mm (actual_max-1mm) - stopping"
                        )
                        self.hybrid_control_active = False
                        self._issue_stop(
                            direction='up',
                            reason='hybrid_top_limit',
                            disable_control=True,
                            complete_task_on_stop='limit_exceeded'
                        )
                        return
            
            # Priority 1: Check if EITHER target reached (OR logic)
            if height_reached or force_reached:
                # CRITICAL: Disable hybrid control before stopping
                self.hybrid_control_active = False
                
                # Determine completion reason
                if height_reached and force_reached:
                    reason = 'both_height_and_force_reached'
                elif height_reached:
                    reason = 'height_reached'
                else:
                    reason = 'force_reached'
                
                # Send stop and schedule task completion
                self._issue_stop(
                    direction=self.movement_state,
                    reason=f'hybrid_control_{reason}',
                    disable_control=False,  # Already disabled above
                    complete_task_on_stop='height_or_force_reached'
                )
                self.get_logger().info(
                    f"[HybridControl] ✅ Target reached: height={self.current_height:.2f}mm (target={self.hybrid_target_height:.2f}mm), "
                    f"force={self.current_force_combined:.2f}N (target={self.hybrid_target_force:.2f}N), reason={reason}"
                )
                return
            
            # Priority 2: 预测提前停（基于当前方向和平均超调）
            # Same logic as goto_height for adaptive early stop
            if self.hybrid_direction == 'up' and self.avg_overshoot_up > OVERSHOOT_MIN_MARGIN:
                threshold_height = self.hybrid_target_height - self.avg_overshoot_up
                if self.current_height >= threshold_height:
                    # Schedule task completion after stop relay OFF verification (early stop expects overshoot to reach target)
                    self.hybrid_control_active = False
                    self._issue_stop(
                        direction='up',
                        reason=f"hybrid_early_stop_up(th={threshold_height:.2f})",
                        disable_control=False,
                        complete_task_on_stop='height_or_force_reached'
                    )
                    self.get_logger().info(
                        f"[HybridControl] ⚠️ Predictive early stop UP: height={self.current_height:.2f}mm >= threshold={threshold_height:.2f}mm (avg_overshoot={self.avg_overshoot_up:.2f}mm)"
                    )
                    return
            elif self.hybrid_direction == 'down' and self.avg_overshoot_down > OVERSHOOT_MIN_MARGIN:
                threshold_height = self.hybrid_target_height + self.avg_overshoot_down
                if self.current_height <= threshold_height:
                    # Schedule task completion after stop relay OFF verification (early stop expects overshoot to reach target)
                    self.hybrid_control_active = False
                    self._issue_stop(
                        direction='down',
                        reason=f"hybrid_early_stop_down(th={threshold_height:.2f})",
                        disable_control=False,
                        complete_task_on_stop='height_or_force_reached'
                    )
                    self.get_logger().info(
                        f"[HybridControl] ⚠️ Predictive early stop DOWN: height={self.current_height:.2f}mm <= threshold={threshold_height:.2f}mm (avg_overshoot={self.avg_overshoot_down:.2f}mm)"
                    )
                    return
            
            # Priority 2.5: Motion Stall Detection (Mechanical Failure Monitoring)
            # Same logic as goto_height for mechanical failure detection
            # Extended to 2.0s to avoid false positives during slow/uneven movement.
            if (self.movement_state in ['up','down'] and
                self.stall_check_start_time is not None):
                elapsed = time.time() - self.stall_check_start_time
                if elapsed >= self.stall_check_duration:
                    position_change = abs(self.current_height - self.stall_check_start_position)
                    if position_change <= self.position_change_tolerance:
                        # Mechanical stall detected - trigger emergency rather than retry
                        self.get_logger().error(
                            f"🚨 HYBRID CONTROL STALL DETECTED: No movement after {elapsed:.2f}s "
                            f"(Δ={position_change:.3f}mm ≤ {self.position_change_tolerance}mm, direction={self.movement_state}) - "
                            f"possible hardware failure (stuck gears/motor seizure)"
                        )
                        # Trigger emergency reset (6-step process)
                        self._trigger_emergency_reset('hybrid_mechanical_stall')
                        return
            
            # Priority 3: Send movement command based on height error direction
            try:
                if height_error > POSITION_TOLERANCE:
                    # Need to move UP
                    if self.movement_state != 'up' and not self.movement_command_sent:
                        self.controller.up()
                        self.movement_command_sent = True
                        self.get_logger().info(
                            f"[HybridControl] ⬆️ Sending UP command: height={self.current_height:.2f}mm (target={self.hybrid_target_height:.2f}mm), "
                            f"force={self.current_force_combined:.2f}N (target={self.hybrid_target_force:.2f}N)"
                        )
                    else:
                        # Already moving up - periodic status log
                        if (time.time() - self.task_start_time) % 2.0 < CONTROL_RATE:  # Log every 2 seconds
                            self.get_logger().debug(
                                f"[HybridControl] ⬆️ Continuing UP: height={self.current_height:.2f}mm (target={self.hybrid_target_height:.2f}mm), "
                                f"force={self.current_force_combined:.2f}N (target={self.hybrid_target_force:.2f}N)"
                            )
                elif height_error < -POSITION_TOLERANCE:
                    # Need to move DOWN
                    if self.movement_state != 'down' and not self.movement_command_sent:
                        self.controller.down()
                        self.movement_command_sent = True
                        self.get_logger().info(
                            f"[HybridControl] ⬇️ Sending DOWN command: height={self.current_height:.2f}mm (target={self.hybrid_target_height:.2f}mm), "
                            f"force={self.current_force_combined:.2f}N (target={self.hybrid_target_force:.2f}N)"
                        )
                    else:
                        # Already moving down - periodic status log
                        if (time.time() - self.task_start_time) % 2.0 < CONTROL_RATE:  # Log every 2 seconds
                            self.get_logger().debug(
                                f"[HybridControl] ⬇️ Continuing DOWN: height={self.current_height:.2f}mm (target={self.hybrid_target_height:.2f}mm), "
                                f"force={self.current_force_combined:.2f}N (target={self.hybrid_target_force:.2f}N)"
                            )
            except Exception as e:
                self.get_logger().error(f"[HybridControl] Movement command error: {e}")
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
        # This check must happen regardless of control_enabled state
        # because overshoot can occur AFTER early stop is triggered
        # Check direction-specific overshoot to avoid false triggers
        # ═══════════════════════════════════════════════════════════════
        height_overshoot_threshold = 10.0  # mm
        
        # Check overshoot during active goto_height task (even if control disabled)
        if self.task_type == 'goto_height' and self.task_state == 'running':
            # Determine expected movement direction based on error
            error = self.target_height - self.current_height
            
            # Only check overshoot in the direction we're moving/moved
            if error > 0 or self.movement_state == 'up' or self.last_stop_direction == 'up':
                # Moving/moved upward: only check upward overshoot
                if self.current_height > self.target_height + height_overshoot_threshold:
                    self.get_logger().error(
                        f"🚨 HEIGHT CONTROL EMERGENCY: Height overshoot detected! "
                        f"height={self.current_height:.2f}mm > target+threshold={self.target_height + height_overshoot_threshold:.2f}mm "
                        f"(overshoot={self.current_height - self.target_height:.2f}mm)"
                    )
                    # Trigger emergency reset (6-step process)
                    self._trigger_emergency_reset('height_overshoot')
                    return
            
            if error < 0 or self.movement_state == 'down' or self.last_stop_direction == 'down':
                # Moving/moved downward: only check downward undershoot
                if self.current_height < self.target_height - height_overshoot_threshold:
                    self.get_logger().error(
                        f"🚨 HEIGHT CONTROL EMERGENCY: Height undershoot detected! "
                        f"height={self.current_height:.2f}mm < target-threshold={self.target_height - height_overshoot_threshold:.2f}mm "
                        f"(undershoot={self.target_height - self.current_height:.2f}mm)"
                    )
                    # Trigger emergency reset (6-step process)
                    self._trigger_emergency_reset('height_undershoot')
                    return
        
        # Priority 0.5: Platform Range Limits Check (same as manual control)
        # Prevents goto_height from exceeding physical platform boundaries
        if self.platform_range_enabled:
            if self.platform_range_min is not None:
                bottom_limit = self.platform_range_min + 1.0
                if self.current_height < bottom_limit:
                    self.get_logger().warn(
                        f"[GotoHeight] Bottom limit reached: height={self.current_height:.2f}mm < "
                        f"limit={bottom_limit:.2f}mm (actual_min+1mm) - stopping"
                    )
                    self._issue_stop(
                        direction='down',
                        reason='goto_height_bottom_limit',
                        disable_control=True,
                        complete_task_on_stop='limit_exceeded'
                    )
                    return
            
            if self.platform_range_max is not None:
                top_limit = self.platform_range_max - 1.0
                if self.current_height > top_limit:
                    self.get_logger().warn(
                        f"[GotoHeight] Top limit reached: height={self.current_height:.2f}mm > "
                        f"limit={top_limit:.2f}mm (actual_max-1mm) - stopping"
                    )
                    self._issue_stop(
                        direction='up',
                        reason='goto_height_top_limit',
                        disable_control=True,
                        complete_task_on_stop='limit_exceeded'
                    )
                    return
        
        # Priority 1: Check if target reached
        if abs_error <= POSITION_TOLERANCE:
            if self.control_enabled:
                # Schedule task completion after stop relay OFF verification completes
                # This ensures task_state and movement_state update synchronously
                self._issue_stop(direction=self.movement_state, reason="target_band", disable_control=True, complete_task_on_stop='target_reached')
            return
        
        # Priority 2: 预测提前停（基于当前方向和平均超调）
        if self.control_enabled:
            # 计算基于方向的提前停阈值
            if self.movement_state == 'up' and self.avg_overshoot_up > OVERSHOOT_MIN_MARGIN:
                threshold_height = self.target_height - self.avg_overshoot_up
                if self.current_height >= threshold_height:
                    # Schedule task completion after stop relay OFF verification (early stop expects overshoot to reach target)
                    # 预测提前停：终止控制，剩余惯性与超调仅记录不纠正
                    self._issue_stop(direction='up', reason=f"early_stop_up(th={threshold_height:.2f})", disable_control=True, complete_task_on_stop='target_reached')
                    return
            elif self.movement_state == 'down' and self.avg_overshoot_down > OVERSHOOT_MIN_MARGIN:
                threshold_height = self.target_height + self.avg_overshoot_down
                if self.current_height <= threshold_height:
                    # Schedule task completion after stop relay OFF verification (early stop expects overshoot to reach target)
                    self._issue_stop(direction='down', reason=f"early_stop_down(th={threshold_height:.2f})", disable_control=True, complete_task_on_stop='target_reached')
                    return

        # Priority 3: 移除时间节流逻辑：只在方向需要变化或到达目标时发送命令。
        # （依靠 movement_state 防止重复脉冲）
        
        # ═══════════════════════════════════════════════════════════════
        # Priority 2.5: Motion Stall Detection (Mechanical Failure Monitoring)
        # Detects extreme mechanical failures where relay verification succeeds
        # but platform physically cannot move (stuck gears, seized motor, etc.)
        # Triggers emergency reset rather than retry - mechanical issues cannot
        # be resolved by resending pulses and indicate critical hardware fault.
        # Extended to 2.0s to avoid false positives during slow/uneven movement.
        # Disabled during range scan to avoid interfering with endpoint detection.
        # ═══════════════════════════════════════════════════════════════
        if (not self.range_scan_active and
            self.movement_state in ['up','down'] and
            self.stall_check_start_time is not None):
            elapsed = time.time() - self.stall_check_start_time
            if elapsed >= self.stall_check_duration:
                position_change = abs(self.current_height - self.stall_check_start_position)
                if position_change <= self.position_change_tolerance:
                    # Mechanical stall detected - trigger emergency rather than retry
                    self.get_logger().error(
                        f"🚨 MECHANICAL STALL DETECTED: No movement after {elapsed:.2f}s "
                        f"(Δ={position_change:.3f}mm ≤ {self.position_change_tolerance}mm, direction={self.movement_state}) - "
                        f"possible hardware failure (stuck gears/motor seizure)"
                    )
                    # Trigger emergency reset (6-step process)
                    self._trigger_emergency_reset('mechanical_stall')
                    return
        
        # Priority 3: Send movement command based on error direction
        try:
            """10Hz timer callback: Detect stationary endpoints during range scan.
            Logic:
            - Active only when range_scan_active=True.
            - Track reference height. If movement exceeds tolerance, refresh reference & timer.
            - If height remains within tolerance for range_scan_duration_required seconds:
                * Mark endpoint for current direction.
                * Record height.
                * If scanning DOWN and HIGH not yet found, automatically start UP scan.
                * If both endpoints found, stop scan and complete task.
            """
            if error > POSITION_TOLERANCE:
                # Need to move up - only send command if not already sent
                if self.movement_state != 'up' and not self.movement_command_sent:
                    # 只发送一次命令，等待继电器验证完成后回调更新 movement_state
                    self.get_logger().info(
                        f"[Control] ⬆️  Sending UP command: current={self.current_height:.2f} target={self.target_height:.2f} err={error:.2f}"
                    )
                    self.controller.up()
                    # Set flag to prevent duplicate sends during flash verification (~50ms)
                    # Will be cleared in _on_flash_complete callback when movement_state is updated
                    self.movement_command_sent = True
                    self.last_command_time = now
                else:
                    # Already moving up - periodic status log
                    if (time.time() - self.task_start_time) % 2.0 < CONTROL_RATE:  # Log every 2 seconds
                        self.get_logger().debug(
                            f"[Control] ⬆️  Continuing UP: current={self.current_height:.2f} target={self.target_height:.2f} err={error:.2f}"
                        )
                
            elif error < -POSITION_TOLERANCE:
                # Need to move down - only send command if not already sent
                if self.movement_state != 'down' and not self.movement_command_sent:
                    self.get_logger().info(
                        f"[Control] ⬇️  Sending DOWN command: current={self.current_height:.2f} target={self.target_height:.2f} err={error:.2f}"
                    )
                    self.controller.down()
                    # Set flag to prevent duplicate sends during flash verification (~50ms)
                    # Will be cleared in _on_flash_complete callback when movement_state is updated
                    self.movement_command_sent = True
                    self.last_command_time = now
                else:
                    # Already moving down - periodic status log
                    if (time.time() - self.task_start_time) % 2.0 < CONTROL_RATE:  # Log every 2 seconds
                        self.get_logger().debug(
                            f"[Control] ⬇️  Continuing DOWN: current={self.current_height:.2f} target={self.target_height:.2f} err={error:.2f}"
                        )
                
                
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
            # Relay flash / range scan diagnostic (helps verify UP pulse actually issued)
            try:
                status['flash_active'] = bool(getattr(self.controller, 'flash_active', False))
                ctx = getattr(self.controller, 'flash_context', None)
                if ctx:
                    status['flash_relay'] = ctx.get('relay')  # 0 STOP / 1 UP / 2 DOWN
                    status['flash_phase'] = ctx.get('phase')
                    status['flash_on_attempts'] = ctx.get('on_attempts')
                    status['flash_off_attempts'] = ctx.get('off_attempts')
                    # expose start time age for debugging long flashes
                    st = ctx.get('start_time')
                    if st:
                        status['flash_age'] = round(time.time() - st, 3)
            except Exception:
                pass
            # Range scan status (to disambiguate web脚本 vs 内部扫描)
            if self.range_scan_active:
                status['range_scan_active'] = True
                status['range_scan_direction'] = self.range_scan_direction
                status['range_scan_low_reached'] = self.range_scan_low_reached
                status['range_scan_high_reached'] = self.range_scan_high_reached
                if self.range_scan_low_height is not None:
                    status['range_scan_low_height'] = round(self.range_scan_low_height, 2)
                if self.range_scan_high_height is not None:
                    status['range_scan_high_height'] = round(self.range_scan_high_height, 2)
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
            
            # Add last goto_height measurement for web calibration
            if self.last_goto_target is not None:
                status['last_goto_target'] = round(self.last_goto_target, 2)
            if self.last_goto_actual is not None:
                status['last_goto_actual'] = round(self.last_goto_actual, 2)
                if self.last_goto_stop_height is not None:
                    status['last_goto_stop_height'] = round(self.last_goto_stop_height, 2)
                status['last_goto_direction'] = self.last_goto_direction
                status['last_goto_timestamp'] = self.last_goto_timestamp
            
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
    # Queued direction pulse helper (for delayed up/down when flash busy)
    # ─────────────────────────────────────────────────────────────
    def _queued_direction_pulse(self, direction, seq_id):
        try:
            # 若仍忙，继续短延时重排队（最多尝试几次可扩展：这里简单递归限制深度）
            if getattr(self.controller, 'flash_active', False):
                # 简单的最多 3 次重试：在 lambda 中不追踪次数会无限，这里直接放弃后续
                # 可扩展为带计数，但保持最小改动先不复杂化。
                self.get_logger().debug(f"[SEQ {seq_id}] Deferred {direction} still busy, give up this queued attempt")
                return
            if direction == 'up':
                # Start flash - movement_state and task will be set in callback
                self.controller.up(seq_id=seq_id)
                self.get_logger().debug(f"[SEQ {seq_id}] Queued UP flash started")
            elif direction == 'down':
                # Start flash - movement_state and task will be set in callback
                self.controller.down(seq_id=seq_id)
                self.get_logger().debug(f"[SEQ {seq_id}] Queued DOWN flash started")
            else:
                self.get_logger().warn(f"[SEQ {seq_id}] Unknown queued direction: {direction}")
        except Exception as e:
            self.get_logger().error(f"Queued pulse error ({direction}): {e}")

    # ─────────────────────────────────────────────────────────────
    # Overshoot helper methods
    # ─────────────────────────────────────────────────────────────
    def _issue_stop(self, direction, reason="stop", disable_control=False, complete_task_on_stop=None):
        """发送停止脉冲并安排超调测量。
        当前策略：所有停止（目标带或提前停）一律 disable_control=True 防止后续相反方向补偿；
        但超调测量与 EMA 更新仍继续，以供人工分析和后续手动调整初始参数。
        
        Args:
            direction: 'up' or 'down' - movement direction before stop
            reason: Stop reason for logging
            disable_control: Whether to disable control_enabled flag
            complete_task_on_stop: If provided (e.g., 'target_reached'), task will be completed
                                   after stop relay OFF verification completes (in _on_flash_complete)
        """
        # Schedule task completion if requested (will be triggered by stop relay OFF verification)
        if complete_task_on_stop:
            self.pending_task_completion = {'reason': complete_task_on_stop}
            self.get_logger().debug(f"[Control] Task completion scheduled after stop OFF verification: {complete_task_on_stop}")
        
        try:
            self.controller.stop()
        except Exception as e:
            self.get_logger().error(f"[Control] Stop command error: {e}")
            # If stop command fails, clear pending completion to avoid stuck state
            self.pending_task_completion = None
            return
            
        if disable_control:
            self.control_enabled = False
        
        # NOTE: movement_state will be updated to 'stop' in _on_flash_complete callback
        # after relay OFF verification completes. This ensures synchronization with task completion.
        # Do NOT update movement_state here - let the callback handle it.
        
        prev_state = self.movement_state
        self.height_at_stop = self.current_height
        
        # Update last_stop_direction for goto_height overshoot detection
        self.last_stop_direction = direction if direction in ('up','down') else prev_state
        
        # CRITICAL: Also update hybrid_last_stop_direction if hybrid control is/was active
        # This ensures hybrid control has its own independent overshoot detection state
        if self.hybrid_control_active or (complete_task_on_stop and 'hybrid' in reason.lower()):
            self.hybrid_last_stop_direction = direction if direction in ('up','down') else prev_state
        
        self.last_stop_time = self.get_clock().now()
        
        # Only measure overshoot for goto_height tasks (when target is set)
        # Manual/force control doesn't need overshoot calibration
        if self.last_goto_target is not None:
            # 取消旧的 overshoot timer
            if self.overshoot_timer and self.overshoot_timer.is_alive():
                self.overshoot_timer.cancel()
            # 计划测量稳定高度 (500ms later, independent of relay verification)
            self.overshoot_timer = threading.Timer(OVERSHOOT_SETTLE_DELAY, self._measure_overshoot)
            self.overshoot_timer.start()
        
        self.get_logger().info(
            f"[Control] 🛑 STOP ({reason}) height_at_stop={self.height_at_stop:.2f} dir={self.last_stop_direction} disable_control={disable_control} pending_completion={complete_task_on_stop}"
        )

    def _measure_overshoot(self):
        """Measure stable position after settle delay for web calibration.
        
        CRITICAL: Measures RESIDUAL overshoot (movement after stop command)
        This is what the EMA algorithm should learn - the inertial drift that 
        occurs even after sending the stop pulse. This value is used to 
        calculate the early-stop threshold (target ± avg_overshoot).
        
        DO NOT measure total error from target - that would make the calibration
        try to compensate for early-stop offsets, creating a feedback loop.
        """
        try:
            stable_height = self.current_height
            if self.height_at_stop is None or self.last_stop_direction is None:
                return
            
            # Store measurement for web calibration interface
            self.last_goto_actual = stable_height
            self.last_goto_stop_height = self.height_at_stop  # Save stop height for web display
            self.last_goto_direction = self.last_stop_direction
            self.last_goto_timestamp = time.time()
            
            # Calculate RESIDUAL overshoot (movement after stop command)
            # This is the inertial drift that EMA should learn
            if self.last_goto_target is not None:
                if self.last_stop_direction == 'up':
                    # Upward: residual overshoot = how much we drifted UP after stopping
                    # Positive value means we drifted upward (normal inertia)
                    # Negative value means we fell back (unusual, but possible)
                    total_overshoot = stable_height - self.height_at_stop
                else:  # down
                    # Downward: residual overshoot = how much we drifted DOWN after stopping  
                    # Positive value means we drifted downward (normal inertia)
                    # Negative value means we bounced back up (unusual)
                    total_overshoot = self.height_at_stop - stable_height
                
                self.get_logger().info(
                    f"[Overshoot] {self.last_stop_direction.upper()} measured: "
                    f"target={self.last_goto_target:.2f} stop_at={self.height_at_stop:.2f} actual={stable_height:.2f} "
                    f"residual_overshoot={total_overshoot:.3f}mm (drift after stop command)"
                )
            else:
                self.get_logger().warn("[Overshoot] Cannot calculate overshoot - target not set")
            
            # Clear temporary tracking state (including target to prevent future measurements)
            self.height_at_stop = None
            self.last_stop_direction = None
            self.last_goto_target = None  # Clear target after measurement completes
            
        except Exception as e:
            self.get_logger().error(f"Overshoot measurement error: {e}")

    # ─────────────────────────────────────────────────────────────
    # Range Scan Endpoint Detection (10Hz timer)
    # ─────────────────────────────────────────────────────────────
    def _range_scan_check(self):
        """检测上下行机械端点（范围扫描模式）。

        核心思想: 利用“位置在容差内保持不动的持续时间”判定到达端点，避免依赖固定高度；
        并提供一个快速底部阈值 (height < 832mm) 的提前判定，用于硬件已自动停下时立即确认低端点。

        状态字段:
        - range_scan_active: 是否处于扫描模式
        - range_scan_direction: 当前扫描方向 'down' 或 'up'
        - range_scan_reference_height: 最近一次显著移动后记录的基准高度
        - range_scan_stall_start_time: 进入“停滞”观察窗口的起始时间
        - range_scan_height_tolerance: 高度变化 ≤ 该值视为未移动
        - range_scan_duration_required: 连续停滞达到该秒数判定端点
        - range_scan_low_reached / range_scan_high_reached: 已完成端点标记

        判定流程:
        1. 初始化 reference_height 与 stall_start_time。
        2. 若高度变化 > tolerance → 认为仍在运动，刷新 reference 与 stall_start_time。
        3. 若高度变化 ≤ tolerance：计算停滞时间；达到 duration_required → 判定端点。
        4. 向下扫描时增加快速阈值 (832mm) 判底逻辑。
        5. 低端点确认后自动切换为向上扫描；高端点确认后结束扫描并完成任务。
        """
        try:
            if not self.range_scan_active:
                return

            h = self.current_height
            direction = self.range_scan_direction

            # 初始化基准
            if self.range_scan_reference_height is None:
                self.range_scan_reference_height = h
                self.range_scan_stall_start_time = time.time()

            delta = abs(h - self.range_scan_reference_height)
            now_ts = time.time()

            # 向下扫描阶段（寻找 LOW）
            if direction == 'down' and not self.range_scan_low_reached:
                # 快速底部阈值：硬件已自动停住时立即认定低端点
                if h < 832.0:
                    self.range_scan_low_reached = True
                    self.range_scan_low_height = h
                    self.get_logger().info(
                        f"[RangeScan] ✅ LOW endpoint (fast threshold) height={h:.2f}mm < 832mm"
                    )
                    # 自动转为向上扫描
                    self.range_scan_direction = 'up'
                    try:
                        self.controller.up()
                        # movement_state will be set in _on_flash_complete callback
                    except Exception as e:
                        self.get_logger().error(f"[RangeScan] Start UP after LOW error: {e}")
                    # 重置参考用于上行端点判定
                    self.range_scan_reference_height = h
                    self.range_scan_stall_start_time = time.time()
                    return

                # 正常停滞判定逻辑
                if delta > self.range_scan_height_tolerance:
                    # 仍在移动 → 刷新基准
                    self.range_scan_reference_height = h
                    self.range_scan_stall_start_time = now_ts
                else:
                    stall_elapsed = now_ts - self.range_scan_stall_start_time
                    if stall_elapsed >= self.range_scan_duration_required:
                        self.range_scan_low_reached = True
                        self.range_scan_low_height = h
                        self.get_logger().info(
                            f"[RangeScan] ✅ LOW endpoint detected: height={h:.2f}mm stall={stall_elapsed:.2f}s tolerance={self.range_scan_height_tolerance}mm"
                        )
                        # 自动开始向上扫描
                        self.range_scan_direction = 'up'
                        try:
                            self.controller.up()
                            # movement_state will be set in _on_flash_complete callback
                        except Exception as e:
                            self.get_logger().error(f"[RangeScan] Start UP after LOW error: {e}")
                        self.range_scan_reference_height = h
                        self.range_scan_stall_start_time = time.time()
                        return

            # 向上扫描阶段（寻找 HIGH）
            if direction == 'up' and not self.range_scan_high_reached:
                if delta > self.range_scan_height_tolerance:
                    self.range_scan_reference_height = h
                    self.range_scan_stall_start_time = now_ts
                else:
                    stall_elapsed = now_ts - self.range_scan_stall_start_time
                    if stall_elapsed >= self.range_scan_duration_required:
                        self.range_scan_high_reached = True
                        self.range_scan_high_height = h
                        self.get_logger().info(
                            f"[RangeScan] ✅ HIGH endpoint detected: height={h:.2f}mm stall={stall_elapsed:.2f}s tolerance={self.range_scan_height_tolerance}mm"
                        )
                        # 若已找到 LOW → 完成扫描
                        if self.range_scan_low_reached:
                            self.range_scan_active = False
                            # Send stop and schedule task completion after relay verification
                            complete_on_stop = 'target_reached' if (self.task_state == 'running' and self.task_type == 'range_scan') else None
                            self._issue_stop(
                                direction=self.range_scan_direction if self.range_scan_direction else 'up',
                                reason='range_scan_complete',
                                disable_control=False,
                                complete_task_on_stop=complete_on_stop
                            )
                            self.get_logger().info(
                                f"[RangeScan] 🏁 Range scan complete: low={self.range_scan_low_height:.2f}mm high={self.range_scan_high_height:.2f}mm"
                            )
                            return
        except Exception as e:
            self.get_logger().error(f"[RangeScan] Check error: {e}")

    # ─────────────────────────────────────────────────────────────
    # Region overshoot helpers
    # ─────────────────────────────────────────────────────────────
    def _eval_poly(self, coeffs, x):
        try:
            deg = len(coeffs) - 1
            y = 0.0
            for k, c in enumerate(coeffs):
                powr = deg - k
                y += c * (x ** powr)
            return y
        except Exception:
            return None

    def _get_fitted_or_region_overshoot(self, height):
        """Prefer polynomial fit if available; otherwise use region selection.
        Returns (overshoot_up, overshoot_down) as ABSOLUTE VALUES (positive).
        Clips x to fit range.
        """
        if self.overshoot_fit:
            x_min = self.overshoot_fit.get('x_min', height)
            x_max = self.overshoot_fit.get('x_max', height)
            x = max(x_min, min(x_max, height))
            upv = self._eval_poly(self.overshoot_fit['coeffs_up'], x)
            dnv = self._eval_poly(self.overshoot_fit['coeffs_down'], x)
            if (upv is not None) and (dnv is not None):
                # CRITICAL: Ensure absolute values (positive) for control logic
                # Control uses: UP stop at (target - overshoot), DOWN stop at (target + overshoot)
                return (abs(float(upv)), abs(float(dnv)))
        # Fallback to region-based
        return self._get_region_overshoot(height)
    def _get_region_overshoot(self, height):
        """Return (overshoot_up, overshoot_down) for a target height.
        If no matching region, fall back to current defaults (avg_overshoot_*).
        Region match rule: lower <= height < upper (upper inclusive if last region).
        """
        if not self.overshoot_regions:
            return (self.avg_overshoot_up, self.avg_overshoot_down)
        for i, r in enumerate(self.overshoot_regions):
            lb = r['lower']; ub = r['upper']
            last = (i == len(self.overshoot_regions) - 1)
            if (height >= lb) and (height < ub or (last and height <= ub)):
                return (r.get('overshoot_up', self.avg_overshoot_up), r.get('overshoot_down', self.avg_overshoot_down))
        # No region matched
        return (self.avg_overshoot_up, self.avg_overshoot_down)

    # ─────────────────────────────────────────────────────────────
    # Force sensor callbacks
    # ─────────────────────────────────────────────────────────────
    def force_cb_right(self, msg):
        try:
            self.current_force_right = msg.data
            self._update_force_combined()
        except Exception as e:
            self.get_logger().warn(f"Force right parse error: {e}")

    def force_cb_left(self, msg):
        try:
            self.current_force_left = msg.data
            self._update_force_combined()
        except Exception as e:
            self.get_logger().warn(f"Force left parse error: {e}")
    
    def _update_force_combined(self):
        """Update combined force reading from available sensors"""
        if self.current_force_right is not None and self.current_force_left is not None:
            self.current_force_combined = self.current_force_right + self.current_force_left
        elif self.current_force_right is not None:
            self.current_force_combined = self.current_force_right
        elif self.current_force_left is not None:
            self.current_force_combined = self.current_force_left
        else:
            self.current_force_combined = None
    
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
            self.hybrid_control_active = False  # CRITICAL: Also disable hybrid control
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
    
    def _on_flash_complete(self, relay, seq_id):
        """Callback when relay flash verification succeeds.
        
        This is called by controller after relay ON→OFF→verify sequence completes.
        For STOP relay (relay 0), OFF verification completion means hardware has truly stopped.
        Updates movement_state and task_state based on verified relay action.
        
        CRITICAL: For STOP relay, task completion happens HERE (if pending) to ensure
        task_state and movement_state update synchronously after relay OFF verification.
        
        Args:
            relay: 0=STOP, 1=UP, 2=DOWN
            seq_id: Sequence ID for tracking
        """
        relay_name = {0: 'STOP', 1: 'UP', 2: 'DOWN'}.get(relay, f'Relay{relay}')
        
        if relay == 0:  # STOP - OFF verification complete, hardware truly stopped
            self.movement_state = 'stop'
            self.get_logger().info(f"[SEQ {seq_id}] Movement state updated: STOP (relay OFF verified)")
            
            # Complete task if pending (from goto_height reaching target)
            # This ensures task_state='completed' happens AFTER movement_state='stop'
            # Both states update synchronously based on relay OFF verification
            if self.pending_task_completion:
                reason = self.pending_task_completion['reason']
                self._complete_task(reason)
                self.get_logger().info(f"[SEQ {seq_id}] Task completed after stop OFF verification: {reason}")
                self.pending_task_completion = None
            
        elif relay == 1:  # UP
            self.movement_state = 'up'
            self.movement_command_sent = False  # Clear flag now that state is confirmed
            self.get_logger().info(f"[SEQ {seq_id}] Movement state updated: UP (relay verified)")
            # Start manual_up task if not already running a task
            if self.task_state != 'running':
                self._start_task('manual_up')
            # Start stall detection timer for control loop
            self.stall_check_start_time = time.time()
            self.stall_check_start_position = self.current_height
            
        elif relay == 2:  # DOWN
            self.movement_state = 'down'
            self.movement_command_sent = False  # Clear flag now that state is confirmed
            self.get_logger().info(f"[SEQ {seq_id}] Movement state updated: DOWN (relay verified)")
            # Start manual_down task if not already running a task
            if self.task_state != 'running':
                self._start_task('manual_down')
            # Start stall detection timer for control loop
            self.stall_check_start_time = time.time()
            self.stall_check_start_position = self.current_height
    
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
