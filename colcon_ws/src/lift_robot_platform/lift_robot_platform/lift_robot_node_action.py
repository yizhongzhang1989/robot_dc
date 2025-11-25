#!/usr/bin/env python3
"""
Lift Robot Platform ROS2 Node - Action-Only Architecture
Complete self-contained Action servers for all control modes.
No Topic-based control, no external control_loop dependency.

Architecture:
- Each Action contains its own 50Hz control loop
- spin_once() only used for reading sensor data (1ms timeout)
- All control logic encapsulated within Action execute callbacks
- Clean separation: Actions for control, callbacks for sensors, timers for status
"""
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup
from std_msgs.msg import String
from lift_robot_interfaces.action import GotoHeight, ForceControl, HybridControl, ManualMove
from .lift_robot_controller import LiftRobotController
import json
import time
import os
import threading
import uuid

# ═══════════════════════════════════════════════════════════════
# Control Parameters
# ═══════════════════════════════════════════════════════════════
CONTROL_RATE = 0.02             # 50 Hz control loop (inside Actions)
POSITION_TOLERANCE = 0.5        # ±0.5 mm target tolerance (increased for sensor noise and movement speed)
FEEDBACK_INTERVAL = 0.1         # 10 Hz Action feedback rate
SENSOR_SPIN_TIMEOUT = 0.001     # 1ms spin for sensor data acquisition

# Pushrod Stall Detection Parameters
PUSHROD_STALL_TOLERANCE = 0.1   # mm - height change threshold to detect stall
PUSHROD_STALL_DURATION = 1.0    # seconds - duration required to confirm stall

# Overshoot calibration defaults
OVERSHOOT_INIT_UP = 2.8
OVERSHOOT_INIT_DOWN = 3.0
OVERSHOOT_MIN_MARGIN = 0.3


class LiftRobotNodeAction(Node):
    def __init__(self):
        super().__init__('lift_robot_platform_action')
        
        # Parameters
        self.declare_parameter('device_id', 1)
        self.declare_parameter('use_ack_patch', True)
        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value
        
        # ═══════════════════════════════════════════════════════════════
        # Shared State (thread-safe access)
        # ═══════════════════════════════════════════════════════════════
        self.state_lock = threading.RLock()
        self.control_lock = threading.RLock()  # For controller emergency reset compatibility
        self.current_height = 0.0
        self.current_force_combined = None  # Combined force from both sensors
        self.current_force_right = None  # Force sensor 1 (right)
        self.current_force_left = None   # Force sensor 2 (left)
        self.movement_state = 'stop'  # 'up', 'down', 'stop'
        self.movement_command_sent = False  # Prevent duplicate commands during flash verification
        
        # Controller compatibility flags (for emergency reset)
        self.reset_in_progress = False
        self.control_enabled = False
        self.force_control_active = False
        self.system_busy = False
        self.active_control_owner = None
        
        # Task state tracking (for Web monitoring compatibility)
        self.task_state = 'idle'  # 'idle', 'running', 'completed', 'aborted'
        self.task_type = None  # 'goto_height', 'force_control', 'hybrid_control', 'manual_move'
        self.task_start_time = None  # Unix timestamp
        self.task_end_time = None  # Unix timestamp
        self.completion_reason = None  # 'target_reached', 'cancelled', 'limit_exceeded', etc.
        
        # Current ManualMove direction (for detecting direction changes)
        self.current_manual_move_direction = None
        
        # Active Action tracking (for mutual exclusion)
        self.active_goal_handle = None  # Currently executing goal
        self.active_action_type = None  # 'goto_height', 'force_control', 'hybrid_control', 'manual_move'
        self.action_lock = threading.Lock()  # Lock for action mutual exclusion
        
        # Platform range limits (loaded from config)
        self.platform_range_min = None
        self.platform_range_max = None
        self.platform_range_enabled = False
        
        # Overshoot calibration
        self.avg_overshoot_up = OVERSHOOT_INIT_UP
        self.avg_overshoot_down = OVERSHOOT_INIT_DOWN
        self.overshoot_regions = []
        self.overshoot_fit = None
        
        # Overshoot measurement (for web calibration interface)
        self.last_goto_target = None      # Target height for last goto command
        self.last_goto_actual = None      # Actual stable height after overshoot
        self.last_goto_stop_height = None # Height when stop command was issued
        self.last_goto_direction = None   # 'up' or 'down'
        self.last_goto_timestamp = None   # Unix timestamp
        
        # Pushrod Stall Detection state
        self.pushrod_stall_reference_height = None  # Reference height for stall detection
        self.pushrod_stall_start_time = None        # Time when stall was first detected
        self.pushrod_stall_active = False           # Flag to track if stall detection is active
        
        # Range Scan state
        self.range_scan_active = False
        self.range_scan_direction = None  # 'up' or 'down'
        self.range_scan_low_height = None
        self.range_scan_high_height = None
        self.range_scan_reference_height = None
        self.range_scan_stall_start_time = None
        self.range_scan_low_reached = False
        self.range_scan_high_reached = False
        self.range_scan_height_tolerance = 0.1  # mm
        self.range_scan_duration_required = 1.0  # seconds
        
        # ═══════════════════════════════════════════════════════════════
        # Load Configuration Files
        # ═══════════════════════════════════════════════════════════════
        self._load_config()
        
        # ═══════════════════════════════════════════════════════════════
        # Controller Initialization
        # ═══════════════════════════════════════════════════════════════
        self.controller = LiftRobotController(
            device_id=self.device_id,
            node=self,
            use_ack_patch=self.use_ack_patch
        )
        self.controller.on_flash_complete_callback = self._on_flash_complete
        self.controller.initialize()
        
        # ═══════════════════════════════════════════════════════════════
        # Sensor Subscriptions (read-only data acquisition)
        # ═══════════════════════════════════════════════════════════════
        # Subscribe to command topic for emergency reset
        self.command_subscription = self.create_subscription(
            String,
            'lift_robot_platform/command',
            self._command_callback,
            10
        )
        
        self.sensor_subscription = self.create_subscription(
            String,
            '/draw_wire_sensor/data',
            self._sensor_callback,
            10
        )
        
        # Subscribe force sensors (Float32 type, same as Topic mode)
        from std_msgs.msg import Float32
        self.force_subscription = self.create_subscription(
            Float32,
            '/force_sensor',
            self._force_sensor_callback,
            10
        )
        
        self.force_subscription_2 = self.create_subscription(
            Float32,
            '/force_sensor_2',
            self._force_sensor_2_callback,
            10
        )
        self.get_logger().info("Subscribed to /force_sensor and /force_sensor_2")
        
        # ═══════════════════════════════════════════════════════════════
        # Status Publisher (10Hz for web monitoring)
        # ═══════════════════════════════════════════════════════════════
        self.status_publisher = self.create_publisher(
            String,
            'lift_robot_platform/status',
            10
        )
        self.status_timer = self.create_timer(0.1, self._publish_status)
        
        # Range Scan detection timer (10Hz)
        self.range_scan_timer = self.create_timer(0.1, self._range_scan_check)
        
        # ═══════════════════════════════════════════════════════════════
        # Action Servers (self-contained control logic)
        # ═══════════════════════════════════════════════════════════════
        self.callback_group = ReentrantCallbackGroup()
        
        self._goto_height_server = ActionServer(
            self,
            GotoHeight,
            'lift_robot/goto_height',
            self._execute_goto_height,
            callback_group=self.callback_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )
        
        self._force_control_server = ActionServer(
            self,
            ForceControl,
            'lift_robot/force_control',
            self._execute_force_control,
            callback_group=self.callback_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )
        
        self._hybrid_control_server = ActionServer(
            self,
            HybridControl,
            'lift_robot/hybrid_control',
            self._execute_hybrid_control,
            callback_group=self.callback_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )
        
        self._manual_move_server = ActionServer(
            self,
            ManualMove,
            'lift_robot/manual_move',
            self._execute_manual_move,
            callback_group=self.callback_group,
            goal_callback=self._goal_callback,
            cancel_callback=self._cancel_callback
        )
        
        self.get_logger().info("Lift platform Action-only node started")
    
    # ═══════════════════════════════════════════════════════════════
    # Configuration Loading
    # ═══════════════════════════════════════════════════════════════
    def _load_config(self):
        """Load overshoot calibration and platform range config"""
        # Determine config directory
        env_dir = os.environ.get('LIFT_ROBOT_CONFIG_DIR')
        if env_dir:
            self.config_dir = os.path.abspath(env_dir)
        else:
            try:
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
        
        os.makedirs(self.config_dir, exist_ok=True)
        
        # Helper function to get config file path (same as Topic version)
        def _cfg(name):
            return os.path.join(self.config_dir, name)
        
        # ═══════════════════════════════════════════════════════════════
        # Load Overshoot Calibration Config
        # ═══════════════════════════════════════════════════════════════
        overshoot_path = _cfg('platform_overshoot_calibration.json')
        self.get_logger().info(f"Overshoot config path resolved: {overshoot_path}")
        
        # Create default overshoot config if it doesn't exist
        if not os.path.exists(overshoot_path):
            try:
                # Default polynomial fit (degree 2, based on empirical data)
                default_config = {
                    'enable': True,
                    'format_version': 2,
                    'regions': [],
                    'samples': [],
                    'fit': {
                        'type': 'poly',
                        'degree': 2,
                        'coeffs_up': [
                            2.2091788116272794e-06,
                            -0.008710729042700554,
                            8.728348317269088
                        ],
                        'coeffs_down': [
                            3.4745013091926766e-06,
                            -0.012317707321611337,
                            11.240744855681426
                        ],
                        'x_min': 800.0,
                        'x_max': 1400.0,
                        'generated_at': time.time(),
                        'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S')
                    }
                }
                with open(overshoot_path, 'w') as f:
                    json.dump(default_config, f, indent=2)
                self.get_logger().info(f"Created default overshoot config with polynomial fit at {overshoot_path}")
            except Exception as e:
                self.get_logger().warn(f"Failed to create default overshoot config: {e}")
        
        # Load overshoot config
        if os.path.exists(overshoot_path):
            try:
                with open(overshoot_path, 'r') as f:
                    config = json.load(f)
                    # New format (v2) stores default + regions
                    if 'default' in config:
                        self.avg_overshoot_up = config['default'].get('overshoot_up', OVERSHOOT_INIT_UP)
                        self.avg_overshoot_down = config['default'].get('overshoot_down', OVERSHOOT_INIT_DOWN)
                    else:
                        # Backward compatibility
                        self.avg_overshoot_up = config.get('overshoot_up', OVERSHOOT_INIT_UP)
                        self.avg_overshoot_down = config.get('overshoot_down', OVERSHOOT_INIT_DOWN)
                    
                    if 'regions' in config and isinstance(config['regions'], list):
                        for r in config['regions']:
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
                                pass
                    
                    if 'fit' in config and config['fit'].get('type') == 'poly':
                        self.overshoot_fit = config['fit']
                
                self.get_logger().info(f"Loaded overshoot config: up={self.avg_overshoot_up:.2f}, down={self.avg_overshoot_down:.2f}")
            except Exception as e:
                self.get_logger().warn(f"Failed to load overshoot config: {e}")
        
        # ═══════════════════════════════════════════════════════════════
        # Load Platform Range Config
        # ═══════════════════════════════════════════════════════════════
        range_path = _cfg('platform_range.json')
        
        # Create default range config if it doesn't exist
        if not os.path.exists(range_path):
            try:
                default_range = {
                    'actual_min': None,
                    'actual_max': None,
                    'safe_min': None,
                    'safe_max': None,
                    'detected_at': None,
                    'detected_at_iso': None
                }
                with open(range_path, 'w') as f:
                    json.dump(default_range, f, indent=2)
                self.get_logger().info(f"Created default platform_range.json at {range_path} (not calibrated)")
            except Exception as e:
                self.get_logger().warn(f"Failed to create default platform_range.json: {e}")
        
        # Load platform range
        if os.path.exists(range_path):
            try:
                with open(range_path, 'r') as f:
                    range_data = json.load(f)
                    actual_min = range_data.get('actual_min')
                    actual_max = range_data.get('actual_max')
                    
                    # Only enable if both values are valid (not None)
                    if actual_min is not None and actual_max is not None:
                        self.platform_range_min = float(actual_min)
                        self.platform_range_max = float(actual_max)
                        self.platform_range_enabled = True
                        self.get_logger().info(
                            f"Platform range limits loaded: "
                            f"min={self.platform_range_min:.2f}mm, max={self.platform_range_max:.2f}mm"
                        )
                    else:
                        self.get_logger().info(
                            f"Platform range config exists but values are null - "
                            f"range limits DISABLED (run range detection to enable)"
                        )
            except Exception as e:
                self.get_logger().warn(f"Failed to load platform range config: {e} - range limits DISABLED")
        else:
            self.get_logger().info(f"No platform range config found - range limits DISABLED")
    
    # ═══════════════════════════════════════════════════════════════
    # Sensor Callbacks (read-only data acquisition)
    # ═══════════════════════════════════════════════════════════════
    # ═══════════════════════════════════════════════════════════════
    # Callback Methods
    # ═══════════════════════════════════════════════════════════════
    
    def _command_callback(self, msg):
        """Handle emergency reset command from topic"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '').lower()
            seq_id_str = command_data.get('seq_id', str(uuid.uuid4())[:8])
            
            if command == 'reset':
                self.get_logger().warn(f"[RESET] 🔴 Emergency reset triggered")
                
                # Step 1: Set emergency_reset state (Actions will detect and abort immediately)
                with self.state_lock:
                    self.task_state = 'emergency_reset'
                    self.reset_in_progress = True
                    self.get_logger().info("[RESET] Step 1: Set emergency_reset state")
                
                # Step 2: Wait for Actions to abort (they check emergency_reset at top of loop)
                time.sleep(CONTROL_RATE * 2)  # 40ms = 2 loop cycles to ensure abort
                self.get_logger().info("[RESET] Step 2: Waited for Actions to abort")
                
                # Step 3: Cancel all timers
                try:
                    self.controller.cancel_all_timers()
                    self.get_logger().info("[RESET] Step 3: All timers cancelled")
                except Exception as e:
                    self.get_logger().error(f"[RESET] Timer cancel failed: {e}")
                
                # Step 4: Reset all relays to 0
                try:
                    seq_id = abs(hash(seq_id_str)) % 65536
                    self.controller.reset_all_relays(seq_id=seq_id)
                    self.get_logger().info("[RESET] Step 4: All relays cleared to 0")
                except Exception as e:
                    self.get_logger().error(f"[RESET] Relay reset failed: {e}")
                
                # Step 4.5: Force abort any active flash (critical for emergency reset)
                # This prevents "Flash already active" errors when sending stop pulses
                try:
                    aborted = self.controller.abort_active_flash()
                    if aborted:
                        self.get_logger().warn("[RESET] Step 4.5: Aborted active flash operation")
                    else:
                        self.get_logger().info("[RESET] Step 4.5: No active flash to abort")
                except Exception as e:
                    self.get_logger().error(f"[RESET] Flash abort failed: {e}")
                
                # Step 5: Send STOP pulses sequentially (platform first, then pushrod)
                # Note: reset_all_relays (Step 4) already cleared all relays to 0.
                # The STOP pulses are safety measures to trigger hardware emergency stop circuits.
                # Must wait for platform stop flash to complete before sending pushrod stop.
                try:
                    # Setup flags to track stop completion and timeout
                    self._reset_platform_stop_done = False
                    self._reset_pushrod_stop_done = False
                    self._reset_seq_id = seq_id
                    self._reset_timeout_reached = False
                    
                    # Temporarily override flash callback to detect stop completion
                    original_callback = self.controller.on_flash_complete_callback
                    
                    def reset_flash_callback(relay_address, seq_id):
                        """Callback when flash completes during reset - chains platform→pushrod stops"""
                        try:
                            if relay_address == 0:  # Platform STOP completed
                                self.get_logger().info("[RESET] Platform STOP flash completed, sending pushrod STOP...")
                                self._reset_platform_stop_done = True
                                # Now send pushrod stop (20ms delay for safety)
                                threading.Timer(0.02, lambda: self.controller.pushrod_stop(seq_id=self._reset_seq_id)).start()
                            elif relay_address == 3:  # Pushrod STOP completed
                                self.get_logger().info("[RESET] Pushrod STOP flash completed - all stops done")
                                self._reset_pushrod_stop_done = True
                                # Restore original callback after sequence completes
                                self.controller.on_flash_complete_callback = original_callback
                            
                            # Call original callback if exists
                            if original_callback and relay_address in [0, 3]:
                                try:
                                    original_callback(relay_address, seq_id)
                                except Exception as e:
                                    self.get_logger().error(f"[RESET] Original callback error: {e}")
                        except Exception as e:
                            self.get_logger().error(f"[RESET] Flash callback error: {e}")
                    
                    def reset_timeout_handler():
                        """Timeout handler if stop sequence takes too long"""
                        if not (self._reset_platform_stop_done and self._reset_pushrod_stop_done):
                            self._reset_timeout_reached = True
                            self.get_logger().error(
                                f"[RESET] ⚠️ STOP sequence timeout! "
                                f"platform_done={self._reset_platform_stop_done}, "
                                f"pushrod_done={self._reset_pushrod_stop_done} - "
                                f"Continuing reset despite incomplete stops (Step 4 already cleared relays)"
                            )
                            # Restore original callback
                            self.controller.on_flash_complete_callback = original_callback
                            # Force abort any stuck flash
                            try:
                                self.controller.abort_active_flash()
                            except:
                                pass
                    
                    # Set temporary callback
                    self.controller.on_flash_complete_callback = reset_flash_callback
                    
                    # Start timeout watchdog (500ms should be enough for 2 stops)
                    timeout_timer = threading.Timer(0.5, reset_timeout_handler)
                    timeout_timer.start()
                    
                    # Send platform stop first (will trigger pushrod stop in callback)
                    self.controller.stop(seq_id=seq_id)
                    self.get_logger().info("[RESET] Step 5: Sent platform STOP, will chain to pushrod STOP after completion...")
                except Exception as e:
                    self.get_logger().error(f"[RESET] Stop pulse sequence failed: {e}")
                    # Restore callback on error
                    if 'original_callback' in locals():
                        self.controller.on_flash_complete_callback = original_callback
                    if 'timeout_timer' in locals() and timeout_timer.is_alive():
                        timeout_timer.cancel()
                
                # Step 6: Immediately recover to idle (frontend handles 5s display)
                with self.state_lock:
                    self.movement_state = 'stop'
                    self.task_state = 'idle'
                    self.task_end_time = time.time()
                    self.completion_reason = 'emergency_reset'
                    self.current_manual_move_direction = None
                    self.reset_in_progress = False
                    # Clear last goto measurement to prevent frontend from reading stale data
                    self.last_goto_target = None
                    self.last_goto_actual = None
                    self.last_goto_stop_height = None
                    self.last_goto_direction = None
                    self.last_goto_timestamp = None
                
                self.get_logger().info("[RESET] ✅ Reset complete: system recovered to idle (ready for new Actions)")
            
            elif command == 'range_scan_down':
                # Start range scan moving downward to find LOW endpoint
                if self.range_scan_active:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] range_scan_down rejected - scan already active")
                    return
                
                # Cancel any running Actions first
                with self.action_lock:
                    if self.active_goal_handle is not None:
                        self.get_logger().info(f"[SEQ {seq_id_str}] Aborting active {self.active_action_type} for range scan")
                        # Actions will abort on next loop check
                
                # Initialize scan state
                with self.state_lock:
                    self.range_scan_active = True
                    self.range_scan_direction = 'down'
                    self.range_scan_low_reached = False
                    self.range_scan_high_reached = False
                    self.range_scan_reference_height = None
                    self.range_scan_stall_start_time = None
                    self.range_scan_low_height = None
                    self.range_scan_high_height = None
                    self.task_state = 'running'
                    self.task_type = 'range_scan'
                    self.task_start_time = time.time()
                
                # Send initial downward pulse
                seq_id = abs(hash(seq_id_str)) % 65536
                self.controller.down(seq_id=seq_id)
                self.get_logger().info(
                    f"[SEQ {seq_id_str}] ▶️ Range scan DOWN started "
                    f"(tolerance={self.range_scan_height_tolerance}mm, duration={self.range_scan_duration_required}s)"
                )
            
            elif command == 'range_scan_up':
                # Start range scan moving upward to find HIGH endpoint
                if self.range_scan_active:
                    self.get_logger().warning(f"[SEQ {seq_id_str}] range_scan_up rejected - scan already active")
                    return
                
                with self.action_lock:
                    if self.active_goal_handle is not None:
                        self.get_logger().info(f"[SEQ {seq_id_str}] Aborting active {self.active_action_type} for range scan")
                
                with self.state_lock:
                    self.range_scan_active = True
                    self.range_scan_direction = 'up'
                    self.range_scan_low_reached = False
                    self.range_scan_high_reached = False
                    self.range_scan_reference_height = None
                    self.range_scan_stall_start_time = None
                    self.range_scan_low_height = None
                    self.range_scan_high_height = None
                    self.task_state = 'running'
                    self.task_type = 'range_scan'
                    self.task_start_time = time.time()
                
                seq_id = abs(hash(seq_id_str)) % 65536
                self.controller.up(seq_id=seq_id)
                self.get_logger().info(
                    f"[SEQ {seq_id_str}] ▶️ Range scan UP started "
                    f"(tolerance={self.range_scan_height_tolerance}mm, duration={self.range_scan_duration_required}s)"
                )
            
            elif command == 'range_scan_cancel':
                # Cancel any active range scan
                if not self.range_scan_active:
                    self.get_logger().info(f"[SEQ {seq_id_str}] range_scan_cancel - no active scan")
                else:
                    with self.state_lock:
                        self.range_scan_active = False
                        self.range_scan_direction = None
                        self.range_scan_reference_height = None
                        self.range_scan_stall_start_time = None
                    self.get_logger().info(f"[SEQ {seq_id_str}] ⏹️ Range scan cancelled")
                
                # Stop motion safely
                try:
                    seq_id = abs(hash(seq_id_str)) % 65536
                    self.controller.stop(seq_id=seq_id)
                    with self.state_lock:
                        self.movement_state = 'stop'
                        if self.task_type == 'range_scan':
                            self.task_state = 'completed'
                            self.task_end_time = time.time()
                            self.completion_reason = 'manual_stop'
                except Exception as e:
                    self.get_logger().error(f"[SEQ {seq_id_str}] Range scan cancel stop error: {e}")
        
        except Exception as e:
            self.get_logger().error(f"Command callback error: {e}")
    
    def _sensor_callback(self, msg):
        """Height sensor callback"""
        try:
            data = json.loads(msg.data)
            with self.state_lock:
                self.current_height = float(data.get('height', 0.0))
        except:
            pass
    
    def _force_sensor_callback(self, msg):
        """Force sensor 1 (right) callback"""
        try:
            with self.state_lock:
                self.current_force_right = msg.data
                self._update_force_combined()
        except Exception as e:
            self.get_logger().warn(f"Force right parse error: {e}")
    
    def _force_sensor_2_callback(self, msg):
        """Force sensor 2 (left) callback"""
        try:
            with self.state_lock:
                self.current_force_left = msg.data
                self._update_force_combined()
        except Exception as e:
            self.get_logger().warn(f"Force left parse error: {e}")
    
    def _update_force_combined(self):
        """Update combined force reading from available sensors (must be called with state_lock held)"""
        if self.current_force_right is not None and self.current_force_left is not None:
            self.current_force_combined = self.current_force_right + self.current_force_left
        elif self.current_force_right is not None:
            self.current_force_combined = self.current_force_right
        elif self.current_force_left is not None:
            self.current_force_combined = self.current_force_left
        else:
            self.current_force_combined = None
    
    def _on_flash_complete(self, relay, seq_id):
        """Relay flash verification complete callback.
        
        This is called by controller when relay flash (ON→OFF→verify) completes.
        Updates movement_state to reflect actual hardware state.
        
        For ManualMove Actions: This signals that the command has been accepted
        and movement has actually started.
        """
        relay_name = {0: 'STOP', 1: 'UP', 2: 'DOWN'}.get(relay, f'Relay{relay}')
        
        with self.state_lock:
            if relay == 0:  # STOP
                self.movement_state = 'stop'
                self.movement_command_sent = False  # Clear flag when state confirmed
                self.get_logger().debug(f"[SEQ {seq_id}] Flash complete: STOP → movement_state='stop'")
            elif relay == 1:  # UP
                self.movement_state = 'up'
                self.movement_command_sent = False  # Clear flag when state confirmed
                self.get_logger().debug(f"[SEQ {seq_id}] Flash complete: UP → movement_state='up'")
            elif relay == 2:  # DOWN
                self.movement_state = 'down'
                self.movement_command_sent = False  # Clear flag when state confirmed
                self.get_logger().debug(f"[SEQ {seq_id}] Flash complete: DOWN → movement_state='down'")
    
    # ═══════════════════════════════════════════════════════════════
    # Status Publishing
    # ═══════════════════════════════════════════════════════════════
    def _publish_status(self):
        """Publish current status (10Hz) - compatible with Web monitoring"""
        with self.state_lock:
            status = {
                'height': self.current_height,
                'movement_state': self.movement_state,
                'force': self.current_force_combined if self.current_force_combined is not None else 0.0,
                # Task state fields for Web compatibility
                'task_state': self.task_state,
                'task_type': self.task_type,
                'task_start_time': self.task_start_time,
                'task_end_time': self.task_end_time,
                'completion_reason': self.completion_reason,
                # Overshoot calibration data for Web interface
                'last_goto_target': self.last_goto_target,
                'last_goto_actual': self.last_goto_actual,
                'last_goto_stop_height': self.last_goto_stop_height,
                'last_goto_direction': self.last_goto_direction,
                'last_goto_timestamp': self.last_goto_timestamp,
                # Range scan data for Web interface
                'range_scan_active': self.range_scan_active,
                'range_scan_direction': self.range_scan_direction,
                'range_scan_low_height': self.range_scan_low_height,
                'range_scan_high_height': self.range_scan_high_height
            }
        
        msg = String()
        msg.data = json.dumps(status)
        self.status_publisher.publish(msg)
    
    def _range_scan_check(self):
        """Range scan endpoint detection (10Hz timer)"""
        if not self.range_scan_active:
            return
        
        try:
            with self.state_lock:
                h = self.current_height
                direction = self.range_scan_direction
            
            if direction is None:
                return
            
            # Initialize reference height on first check
            if self.range_scan_reference_height is None:
                self.range_scan_reference_height = h
                self.range_scan_stall_start_time = time.time()
                self.get_logger().info(f"[RangeScan] Initialized: height={h:.2f}mm, direction={direction}")
                return
            
            # Check if height is stable (stalled)
            height_change = abs(h - self.range_scan_reference_height)
            
            if height_change <= self.range_scan_height_tolerance:
                # Height stable - check duration
                if self.range_scan_stall_start_time is None:
                    self.range_scan_stall_start_time = time.time()
                
                stall_elapsed = time.time() - self.range_scan_stall_start_time
                
                # Debug log every 5 seconds
                if int(stall_elapsed) % 5 == 0 and int(stall_elapsed * 10) % 10 == 0:
                    self.get_logger().info(
                        f"[RangeScan] Stall progress: {stall_elapsed:.1f}s at {h:.2f}mm "
                        f"(need {self.range_scan_duration_required}s)"
                    )
                
                if stall_elapsed >= self.range_scan_duration_required:
                    # Endpoint detected
                    if direction == 'down' and not self.range_scan_low_reached:
                        with self.state_lock:
                            self.range_scan_low_reached = True
                            self.range_scan_low_height = h
                        self.get_logger().info(
                            f"[RangeScan] ✅ LOW endpoint detected: height={h:.2f}mm "
                            f"stall={stall_elapsed:.2f}s tolerance={self.range_scan_height_tolerance}mm"
                        )
                        # Switch to UP direction
                        self.controller.up()
                        self.range_scan_direction = 'up'
                        self.range_scan_reference_height = None
                        self.range_scan_stall_start_time = None
                    
                    elif direction == 'up' and not self.range_scan_high_reached:
                        with self.state_lock:
                            self.range_scan_high_reached = True
                            self.range_scan_high_height = h
                        self.get_logger().info(
                            f"[RangeScan] ✅ HIGH endpoint detected: height={h:.2f}mm "
                            f"stall={stall_elapsed:.2f}s tolerance={self.range_scan_height_tolerance}mm"
                        )
                        
                        # Both endpoints found - save and complete
                        if self.range_scan_low_height is not None:
                            self._save_range_config()
                            self.get_logger().info(
                                f"[RangeScan] 🏁 Range scan complete: "
                                f"low={self.range_scan_low_height:.2f}mm high={self.range_scan_high_height:.2f}mm"
                            )
                        
                        # Stop scan
                        self.controller.stop()
                        with self.state_lock:
                            self.range_scan_active = False
                            self.range_scan_direction = None
                            self.task_state = 'completed'
                            self.task_end_time = time.time()
                            self.completion_reason = 'range_detected'
            else:
                # Height changed - reset stall timer
                self.range_scan_reference_height = h
                self.range_scan_stall_start_time = time.time()
        
        except Exception as e:
            self.get_logger().error(f"[RangeScan] Check error: {e}")
    
    def _save_range_config(self):
        """Save detected range to config file"""
        try:
            range_path = os.path.join(self.config_dir, 'platform_range.json')
            
            # Calculate safe range (50mm margin from actual limits)
            safe_min = self.range_scan_low_height + 50.0
            safe_max = self.range_scan_high_height - 50.0
            
            range_data = {
                'actual_min': round(self.range_scan_low_height, 2),
                'actual_max': round(self.range_scan_high_height, 2),
                'safe_min': round(safe_min, 2),
                'safe_max': round(safe_max, 2),
                'detected_at': time.time(),
                'detected_at_iso': time.strftime('%Y-%m-%d %H:%M:%S')
            }
            with open(range_path, 'w') as f:
                json.dump(range_data, f, indent=2)
            
            # Update runtime values
            with self.state_lock:
                self.platform_range_min = self.range_scan_low_height
                self.platform_range_max = self.range_scan_high_height
                self.platform_range_enabled = True
            
            self.get_logger().info(
                f"[RangeScan] ✅ Range config saved: "
                f"actual=[{self.range_scan_low_height:.2f}, {self.range_scan_high_height:.2f}]mm, "
                f"safe=[{safe_min:.2f}, {safe_max:.2f}]mm"
            )
        except Exception as e:
            self.get_logger().error(f"[RangeScan] Failed to save range config: {e}")
    
    # ═══════════════════════════════════════════════════════════════
    # Action Server Callbacks
    # ═══════════════════════════════════════════════════════════════
    def _goal_callback(self, goal_request):
        """Accept or reject goals based on mutual exclusion policy.
        
        Policy:
        - Only one Action can execute at a time
        - Same type Actions can interrupt each other (e.g., ManualMove interrupts ManualMove)
        - Different type Actions are rejected if another Action is running
        - Range scan blocks all Actions
        """
        # Reject if range scan is active
        if self.range_scan_active:
            self.get_logger().warn("[Action] Rejecting goal - range scan is active")
            return GoalResponse.REJECT
        
        # Accept all goals - Actions will abort in execute loop if emergency_reset is active
        # Determine incoming action type
        # CRITICAL: Check more specific conditions FIRST (target_height+target_force before direction)
        incoming_type = None
        if hasattr(goal_request, 'target_height') and hasattr(goal_request, 'target_force'):  # HybridControl
            incoming_type = 'hybrid_control'
        elif hasattr(goal_request, 'target_force'):  # ForceControl (also has direction)
            incoming_type = 'force_control'
        elif hasattr(goal_request, 'target_height'):  # GotoHeight
            incoming_type = 'goto_height'
        elif hasattr(goal_request, 'direction'):  # ManualMove (only direction, no target_*)
            incoming_type = 'manual_move'
        
        with self.action_lock:
            # If no active Action, accept
            if self.active_goal_handle is None:
                self.get_logger().info(f"[Action] Accepting {incoming_type} - no active Action")
                return GoalResponse.ACCEPT
            
            # If same type Action, allow interruption
            # Old Action will detect state change and exit gracefully
            if self.active_action_type == incoming_type:
                self.get_logger().info(f"[Action] Accepting {incoming_type} - interrupting previous {self.active_action_type}")
                return GoalResponse.ACCEPT
            
            # Different type Action while another is running - reject
            self.get_logger().warn(
                f"[Action] Rejecting {incoming_type} - {self.active_action_type} is currently running"
            )
            return GoalResponse.REJECT
    
    def _cancel_callback(self, goal_handle):
        """Accept all cancel requests"""
        return CancelResponse.ACCEPT
    
    def _register_active_action(self, goal_handle, action_type):
        """Register an Action as active (for mutual exclusion)"""
        with self.action_lock:
            self.active_goal_handle = goal_handle
            self.active_action_type = action_type
            self.get_logger().debug(f"[Action] Registered active: {action_type} (handle={id(goal_handle)})")
    
    def _clear_active_action(self, goal_handle, action_type):
        """Clear active Action registration - only if this goal_handle is still active"""
        with self.action_lock:
            # Only clear if this is the currently active goal_handle
            # This prevents old Action from clearing new Action's registration
            if self.active_goal_handle is goal_handle:
                self.active_goal_handle = None
                self.active_action_type = None
                self.get_logger().debug(f"[Action] Cleared active: {action_type} (handle={id(goal_handle)})")
            else:
                self.get_logger().debug(f"[Action] Skip clear {action_type} - not active (handle={id(goal_handle)})")
    
    # ═══════════════════════════════════════════════════════════════
    # Utility Functions
    # ═══════════════════════════════════════════════════════════════
    def _get_overshoot(self, target_height):
        """Get overshoot values for target height"""
        # Use polynomial fit if available
        if self.overshoot_fit:
            try:
                x = target_height
                coeffs_up = self.overshoot_fit['coeffs_up']
                coeffs_down = self.overshoot_fit['coeffs_down']
                
                overshoot_up = sum(c * (x ** i) for i, c in enumerate(reversed(coeffs_up)))
                overshoot_down = sum(c * (x ** i) for i, c in enumerate(reversed(coeffs_down)))
                
                return abs(overshoot_up), abs(overshoot_down)
            except:
                pass
        
        # Use region-based if available
        for region in self.overshoot_regions:
            if region['lower'] <= target_height <= region['upper']:
                return region['overshoot_up'], region['overshoot_down']
        
        # Fall back to default
        return self.avg_overshoot_up, self.avg_overshoot_down
    
    def _check_range_limits(self, current_height, direction):
        """Check if platform has reached range limits.
        
        只在运动方向达到限制时才触发紧急停止:
        - 向下运动 AND 达到 min+1mm
        - 向上运动 AND 达到 max-1mm
        """
        if not self.platform_range_enabled:
            return False
        
        # Disable range limits during range scan to reach true mechanical endpoints
        if self.range_scan_active:
            return False
        
        # 只在运动方向达到限制时才触发
        if direction == 'down' and self.platform_range_min is not None:
            if current_height <= self.platform_range_min + 1.0:
                return True
        elif direction == 'up' and self.platform_range_max is not None:
            if current_height >= self.platform_range_max - 1.0:
                return True
        
        return False
    
    def _check_pushrod_stall(self, current_height, direction):
        """Check if pushrod has stalled (reached mechanical limit).
        
        Detects when pushrod stops moving in the commanded direction,
        indicating it has reached the top or bottom mechanical limit.
        
        Args:
            current_height: Current height reading (mm)
            direction: Movement direction ('up' or 'down')
        
        Returns:
            True if stall detected and confirmed for required duration
        """
        # Initialize reference height on first check
        if self.pushrod_stall_reference_height is None:
            self.pushrod_stall_reference_height = current_height
            self.pushrod_stall_start_time = time.time()
            return False
        
        # Calculate height change from reference
        height_change = abs(current_height - self.pushrod_stall_reference_height)
        
        # Check if height is stable (not moving)
        if height_change <= PUSHROD_STALL_TOLERANCE:
            # Height stable - check duration
            if self.pushrod_stall_start_time is None:
                self.pushrod_stall_start_time = time.time()
            
            stall_duration = time.time() - self.pushrod_stall_start_time
            
            # Stall confirmed if stable for required duration
            if stall_duration >= PUSHROD_STALL_DURATION:
                return True
        else:
            # Height changed - reset stall detection
            self.pushrod_stall_reference_height = current_height
            self.pushrod_stall_start_time = time.time()
        
        return False
    
    def _reset_pushrod_stall_detection(self):
        """Reset pushrod stall detection state."""
        self.pushrod_stall_reference_height = None
        self.pushrod_stall_start_time = None
        self.pushrod_stall_active = False
    
    # ═══════════════════════════════════════════════════════════════
    # GotoHeight Action
    # ═══════════════════════════════════════════════════════════════
    async def _execute_goto_height(self, goal_handle):
        """Execute GotoHeight action with internal 50Hz control loop"""
        target = goal_handle.request.target  # "platform" or "pushrod"
        input_height = goal_handle.request.target_height
        mode = goal_handle.request.mode if hasattr(goal_handle.request, 'mode') and goal_handle.request.mode else 'absolute'
        
        # Calculate actual target height based on mode
        with self.state_lock:
            current_height = self.current_height
        
        if mode == 'relative':
            # Relative mode: add input to current height
            target_height = current_height + input_height
            self.get_logger().info(
                f"GotoHeight started: target={target}, mode=RELATIVE, "
                f"input={input_height:+.2f}mm, current={current_height:.2f}mm, "
                f"target={target_height:.2f}mm"
            )
        else:
            # Absolute mode (default): use input as absolute target
            target_height = input_height
            self.get_logger().info(
                f"GotoHeight started: target={target}, mode=ABSOLUTE, "
                f"target_height={target_height:.2f}mm"
            )
        
        # Validate target
        if target not in ['platform', 'pushrod']:
            self.get_logger().error(f"Invalid target: {target}")
            result = GotoHeight.Result()
            result.success = False
            result.final_height = 0.0
            result.execution_time = 0.0
            result.completion_reason = 'invalid_target'
            goal_handle.abort()
            return result
        
        # Register as active Action
        self._register_active_action(goal_handle, 'goto_height')
        
        try:
            start_time = time.time()
            
            # Update task state (single lock for all state updates)
            # Clear last goto measurement to prevent frontend from re-reading old data
            with self.state_lock:
                self.task_state = 'running'
                self.task_type = 'goto_height'
                self.task_start_time = start_time
                self.task_end_time = None
                self.completion_reason = None
                initial_height = self.current_height  # Also read initial state
                # Clear last goto measurement to prevent duplicate recording
                self.last_goto_target = None
                self.last_goto_actual = None
                self.last_goto_stop_height = None
                self.last_goto_direction = None
                self.last_goto_timestamp = None
                # CRITICAL: Clear movement command flag for new Action
                self.movement_command_sent = False
            
            # Reset pushrod stall detection for new Action (only for pushrod)
            if target == 'pushrod':
                self._reset_pushrod_stall_detection()
                self.pushrod_stall_active = True  # Enable stall detection for this Action
            
            # Get overshoot calibration (only for platform)
            if target == 'platform':
                overshoot_up, overshoot_down = self._get_overshoot(target_height)
            else:  # pushrod - uses simple real-time control (no overshoot)
                overshoot_up, overshoot_down = None, None  # Flag to skip early stop logic
            
            # Feedback preparation
            feedback_msg = GotoHeight.Feedback()
            last_feedback_time = start_time
            
            # Control loop at 50Hz
            while rclpy.ok():
                loop_start = time.time()
                
                # PRIORITY 0: Check reset flag (emergency reset has highest priority)
                if self.reset_in_progress or self.task_state == 'emergency_reset':
                    self.get_logger().warn("GotoHeight aborted: emergency reset in progress")
                    
                    result = GotoHeight.Result()
                    result.success = False
                    result.final_height = self.current_height
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'emergency_reset'
                    
                    goal_handle.abort()
                    return result
                
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    # Send stop command based on target
                    if target == 'platform':
                        self.controller.stop()
                    else:  # pushrod
                        self.controller.pushrod_stop()
                    
                    result = GotoHeight.Result()
                    result.success = False
                    result.final_height = self.current_height
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'cancelled'
                    
                    # Update task state
                    with self.state_lock:
                        self.task_state = 'aborted'
                        self.task_end_time = time.time()
                        self.completion_reason = 'cancelled'
                    
                    self.get_logger().info("GotoHeight cancelled")
                    return result
                
                # MultiThreadedExecutor handles all callbacks (sensors, modbus, etc.)
                # Do NOT spin_once here - blocks modbus future callbacks
                
                with self.state_lock:
                    current_height = self.current_height
                    movement_state = self.movement_state
                
                # Calculate error
                error = target_height - current_height
                abs_error = abs(error)
                
                # Check if target reached or overshot
                # For pushrod: detect both tolerance range AND direction reversal (overshot)
                target_reached = False
                if abs_error <= POSITION_TOLERANCE:
                    target_reached = True
                elif target == 'pushrod':
                    # Pushrod-specific: detect overshoot (crossed target)
                    if movement_state == 'up' and error < -POSITION_TOLERANCE:
                        # Moving up but current > target (overshot)
                        target_reached = True
                    elif movement_state == 'down' and error > POSITION_TOLERANCE:
                        # Moving down but current < target (overshot)
                        target_reached = True
                
                if target_reached:
                    stop_height = current_height
                    direction_before_stop = movement_state
                    # Send stop command based on target
                    if target == 'platform':
                        self.controller.stop()
                    else:  # pushrod
                        self.controller.pushrod_stop()
                    # Clear movement_command_sent to allow stop to be processed
                    with self.state_lock:
                        self.movement_command_sent = False
                    # Wait for stop verification and platform to settle
                    time.sleep(0.5)
                    
                    # Read final stable height
                    with self.state_lock:
                        final_height = self.current_height
                    
                    # Measure and save overshoot for web calibration
                    if direction_before_stop in ['up', 'down']:
                        residual_overshoot = (final_height - stop_height) if direction_before_stop == 'up' else (stop_height - final_height)
                        with self.state_lock:
                            self.last_goto_target = target_height
                            self.last_goto_actual = final_height
                            self.last_goto_stop_height = stop_height
                            self.last_goto_direction = direction_before_stop
                            self.last_goto_timestamp = time.time()
                        
                        self.get_logger().info(
                            f"[Overshoot] {direction_before_stop.upper()} measured: target={target_height:.2f} "
                            f"stop_at={stop_height:.2f} actual={final_height:.2f} residual_overshoot={residual_overshoot:.3f}mm"
                        )
                    
                    result = GotoHeight.Result()
                    result.success = True
                    result.final_height = final_height
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'target_reached'
                    
                    # Update task state
                    with self.state_lock:
                        self.task_state = 'completed'
                        self.task_end_time = time.time()
                        self.completion_reason = 'target_reached'
                    
                    goal_handle.succeed()
                    self.get_logger().info(f"GotoHeight succeeded: final={current_height:.2f}mm, time={result.execution_time:.2f}s")
                    return result
                
                # Check pushrod stall (mechanical limit reached) - only for pushrod
                if target == 'pushrod' and self.pushrod_stall_active:
                    direction = 'up' if error > 0 else 'down'
                    if self._check_pushrod_stall(current_height, direction):
                        # Stall detected - pushrod reached mechanical limit
                        self.controller.pushrod_stop()
                        time.sleep(0.1)
                        
                        result = GotoHeight.Result()
                        result.success = True
                        result.final_height = current_height
                        result.execution_time = time.time() - start_time
                        result.completion_reason = 'stall_detected'
                        
                        # Update task state
                        with self.state_lock:
                            self.task_state = 'completed'
                            self.task_end_time = time.time()
                            self.completion_reason = 'stall_detected'
                        
                        goal_handle.succeed()
                        self.get_logger().info(
                            f"GotoHeight succeeded (pushrod stall): {direction.upper()} stalled at {current_height:.2f}mm, "
                            f"time={result.execution_time:.2f}s"
                        )
                        return result
                
                # Check range limits (only for platform)
                if target == 'platform':
                    direction = 'up' if error > 0 else 'down'
                    if self._check_range_limits(current_height, direction):
                        self.controller.stop()
                        time.sleep(0.1)
                        
                        result = GotoHeight.Result()
                        result.success = True
                        result.final_height = current_height
                        result.execution_time = time.time() - start_time
                        result.completion_reason = 'limit_reached'
                        
                        # Update task state
                        with self.state_lock:
                            self.task_state = 'completed'
                            self.task_end_time = time.time()
                            self.completion_reason = 'limit_reached'
                        
                        goal_handle.succeed()
                        self.get_logger().info(f"GotoHeight completed: limit reached at {current_height:.2f}mm")
                        return result
                
                # Early stop with overshoot compensation (ONLY for platform, pushrod uses simple control)
                # Platform: Topic版本逻辑: 提前停后disable_control并complete_task,控制循环退出
                #           Action版本逻辑: 提前停后发送STOP,等待静止,测量overshoot,然后succeed返回
                # Pushrod: Skip early stop logic - uses simple real-time control below
                if target == 'platform' and error > 0:  # Moving up
                    early_stop_height = target_height - max(overshoot_up - OVERSHOOT_MIN_MARGIN, OVERSHOOT_MIN_MARGIN)
                    if current_height >= early_stop_height and movement_state == 'up':
                        stop_height = current_height  # Record height when stop command issued
                        # Send stop command based on target
                        if target == 'platform':
                            self.controller.stop()
                        else:  # pushrod
                            self.controller.pushrod_stop()
                        self.get_logger().info(f"Early stop UP: height={current_height:.2f}, target={target_height:.2f}, overshoot={overshoot_up:.2f}")
                        
                        # Wait for platform to settle (like Topic version's OVERSHOOT_SETTLE_DELAY)
                        time.sleep(0.5)
                        
                        # Read final stable height
                        with self.state_lock:
                            final_height = self.current_height
                        
                        # Measure and save overshoot for web calibration
                        residual_overshoot = final_height - stop_height
                        with self.state_lock:
                            self.last_goto_target = target_height
                            self.last_goto_actual = final_height
                            self.last_goto_stop_height = stop_height
                            self.last_goto_direction = 'up'
                            self.last_goto_timestamp = time.time()
                        
                        self.get_logger().info(
                            f"[Overshoot] UP measured: target={target_height:.2f} stop_at={stop_height:.2f} "
                            f"actual={final_height:.2f} residual_overshoot={residual_overshoot:.3f}mm"
                        )
                        
                        result = GotoHeight.Result()
                        result.success = True
                        result.final_height = final_height
                        result.execution_time = time.time() - start_time
                        result.completion_reason = 'target_reached'
                        
                        # Update task state
                        with self.state_lock:
                            self.task_state = 'completed'
                            self.task_end_time = time.time()
                            self.completion_reason = 'target_reached'
                        
                        goal_handle.succeed()
                        self.get_logger().info(f"GotoHeight succeeded (early stop): final={final_height:.2f}mm, time={result.execution_time:.2f}s")
                        return result
                        
                elif target == 'platform' and error < 0:  # Moving down
                    early_stop_height = target_height + max(overshoot_down - OVERSHOOT_MIN_MARGIN, OVERSHOOT_MIN_MARGIN)
                    if current_height <= early_stop_height and movement_state == 'down':
                        stop_height = current_height  # Record height when stop command issued
                        # Send stop command based on target
                        if target == 'platform':
                            self.controller.stop()
                        else:  # pushrod
                            self.controller.pushrod_stop()
                        self.get_logger().info(f"Early stop DOWN: height={current_height:.2f}, target={target_height:.2f}, overshoot={overshoot_down:.2f}")
                        
                        # Wait for platform to settle
                        time.sleep(0.5)
                        
                        # Read final stable height
                        with self.state_lock:
                            final_height = self.current_height
                        
                        # Measure and save overshoot for web calibration
                        residual_overshoot = stop_height - final_height
                        with self.state_lock:
                            self.last_goto_target = target_height
                            self.last_goto_actual = final_height
                            self.last_goto_stop_height = stop_height
                            self.last_goto_direction = 'down'
                            self.last_goto_timestamp = time.time()
                        
                        self.get_logger().info(
                            f"[Overshoot] DOWN measured: target={target_height:.2f} stop_at={stop_height:.2f} "
                            f"actual={final_height:.2f} residual_overshoot={residual_overshoot:.3f}mm"
                        )
                        
                        result = GotoHeight.Result()
                        result.success = True
                        result.final_height = final_height
                        result.execution_time = time.time() - start_time
                        result.completion_reason = 'target_reached'
                        
                        # Update task state
                        with self.state_lock:
                            self.task_state = 'completed'
                            self.task_end_time = time.time()
                            self.completion_reason = 'target_reached'
                        
                        goal_handle.succeed()
                        self.get_logger().info(f"GotoHeight succeeded (early stop): final={final_height:.2f}mm, time={result.execution_time:.2f}s")
                        return result
                
                # Send movement command - simple real-time control (same for platform and pushrod)
                # Platform uses early stop above, pushrod uses this simple control directly
                # CRITICAL: Use movement_command_sent flag to prevent duplicate commands during flash verification
                # Reset flag once movement_state confirms the command took effect
                if error > 0:
                    if movement_state == 'up':
                        # Already moving in correct direction - reset flag to allow stop later
                        with self.state_lock:
                            self.movement_command_sent = False
                    elif not self.movement_command_sent:
                        # Need to send UP command
                        with self.state_lock:
                            if target == 'platform':
                                self.controller.up()
                            else:  # pushrod
                                self.controller.pushrod_up()
                            self.movement_command_sent = True  # Prevent duplicates during verification
                            self.get_logger().debug(f"[{target.upper()}] ⬆️ UP error={error:.2f}mm (cmd sent)")
                elif error < 0:
                    if movement_state == 'down':
                        # Already moving in correct direction - reset flag to allow stop later
                        with self.state_lock:
                            self.movement_command_sent = False
                    elif not self.movement_command_sent:
                        # Need to send DOWN command
                        with self.state_lock:
                            if target == 'platform':
                                self.controller.down()
                            else:  # pushrod
                                self.controller.pushrod_down()
                            self.movement_command_sent = True  # Prevent duplicates during verification
                            self.get_logger().debug(f"[{target.upper()}] ⬇️ DOWN error={error:.2f}mm (cmd sent)")
                
                # Publish feedback at 10Hz
                now = time.time()
                if now - last_feedback_time >= FEEDBACK_INTERVAL:
                    feedback_msg.current_height = current_height
                    feedback_msg.error = error
                    # Calculate progress based on distance traveled vs total distance
                    total_distance = abs(target_height - initial_height)
                    if total_distance > 0:
                        traveled = abs(current_height - initial_height)
                        feedback_msg.progress = min(1.0, traveled / total_distance)
                    else:
                        feedback_msg.progress = 1.0
                    feedback_msg.movement_state = movement_state
                    goal_handle.publish_feedback(feedback_msg)
                    last_feedback_time = now
                
                # Sleep to maintain 50Hz
                elapsed = time.time() - loop_start
                if elapsed < CONTROL_RATE:
                    time.sleep(CONTROL_RATE - elapsed)
        
        finally:
            # Always clear active Action on exit
            self._clear_active_action(goal_handle, 'goto_height')
            # Clear movement command flag to allow next Action to send commands
            with self.state_lock:
                self.movement_command_sent = False
            # Reset pushrod stall detection
            if target == 'pushrod':
                self._reset_pushrod_stall_detection()
    
    # ═══════════════════════════════════════════════════════════════
    # ForceControl Action
    # ═══════════════════════════════════════════════════════════════
    async def _execute_force_control(self, goal_handle):
        """Execute ForceControl action with internal 50Hz control loop"""
        self.get_logger().info(f"ForceControl started: target={goal_handle.request.target_force:.2f}N, direction={goal_handle.request.direction}")
        
        # Register as active action
        self._register_active_action(goal_handle, 'force_control')
        
        try:
            target_force = goal_handle.request.target_force
            direction = goal_handle.request.direction
            start_time = time.time()
            
            # Reset movement command flag
            with self.state_lock:
                self.movement_command_sent = False
            
            # Update task state
            with self.state_lock:
                self.task_state = 'running'
                self.task_type = f'force_{direction}'
                self.task_start_time = start_time
                self.task_end_time = None
                self.completion_reason = None
            
            # Validate direction
            if direction not in ['up', 'down']:
                result = ForceControl.Result()
                result.success = False
                result.final_force = 0.0
                result.execution_time = 0.0
                result.completion_reason = 'invalid_direction'
                goal_handle.abort()
                return result
            
            feedback_msg = ForceControl.Feedback()
            last_feedback_time = start_time
            
            # Control loop at 50Hz
            while rclpy.ok():
                loop_start = time.time()
                
                # PRIORITY 0: Check emergency stop (highest priority)
                if self.reset_in_progress or self.task_state == 'emergency_reset':
                    self.get_logger().warn("ForceControl aborted: emergency reset in progress")
                    
                    result = ForceControl.Result()
                    result.success = False
                    result.final_force = self.current_force_combined or 0.0
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'emergency_reset'
                    
                    goal_handle.abort()
                    return result
                
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.controller.stop()
                    
                    result = ForceControl.Result()
                    result.success = False
                    result.final_force = self.current_force_combined or 0.0
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'cancelled'
                    return result
                
                # MultiThreadedExecutor handles all callbacks automatically
                
                with self.state_lock:
                    current_force = self.current_force_combined
                    movement_state = self.movement_state
                    current_height = self.current_height
                    cmd_sent = self.movement_command_sent
                
                # Debug log every 1 second
                if int(time.time() - start_time) % 1 == 0 and int((time.time() - start_time) * 10) % 10 == 0:
                    self.get_logger().info(
                        f"[ForceControl] Loop: force={current_force}, state={movement_state}, "
                        f"cmd_sent={cmd_sent}, target={target_force:.1f}N"
                    )
                
                # Check if force sensor available
                if current_force is None:
                    time.sleep(CONTROL_RATE)
                    continue
                
                # Check target reached
                force_reached = False
                if direction == 'up' and current_force >= target_force:
                    force_reached = True
                elif direction == 'down' and current_force <= target_force:
                    force_reached = True
                
                if force_reached:
                    self.controller.stop()
                    time.sleep(0.1)
                    result = ForceControl.Result()
                    result.success = True
                    result.final_force = current_force
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'target_reached'
                    
                    # Update task state
                    with self.state_lock:
                        self.task_state = 'completed'
                        self.task_end_time = time.time()
                        self.completion_reason = 'target_reached'
                    
                    goal_handle.succeed()
                    self.get_logger().info(f"ForceControl succeeded: force={current_force:.2f}N")
                    return result
                
                # Check range limits
                if self._check_range_limits(current_height, direction):
                    self.controller.stop()
                    result = ForceControl.Result()
                    result.success = False
                    result.final_force = current_force
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'limit_exceeded'
                    goal_handle.abort()
                    return result
                
                # Send movement command (prevent duplicate sends during flash verification)
                if direction == 'up' and movement_state != 'up':
                    with self.state_lock:
                        if not self.movement_command_sent:
                            self.get_logger().info(f"[ForceControl] Sending UP command (state={movement_state})")
                            self.controller.up()
                            self.movement_command_sent = True  # Will be cleared when flash completes
                        else:
                            self.get_logger().debug(f"[ForceControl] Skipping UP - cmd already sent")
                elif direction == 'down' and movement_state != 'down':
                    with self.state_lock:
                        if not self.movement_command_sent:
                            self.get_logger().info(f"[ForceControl] Sending DOWN command (state={movement_state})")
                            self.controller.down()
                            self.movement_command_sent = True  # Will be cleared when flash completes
                        else:
                            self.get_logger().debug(f"[ForceControl] Skipping DOWN - cmd already sent")
                
                # Publish feedback
                now = time.time()
                if now - last_feedback_time >= FEEDBACK_INTERVAL:
                    feedback_msg.current_force = current_force
                    feedback_msg.error = target_force - current_force if direction == 'up' else current_force - target_force
                    feedback_msg.progress = min(100.0, abs(current_force / target_force) * 100.0) if target_force != 0 else 0.0
                    feedback_msg.movement_state = movement_state
                    goal_handle.publish_feedback(feedback_msg)
                    last_feedback_time = now
                
                # Maintain 50Hz
                elapsed = time.time() - loop_start
                if elapsed < CONTROL_RATE:
                    time.sleep(CONTROL_RATE - elapsed)
        
        finally:
            # Always clear active action on exit
            self._clear_active_action(goal_handle, 'force_control')
    
    # ═══════════════════════════════════════════════════════════════
    # HybridControl Action
    # ═══════════════════════════════════════════════════════════════
    async def _execute_hybrid_control(self, goal_handle):
        """Execute HybridControl action - stops when EITHER height OR force target reached"""
        self.get_logger().info(
            f"HybridControl started: height={goal_handle.request.target_height:.2f}mm, "
            f"force={goal_handle.request.target_force:.2f}N (direction auto-determined)"
        )
        
        # Register as active action
        self._register_active_action(goal_handle, 'hybrid_control')
        
        try:
            target_height = goal_handle.request.target_height
            target_force = goal_handle.request.target_force
            start_time = time.time()
            
            # Get regionalized overshoot values for target height (CRITICAL: use polynomial fit)
            overshoot_up, overshoot_down = self._get_overshoot(target_height)
            
            # Reset movement command flag
            with self.state_lock:
                self.movement_command_sent = False
            
            # Update task state
            with self.state_lock:
                self.task_state = 'running'
                self.task_type = 'height_force_hybrid'
                self.task_start_time = start_time
                self.task_end_time = None
                self.completion_reason = None
            
            # Get regionalized overshoot compensation (same as GotoHeight)
            overshoot_up, overshoot_down = self._get_overshoot(target_height)
            self.get_logger().info(
                f"HybridControl using overshoot: up={overshoot_up:.2f}mm, down={overshoot_down:.2f}mm "
                f"for target_height={target_height:.2f}mm"
            )
            
            # Determine initial force control direction based on initial height error
            # This direction is fixed throughout the control loop (does not change)
            with self.state_lock:
                initial_height = self.current_height
            initial_height_error = target_height - initial_height
            
            # Determine force control direction automatically from height error
            if initial_height_error > POSITION_TOLERANCE:
                # Need to move up → force increases
                force_control_direction = 'up'
            elif initial_height_error < -POSITION_TOLERANCE:
                # Need to move down → force decreases
                force_control_direction = 'down'
            else:
                # Height already at target, default to 'up' for force control
                force_control_direction = 'up'
            
            self.get_logger().info(
                f"HybridControl force direction: {force_control_direction} "
                f"(initial_height={initial_height:.2f}mm, initial_error={initial_height_error:.2f}mm)"
            )
            
            feedback_msg = HybridControl.Feedback()
            last_feedback_time = start_time
            
            # Control loop at 50Hz
            while rclpy.ok():
                loop_start = time.time()
                
                # PRIORITY 0: Check emergency stop (highest priority)
                if self.reset_in_progress or self.task_state == 'emergency_reset':
                    self.get_logger().warn("HybridControl aborted: emergency reset in progress")
                    
                    result = HybridControl.Result()
                    result.success = False
                    result.final_height = self.current_height
                    result.final_force = self.current_force_combined or 0.0
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'emergency_reset'
                    
                    goal_handle.abort()
                    return result
                
                if goal_handle.is_cancel_requested:
                    goal_handle.canceled()
                    self.controller.stop()
                    
                    result = HybridControl.Result()
                    result.success = False
                    result.final_height = self.current_height
                    result.final_force = self.current_force_combined or 0.0
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'cancelled'
                    return result
                
                # MultiThreadedExecutor handles all callbacks automatically
                
                with self.state_lock:
                    current_height = self.current_height
                    current_force = self.current_force_combined
                    movement_state = self.movement_state
                
                # Calculate height error
                height_error = target_height - current_height
                abs_height_error = abs(height_error)
                
                # Check both conditions (OR logic - whichever reaches first will stop)
                height_reached = abs_height_error <= POSITION_TOLERANCE
                
                force_reached = False
                if current_force is not None:
                    if force_control_direction == 'up':
                        # Moving up compresses sensor → force increases
                        force_reached = current_force >= target_force
                    elif force_control_direction == 'down':
                        # Moving down releases sensor → force decreases
                        force_reached = current_force <= target_force
                
                # Stop when EITHER height OR force target is reached (whichever first)
                if height_reached or force_reached:
                    self.controller.stop()
                    time.sleep(0.1)
                    result = HybridControl.Result()
                    result.success = True
                    result.final_height = current_height
                    result.final_force = current_force if current_force is not None else 0.0
                    result.execution_time = time.time() - start_time
                    
                    if height_reached and force_reached:
                        result.completion_reason = 'both_height_and_force_reached'
                    elif height_reached:
                        result.completion_reason = 'height_reached'
                    else:
                        result.completion_reason = 'force_reached'
                    
                    # Update task state
                    with self.state_lock:
                        self.task_state = 'completed'
                        self.task_end_time = time.time()
                        self.completion_reason = result.completion_reason
                    
                    goal_handle.succeed()
                    self.get_logger().info(f"HybridControl succeeded: {result.completion_reason}")
                    return result
                
                # Check range limits (use actual movement direction based on height_error)
                actual_direction = 'up' if height_error > POSITION_TOLERANCE else 'down' if height_error < -POSITION_TOLERANCE else None
                if actual_direction and self._check_range_limits(current_height, actual_direction):
                    self.controller.stop()
                    time.sleep(0.1)
                    
                    result = HybridControl.Result()
                    result.success = True
                    result.final_height = current_height
                    result.final_force = current_force if current_force is not None else 0.0
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'limit_reached'
                    
                    # Update task state
                    with self.state_lock:
                        self.task_state = 'completed'
                        self.task_end_time = time.time()
                        self.completion_reason = 'limit_reached'
                    
                    goal_handle.succeed()
                    self.get_logger().info(f"HybridControl completed: limit reached at {current_height:.2f}mm")
                    return result
                
                # Priority 2: Predictive early stop (based on actual movement direction from height_error)
                # Use the overshoot values calculated before the loop (from _get_overshoot)
                if height_error > POSITION_TOLERANCE and overshoot_up > OVERSHOOT_MIN_MARGIN:
                    # Moving up (height_error > 0)
                    early_stop_height = target_height - max(overshoot_up - OVERSHOOT_MIN_MARGIN, OVERSHOOT_MIN_MARGIN)
                    if current_height >= early_stop_height and movement_state == 'up':
                        self.controller.stop()
                        self.get_logger().info(
                            f"[HybridControl] Early stop UP: height={current_height:.2f}, target={target_height:.2f}, overshoot={overshoot_up:.2f}"
                        )
                        
                        # Wait for platform to settle
                        time.sleep(0.5)
                        
                        # Read final stable height and force
                        with self.state_lock:
                            final_height = self.current_height
                            final_force = self.current_force_combined if self.current_force_combined is not None else 0.0
                        
                        result = HybridControl.Result()
                        result.success = True
                        result.final_height = final_height
                        result.final_force = final_force
                        result.execution_time = time.time() - start_time
                        result.completion_reason = 'height_reached_with_overshoot'
                        
                        # Update task state
                        with self.state_lock:
                            self.task_state = 'completed'
                            self.task_end_time = time.time()
                            self.completion_reason = result.completion_reason
                        
                        goal_handle.succeed()
                        self.get_logger().info(f"HybridControl succeeded (early stop): final={final_height:.2f}mm, force={final_force:.2f}N")
                        return result
                        
                elif height_error < -POSITION_TOLERANCE and overshoot_down > OVERSHOOT_MIN_MARGIN:
                    # Moving down (height_error < 0)
                    early_stop_height = target_height + max(overshoot_down - OVERSHOOT_MIN_MARGIN, OVERSHOOT_MIN_MARGIN)
                    if current_height <= early_stop_height and movement_state == 'down':
                        self.controller.stop()
                        self.get_logger().info(
                            f"[HybridControl] Early stop DOWN: height={current_height:.2f}, target={target_height:.2f}, overshoot={overshoot_down:.2f}"
                        )
                        
                        # Wait for platform to settle
                        time.sleep(0.5)
                        
                        # Read final stable height and force
                        with self.state_lock:
                            final_height = self.current_height
                            final_force = self.current_force_combined if self.current_force_combined is not None else 0.0
                        
                        result = HybridControl.Result()
                        result.success = True
                        result.final_height = final_height
                        result.final_force = final_force
                        result.execution_time = time.time() - start_time
                        result.completion_reason = 'height_reached_with_overshoot'
                        
                        # Update task state
                        with self.state_lock:
                            self.task_state = 'completed'
                            self.task_end_time = time.time()
                            self.completion_reason = result.completion_reason
                        
                        goal_handle.succeed()
                        self.get_logger().info(f"HybridControl succeeded (early stop): final={final_height:.2f}mm, force={final_force:.2f}N")
                        return result
                
                # Priority 3: Send movement command based on height error direction
                # (prevent duplicate sends during flash verification)
                if height_error > POSITION_TOLERANCE:
                    # Need to move UP
                    if movement_state != 'up':
                        with self.state_lock:
                            if not self.movement_command_sent:
                                self.controller.up()
                                self.movement_command_sent = True  # Will be cleared when flash completes
                elif height_error < -POSITION_TOLERANCE:
                    # Need to move DOWN
                    if movement_state != 'down':
                        with self.state_lock:
                            if not self.movement_command_sent:
                                self.controller.down()
                                self.movement_command_sent = True  # Will be cleared when flash completes
                
                # Publish feedback
                now = time.time()
                if now - last_feedback_time >= FEEDBACK_INTERVAL:
                    feedback_msg.current_height = current_height
                    feedback_msg.current_force = current_force if current_force is not None else 0.0
                    feedback_msg.height_error = height_error
                    feedback_msg.force_error = (target_force - current_force) if current_force is not None else 0.0
                    feedback_msg.movement_state = movement_state
                    goal_handle.publish_feedback(feedback_msg)
                    last_feedback_time = now
                
                # Maintain 50Hz
                elapsed = time.time() - loop_start
                if elapsed < CONTROL_RATE:
                    time.sleep(CONTROL_RATE - elapsed)
        
        finally:
            # Always clear active action on exit
            self._clear_active_action(goal_handle, 'hybrid_control')
    
    # ═══════════════════════════════════════════════════════════════
    # ManualMove Action
    # ═══════════════════════════════════════════════════════════════
    async def _execute_manual_move(self, goal_handle):
        """Execute ManualMove action - continues until limit or cancel"""
        target = goal_handle.request.target  # "platform" or "pushrod"
        direction = goal_handle.request.direction
        
        self.get_logger().info(f"ManualMove started: target={target}, direction={direction}")
        
        # Validate target
        if target not in ['platform', 'pushrod']:
            self.get_logger().error(f"Invalid target: {target}")
            result = ManualMove.Result()
            result.success = False
            result.final_height = 0.0
            result.execution_time = 0.0
            result.completion_reason = 'invalid_target'
            goal_handle.abort()
            return result
        
        # Register as active Action
        self._register_active_action(goal_handle, 'manual_move')
        
        # Update current direction - this signals old Action to exit
        self.current_manual_move_direction = direction
        start_time = time.time()
        
        try:
            # Update task state
            with self.state_lock:
                self.task_state = 'running'
                self.task_type = f'manual_{direction}'
                self.task_start_time = start_time
                self.task_end_time = None
                self.completion_reason = None
                # Update movement_state immediately (will be updated again by flash callback)
                self.movement_state = direction
            
            # Reset pushrod stall detection for manual move (only for pushrod)
            if target == 'pushrod':
                self._reset_pushrod_stall_detection()
                self.pushrod_stall_active = True  # Enable stall detection for manual move
            
            if direction not in ['up', 'down']:
                result = ManualMove.Result()
                result.success = False
                result.final_height = 0.0
                result.execution_time = 0.0
                result.completion_reason = 'invalid_direction'
                goal_handle.abort()
                return result
            
            # Send movement command based on target device
            # Controller handles flash conflicts internally
            # If flash is active, controller will reject and we need to keep retrying
            if target == 'platform':
                if direction == 'up':
                    self.controller.up()
                else:
                    self.controller.down()
            else:  # pushrod
                if direction == 'up':
                    self.controller.pushrod_up()
                else:
                    self.controller.pushrod_down()
            
            feedback_msg = ManualMove.Feedback()
            last_feedback_time = start_time
            
            # Track if command has been accepted (movement started)
            command_accepted = False
            
            # Monitor loop at 50Hz
            loop_count = 0
            while rclpy.ok():
                loop_start = time.time()
                loop_count += 1
                
                # Debug log every 50 iterations (1 second)
                if loop_count % 50 == 0:
                    with self.state_lock:
                        debug_movement = self.movement_state
                        debug_direction = self.current_manual_move_direction
                    self.get_logger().info(
                        f"[DEBUG] ManualMove({direction}) loop #{loop_count}: "
                        f"movement_state={debug_movement}, "
                        f"current_direction={debug_direction}, "
                        f"command_accepted={command_accepted}, "
                        f"reset={self.reset_in_progress}"
                    )
                
                # PRIORITY 0: Check reset flag (emergency reset has highest priority)
                if self.reset_in_progress or self.task_state == 'emergency_reset':
                    self.get_logger().warn(f"ManualMove({direction}) aborted: emergency reset in progress")
                    
                    result = ManualMove.Result()
                    result.success = False
                    result.final_height = self.current_height
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'emergency_reset'
                    
                    goal_handle.abort()
                    return result
                
                # Check if command was accepted by checking if movement has started
                # We know command is accepted when movement_state matches our direction
                # This is set by controller's flash callback after relay verification
                with self.state_lock:
                    current_movement = self.movement_state
                
                if not command_accepted:
                    if current_movement == direction:
                        command_accepted = True
                        self.get_logger().info(f"ManualMove({target}/{direction}) command accepted, movement started")
                    else:
                        # Command not yet accepted, retry
                        if loop_count % 10 == 0:  # Log every 10 retries
                            self.get_logger().warn(
                                f"ManualMove({target}/{direction}) command not accepted yet "
                                f"(current_movement={current_movement}), retrying..."
                            )
                        if target == 'platform':
                            if direction == 'up':
                                self.controller.up()
                            else:
                                self.controller.down()
                        else:  # pushrod
                            if direction == 'up':
                                self.controller.pushrod_up()
                            else:
                                self.controller.pushrod_down()
                
                # Check if direction changed (new opposing ManualMove started)
                if self.current_manual_move_direction != direction:
                    # New Action has taken over, just exit quietly
                    # Do NOT send stop() - new direction will naturally override old one
                    # Do NOT modify shared state (task_state, movement_state, current_manual_move_direction)
                    # because new Action has already set them
                    
                    result = ManualMove.Result()
                    result.success = False
                    result.final_height = self.current_height
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'direction_changed'
                    
                    self.get_logger().info(f"ManualMove({direction}) aborted: direction changed to {self.current_manual_move_direction}")
                    goal_handle.abort()
                    return result
                
                # Check for cancellation
                if goal_handle.is_cancel_requested:
                    self.get_logger().info(f"ManualMove({target}/{direction}) received cancel request")
                    goal_handle.canceled()
                    # Send stop command based on target
                    if target == 'platform':
                        self.controller.stop()
                    else:  # pushrod
                        self.controller.pushrod_stop()
                    
                    # Update task state and movement_state
                    with self.state_lock:
                        self.task_state = 'completed'
                        self.task_end_time = time.time()
                        self.completion_reason = 'manual_stop'
                        self.movement_state = 'stop'
                        # Clear direction only if it's still our direction
                        if self.current_manual_move_direction == direction:
                            self.current_manual_move_direction = None
                    
                    result = ManualMove.Result()
                    result.success = True
                    result.final_height = self.current_height
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'cancelled'
                    
                    self.get_logger().info(f"ManualMove({direction}) stopped by user")
                    return result
                    
                    self.get_logger().info("ManualMove stopped by user")
                    return result
                
                # Check pushrod stall (mechanical limit reached) - only for pushrod
                with self.state_lock:
                    current_height = self.current_height
                
                if target == 'pushrod' and self.pushrod_stall_active:
                    if self._check_pushrod_stall(current_height, direction):
                        # Stall detected - pushrod reached mechanical limit
                        self.controller.pushrod_stop()
                        time.sleep(0.1)
                        
                        result = ManualMove.Result()
                        result.success = True
                        result.final_height = current_height
                        result.execution_time = time.time() - start_time
                        result.completion_reason = 'stall_detected'
                        
                        with self.state_lock:
                            self.task_state = 'completed'
                            self.task_end_time = time.time()
                            self.completion_reason = 'stall_detected'
                            self.movement_state = 'stop'
                            # Clear direction only if it's still our direction
                            if self.current_manual_move_direction == direction:
                                self.current_manual_move_direction = None
                        
                        goal_handle.succeed()
                        self.get_logger().info(
                            f"ManualMove({direction}) succeeded (pushrod stall): {direction.upper()} stalled at {current_height:.2f}mm, "
                            f"time={result.execution_time:.2f}s"
                        )
                        return result
                
                # Check range limits (only for platform)
                if target == 'platform' and self._check_range_limits(current_height, direction):
                    # Send stop command based on target
                    if target == 'platform':
                        self.controller.stop()
                    else:  # pushrod
                        self.controller.pushrod_stop()
                    time.sleep(0.1)
                    
                    result = ManualMove.Result()
                    result.success = True
                    result.final_height = current_height
                    result.execution_time = time.time() - start_time
                    result.completion_reason = 'limit_reached'
                    
                    with self.state_lock:
                        self.task_state = 'completed'
                        self.task_end_time = time.time()
                        self.completion_reason = 'limit_exceeded'
                    
                    goal_handle.succeed()
                    self.get_logger().info(f"ManualMove completed: limit reached at {current_height:.2f}mm")
                    return result
                
                # Publish feedback
                now = time.time()
                if now - last_feedback_time >= FEEDBACK_INTERVAL:
                    feedback_msg.current_height = current_height
                    feedback_msg.movement_state = f"moving_{direction}"
                    feedback_msg.elapsed_time = now - start_time
                    
                    # Warn if approaching limit
                    if self.platform_range_enabled:
                        if direction == 'down' and self.platform_range_min is not None:
                            feedback_msg.limit_warning = (current_height - self.platform_range_min) < 10.0
                        elif direction == 'up' and self.platform_range_max is not None:
                            feedback_msg.limit_warning = (self.platform_range_max - current_height) < 10.0
                    else:
                        feedback_msg.limit_warning = False
                    
                    goal_handle.publish_feedback(feedback_msg)
                    last_feedback_time = now
                
                # Maintain 50Hz
                elapsed = time.time() - loop_start
                if elapsed < CONTROL_RATE:
                    time.sleep(CONTROL_RATE - elapsed)
        
        finally:
            # Always clear active action on exit
            self._clear_active_action(goal_handle, 'manual_move')
            # Reset pushrod stall detection
            if target == 'pushrod':
                self._reset_pushrod_stall_detection()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LiftRobotNodeAction()
        # Use MultiThreadedExecutor for concurrent Action handling
        executor = MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()
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
