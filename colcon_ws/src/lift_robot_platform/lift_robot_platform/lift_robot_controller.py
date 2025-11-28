from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *
import threading
import time
from collections import deque
import rclpy

class LiftRobotController(ModbusDevice):
    """
    Lift platform controller - drives standard relay outputs via Modbus FC05 (write single coil)

    Relay mapping (Platform):
    - Relay 0: stop
    - Relay 1: up
    - Relay 2: down

    Relay mapping (Pushrod):
    - Relay 3: stop
    - Relay 4: down
    - Relay 5: up

    Communication:
    - Baudrate: 115200
    - Device ID: 50 (overridden by parameter at runtime)
    - Function code 0x05 to write single coil

    Pulse (flash) behavior:
    - Open relay (0xFF00) -> 100ms delay -> Close relay (0x0000)
    """
    
    def __init__(self, device_id, node, use_ack_patch, callback_group=None):
        super().__init__(device_id, node, use_ack_patch, callback_group)
        
        # Relay address constants
        # Platform relays
        self.RELAY_PLATFORM_STOP = 0
        self.RELAY_PLATFORM_UP = 1
        self.RELAY_PLATFORM_DOWN = 2
        # Pushrod relays
        self.RELAY_PUSHROD_STOP = 3
        self.RELAY_PUSHROD_DOWN = 4
        self.RELAY_PUSHROD_UP = 5
        
        # Timer-based movement attributes
        self.active_timers = {}  # Dictionary to store active timers
        self.timer_lock = threading.Lock()  # Lock for thread safety
        
        # Callbacks (set by node)
        self.on_auto_stop_callback = None  # Callback for timed operation auto-stop
        self.on_flash_complete_callback = None  # Callback for relay flash verification success
        
        # Command queue for timed operations
        self.timed_cmd_queue = deque()
        self.waiting_for_timed_ack = False

    def _get_relay_name(self, relay_address):
        """Get human-readable name for relay address."""
        relay_names = {
            0: 'PLATFORM_STOP', 
            1: 'PLATFORM_UP', 
            2: 'PLATFORM_DOWN',
            3: 'PUSHROD_STOP',
            4: 'PUSHROD_DOWN',
            5: 'PUSHROD_UP'
        }
        return relay_names.get(relay_address, f'Relay{relay_address}')

    def initialize(self):
        """Initialize lift platform: reset all relays once Modbus service is ready."""
        try:
            self.node.get_logger().info("Waiting for Modbus service ready ...")

            def retry_reset():
                try:
                    if hasattr(self, 'cli') and self.cli.service_is_ready():
                        self.reset_all_relays()
                        self.node.get_logger().info("Initialization done, all relays reset")
                    else:
                        self.node.get_logger().warn("Modbus service not ready, retry in 1s ...")
                        timer = threading.Timer(1.0, retry_reset)
                        timer.start()
                except Exception as e:
                    self.node.get_logger().error(f"Retry reset error: {e}")
                    # Try again in 2 seconds
                    timer = threading.Timer(2.0, retry_reset)
                    timer.start()

            if hasattr(self, 'cli') and self.cli.service_is_ready():
                self.reset_all_relays()
                self.node.get_logger().info("Initialization done, all relays reset")
            else:
                retry_reset()
        except Exception as e:
            self.node.get_logger().error(f"Controller initialization error: {e}")
            # Continue without initialization - commands may fail but node won't crash

    def reset_all_relays(self, seq_id=None):
        """Reset all relays (example Modbus frame: 01 05 00 FF 00 00 FD FA)."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Reset all relays")
        reset_address = 0x00FF  # per hardware mapping
        reset_value = 0x0000
        self.node.get_logger().info(
            f"[SEQ {seq_id}] Send reset: addr=0x{reset_address:04X}, value=0x{reset_value:04X}"
        )
        self.send(5, reset_address, [reset_value], seq_id=seq_id)

    def abort_active_flash(self):
        """Force abort any active flash operation (for emergency reset only)."""
        with getattr(self, 'flash_lock', threading.Lock()):
            if getattr(self, 'flash_active', False):
                ctx = getattr(self, 'flash_context', None)
                if ctx:
                    # Cancel watchdog
                    wd = ctx.get('watchdog')
                    if wd and wd.is_alive():
                        try:
                            wd.cancel()
                        except Exception:
                            pass
                    seq_id = ctx.get('seq_id')
                    relay_name = self._get_relay_name(ctx.get('relay', -1))
                    self.node.get_logger().warn(
                        f"[SEQ {seq_id}] Force aborting active flash: {relay_name} "
                        f"(phase={ctx.get('phase')}, on_attempts={ctx.get('on_attempts')}, "
                        f"off_attempts={ctx.get('off_attempts')})"
                    )
                self.flash_context = None
                self.flash_active = False
                return True
            return False

    def stop(self, seq_id=None):
        """Stop motion (pulse relay 0).
        
        Returns:
            bool: True if flash started, False if rejected due to conflict
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] Stop command (relay 0 pulse)")
        return self.start_flash_async(relay_address=0, seq_id=seq_id, max_attempts=10)

    def up(self, seq_id=None):
        """Move up (pulse relay 1). Pulse width is fixed, velocity is hardware-defined.
        
        Returns:
            bool: True if flash started, False if rejected due to conflict
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] Up command (relay 1 pulse)")
        return self.start_flash_async(relay_address=1, seq_id=seq_id, max_attempts=10)

    def down(self, seq_id=None):
        """Move down (pulse relay 2). Pulse width is fixed, velocity is hardware-defined.
        
        Returns:
            bool: True if flash started, False if rejected due to conflict
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] Down command (relay 2 pulse)")
        return self.start_flash_async(relay_address=2, seq_id=seq_id, max_attempts=10)

    # ═══════════════════════════════════════════════════════════════
    # Pushrod Control Methods (Relay 3, 4, 5)
    # ═══════════════════════════════════════════════════════════════
    def pushrod_stop(self, seq_id=None):
        """Stop pushrod motion (pulse relay 3).
        
        Returns:
            bool: True if flash started, False if rejected due to conflict
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] Pushrod Stop command (relay 3 pulse)")
        return self.start_flash_async(relay_address=self.RELAY_PUSHROD_STOP, seq_id=seq_id, max_attempts=10)

    def pushrod_up(self, seq_id=None):
        """Move pushrod up (pulse relay 5). Pulse width is fixed, velocity is hardware-defined.
        
        Returns:
            bool: True if flash started, False if rejected due to conflict
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] Pushrod Up command (relay 5 pulse)")
        return self.start_flash_async(relay_address=self.RELAY_PUSHROD_UP, seq_id=seq_id, max_attempts=10)

    def pushrod_down(self, seq_id=None):
        """Move pushrod down (pulse relay 4). Pulse width is fixed, velocity is hardware-defined.
        
        Returns:
            bool: True if flash started, False if rejected due to conflict
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] Pushrod Down command (relay 4 pulse)")
        return self.start_flash_async(relay_address=self.RELAY_PUSHROD_DOWN, seq_id=seq_id, max_attempts=10)

    def open_relay(self, relay_address, seq_id=None):
        """Open relay.

        Reference frames:
        - Relay0 ON: 01 05 00 00 FF 00 8C 3A
        - Relay1 ON: 01 05 00 01 FF 00 DD FA
        - Relay2 ON: 01 05 00 02 FF 00 2D FA
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] Open relay {relay_address}")
        modbus_address = relay_address  # 0x0000 - 0x000F
        open_value = 0xFF00
        self.node.get_logger().info(
            f"[SEQ {seq_id}] Relay ON: addr=0x{modbus_address:04X}, value=0x{open_value:04X}"
        )
        self.send(5, modbus_address, [open_value], seq_id=seq_id)

    def close_relay(self, relay_address, seq_id=None):
        """Close relay.

        Reference frames:
        - Relay0 OFF: 01 05 00 00 00 00 CD CA
        - Relay1 OFF: 01 05 00 01 00 00 9C 0A
        - Relay2 OFF: 01 05 00 02 00 00 6C 0A
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] Close relay {relay_address}")
        modbus_address = relay_address
        close_value = 0x0000
        self.node.get_logger().info(
            f"[SEQ {seq_id}] Relay OFF: addr=0x{modbus_address:04X}, value=0x{close_value:04X}"
        )
        self.send(5, modbus_address, [close_value], seq_id=seq_id)

    # ─────────────────────────────────────────────────────────────
    # Simplified flash (pulse) mechanism - no verification
    # Send ON twice, wait 100ms, send OFF twice, assume complete
    # ─────────────────────────────────────────────────────────────
    def start_flash_async(self, relay_address, seq_id=None, max_attempts=3):
        """Simplified relay pulse: 2×ON + 100ms + 2×OFF, no verification.
        
        Simple and deterministic: assumes relay always works.
        Only one flash allowed at a time (mutual exclusion).
        
        Returns:
            bool: True if flash started, False if rejected due to conflict
        """
        with getattr(self, 'flash_lock', threading.Lock()):
            if getattr(self, 'flash_active', False):
                self.node.get_logger().warn(f"[SEQ {seq_id}] Flash already active, ignore new request relay={relay_address}")
                return False
            
            self.flash_active = True
            self.flash_context = {
                'relay': relay_address,
                'seq_id': seq_id,
                'start_time': time.time()
            }
        
        relay_name = self._get_relay_name(relay_address)
        self.node.get_logger().info(f"[SEQ {seq_id}] Flash start: {relay_name} (simplified mode)")
        
        # Schedule the flash sequence
        threading.Thread(target=self._execute_simple_flash, args=(relay_address, seq_id, relay_name), daemon=True).start()
        
        return True
    
    def _execute_simple_flash(self, relay_address, seq_id, relay_name):
        """Execute simplified flash sequence in background thread.
        
        Each send() is individually wrapped in try-catch to ensure all 4 commands
        are attempted even if some fail.
        """
        start_time = time.time()
        
        # Step 1: Send ON command twice (each with independent error handling)
        self.node.get_logger().info(f"[SEQ {seq_id}] {relay_name} ON×2")
        try:
            self.send(5, relay_address, [0xFF00], seq_id=seq_id)
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] ON #1 send error: {e}")
        
        time.sleep(0.01)  # 10ms between commands
        
        try:
            self.send(5, relay_address, [0xFF00], seq_id=seq_id)
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] ON #2 send error: {e}")
        
        # Step 2: Wait 100ms
        time.sleep(0.1)
        
        # Step 3: Send OFF command twice (each with independent error handling)
        self.node.get_logger().info(f"[SEQ {seq_id}] {relay_name} OFF×2")
        try:
            self.send(5, relay_address, [0x0000], seq_id=seq_id)
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] OFF #1 send error: {e}")
        
        time.sleep(0.01)  # 10ms between commands
        
        try:
            self.send(5, relay_address, [0x0000], seq_id=seq_id)
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] OFF #2 send error: {e}")
        
        # Step 4: Assume complete (always execute callback regardless of errors)
        total_ms = (time.time() - start_time) * 1000
        self.node.get_logger().info(f"[SEQ {seq_id}] ✅ Flash complete: {relay_name} ({total_ms:.1f}ms)")
        
        # Notify success
        if hasattr(self, 'on_flash_complete_callback') and self.on_flash_complete_callback:
            try:
                self.on_flash_complete_callback(relay_address, seq_id)
            except Exception as e:
                self.node.get_logger().error(f"[SEQ {seq_id}] Flash complete callback error: {e}")
        
        # Clear flash state
        self.flash_context = None
        self.flash_active = False



    # write_relay_verified removed - using simplified flash instead

    def _trigger_emergency_reset(self, seq_id, reason):
        """Trigger emergency reset - clear all states and recover to idle.
        
        Args:
            seq_id: sequence id for logging
            reason: reason for emergency reset
        """
        self.node.get_logger().error(
            f"[SEQ {seq_id}] 🔴 EMERGENCY RESET TRIGGERED - Reason: {reason}"
        )
        
        try:
            # Step 1: Set emergency_reset state (Actions will detect and abort)
            with self.node.control_lock:
                self.node.reset_in_progress = True
                self.node.control_enabled = False
                self.node.force_control_active = False
                self.node.task_state = 'emergency_reset'
                self.node.completion_reason = f'relay_failure: {reason}'
                self.node.get_logger().info(f"[SEQ {seq_id}] Emergency reset step 1: Set reset state")
            
            # Step 2: Wait for Actions to abort
            time.sleep(0.04)  # 40ms = 2 control loop cycles
            self.node.get_logger().info(f"[SEQ {seq_id}] Emergency reset step 2: Waited for Actions to abort")
            
            # Step 3: Cancel all timers
            try:
                self.cancel_all_timers()
                self.node.get_logger().info(f"[SEQ {seq_id}] Emergency reset step 3: All timers cancelled")
            except Exception as e:
                self.node.get_logger().error(f"[SEQ {seq_id}] Emergency reset timer cancel failed: {e}")
            
            # Step 4: Reset all relays to 0
            self.reset_all_relays(seq_id=seq_id)
            self.node.get_logger().info(f"[SEQ {seq_id}] Emergency reset step 4: All relays cleared to 0")
            
            # Step 5: Force abort any active flash
            try:
                aborted = self.abort_active_flash()
                if aborted:
                    self.node.get_logger().warn(f"[SEQ {seq_id}] Emergency reset step 5: Aborted active flash")
                else:
                    self.node.get_logger().info(f"[SEQ {seq_id}] Emergency reset step 5: No active flash to abort")
            except Exception as e:
                self.node.get_logger().error(f"[SEQ {seq_id}] Emergency reset flash abort failed: {e}")
            
            # Step 6: Send physical STOP pulse (non-blocking, best effort)
            try:
                self.open_relay(0, seq_id=seq_id)  # Relay 0 ON
                time.sleep(0.1)
                self.close_relay(0, seq_id=seq_id)  # Relay 0 OFF
                self.node.get_logger().info(f"[SEQ {seq_id}] Emergency reset step 6: Physical STOP pulse sent")
            except Exception as e:
                self.node.get_logger().error(f"[SEQ {seq_id}] ❌ Emergency STOP pulse failed: {e}")
            
            # Step 7: Clear all node states and recover to idle
            with self.node.control_lock:
                self.node.movement_state = 'stop'
                self.node.task_state = 'idle'
                self.node.task_end_time = time.time()
                self.node.completion_reason = 'emergency_reset'
                self.node.current_manual_move_direction = None
                self.node.reset_in_progress = False
                self.node.system_busy = False
                self.node.active_control_owner = None
                # Clear goto measurement data
                self.node.last_goto_target = None
                self.node.last_goto_actual = None
                self.node.last_goto_stop_height = None
                self.node.last_goto_direction = None
                self.node.last_goto_timestamp = None
                # Clear range scan state
                self.node.range_scan_active = False
                self.node.range_scan_phase = None
                self.node.range_scan_direction = None
                self.node.range_scan_reference_height = None
                self.node.range_scan_stall_start_time = None
            
            # Step 8: Reset pushrod stall detection
            if hasattr(self.node, '_reset_pushrod_stall_detection'):
                try:
                    self.node._reset_pushrod_stall_detection()
                except Exception as e:
                    self.node.get_logger().error(f"[SEQ {seq_id}] Pushrod stall reset failed: {e}")
            
            self.node.get_logger().warn(
                f"[SEQ {seq_id}] ✅ Emergency reset complete - System recovered to idle state"
            )
            
        except Exception as e:
            self.node.get_logger().error(
                f"[SEQ {seq_id}] ❌ Emergency reset procedure failed: {e}"
            )

    def timed_up(self, duration, seq_id=None):
        """Timed up movement.

        Args:
            duration: seconds to move up
            seq_id: sequence id
        """
        with self.timer_lock:
            # Cancel previous timers
            self.cancel_all_timers()
            self.node.get_logger().info(f"[SEQ {seq_id}] Timed up {duration}s")
            # Up command
            self.up(seq_id=seq_id)
            # Auto-stop timer (with callback notification)
            def auto_stop_with_callback():
                self.stop(seq_id=seq_id)
                # Notify node that timed operation completed
                if self.on_auto_stop_callback:
                    try:
                        self.on_auto_stop_callback()
                    except Exception as e:
                        self.node.get_logger().error(f"Auto-stop callback error: {e}")
            
            timer = threading.Timer(duration, auto_stop_with_callback)
            timer.start()
            self.active_timers['timed_up'] = timer

    def timed_down(self, duration, seq_id=None):
        """Timed down movement.

        Args:
            duration: seconds to move down
            seq_id: sequence id
        """
        with self.timer_lock:
            # Cancel previous timers
            self.cancel_all_timers()
            self.node.get_logger().info(f"[SEQ {seq_id}] Timed down {duration}s")
            # Down command
            self.down(seq_id=seq_id)
            # Auto-stop timer (with callback notification)
            def auto_stop_with_callback():
                self.stop(seq_id=seq_id)
                # Notify node that timed operation completed
                if self.on_auto_stop_callback:
                    try:
                        self.on_auto_stop_callback()
                    except Exception as e:
                        self.node.get_logger().error(f"Auto-stop callback error: {e}")
            
            timer = threading.Timer(duration, auto_stop_with_callback)
            timer.start()
            self.active_timers['timed_down'] = timer

    def stop_timed(self, seq_id=None):
        """Stop all timed motions."""
        with self.timer_lock:
            self.node.get_logger().info(f"[SEQ {seq_id}] Stop all timed motions")
            self.cancel_all_timers()
            self.stop(seq_id=seq_id)

    def cancel_all_timers(self):
        """Cancel all active timers."""
        for timer_name, timer in self.active_timers.items():
            if timer.is_alive():
                timer.cancel()
                self.node.get_logger().info(f"Cancelled timer: {timer_name}")
        self.active_timers.clear()

    def cleanup(self):
        """Cleanup resources."""
        with self.timer_lock:
            self.cancel_all_timers()
            self.node.get_logger().info("All timers cleaned up")

    # ─────────────────────────────────────────────────────────────
    # Force-controlled movement wrappers (start pulses only)
    # Actual force threshold stop logic handled in node control loop
    # ─────────────────────────────────────────────────────────────
    def force_up_start(self, seq_id=None):
        """Start upward movement under force control (single pulse)."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Force-UP start")
        self.up(seq_id=seq_id)

    def force_down_start(self, seq_id=None):
        """Start downward movement under force control (single pulse)."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Force-DOWN start")
        self.down(seq_id=seq_id)
