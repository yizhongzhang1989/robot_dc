import threading
from modbus_devices.base_device import ModbusDevice


class PushrodController(ModbusDevice):
    """Pushrod controller using relay pulses (relay3 stop, relay5 up, relay4 down).

    FINAL Relay mapping (confirmed):
        - Relay 3: STOP (pulse)
        - Relay 5: UP (pulse)
        - Relay 4: DOWN (pulse)

    Motion pattern:
        1. Direction pulse (UP/DOWN) -> board latches motion.
        2. Motion continues until a STOP pulse is sent (or power removed).

    Timed operations:
        - Pulse direction relay.
        - Schedule a STOP pulse after duration seconds.
        - Any new command cancels pending auto-stop (interruptible).

    Implementation notes:
        - Only one long-lived timer (self.stop_timer) tracks pending auto-stop.
        - Short pulse OFF timers are fire-and-forget and not stored.
    """

    def __init__(self, device_id, node, use_ack_patch=True):
        super().__init__(device_id, node, use_ack_patch)
        # Relay mapping
        self.RELAY_STOP = 3
        self.RELAY_UP = 5
        self.RELAY_DOWN = 4
        # Synchronization
        self.timer_lock = threading.RLock()
        # Single pending auto-stop timer (threading.Timer or None)
        self.stop_timer = None
        # Track the timer identity for callback race prevention
        self._stop_timer_id = 0
        # Callback for when auto-stop completes (for offset tracking)
        self.on_auto_stop_callback = None
        # Discrete named points (seconds up from base). Base assumed 0.
        # Only 'base' is supported now
        self.points = {
            'base': 0.0,
        }
        # Track current position in seconds from base (approx). Starts at base.
        self.current_position = 0.0
        self.current_point = 'base'
        # Movement tracking
        self._move_target_seconds = None  # absolute target (seconds from base)
        self._move_direction = None  # 'up' or 'down'
        self._move_timer_duration = None  # scheduled duration

    def initialize(self):
        self.node.get_logger().info("Pushrod controller initialized.")

    def open_relay(self, relay_address, seq_id=None):
        """Open relay (turn ON)."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Open pushrod relay {relay_address}")
        modbus_address = relay_address
        open_value = 0xFF00
        self.node.get_logger().info(
            f"[SEQ {seq_id}] Pushrod Relay ON: addr=0x{modbus_address:04X}, value=0x{open_value:04X}"
        )
        self.send(5, modbus_address, [open_value], seq_id=seq_id)

    def close_relay(self, relay_address, seq_id=None):
        """Close relay (turn OFF)."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Close pushrod relay {relay_address}")
        modbus_address = relay_address
        close_value = 0x0000
        self.node.get_logger().info(
            f"[SEQ {seq_id}] Pushrod Relay OFF: addr=0x{modbus_address:04X}, value=0x{close_value:04X}"
        )
        self.send(5, modbus_address, [close_value], seq_id=seq_id)

    def flash_relay(self, relay_address, duration_ms=100, seq_id=None):
        """Pulse relay: open -> delay -> close."""
        self.node.get_logger().info(
            f"[SEQ {seq_id}] Pushrod relay {relay_address} pulse: {duration_ms}ms"
        )
        self.open_relay(relay_address, seq_id=seq_id)
        threading.Timer(duration_ms / 1000.0, self.close_relay,
                        args=[relay_address], kwargs={'seq_id': seq_id}).start()

    def _cancel_stop_timer(self):
        """Cancel pending auto-stop timer if it exists."""
        if self.stop_timer and self.stop_timer.is_alive():
            try:
                self.stop_timer.cancel()
            except Exception:
                pass
        self.stop_timer = None
        self._stop_timer_id += 1  # invalidate previous callbacks

    def stop(self, seq_id=None):
        """Stop motion (pulse relay 3). Cancels any scheduled auto-stop timer."""
        with self.timer_lock:
            self._cancel_stop_timer()
        self.node.get_logger().info(f"[SEQ {seq_id}] Pushrod STOP command (relay 3 pulse)")
        self.flash_relay(relay_address=self.RELAY_STOP, duration_ms=100, seq_id=seq_id)

    def up(self, seq_id=None):
        """Move up (pulse relay 5). Cancels any scheduled auto-stop timer."""
        with self.timer_lock:
            self._cancel_stop_timer()
        self.node.get_logger().info(f"[SEQ {seq_id}] Pushrod UP command (relay 5 pulse)")
        self.flash_relay(relay_address=self.RELAY_UP, duration_ms=100, seq_id=seq_id)

    def down(self, seq_id=None):
        """Move down (pulse relay 4). Cancels any scheduled auto-stop timer."""
        with self.timer_lock:
            self._cancel_stop_timer()
        self.node.get_logger().info(f"[SEQ {seq_id}] Pushrod DOWN command (relay 4 pulse)")
        self.flash_relay(relay_address=self.RELAY_DOWN, duration_ms=100, seq_id=seq_id)

    def _schedule_auto_stop(self, duration, seq_id):
        """Internal: schedule auto-stop timer if duration > 0."""
        if duration <= 0:
            return
        with self.timer_lock:
            self._cancel_stop_timer()
            timer_id = self._stop_timer_id + 1
            self._stop_timer_id = timer_id
            timer = threading.Timer(duration, self._auto_stop_callback, args=(seq_id, timer_id))
            self.stop_timer = timer
            timer.start()
        import time
        self.node.get_logger().info(f"[SEQ {seq_id}] Auto-stop scheduled in {duration}s (timer_id={timer_id}) at t={time.time():.3f}")

    def timed_up(self, duration, seq_id=None):
        """Timed UP: pulse UP then schedule auto STOP after duration seconds."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Timed UP request {duration}s")
        self.up(seq_id=seq_id)
        if duration <= 0:
            self.node.get_logger().warning(f"[SEQ {seq_id}] Non-positive duration -> immediate UP only")
            return
        self._schedule_auto_stop(duration, seq_id)
        # Mark movement tracking for position estimate
        with self.timer_lock:
            self._move_direction = 'up'
            self._move_timer_duration = duration
            self._move_target_seconds = self.current_position + duration

    def timed_down(self, duration, seq_id=None):
        """Timed DOWN: pulse DOWN then schedule auto STOP after duration seconds."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Timed DOWN request {duration}s")
        self.down(seq_id=seq_id)
        if duration <= 0:
            self.node.get_logger().warning(f"[SEQ {seq_id}] Non-positive duration -> immediate DOWN only")
            return
        self._schedule_auto_stop(duration, seq_id)
        with self.timer_lock:
            self._move_direction = 'down'
            self._move_timer_duration = duration
            self._move_target_seconds = max(0.0, self.current_position - duration)

    def stop_timed(self, seq_id=None):
        """Explicit cancellation of timed operation then STOP pulse."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Stop TIMED operation request")
        self.stop(seq_id=seq_id)

    def cancel_all_timers(self):
        """Compat shim: cancel auto-stop timer (if any)."""
        with self.timer_lock:
            self._cancel_stop_timer()
        self.node.get_logger().info("Cancelled auto-stop timer (if any)")

    def _auto_stop_callback(self, seq_id, timer_id):
        """Auto-stop callback: verify timer identity then pulse STOP."""
        import time
        t_callback = time.time()
        with self.timer_lock:
            if timer_id != self._stop_timer_id:
                # Obsolete timer
                self.node.get_logger().warning(f"[SEQ {seq_id}] Auto-stop callback OBSOLETE timer_id={timer_id} vs current={self._stop_timer_id} at t={t_callback:.3f}")
                return
            self.stop_timer = None
        self.node.get_logger().info(f"[SEQ {seq_id}] Auto-stop timer elapsed (timer_id={timer_id}) at t={t_callback:.3f} -> STOP pulse")
        # Direct STOP pulse; don't call self.stop() to avoid double cancel sequence id churn
        self.flash_relay(relay_address=self.RELAY_STOP, duration_ms=100, seq_id=seq_id)
        # Update estimated position & point after completing timed move
        with self.timer_lock:
            if self._move_direction and self._move_timer_duration is not None:
                if self._move_direction == 'up':
                    self.current_position += self._move_timer_duration
                elif self._move_direction == 'down':
                    self.current_position = max(0.0, self.current_position - self._move_timer_duration)
                # Clamp to known range (0 to max point)
                max_point = max(self.points.values())
                if self.current_position < 0:
                    self.current_position = 0.0
                if self.current_position > max_point:
                    self.current_position = max_point
                # Determine closest named point
                closest = min(self.points.items(), key=lambda kv: abs(kv[1] - self.current_position))
                if abs(closest[1] - self.current_position) <= 0.25:  # within 0.25s tolerance
                    self.current_point = closest[0]
                else:
                    self.current_point = 'custom'
            # Reset move tracking
            self._move_direction = None
            self._move_timer_duration = None
            self._move_target_seconds = None
        
        # Call the callback to notify node (for offset tracking)
        if self.on_auto_stop_callback:
            try:
                self.node.get_logger().info(f"[SEQ {seq_id}] Calling on_auto_stop_callback at t={t_callback:.3f}")
                self.on_auto_stop_callback()
                self.node.get_logger().info(f"[SEQ {seq_id}] on_auto_stop_callback completed at t={time.time():.3f}")
            except Exception as e:
                self.node.get_logger().error(f"Auto-stop callback error: {e}")

    def goto_point(self, point_name, seq_id=None):
        """Move from current_point to target named point.
        Only 'base' is supported - goes to hardware bottom limit.
        """
        if point_name != 'base':
            self.node.get_logger().warning(f"[SEQ {seq_id}] Only 'base' point is supported, got '{point_name}'")
            return
        
        # Go to base: DOWN until hardware limit switch stop
        self.node.get_logger().info(f"[SEQ {seq_id}] Goto 'base' -> DOWN until hardware bottom")
        # Cancel any existing auto-stop timer; then issue a down pulse only
        with self.timer_lock:
            self._cancel_stop_timer()
            # Mark movement as downward to bottom
            self._move_direction = 'down'
            self._move_timer_duration = None
            self._move_target_seconds = 0.0
        self.down(seq_id=seq_id)
        # Assume hardware will stop itself; optimistic position update
        with self.timer_lock:
            self.current_position = 0.0
            self.current_point = 'base'
            # Clear tracking (since stop not timed)
            self._move_direction = None
            self._move_timer_duration = None
            self._move_target_seconds = None


    def cleanup(self):
        with self.timer_lock:
            self._cancel_stop_timer()
        self.node.get_logger().info("Pushrod controller cleanup complete")
