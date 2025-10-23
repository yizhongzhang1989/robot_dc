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
        self.node.get_logger().info(f"[SEQ {seq_id}] Auto-stop scheduled in {duration}s (timer_id={timer_id})")

    def timed_up(self, duration, seq_id=None):
        """Timed UP: pulse UP then schedule auto STOP after duration seconds."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Timed UP request {duration}s")
        self.up(seq_id=seq_id)
        if duration <= 0:
            self.node.get_logger().warning(f"[SEQ {seq_id}] Non-positive duration -> immediate UP only")
            return
        self._schedule_auto_stop(duration, seq_id)

    def timed_down(self, duration, seq_id=None):
        """Timed DOWN: pulse DOWN then schedule auto STOP after duration seconds."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Timed DOWN request {duration}s")
        self.down(seq_id=seq_id)
        if duration <= 0:
            self.node.get_logger().warning(f"[SEQ {seq_id}] Non-positive duration -> immediate DOWN only")
            return
        self._schedule_auto_stop(duration, seq_id)

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
        with self.timer_lock:
            if timer_id != self._stop_timer_id:
                # Obsolete timer
                return
            self.stop_timer = None
        self.node.get_logger().info(f"[SEQ {seq_id}] Auto-stop timer elapsed (timer_id={timer_id}) -> STOP pulse")
        # Direct STOP pulse; don't call self.stop() to avoid double cancel sequence id churn
        self.flash_relay(relay_address=self.RELAY_STOP, duration_ms=100, seq_id=seq_id)

    def cleanup(self):
        with self.timer_lock:
            self._cancel_stop_timer()
        self.node.get_logger().info("Pushrod controller cleanup complete")
