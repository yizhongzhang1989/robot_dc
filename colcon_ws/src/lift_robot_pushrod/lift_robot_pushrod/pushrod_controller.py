import threading
import time
from modbus_devices.base_device import ModbusDevice
from modbus_driver_interfaces.srv import ModbusRequest as ModbusRequestSrv


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
        
        # Async flash verification (similar to platform_controller)
        self.flash_lock = threading.Lock()
        self.flash_active = False
        self.flash_context = None
        
        # Callbacks
        self.on_auto_stop_callback = None  # Callback for when auto-stop completes (for offset tracking)
        self.on_flash_complete_callback = None  # Callback for relay flash verification success
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
        self.start_flash_async(relay_address=self.RELAY_STOP, seq_id=seq_id)

    def up(self, seq_id=None):
        """Move up (pulse relay 5). Cancels any scheduled auto-stop timer."""
        with self.timer_lock:
            self._cancel_stop_timer()
        self.node.get_logger().info(f"[SEQ {seq_id}] Pushrod UP command (relay 5 pulse)")
        self.start_flash_async(relay_address=self.RELAY_UP, seq_id=seq_id)

    def down(self, seq_id=None):
        """Move down (pulse relay 4). Cancels any scheduled auto-stop timer."""
        with self.timer_lock:
            self._cancel_stop_timer()
        self.node.get_logger().info(f"[SEQ {seq_id}] Pushrod DOWN command (relay 4 pulse)")
        self.start_flash_async(relay_address=self.RELAY_DOWN, seq_id=seq_id)

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

    # ─────────────────────────────────────────────────────────────
    # Async flash (pulse) verification - matches platform_controller
    # ─────────────────────────────────────────────────────────────
    def start_flash_async(self, relay_address, seq_id=None, max_attempts=3):
        """Begin an asynchronous relay pulse with immediate verification & retry.
        
        Non-blocking: scheduling is done via futures and threading.Timer so we never
        spin or sleep inside a ROS callback thread. Only one flash is allowed at a time.
        If another flash is active, the new request is ignored.
        """
        with self.flash_lock:
            if self.flash_active:
                self.node.get_logger().warn(f"[SEQ {seq_id}] Flash already active, ignore new request relay={relay_address}")
                return
            # Initialize context
            self.flash_active = True
            self.flash_context = {
                'relay': relay_address,
                'seq_id': seq_id,
                'phase': 'ON',
                'on_attempts': 0,
                'off_attempts': 0,
                'max': max_attempts,
                'start_time': time.time(),
                'on_eval_done': False,
                'on_inner_read_count': 0,
                'off_eval_done': False,
                'off_inner_read_count': 0
            }
        relay_name = {3: 'STOP', 5: 'UP', 4: 'DOWN'}.get(relay_address, f'Relay{relay_address}')
        self.node.get_logger().info(f"[SEQ {seq_id}] Async flash start: {relay_name} (max_attempts={max_attempts})")
        self._flash_attempt_on()

    def _flash_attempt_on(self):
        ctx = self.flash_context
        if ctx is None:
            return
        ctx['on_attempts'] += 1
        attempt = ctx['on_attempts']
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        relay_name = {3: 'STOP', 5: 'UP', 4: 'DOWN'}.get(relay, f'Relay{relay}')
        self.node.get_logger().info(f"[SEQ {seq_id}] ON attempt {attempt}/{ctx['max']} for {relay_name}")
        ctx['on_eval_done'] = False
        ctx['on_inner_read_count'] = 0
        # Send ON write then schedule read after 10ms
        self.send(5, relay, [0xFF00], seq_id=seq_id)
        threading.Timer(0.01, self._flash_read_on_result).start()

    def _flash_read_on_result(self):
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'ON':
            return
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        # Read relay status to verify ON
        req = ModbusRequestSrv.Request()
        req.slave_id = self.device_id
        req.function_code = 1  # Read coils
        req.address = 0x0000
        req.count = 6  # Read relays 0-5
        req.values = []
        req.seq_id = seq_id if seq_id is not None else 0
        future = self.cli.call_async(req)
        def first_done(f):
            ctx_local = self.flash_context
            if ctx_local is None or ctx_local.get('phase') != 'ON':
                return
            if ctx_local['on_eval_done']:
                return
            ctx_local['on_eval_done'] = True
            self._flash_on_evaluate(f)
        future.add_done_callback(first_done)
        # Schedule micro-timeout extra read
        threading.Timer(0.002, self._flash_on_second_read, args=[future]).start()

    def _flash_on_second_read(self, first_future):
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'ON':
            return
        if ctx['on_eval_done'] or first_future.done():
            return
        if ctx['on_inner_read_count'] >= 2:
            return
        ctx['on_inner_read_count'] += 1
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        req = ModbusRequestSrv.Request()
        req.slave_id = self.device_id
        req.function_code = 1
        req.address = 0x0000
        req.count = 6
        req.values = []
        req.seq_id = seq_id if seq_id is not None else 0
        second_future = self.cli.call_async(req)
        def second_done(f):
            ctx_local = self.flash_context
            if ctx_local is None or ctx_local.get('phase') != 'ON':
                return
            if ctx_local['on_eval_done']:
                return
            ctx_local['on_eval_done'] = True
            self._flash_on_evaluate(f)
        second_future.add_done_callback(second_done)
        if ctx['on_inner_read_count'] < 2:
            threading.Timer(0.01, self._flash_on_second_read, args=[first_future]).start()

    def _flash_on_evaluate(self, future):
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'ON':
            return
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        relay_name = {3: 'STOP', 5: 'UP', 4: 'DOWN'}.get(relay, f'Relay{relay}')
        try:
            resp = future.result()
            ok = resp is not None and resp.success and len(resp.response) >= (relay+1)
            status = None
            if ok:
                status = bool(resp.response[relay])
            if ok and status:
                elapsed_ms = (time.time() - ctx['start_time']) * 1000
                self.node.get_logger().info(f"[SEQ {seq_id}] ✅ {relay_name} ON verified at attempt {ctx['on_attempts']} ({elapsed_ms:.1f}ms)")
                ctx['phase'] = 'OFF'
                ctx['off_eval_done'] = False
                ctx['off_inner_read_count'] = 0
                self._flash_attempt_off()
            else:
                self.node.get_logger().warn(f"[SEQ {seq_id}] ⚠️ {relay_name} ON verify failed attempt {ctx['on_attempts']} (status={status})")
                if ctx['on_attempts'] < ctx['max']:
                    self._flash_attempt_on()
                else:
                    self._flash_fail(f"{relay_name} ON phase failed after {ctx['max']} attempts")
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] ON evaluate exception: {e}")
            if ctx['on_attempts'] < ctx['max']:
                self._flash_attempt_on()
            else:
                self._flash_fail(f"{relay_name} ON exception: {e}")

    def _flash_attempt_off(self):
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'OFF':
            return
        ctx['off_attempts'] += 1
        attempt = ctx['off_attempts']
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        relay_name = {3: 'STOP', 5: 'UP', 4: 'DOWN'}.get(relay, f'Relay{relay}')
        self.node.get_logger().info(f"[SEQ {seq_id}] OFF attempt {attempt}/{ctx['max']} for {relay_name}")
        ctx['off_eval_done'] = False
        ctx['off_inner_read_count'] = 0
        # Send OFF write then schedule read
        self.send(5, relay, [0x0000], seq_id=seq_id)
        threading.Timer(0.01, self._flash_read_off_result).start()

    def _flash_read_off_result(self):
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'OFF':
            return
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        req = ModbusRequestSrv.Request()
        req.slave_id = self.device_id
        req.function_code = 1
        req.address = 0x0000
        req.count = 6
        req.values = []
        req.seq_id = seq_id if seq_id is not None else 0
        future = self.cli.call_async(req)
        def first_done(f):
            ctx_local = self.flash_context
            if ctx_local is None or ctx_local.get('phase') != 'OFF':
                return
            if ctx_local['off_eval_done']:
                return
            ctx_local['off_eval_done'] = True
            self._flash_off_evaluate(f)
        future.add_done_callback(first_done)
        threading.Timer(0.002, self._flash_off_second_read, args=[future]).start()

    def _flash_off_second_read(self, first_future):
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'OFF':
            return
        if ctx['off_eval_done'] or first_future.done():
            return
        if ctx['off_inner_read_count'] >= 2:
            return
        ctx['off_inner_read_count'] += 1
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        req = ModbusRequestSrv.Request()
        req.slave_id = self.device_id
        req.function_code = 1
        req.address = 0x0000
        req.count = 6
        req.values = []
        req.seq_id = seq_id if seq_id is not None else 0
        second_future = self.cli.call_async(req)
        def second_done(f):
            ctx_local = self.flash_context
            if ctx_local is None or ctx_local.get('phase') != 'OFF':
                return
            if ctx_local['off_eval_done']:
                return
            ctx_local['off_eval_done'] = True
            self._flash_off_evaluate(f)
        second_future.add_done_callback(second_done)
        if ctx['off_inner_read_count'] < 2:
            threading.Timer(0.01, self._flash_off_second_read, args=[first_future]).start()

    def _flash_off_evaluate(self, future):
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'OFF':
            return
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        relay_name = {3: 'STOP', 5: 'UP', 4: 'DOWN'}.get(relay, f'Relay{relay}')
        try:
            resp = future.result()
            ok = resp is not None and resp.success and len(resp.response) >= (relay+1)
            status = None
            if ok:
                status = bool(resp.response[relay])
            if ok and (not status):
                elapsed_ms = (time.time() - ctx['start_time']) * 1000
                self.node.get_logger().info(f"[SEQ {seq_id}] ✅ {relay_name} OFF verified at attempt {ctx['off_attempts']} ({elapsed_ms:.1f}ms) total")
                self._flash_complete()
            else:
                self.node.get_logger().warn(f"[SEQ {seq_id}] ⚠️ {relay_name} OFF verify failed attempt {ctx['off_attempts']} (status={status})")
                if ctx['off_attempts'] < ctx['max']:
                    self._flash_attempt_off()
                else:
                    self._flash_fail(f"{relay_name} OFF phase failed after {ctx['max']} attempts")
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] OFF evaluate exception: {e}")
            if ctx['off_attempts'] < ctx['max']:
                self._flash_attempt_off()
            else:
                self._flash_fail(f"{relay_name} OFF exception: {e}")

    def _flash_complete(self):
        ctx = self.flash_context
        if ctx is None:
            return
        seq_id = ctx['seq_id']
        relay = ctx['relay']
        relay_name = {3: 'STOP', 5: 'UP', 4: 'DOWN'}.get(relay, f'Relay{relay}')
        total_ms = (time.time() - ctx['start_time']) * 1000
        self.node.get_logger().info(f"[SEQ {seq_id}] ✅ Async flash SUCCESS {relay_name} total={total_ms:.1f}ms")
        
        # Notify node that relay verification succeeded
        if hasattr(self, 'on_flash_complete_callback') and self.on_flash_complete_callback:
            try:
                self.on_flash_complete_callback(relay, seq_id)
            except Exception as e:
                self.node.get_logger().error(f"[SEQ {seq_id}] Flash complete callback error: {e}")
        
        self.flash_context = None
        self.flash_active = False

    def _flash_fail(self, reason):
        ctx = self.flash_context
        seq_id = ctx['seq_id'] if ctx else None
        self.node.get_logger().error(f"[SEQ {seq_id}] ❌ Async flash FAILED: {reason}")
        self.flash_context = None
        self.flash_active = False

    def cleanup(self):
        with self.timer_lock:
            self._cancel_stop_timer()
        self.node.get_logger().info("Pushrod controller cleanup complete")
