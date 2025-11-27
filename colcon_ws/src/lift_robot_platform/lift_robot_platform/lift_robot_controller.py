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
    
    def __init__(self, device_id, node, use_ack_patch):
        super().__init__(device_id, node, use_ack_patch)
        
        # Relay address constants
        # Platform relays
        self.RELAY_PLATFORM_STOP = 0
        self.RELAY_PLATFORM_UP = 1
        self.RELAY_PLATFORM_DOWN = 2
        # Pushrod relays
        self.RELAY_PUSHROD_STOP = 3
        self.RELAY_PUSHROD_DOWN = 4
        self.RELAY_PUSHROD_UP = 5

    @staticmethod
    def _wrap_response_as_future(response):
        """包装recv()回调响应为Future兼容对象,用于evaluate函数."""
        class FakeFuture:
            def __init__(self, resp):
                self.resp = resp
            def result(self):
                class FakeResult:
                    def __init__(self, r):
                        self.success = r is not None and len(r) > 0
                        self.response = r if r else []
                return FakeResult(self.resp)
        return FakeFuture(response)
        
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
    # Async flash (pulse) state machine WITHOUT blocking spin
    # Uses service call futures + timers to chain ON/OFF attempts.
    # ─────────────────────────────────────────────────────────────
    def start_flash_async(self, relay_address, seq_id=None, max_attempts=3):
        """Begin an asynchronous relay pulse with immediate verification & retry.

        Non-blocking: scheduling is done via futures and threading.Timer so we never
        spin or sleep inside a ROS callback thread. Only one flash is allowed at a time.
        If another flash is active, the new request is ignored.
        
        Returns:
            bool: True if flash started, False if rejected due to conflict
        """
        with getattr(self, 'flash_lock', threading.Lock()):
            if getattr(self, 'flash_active', False):
                self.node.get_logger().warn(f"[SEQ {seq_id}] Flash already active, ignore new request relay={relay_address}")
                return False
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
                # Micro-timeout flags (ON phase)
                'on_eval_done': False,
                'on_inner_read_count': 0,  # number of extra micro-timeout reads issued (excluding primary)
                # Micro-timeout flags (OFF phase)
                'off_eval_done': False,
                'off_inner_read_count': 0
            }
        relay_name = {
            0: 'PLATFORM_STOP', 
            1: 'PLATFORM_UP', 
            2: 'PLATFORM_DOWN',
            3: 'PUSHROD_STOP',
            4: 'PUSHROD_DOWN',
            5: 'PUSHROD_UP'
        }.get(relay_address, f'Relay{relay_address}')
        self.node.get_logger().info(f"[SEQ {seq_id}] Async flash start: {relay_name} (max_attempts={max_attempts})")
        self._try_on()
        return True

    def _try_on(self):
        """尝试打开继电器: 发送ON命令 → 10ms后验证."""
        ctx = self.flash_context
        if ctx is None:
            return
        ctx['on_attempts'] += 1
        attempt = ctx['on_attempts']
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        relay_name = self._get_relay_name(relay)
        self.node.get_logger().info(f"[SEQ {seq_id}] ON attempt {attempt}/{ctx['max']} for {relay_name}")
        # 重置验证标志
        ctx['on_eval_done'] = False
        ctx['on_inner_read_count'] = 0
        # 发送ON命令,10ms后验证
        try: 
            self.send(5, relay, [0xFF00], seq_id=seq_id)
        except Exception as e:
            self.node.get_logger().error(f"[***SEQ {seq_id}] Exception during send ON: {e}")
        threading.Timer(0.01, self._verify_on).start()

    def _verify_on(self):
        """验证ON状态: 主读取 + 补充读取(最多2次) + 超时保护."""
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'ON':
            return
        
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        done = {'flag': False}  # 主读取完成标志
        
        def on_response(response):
            if self.flash_context is None or self.flash_context.get('phase') != 'ON':
                return
            if self.flash_context['on_eval_done']:
                return
            self.flash_context['on_eval_done'] = True
            done['flag'] = True
            self._check_on(self._wrap_response_as_future(response))
        
        # 主读取: FC01读取6个继电器
        try:
            self.recv(1, 0x0000, 6, callback=on_response, seq_id=seq_id)
        except Exception as e:
            self.node.get_logger().error(f"[***SEQ {seq_id}] Exception during recv ON: {e}")
        
        # 10ms后启动补充读取链
        threading.Timer(0.01, self._retry_verify_on, args=[done]).start()
        
        # 100ms验证超时: 如果所有读取都没返回,强制判定为失败并重试
        def on_verify_timeout():
            if self.flash_context is None or self.flash_context.get('phase') != 'ON':
                return
            if self.flash_context['on_eval_done']:
                return  # 已经有回调返回了
            
            # 所有读取都超时,强制标记为已处理并判定失败
            self.flash_context['on_eval_done'] = True
            relay_name = self._get_relay_name(relay)
            self.node.get_logger().warn(
                f"[SEQ {seq_id}] ⏱️  {relay_name} ON verification TIMEOUT (100ms) - "
                f"no response from FC01, attempt {ctx['on_attempts']}/{ctx['max']}"
            )
            
            # 创建失败的future并调用check
            class TimeoutFuture:
                def result(self):
                    class TimeoutResult:
                        success = False
                        response = []
                    return TimeoutResult()
            
            self._check_on(TimeoutFuture())
        
        threading.Timer(0.1, on_verify_timeout).start()

    def _retry_verify_on(self, done):
        """补充读取ON: 主读取未完成时重试(递归最多2次)."""
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'ON':
            return
        if ctx['on_eval_done'] or done['flag']:
            return  # 已完成
        if ctx['on_inner_read_count'] >= 2:
            return  # 超过限制
        
        ctx['on_inner_read_count'] += 1
        
        def on_retry_response(response):
            if self.flash_context is None or self.flash_context.get('phase') != 'ON':
                return
            if self.flash_context['on_eval_done']:
                return
            self.flash_context['on_eval_done'] = True
            self._check_on(self._wrap_response_as_future(response))
        
        # 补充读取: FC01读取3个继电器
        self.recv(1, 0x0000, 3, callback=on_retry_response, seq_id=ctx['seq_id'])
        # 10ms后递归
        if ctx['on_inner_read_count'] < 2:
            threading.Timer(0.01, self._retry_verify_on, args=[done]).start()

    def _check_on(self, future):
        """检查ON验证结果: 成功→进入OFF阶段, 失败→重试."""
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'ON':
            return
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        relay_name = self._get_relay_name(relay)
        try:
            resp = future.result()
            ok = resp is not None and resp.success and len(resp.response) >= (relay+1)
            status = None
            if ok:
                status = bool(resp.response[relay])
            if ok and status:
                elapsed_ms = (time.time() - ctx['start_time']) * 1000
                self.node.get_logger().info(f"[SEQ {seq_id}] ✅ {relay_name} ON verified at attempt {ctx['on_attempts']} ({elapsed_ms:.1f}ms)")
                # Move to OFF phase
                ctx['phase'] = 'OFF'
                # Reset OFF phase micro-timeout flags
                ctx['off_eval_done'] = False
                ctx['off_inner_read_count'] = 0
                self._try_off()
            else:
                self.node.get_logger().warn(f"[SEQ {seq_id}] ⚠️ {relay_name} ON verify failed attempt {ctx['on_attempts']} (status={status})")
                if ctx['on_attempts'] < ctx['max']:
                    self._try_on()
                else:
                    self._flash_fail(f"{relay_name} ON phase failed after {ctx['max']} attempts")
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] ON evaluate exception: {e}")
            if ctx['on_attempts'] < ctx['max']:
                self._try_on()
            else:
                self._flash_fail(f"{relay_name} ON exception: {e}")

    def _try_off(self):
        """尝试关闭继电器: 发送OFF命令 → 10ms后验证."""
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'OFF':
            return
        ctx['off_attempts'] += 1
        attempt = ctx['off_attempts']
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        relay_name = self._get_relay_name(relay)
        self.node.get_logger().info(f"[SEQ {seq_id}] OFF attempt {attempt}/{ctx['max']} for {relay_name}")
        ctx['off_eval_done'] = False
        ctx['off_inner_read_count'] = 0
        self.send(5, relay, [0x0000], seq_id=seq_id)
        threading.Timer(0.01, self._verify_off).start()

    def _verify_off(self):
        """验证OFF状态: 主读取 + 补充读取(镜像ON) + 超时保护."""
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'OFF':
            return
        
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        done = {'flag': False}
        
        def off_response(response):
            if self.flash_context is None or self.flash_context.get('phase') != 'OFF':
                return
            if self.flash_context['off_eval_done']:
                return
            self.flash_context['off_eval_done'] = True
            done['flag'] = True
            self._check_off(self._wrap_response_as_future(response))
        
        self.recv(1, 0x0000, 6, callback=off_response, seq_id=seq_id)
        
        # 10ms后启动补充读取链
        threading.Timer(0.01, self._retry_verify_off, args=[done]).start()
        
        # 100ms验证超时: 如果所有读取都没返回,强制判定为失败并重试
        def off_verify_timeout():
            if self.flash_context is None or self.flash_context.get('phase') != 'OFF':
                return
            if self.flash_context['off_eval_done']:
                return  # 已经有回调返回了
            
            # 所有读取都超时,强制标记为已处理并判定失败
            self.flash_context['off_eval_done'] = True
            relay_name = self._get_relay_name(relay)
            self.node.get_logger().warn(
                f"[SEQ {seq_id}] ⏱️  {relay_name} OFF verification TIMEOUT (100ms) - "
                f"no response from FC01, attempt {ctx['off_attempts']}/{ctx['max']}"
            )
            
            # 创建失败的future并调用check
            class TimeoutFuture:
                def result(self):
                    class TimeoutResult:
                        success = False
                        response = []
                    return TimeoutResult()
            
            self._check_off(TimeoutFuture())
        
        threading.Timer(0.1, off_verify_timeout).start()

    def _retry_verify_off(self, done):
        """补充读取OFF: 主读取未完成时重试(递归最多2次)."""
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'OFF':
            return
        if ctx['off_eval_done'] or done['flag']:
            return
        if ctx['off_inner_read_count'] >= 2:
            return
        
        ctx['off_inner_read_count'] += 1
        
        def off_retry_response(response):
            if self.flash_context is None or self.flash_context.get('phase') != 'OFF':
                return
            if self.flash_context['off_eval_done']:
                return
            self.flash_context['off_eval_done'] = True
            self._check_off(self._wrap_response_as_future(response))
        
        self.recv(1, 0x0000, 6, callback=off_retry_response, seq_id=ctx['seq_id'])
        if ctx['off_inner_read_count'] < 2:
            threading.Timer(0.01, self._retry_verify_off, args=[done]).start()

    def _check_off(self, future):
        """检查OFF验证结果: 成功→完成, 失败→重试."""
        ctx = self.flash_context
        if ctx is None or ctx.get('phase') != 'OFF':
            return
        relay = ctx['relay']
        seq_id = ctx['seq_id']
        relay_name = self._get_relay_name(relay)
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
                    self._try_off()
                else:
                    self._flash_fail(f"{relay_name} OFF phase failed after {ctx['max']} attempts")
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] OFF evaluate exception: {e}")
            if ctx['off_attempts'] < ctx['max']:
                self._try_off()
            else:
                self._flash_fail(f"{relay_name} OFF exception: {e}")

    def _flash_complete(self):
        ctx = self.flash_context
        if ctx is None:
            return
        seq_id = ctx['seq_id']
        relay = ctx['relay']
        relay_name = self._get_relay_name(relay)
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
        relay_addr = ctx['relay'] if ctx else -1
        
        # Check if we're already in emergency reset (to avoid infinite loop)
        # Emergency reset uses relay 0 (platform stop) and relay 3 (pushrod stop)
        is_reset_flash = relay_addr in [0, 3] and getattr(self.node, 'reset_in_progress', False)
        
        if is_reset_flash:
            # Flash failure during emergency reset - do NOT trigger another reset
            self.node.get_logger().error(
                f"[SEQ {seq_id}] ❌ STOP flash FAILED during emergency reset: {reason} - "
                f"Relay cleared by reset_all_relays, system will continue to idle state"
            )
        else:
            # Normal flash failure - trigger emergency reset
            self.node.get_logger().error(f"[SEQ {seq_id}] ❌ Async flash FAILED: {reason} -> EMERGENCY RESET")
        
        # Trigger emergency reset only if not already in reset
        if not is_reset_flash:
            try:
                self._trigger_emergency_reset(seq_id, reason)
            except Exception as e:
                self.node.get_logger().error(f"[SEQ {seq_id}] Emergency reset invocation error: {e}")
        
        self.flash_context = None
        self.flash_active = False

    # Legacy sync method retained for compatibility (not used now)
    def flash_relay(self, relay_address, duration_ms=100, seq_id=None):
        self.node.get_logger().warn("flash_relay deprecated - using start_flash_async instead")
        self.start_flash_async(relay_address=relay_address, seq_id=seq_id)

    # write_relay_verified removed in async mode (verification handled by state machine)

    def flash_relay(self, relay_address, duration_ms=100, seq_id=None):
        """Pulse relay with immediate retry detection: ON -> verify -> OFF -> verify.
        
        Uses immediate detection with max 3 retries for both ON and OFF phases.
        If all 3 attempts fail, triggers emergency reset.

        Args:
            relay_address: 0=stop,1=up,2=down
            duration_ms: deprecated (kept for compatibility, not used)
            seq_id: sequence id
            
        Returns:
            bool: True if flash successful, False if failed and emergency reset triggered
        """
        relay_name = {0: 'STOP', 1: 'UP', 2: 'DOWN'}.get(relay_address, f'Relay{relay_address}')
        
        try:
            self.node.get_logger().info(
                f"[SEQ {seq_id}] {relay_name} relay flash with retry detection"
            )
            
            start_time = time.time()
            max_attempts = 3
            
            # ─────────────────────────────────────────────────────────
            # Phase 1: Turn ON relay (with retry)
            # ─────────────────────────────────────────────────────────
            relay_turned_on = False
            
            for attempt in range(1, max_attempts + 1):
                if self.write_relay_verified(relay_address, 0xFF00):
                    elapsed_ms = (time.time() - start_time) * 1000
                    self.node.get_logger().info(
                        f"[SEQ {seq_id}] ✅ {relay_name} ON verified (attempt {attempt}/{max_attempts}, {elapsed_ms:.1f}ms)"
                    )
                    relay_turned_on = True
                    break
                else:
                    self.node.get_logger().warn(
                        f"[SEQ {seq_id}] ⚠️  {relay_name} ON failed (attempt {attempt}/{max_attempts}), retrying..."
                    )
            
            if not relay_turned_on:
                total_ms = (time.time() - start_time) * 1000
                self.node.get_logger().error(
                    f"[SEQ {seq_id}] ❌ {relay_name} failed to turn ON after {max_attempts} attempts ({total_ms:.1f}ms)"
                )
                self._trigger_emergency_reset(seq_id, f"{relay_name} relay ON failure")
                return False
            
            # ─────────────────────────────────────────────────────────
            # Phase 2: Turn OFF relay (with retry)
            # ─────────────────────────────────────────────────────────
            relay_turned_off = False
            
            for attempt in range(1, max_attempts + 1):
                if self.write_relay_verified(relay_address, 0x0000):
                    elapsed_ms = (time.time() - start_time) * 1000
                    self.node.get_logger().info(
                        f"[SEQ {seq_id}] ✅ {relay_name} OFF verified (attempt {attempt}/{max_attempts}, {elapsed_ms:.1f}ms)"
                    )
                    relay_turned_off = True
                    break
                else:
                    self.node.get_logger().warn(
                        f"[SEQ {seq_id}] ⚠️  {relay_name} OFF failed (attempt {attempt}/{max_attempts}), retrying..."
                    )
            
            total_ms = (time.time() - start_time) * 1000
            
            if not relay_turned_off:
                self.node.get_logger().error(
                    f"[SEQ {seq_id}] ❌ {relay_name} failed to turn OFF after {max_attempts} attempts ({total_ms:.1f}ms)"
                )
                self._trigger_emergency_reset(seq_id, f"{relay_name} relay OFF failure")
                return False
            
            self.node.get_logger().info(
                f"[SEQ {seq_id}] ✅ {relay_name} flash completed successfully ({total_ms:.1f}ms)"
            )
            return True
            
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] ❌ Flash relay {relay_address} exception: {e}")
            self._trigger_emergency_reset(seq_id, f"{relay_name} relay exception: {e}")
            return False

    def _trigger_emergency_reset(self, seq_id, reason):
        """Trigger emergency reset - set state to emergency_reset for temporary display.
        
        Args:
            seq_id: sequence id for logging
            reason: reason for emergency reset
        """
        self.node.get_logger().error(
            f"[SEQ {seq_id}] 🔴 EMERGENCY RESET TRIGGERED - Reason: {reason}"
        )
        
        try:
            # Signal node to perform emergency reset
            with self.node.control_lock:
                self.node.reset_in_progress = True
                self.node.control_enabled = False
                self.node.force_control_active = False
                # Set emergency_reset state (flash failures during reset won't block new Actions)
                self.node.task_state = 'emergency_reset'
                self.node.completion_reason = f'relay_failure: {reason}'
            
            # Cancel all timers
            self.cancel_all_timers()
            
            # Reset all relays
            self.reset_all_relays(seq_id=seq_id)
            
            # Send physical STOP pulse
            # Note: This might also fail, but we try anyway
            try:
                self.open_relay(0, seq_id=seq_id)  # Relay 0 ON
                time.sleep(0.1)
                self.close_relay(0, seq_id=seq_id)  # Relay 0 OFF
            except Exception as e:
                self.node.get_logger().error(f"[SEQ {seq_id}] ❌ Emergency STOP pulse failed: {e}")
            
            # Clear reset flag
            with self.node.control_lock:
                self.node.reset_in_progress = False
                self.node.system_busy = False
                self.node.active_control_owner = None
            
            self.node.get_logger().warn(
                f"[SEQ {seq_id}] 🔴 Emergency reset complete - System in emergency_stop state"
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
