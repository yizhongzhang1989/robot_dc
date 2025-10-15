import threading
import json
from collections import deque
from modbus_devices.base_device import ModbusDevice

class PushrodController(ModbusDevice):
    """
    Pushrod controller using two relay channels (CH5 and CH6).

    Logic:
    - CH5=1, CH6=0 -> up
    - CH5=0, CH6=1 -> down
    - CH5=0, CH6=0 or CH5=1, CH6=1 -> stop

    Implementation writes single coils (FC05): 0xFF00=ON, 0x0000=OFF.

    Timed operations use threading.Timer similar to lift_robot_platform.
    """
    def __init__(self, device_id, node, use_ack_patch=True):
        super().__init__(device_id, node, use_ack_patch)
        self.timer_lock = threading.Lock()
        self.active_timers = {}
        self.timed_cmd_queue = deque()
        # Relay addresses: platform used 0,1,2 for CH1-CH3 so CH5=coil 4, CH6=coil 5
        self.CH5_ADDR = 4
        self.CH6_ADDR = 5

    def initialize(self):
        self.node.get_logger().info("Pushrod controller initialized (no reset sequence).")

    def _write_relay(self, relay_addr, on, seq_id=None):
        value = 0xFF00 if on else 0x0000
        self.send(5, relay_addr, [value], seq_id=seq_id)

    def _apply_state(self, ch5_on, ch6_on, seq_id=None):
        self.node.get_logger().info(f"[SEQ {seq_id}] Set CH5={int(ch5_on)} CH6={int(ch6_on)}")
        self._write_relay(self.CH5_ADDR, ch5_on, seq_id=seq_id)
        self._write_relay(self.CH6_ADDR, ch6_on, seq_id=seq_id)

    def stop(self, seq_id=None):
        self.node.get_logger().info(f"[SEQ {seq_id}] Stop pushrod")
        self._apply_state(False, False, seq_id=seq_id)

    def up(self, seq_id=None):
        self.node.get_logger().info(f"[SEQ {seq_id}] Pushrod up")
        self._apply_state(True, False, seq_id=seq_id)

    def down(self, seq_id=None):
        self.node.get_logger().info(f"[SEQ {seq_id}] Pushrod down")
        self._apply_state(False, True, seq_id=seq_id)

    def timed_up(self, duration, seq_id=None):
        with self.timer_lock:
            self.cancel_all_timers()
            self.up(seq_id=seq_id)
            timer = threading.Timer(duration, self.stop)
            timer.start()
            self.active_timers['timed_up'] = timer
            self.node.get_logger().info(f"[SEQ {seq_id}] Timed up {duration}s")

    def timed_down(self, duration, seq_id=None):
        with self.timer_lock:
            self.cancel_all_timers()
            self.down(seq_id=seq_id)
            timer = threading.Timer(duration, self.stop)
            timer.start()
            self.active_timers['timed_down'] = timer
            self.node.get_logger().info(f"[SEQ {seq_id}] Timed down {duration}s")

    def stop_timed(self, seq_id=None):
        with self.timer_lock:
            self.node.get_logger().info(f"[SEQ {seq_id}] Stop timed pushrod motions")
            self.cancel_all_timers()
            self.stop(seq_id=seq_id)

    def cancel_all_timers(self):
        for name, timer in self.active_timers.items():
            if timer.is_alive():
                timer.cancel()
                self.node.get_logger().info(f"Cancelled timer: {name}")
        self.active_timers.clear()

    def cleanup(self):
        with self.timer_lock:
            self.cancel_all_timers()
            self.stop()
            self.node.get_logger().info("Pushrod controller cleanup complete")
