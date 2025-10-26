from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *
import threading
import time
from collections import deque

class LiftRobotController(ModbusDevice):
    """
    Lift platform controller - drives standard relay outputs via Modbus FC05 (write single coil)

    Relay mapping:
    - Relay 0: stop
    - Relay 1: up
    - Relay 2: down

    Communication:
    - Baudrate: 115200
    - Device ID: 50 (overridden by parameter at runtime)
    - Function code 0x05 to write single coil

    Pulse (flash) behavior:
    - Open relay (0xFF00) -> 100ms delay -> Close relay (0x0000)
    """
    
    def __init__(self, device_id, node, use_ack_patch):
        super().__init__(device_id, node, use_ack_patch)
        
        # Timer-based movement attributes
        self.active_timers = {}  # Dictionary to store active timers
        self.timer_lock = threading.Lock()  # Lock for thread safety
        
        # Command queue for timed operations
        self.timed_cmd_queue = deque()
        self.waiting_for_timed_ack = False

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

    def stop(self, seq_id=None):
        """Stop motion (pulse relay 0)."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Stop command (relay 0 pulse)")
        self.flash_relay(relay_address=0, duration_ms=100, seq_id=seq_id)

    def up(self, seq_id=None):
        """Move up (pulse relay 1). Pulse width is fixed, velocity is hardware-defined."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Up command (relay 1 pulse)")
        self.flash_relay(relay_address=1, duration_ms=100, seq_id=seq_id)

    def down(self, seq_id=None):
        """Move down (pulse relay 2). Pulse width is fixed, velocity is hardware-defined."""
        self.node.get_logger().info(f"[SEQ {seq_id}] Down command (relay 2 pulse)")
        self.flash_relay(relay_address=2, duration_ms=100, seq_id=seq_id)

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

    def flash_relay(self, relay_address, duration_ms=100, seq_id=None):
        """Pulse relay: open -> delay -> close.

        Args:
            relay_address: 0=stop,1=up,2=down
            duration_ms: pulse width in ms
            seq_id: sequence id
        """
        try:
            self.node.get_logger().info(
                f"[SEQ {seq_id}] Relay {relay_address} pulse: {duration_ms}ms"
            )
            self.open_relay(relay_address, seq_id=seq_id)
            flash_timer = threading.Timer(duration_ms / 1000.0, self.close_relay,
                                           args=[relay_address], kwargs={'seq_id': seq_id})
            flash_timer.start()
            with self.timer_lock:
                timer_name = f'flash_relay_{relay_address}'
                if timer_name in self.active_timers:
                    self.active_timers[timer_name].cancel()
                self.active_timers[timer_name] = flash_timer
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] Flash relay {relay_address} error: {e}")
            # Continue operation - this specific command failed but system should stay up

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
            # Auto-stop timer
            timer = threading.Timer(duration, self.stop)
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
            # Auto-stop timer
            timer = threading.Timer(duration, self.stop)
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
