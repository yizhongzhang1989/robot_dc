from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *

class DrawWireSensorController(ModbusDevice):
    """Draw-wire sensor controller (formerly CableSensorController)."""
    def __init__(self, device_id, node, use_ack_patch):
        super().__init__(device_id, node, use_ack_patch)
        self.sensor_data = [0, 0]
        self.last_read_time = None
        # Error tracking
        self.error_count = 0
        self.last_error = None
        self.last_error_time = None
        self.consecutive_errors = 0
        # Sensor disable mechanism (after consecutive failures)
        self.sensor_disabled = False
        self.disable_threshold = 2  # Disable after 2 consecutive failures (prevent jitter)

    def initialize(self):
        self.node.get_logger().info("Draw-wire sensor initialized")

    def read_sensor_data(self, seq_id=None):
        try:
            # Removed info log to reduce output
            def handle_sensor_response(response):
                try:
                    # Check if response is None or empty (error indicators from base_device)
                    if response is None or not response or len(response) < 2:
                        self.error_count += 1
                        self.consecutive_errors += 1
                        self.last_error = "Modbus recv no response (timeout or hardware failure)"
                        self.last_error_time = __import__('time').time()
                        # Check if we should disable this sensor
                        if self.consecutive_errors >= self.disable_threshold:
                            if not self.sensor_disabled:
                                self.sensor_disabled = True
                                self.node.get_logger().error(
                                    f"[SEQ {seq_id}] 🔴 Sensor reached disable threshold ({self.consecutive_errors}/{self.disable_threshold}) - stopping Modbus commands"
                                )
                        self.node.get_logger().error(f"[SEQ {seq_id}] Sensor read error ({self.consecutive_errors}/{self.disable_threshold}): {self.last_error}")
                        return
                    
                    if response and len(response) >= 2:
                        self.sensor_data = response[:2]
                        import time
                        self.last_read_time = time.time()
                        # Reset error counters on success
                        self.consecutive_errors = 0
                        self.last_error = None
                        # Removed info log - data published via topic
                    else:
                        # Should not reach here after the check above
                        self.error_count += 1
                        self.consecutive_errors += 1
                        self.last_error = f"Read failed / invalid format: {response}"
                        self.last_error_time = __import__('time').time()
                        if self.consecutive_errors >= self.disable_threshold:
                            if not self.sensor_disabled:
                                self.sensor_disabled = True
                                self.node.get_logger().error(
                                    f"[SEQ {seq_id}] 🔴 Sensor reached disable threshold ({self.consecutive_errors}/{self.disable_threshold}) - stopping Modbus commands"
                                )
                        self.node.get_logger().error(f"[SEQ {seq_id}] {self.last_error}")
                except Exception as e:
                    self.error_count += 1
                    self.consecutive_errors += 1
                    self.last_error = f"Sensor response handler error: {e}"
                    self.last_error_time = __import__('time').time()
                    if self.consecutive_errors >= self.disable_threshold:
                        if not self.sensor_disabled:
                            self.sensor_disabled = True
                            self.node.get_logger().error(
                                f"[SEQ {seq_id}] 🔴 Sensor reached disable threshold ({self.consecutive_errors}/{self.disable_threshold}) - stopping Modbus commands"
                            )
                    self.node.get_logger().error(f"[SEQ {seq_id}] {self.last_error}")
            self.recv(3, 0x0000, 2, handle_sensor_response, seq_id=seq_id)
        except Exception as e:
            self.error_count += 1
            self.consecutive_errors += 1
            self.last_error = f"Read sensor data error: {e}"
            self.last_error_time = __import__('time').time()
            if self.consecutive_errors >= self.disable_threshold:
                if not self.sensor_disabled:
                    self.sensor_disabled = True
                    self.node.get_logger().error(
                        f"[SEQ {seq_id}] 🔴 Sensor reached disable threshold ({self.consecutive_errors}/{self.disable_threshold}) - stopping Modbus commands"
                    )
            self.node.get_logger().error(f"[SEQ {seq_id}] {self.last_error}")
            # Continue operation - sensor read failed but system should stay up

    def get_sensor_data(self):
        return self.sensor_data[0], self.sensor_data[1], self.last_read_time
    
    def get_error_status(self):
        """Get error status for reporting"""
        return {
            'error_count': self.error_count,
            'last_error': self.last_error,
            'last_error_time': self.last_error_time,
            'consecutive_errors': self.consecutive_errors,
        }

    def cleanup(self):
        self.node.get_logger().info("Draw-wire sensor cleanup done")
