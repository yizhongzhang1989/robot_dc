from modbus_devices.base_device import ModbusDevice
import time, struct

class ForceSensorController(ModbusDevice):
    """Single-channel force sensor controller (function code 0x03 read holding registers).

    Hardware: Bipolar force sensor (supports both compression and tension)
    - Register address: 0x0000 (40001)
    - Data format: Signed 16-bit two's complement
    - Range: -32768 to +32767 (bipolar mode)
    - Positive values: Compression force
    - Negative values: Tension force
    
    Read frame example (device address 52 = 0x34):
      Request: 34 03 00 00 00 01 CRC(lo) CRC(hi)
      Response: 34 03 02 HH LL CRC(lo) CRC(hi)
        02 = byte count (2 bytes data)
        HH LL = 16-bit signed value in two's complement form
    
    Value conversion examples:
      - 0x0014 (20) → +20 N (compression)
      - 0xFFEC (65516) → -20 N (tension, two's complement)
      - 0x7FFF (32767) → +32767 N (max compression)
      - 0x8000 (32768) → -32768 N (max tension)
    """
    REG_FORCE = 0x0000

    def __init__(self, device_id, node, use_ack_patch=False):
        super().__init__(device_id, node, use_ack_patch)
        self.last_force_value = None
        self.last_force_reg = None
        self.last_force_ts = None
        # Error tracking
        self.error_count = 0
        self.last_error = None
        self.last_error_time = None
        self.consecutive_errors = 0
        # Sensor disable mechanism (after consecutive failures)
        self.sensor_disabled = False
        self.disable_threshold = 2  # Disable after 2 consecutive failures (prevent jitter)

    def initialize(self):
        self.node.get_logger().info(
            f"Bipolar force sensor controller initialized (device_id={self.device_id}) "
            f"REG_FORCE@{self.REG_FORCE}, mode=signed 16-bit (bipolar)"
        )

    def _parse_int16(self, regs):
        """Parse register as signed 16-bit integer (two's complement for bipolar sensor)
        
        Converts unsigned 16-bit Modbus register to signed value:
        - 0x0000 to 0x7FFF (0 to 32767) → positive (compression)
        - 0x8000 to 0xFFFF (32768 to 65535) → negative (tension)
        
        Examples:
        - 65516 (0xFFEC) → -20 N (tension)
        - 20 (0x0014) → +20 N (compression)
        - 0 (0x0000) → 0 N (no force)
        """
        if not regs or len(regs) < 1:
            return None
        try:
            # Read as unsigned 16-bit first
            unsigned_val = int(regs[0]) & 0xFFFF
            
            # Convert to signed 16-bit using two's complement
            # If value >= 0x8000 (32768), it's negative
            if unsigned_val >= 0x8000:
                return unsigned_val - 0x10000  # Convert to negative (65536 - unsigned_val)
            else:
                return unsigned_val  # Already positive
        except Exception:
            return None

    def read_force(self, seq_id=None):
        """异步读取单通道力值 (REG_FORCE)。"""
        def cb(resp):
            try:
                # Check if response is None or empty (error indicators from base_device)
                if resp is None or not resp or len(resp) == 0:
                    self.error_count += 1
                    self.consecutive_errors += 1
                    self.last_error = "Modbus recv no response (timeout or hardware failure)"
                    self.last_error_time = time.time()
                    # Check if we should disable this sensor
                    if self.consecutive_errors >= self.disable_threshold:
                        if not self.sensor_disabled:
                            self.sensor_disabled = True
                            self.node.get_logger().error(
                                f"[SEQ {seq_id}] 🔴 Sensor reached disable threshold ({self.consecutive_errors}/{self.disable_threshold}) - stopping Modbus commands"
                            )
                    self.node.get_logger().error(f"[SEQ {seq_id}] FORCE read error ({self.consecutive_errors}/{self.disable_threshold}): {self.last_error}")
                    return
                
                ts = time.time()
                if resp and len(resp) >= 1:
                    val = self._parse_int16(resp)
                    self.last_force_value = val
                    self.last_force_reg = resp[0]
                    self.last_force_ts = ts
                    # Reset error counters on success
                    self.consecutive_errors = 0
                    self.last_error = None
                    self.node.get_logger().debug(f"[SEQ {seq_id}] FORCE reg: 0x{resp[0]:04X} value={val}N")
                else:
                    # Should not reach here after the check above, but keep for safety
                    self.error_count += 1
                    self.consecutive_errors += 1
                    self.last_error = f"Invalid FORCE response: {resp}"
                    self.last_error_time = ts
                    if self.consecutive_errors >= self.disable_threshold:
                        if not self.sensor_disabled:
                            self.sensor_disabled = True
                            self.node.get_logger().error(
                                f"[SEQ {seq_id}] 🔴 Sensor reached disable threshold ({self.consecutive_errors}/{self.disable_threshold}) - stopping Modbus commands"
                            )
                    self.node.get_logger().warn(f"[SEQ {seq_id}] {self.last_error}")
            except Exception as e:
                self.error_count += 1
                self.consecutive_errors += 1
                self.last_error = f"Force callback error: {e}"
                self.last_error_time = time.time()
                if self.consecutive_errors >= self.disable_threshold:
                    if not self.sensor_disabled:
                        self.sensor_disabled = True
                        self.node.get_logger().error(
                            f"[SEQ {seq_id}] 🔴 Sensor reached disable threshold ({self.consecutive_errors}/{self.disable_threshold}) - stopping Modbus commands"
                        )
                self.node.get_logger().error(f"[SEQ {seq_id}] {self.last_error}")
                # Don't propagate exception - keep node running
        
        # Read 1 holding register - wrap to catch Modbus errors
        try:
            self.recv(3, self.REG_FORCE, 1, callback=cb, seq_id=seq_id)
        except Exception as e:
            self.error_count += 1
            self.consecutive_errors += 1
            self.last_error = f"Force recv error: {e}"
            self.last_error_time = time.time()
            if self.consecutive_errors >= self.disable_threshold:
                if not self.sensor_disabled:
                    self.sensor_disabled = True
                    self.node.get_logger().error(
                        f"[SEQ {seq_id}] 🔴 Sensor reached disable threshold ({self.consecutive_errors}/{self.disable_threshold}) - stopping Modbus commands"
                    )
            self.node.get_logger().error(f"[SEQ {seq_id}] {self.last_error}")
            # Don't propagate - allow system to continue

    def get_last(self):
        # For backward compatibility, return same value for both channels
        return {
            'right_value': self.last_force_value,
            'left_value': self.last_force_value,
            'force_reg': self.last_force_reg,
            'right_ts': self.last_force_ts,
            'left_ts': self.last_force_ts,
            'error_count': self.error_count,
            'last_error': self.last_error,
            'last_error_time': self.last_error_time,
            'consecutive_errors': self.consecutive_errors,
        }

    def cleanup(self):
        self.node.get_logger().info("Single force sensor controller cleanup done")
