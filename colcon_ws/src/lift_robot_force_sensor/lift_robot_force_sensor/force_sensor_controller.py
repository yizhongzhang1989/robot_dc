from modbus_devices.base_device import ModbusDevice
import time, struct

class ForceSensorController(ModbusDevice):
    """Controller for dual-channel force sensor (CH2 & CH3) on Modbus.

    Requirement:
      - Read CH2 at holding register address 202 (0x00CA), length 2 (32-bit float)
      - Read CH3 at holding register address 204 (0x00CC), length 2 (32-bit float)
    Function code: 0x03 (Read Holding Registers)

    Parsing:
      Each channel returns two 16-bit registers (high word first) representing a
      big-endian IEEE-754 float. Example:
        resp[0] = high 16 bits
        resp[1] = low 16 bits
        value = struct.unpack('!f', bytes.fromhex(f"{resp[0]:04X}{resp[1]:04X}"))[0]
    """
    REG_CH2 = 202  # 0x00CA
    REG_CH3 = 204  # 0x00CC

    def __init__(self, device_id, node, use_ack_patch=False):
        super().__init__(device_id, node, use_ack_patch)
        self.last_right_value = None  # CH2 -> right
        self.last_left_value = None   # CH3 -> left
        self.last_right_regs = (None, None)
        self.last_left_regs = (None, None)
        self.last_right_ts = None
        self.last_left_ts = None

    def initialize(self):
        self.node.get_logger().info(f"Force sensor controller initialized (device_id={self.device_id}) CH2@{self.REG_CH2} CH3@{self.REG_CH3}")

    def _parse_float(self, regs):
        if not regs or len(regs) < 2:
            return None
        try:
            # Combine as big-endian high word then low word
            packed = struct.pack('!HH', regs[0], regs[1])
            return struct.unpack('!f', packed)[0]
        except Exception:
            return None

    def read_ch2_ch3(self, seq_id=None):
        """Issue two asynchronous read requests for CH2 and CH3."""
        # CH2
        def cb_ch2(resp):
            ts = time.time()
            if resp and len(resp) >= 2:
                val = self._parse_float(resp)
                self.last_right_value = val
                self.last_right_regs = (resp[0], resp[1])
                self.last_right_ts = ts
                self.node.get_logger().debug(f"[SEQ {seq_id}] CH2(right) regs: 0x{resp[0]:04X} 0x{resp[1]:04X} value={val}")
            else:
                self.node.get_logger().warn(f"[SEQ {seq_id}] Invalid CH2 response: {resp}")
        self.recv(3, self.REG_CH2, 2, callback=cb_ch2, seq_id=seq_id)

        # CH3
        def cb_ch3(resp):
            ts = time.time()
            if resp and len(resp) >= 2:
                val = self._parse_float(resp)
                self.last_left_value = val
                self.last_left_regs = (resp[0], resp[1])
                self.last_left_ts = ts
                self.node.get_logger().debug(f"[SEQ {seq_id}] CH3(left) regs: 0x{resp[0]:04X} 0x{resp[1]:04X} value={val}")
            else:
                self.node.get_logger().warn(f"[SEQ {seq_id}] Invalid CH3 response: {resp}")
        self.recv(3, self.REG_CH3, 2, callback=cb_ch3, seq_id=seq_id)

    def get_last(self):
        return {
            'right_value': self.last_right_value,
            'left_value': self.last_left_value,
            'right_regs': self.last_right_regs,
            'left_regs': self.last_left_regs,
            'right_ts': self.last_right_ts,
            'left_ts': self.last_left_ts,
        }

    def cleanup(self):
        self.node.get_logger().info("Force sensor controller cleanup done")
