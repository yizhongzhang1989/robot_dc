from modbus_devices.base_device import ModbusDevice
import time, struct

class ForceSensorController(ModbusDevice):
    """Controller for single-channel force sensor (CH2 only) on Modbus.

    Requirement:
      - Read CH2 at holding register address 202 (0x00CA), length 2 (32-bit float)
        Raw Modbus frame (hex, before CRC): 3C 03 00 CA 00 02
        (Slave=0x3C=60, FC=0x03, start=0x00CA, length=0x0002)
    Function code: 0x03 (Read Holding Registers)

    Parsing:
      Channel returns two 16-bit registers (high word first) representing a
      big-endian IEEE-754 float.
    """
    REG_CH2 = 202  # 0x00CA

    def __init__(self, device_id, node, use_ack_patch=False):
        super().__init__(device_id, node, use_ack_patch)
        self.last_value = None  # CH2 value
        self.last_regs = (None, None)
        self.last_ts = None

    def initialize(self):
        self.node.get_logger().info(f"Force sensor controller initialized (device_id={self.device_id}) CH2@{self.REG_CH2}")

    def _parse_float(self, regs):
        if not regs or len(regs) < 2:
            return None
        try:
            # Combine as big-endian high word then low word
            packed = struct.pack('!HH', regs[0], regs[1])
            return struct.unpack('!f', packed)[0]
        except Exception:
            return None

    def read_ch2(self, seq_id=None):
        """Issue asynchronous read request for CH2 only.
        
        The callback function updates self.last_value when the Modbus response arrives.
        Node's periodic_read() publishes the last_value from previous cycle while
        issuing new read for current cycle.
        """
        def cb(resp):
            ts = time.time()
            if resp and len(resp) >= 2:
                val = self._parse_float(resp)
                # Update last_value for next cycle's publication
                self.last_value = val
                self.last_regs = (resp[0], resp[1])
                self.last_ts = ts
                self.node.get_logger().debug(f"[SEQ {seq_id}] CH2 regs: 0x{resp[0]:04X} 0x{resp[1]:04X} value={val}")
            else:
                self.node.get_logger().warn(f"[SEQ {seq_id}] Invalid CH2 response: {resp}")
        self.recv(3, self.REG_CH2, 2, callback=cb, seq_id=seq_id)

    def get_last(self):
        return {
            'value': self.last_value,
            'regs': self.last_regs,
            'ts': self.last_ts,
        }

    def cleanup(self):
        self.node.get_logger().info("Force sensor controller cleanup done (single channel)")