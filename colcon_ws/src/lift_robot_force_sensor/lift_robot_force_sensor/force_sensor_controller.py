from modbus_devices.base_device import ModbusDevice
import time, struct

class ForceSensorController(ModbusDevice):
    """Single-channel force sensor controller (function code 0x03 read holding registers).

    New hardware: Only one force value register, address 0x0000 (decimal 0), length 1 (returns 1 16-bit register).
    Read frame example (device address assumed 0x34 = 52 decimal):
      Request: 34 03 00 00 00 01 CRC(lo) CRC(hi)
      Response: 34 03 02 HH LL CRC(lo) CRC(hi)
        02 = following byte count (2 bytes data); HH LL are 16-bit unsigned high/low bytes.
    Value conversion: value = (HH << 8) | LL  (unit N, no decimal places).
    """
    REG_FORCE = 0x0000

    def __init__(self, device_id, node, use_ack_patch=False):
        super().__init__(device_id, node, use_ack_patch)
        self.last_force_value = None
        self.last_force_reg = None
        self.last_force_ts = None

    def initialize(self):
        self.node.get_logger().info(f"Single force sensor controller initialized (device_id={self.device_id}) REG_FORCE@{self.REG_FORCE}")

    def _parse_int16(self, regs):
        if not regs or len(regs) < 1:
            return None
        try:
            raw_value = int(regs[0]) & 0xFFFF
            # Zero drift elimination: filter values > 65336 to 0
            # This handles sensor noise/drift when no force is applied
            if raw_value > 65336:
                return 0
            return raw_value
        except Exception:
            return None

    def read_force(self, seq_id=None):
        """异步读取单通道力值 (REG_FORCE)。"""
        def cb(resp):
            try:
                ts = time.time()
                if resp and len(resp) >= 1:
                    val = self._parse_int16(resp)
                    self.last_force_value = val
                    self.last_force_reg = resp[0]
                    self.last_force_ts = ts
                    self.node.get_logger().debug(f"[SEQ {seq_id}] FORCE reg: 0x{resp[0]:04X} value={val}N")
                else:
                    self.node.get_logger().warn(f"[SEQ {seq_id}] Invalid FORCE response: {resp}")
            except Exception as e:
                self.node.get_logger().error(f"[SEQ {seq_id}] Force callback error: {e}")
                # Don't propagate exception - keep node running
        
        # 读取 1 个保持寄存器 - wrap to catch Modbus errors
        try:
            self.recv(3, self.REG_FORCE, 1, callback=cb, seq_id=seq_id)
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] Force recv error: {e}")
            # Don't propagate - allow system to continue

    def get_last(self):
        # 为兼容旧接口，返回左右两路相同值
        return {
            'right_value': self.last_force_value,
            'left_value': self.last_force_value,
            'force_reg': self.last_force_reg,
            'right_ts': self.last_force_ts,
            'left_ts': self.last_force_ts,
        }

    def cleanup(self):
        self.node.get_logger().info("Single force sensor controller cleanup done")
