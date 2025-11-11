from modbus_devices.base_device import ModbusDevice
import time

class ForceSensorController(ModbusDevice):
    """单通道力传感器控制器（功能码 0x03 读保持寄存器）。

    设备：第二个力传感器 (device_id=53)。只读取一个力值寄存器 0x0000。
    应答帧格式:
      请求: <ID> 03 00 00 00 01 CRC(lo) CRC(hi)
      应答: <ID> 03 02 HH LL CRC(lo) CRC(hi)
    数值: value = (HH << 8) | LL  (单位 N)
    """
    REG_FORCE = 0x0000

    def __init__(self, device_id, node, use_ack_patch=False):
        super().__init__(device_id, node, use_ack_patch)
        self.last_force_value = None
        self.last_force_reg = None
        self.last_force_ts = None

    def initialize(self):
        self.node.get_logger().info(f"[Sensor2] controller initialized (device_id={self.device_id}) REG_FORCE@{self.REG_FORCE}")

    def _parse_int16(self, regs):
        if not regs or len(regs) < 1:
            return None
        try:
            return int(regs[0]) & 0xFFFF
        except Exception:
            return None

    def read_force(self, seq_id=None):
        def cb(resp):
            ts = time.time()
            if resp and len(resp) >= 1:
                val = self._parse_int16(resp)
                self.last_force_value = val
                self.last_force_reg = resp[0]
                self.last_force_ts = ts
                self.node.get_logger().debug(f"[Sensor2 SEQ {seq_id}] FORCE reg: 0x{resp[0]:04X} value={val}N")
            else:
                self.node.get_logger().warn(f"[Sensor2 SEQ {seq_id}] Invalid FORCE response: {resp}")
        self.recv(3, self.REG_FORCE, 1, callback=cb, seq_id=seq_id)

    def get_last(self):
        return {
            'right_value': self.last_force_value,
            'left_value': self.last_force_value,
            'force_reg': self.last_force_reg,
            'right_ts': self.last_force_ts,
            'left_ts': self.last_force_ts,
        }

    def cleanup(self):
        self.node.get_logger().info("[Sensor2] controller cleanup done")
