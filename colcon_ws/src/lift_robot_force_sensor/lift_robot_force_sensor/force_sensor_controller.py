from modbus_devices.base_device import ModbusDevice
import time, struct

class ForceSensorController(ModbusDevice):
    """单通道力传感器控制器（功能码 0x03 读保持寄存器）。

    新硬件：只有一个力值寄存器，地址 0x0000 (decimal 0)，长度 1（返回 1 个 16bit 寄存器）。
    读取帧示例（设备地址假设 0x34 = 52 十进制）：
      请求: 34 03 00 00 00 01 CRC(lo) CRC(hi)
      应答: 34 03 02 HH LL CRC(lo) CRC(hi)
        02 = 后续字节数 (2 字节数据)；HH LL 为 16 位无符号数高低字节。
    数值转换: value = (HH << 8) | LL  (单位 N, 无小数位)。
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
            return int(regs[0]) & 0xFFFF
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
