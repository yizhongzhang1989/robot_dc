from modbus_devices.base_device import ModbusDevice
import time

class ForceSensorController(ModbusDevice):
    """Controller for single-channel force sensor on Modbus.

    Performs FC03 (Read Holding Registers) starting at address 0x0000 count=2.
    Slave/device ID is provided by parent node (expected 52).

    Response registers (2 x 16-bit) combined to 32-bit value:
        raw32 = (reg0 << 16) | reg1
    """
    def __init__(self, device_id, node, use_ack_patch=False):
        super().__init__(device_id, node, use_ack_patch)
        self.last_value = None
        self.last_regs = (None, None)
        self.last_timestamp = None

    def initialize(self):
        self.node.get_logger().info(f"Force sensor controller initialized (device_id={self.device_id})")

    def read_once(self, seq_id=None):
        """Read two registers and parse 32-bit value."""
        def cb(resp):
            ts = time.time()
            if resp and len(resp) >= 2:
                reg0, reg1 = resp[0], resp[1]
                raw32 = (reg0 << 16) | reg1
                self.last_value = raw32
                self.last_regs = (reg0, reg1)
                self.last_timestamp = ts
                self.node.get_logger().info(f"[SEQ {seq_id}] Force sensor read: reg0=0x{reg0:04X} reg1=0x{reg1:04X} raw32={raw32} (dec) ts={ts:.3f}")
            else:
                self.node.get_logger().warn(f"[SEQ {seq_id}] Invalid force sensor response: {resp}")
        # Function code 3, address 0, count 2
        self.recv(3, 0x0000, 2, callback=cb, seq_id=seq_id)

    def get_last(self):
        return self.last_value, self.last_regs, self.last_timestamp

    def cleanup(self):
        self.node.get_logger().info("Force sensor controller cleanup done")
