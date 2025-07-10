from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *

class PlatformController(ModbusDevice):
    def __init__(self, device_id, node, use_ack_patch):
        super().__init__(device_id, node)
        if use_ack_patch:
            self._orig_send = self.send
            def send_with_ack(*args, **kwargs):
                def ack_callback(fut):
                    result = fut.result()
                    if hasattr(result, 'ack') and result.ack == 1:
                        if hasattr(self.node, 'waiting_for_ack'):
                            self.node.waiting_for_ack = False
                            if hasattr(self.node, 'process_next_command'):
                                self.node.process_next_command()
                kwargs['callback'] = ack_callback
                return self._orig_send(*args, **kwargs)
            self.send = send_with_ack

    def initialize(self):
        pass

    def up(self, move_flag=True, seq_id=None):    # move_flag is True to move up, False to stop
        if move_flag:
            self.send(5, 0x0002, [0], seq_id=seq_id)
            self.send(5, 0x0003, [1], seq_id=seq_id)
        else:
            self.send(5, 0x0002, [0], seq_id=seq_id)
            self.send(5, 0x0003, [0], seq_id=seq_id)

    def down(self, move_flag=True, seq_id=None):  # move_flag is True to move down, False to stop
        if move_flag:
            self.send(5, 0x0002, [1], seq_id=seq_id)
            self.send(5, 0x0003, [0], seq_id=seq_id)
        else:
            self.send(5, 0x0002, [0], seq_id=seq_id)
            self.send(5, 0x0003, [0], seq_id=seq_id)

    def forward(self, move_flag=True, seq_id=None):  # move_flag is True to move forward, False to stop
        if move_flag:
            self.send(5, 0x0000, [1], seq_id=seq_id)
            self.send(5, 0x0001, [0], seq_id=seq_id)
        else:
            self.send(5, 0x0000, [0], seq_id=seq_id)
            self.send(5, 0x0001, [0], seq_id=seq_id)

    def backward(self, move_flag=True, seq_id=None): # move_flag is True to move backward, False to stop
        if move_flag:
            self.send(5, 0x0000, [0], seq_id=seq_id)
            self.send(5, 0x0001, [1], seq_id=seq_id)
        else:
            self.send(5, 0x0000, [0], seq_id=seq_id)
            self.send(5, 0x0001, [0], seq_id=seq_id)
