from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *

class PlatformController(ModbusDevice):
    def __init__(self, device_id, node):
        super().__init__(device_id, node)

    def initialize(self):
        pass

    def up(self, move_flag=True):    # move_flag is True to move up, False to stop
        if move_flag:
            self.send(5, 0x0002, [0])
            self.send(5, 0x0003, [1])
        else:
            self.send(5, 0x0002, [0])
            self.send(5, 0x0003, [0])

    def down(self, move_flag=True):  # move_flag is True to move down, False to stop
        if move_flag:
            self.send(5, 0x0002, [1])
            self.send(5, 0x0003, [0])
        else:
            self.send(5, 0x0002, [0])
            self.send(5, 0x0003, [0])

    def forward(self, move_flag=True):  # move_flag is True to move forward, False to stop
        if move_flag:
            self.send(5, 0x0000, [1])
            self.send(5, 0x0001, [0])
        else:
            self.send(5, 0x0000, [0])
            self.send(5, 0x0001, [0])

    def backward(self, move_flag=True): # move_flag is True to move backward, False to stop
        if move_flag:
            self.send(5, 0x0000, [0])
            self.send(5, 0x0001, [1])
        else:
            self.send(5, 0x0000, [0])
            self.send(5, 0x0001, [0])
