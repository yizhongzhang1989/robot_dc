from pymodbus.client import ModbusSerialClient as ModbusClient


class ModbusRTUInterface:
    def __init__(self, port, baudrate=38400, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.client = ModbusClient(port=port, baudrate=baudrate, timeout=timeout)

    def connect(self):
        return self.client.connect()

    def disconnect(self):
        self.client.close()

    def read_register(self, address, unit_id, count):
        return self.client.read_holding_registers(address=address, count=count, unit=unit_id)

    def write_register(self, address, value, unit_id):
        return self.client.write_register(address=address, value=value, unit=unit_id)

    def write_registers(self, address, values, unit_id):
        return self.client.write_registers(address=address, values=values, unit=unit_id)
