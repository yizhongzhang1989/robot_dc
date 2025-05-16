from pymodbus.client import ModbusSerialClient

class ModbusRTUInterface:
    def __init__(self, port, baudrate=38400, timeout=1):
        self.client = ModbusSerialClient(
            port=port,
            baudrate=baudrate,
            stopbits=1,
            parity='N',
            bytesize=8,
            timeout=timeout
        )

    def connect(self):
        return self.client.connect()

    def disconnect(self):
        self.client.close()

    def write_register(self, addr, value, slave_id):
        return self.client.write_register(address=addr, value=value, slave=slave_id)

    def read_register(self, addr, slave_id, count=1):
        return self.client.read_holding_registers(address=addr, count=count, slave=slave_id)
