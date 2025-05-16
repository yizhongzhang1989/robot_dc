from modbus_driver.modbus_rtu_interface import ModbusRTUInterface

class LeadshineMotor:
    JOG_LEFT = 0x4001
    JOG_RIGHT = 0x4002
    REGISTER_ADDR = 0x1801

    def __init__(self, modbus: ModbusRTUInterface, motor_id: int):
        self.modbus = modbus
        self.motor_id = motor_id

    def jog(self, direction: str):
        value = self.JOG_LEFT if direction == 'left' else self.JOG_RIGHT
        return self.modbus.write_register(self.REGISTER_ADDR, value, self.motor_id)
