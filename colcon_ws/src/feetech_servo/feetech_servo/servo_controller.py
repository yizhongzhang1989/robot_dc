from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *

class FeetechServo(ModbusDevice):
    def __init__(self, device_id, send_request_fn, recv_request_fn):
        super().__init__(device_id, send_request_fn, recv_request_fn)

        self.motion_mode = None

        self.target_position = 0
        self.enable_torque = 0
        self.target_acceleration = 0
        self.target_velocity = 0
        self.target_torque_limit = 0

        self.curr_status = 0
        self.curr_position = 0
        self.curr_velocity = 0
        self.curr_PWM = 0
        self.curr_voltage = 0
        self.curr_temperature = 0
        self.curr_moving_flag = 0
        self.curr_current = 0

        self.initialize_servo()

    def initialize_servo(self):
        self.set_target_position(2048)  # Set initial target position to midpoint
        self.set_enable_torque(True)     # Enable torque
        self.set_target_acceleration(100) # Set target acceleration
        self.set_target_velocity(200)     # Set target velocity
        self.set_target_torque_limit(1000) # Set target torque limit

    def set_target_position(self, position):
        self.target_position = position
        self.send(6, 0x0080, [position])

    def set_enable_torque(self, enable):
        self.enable_torque = 1 if enable else 0
        self.send(6, 0x0081, [self.enable_torque])

    def set_target_acceleration(self, acceleration):
        self.target_acceleration = acceleration
        self.send(6, 0x0082, [acceleration])

    def set_target_velocity(self, velocity):
        self.target_velocity = velocity
        self.send(6, 0x0083, [velocity])

    def set_target_torque_limit(self, torque_limit):
        self.target_torque_limit = torque_limit
        self.send(6, 0x0084, [torque_limit])

    