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
        self.set_target_acceleration(10) # Set target acceleration
        self.set_target_velocity(5)     # Set target velocity
        self.set_target_torque_limit(1000) # Set target torque limit

    def stop(self):
        # Stop the servo by setting target position to current position
        def callback(response):
            if response:
                self.target_position = response[0]
                self.set_target_position(self.target_position)
            else:
                raise ValueError("Failed to read current position for stopping")
        self.recv(3, 0x0101, 1, callback)

    def get_current_status(self):
        # Request current status (8 uint16) from the servo from address 0x100
        def callback(response):
            if response:
                self.curr_status = response[0]
                self.curr_position = response[1]
                self.curr_velocity = response[2]
                self.curr_PWM = response[3]
                self.curr_voltage = response[4]
                self.curr_temperature = response[5]
                self.curr_moving_flag = response[6]
                self.curr_current = response[7]
            else:
                raise ValueError("Failed to read current status")

        self.recv(3, 0x0100, 8, callback)



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
