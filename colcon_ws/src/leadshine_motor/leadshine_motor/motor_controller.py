from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *

class LeadshineMotor(ModbusDevice):
    VALID_MOTION_MODES = {
        0x0001: 'absolute',
        0x0041: 'relative',
        0x0002: 'velocity'
    }

    def __init__(self, device_id, node):
        super().__init__(device_id, node)

        self.motion_mode = None

        self.target_position = 0
        self.target_velocity = 0
        self.target_acceleration = 0
        self.target_deceleration = 0

        self.curr_position = 0

    def initialize(self):
        self.send(6, 0x0001, [10000])   # Set counts per round
        self.send(6, 0x0003, [2])       # Set loop mode
        self.send(6, 0x0007, [0])       # Set direction

        self._get_motion_target()
        self._get_motion_mode()

    def _get_motion_target(self):
        def callback(response):
            if response:
                self.target_position = from_unsigned_16bit_regs_to_signed_32bit(response[0], response[1])
                self.target_velocity = from_unsigned_16bit_to_signed(response[2])
                self.target_acceleration = from_unsigned_16bit_to_signed(response[3])
                self.target_deceleration = from_unsigned_16bit_to_signed(response[4])

                print(f"Motor {self.device_id} initialized with:")
                print(f"  Target position: {self.target_position}")
                print(f"  Velocity: {self.target_velocity}")
                print(f"  Acceleration: {self.target_acceleration}")
                print(f"  Deceleration: {self.target_deceleration}")
            else:
                raise ValueError("Failed to read motion target")

        self.recv(3, 0x6201, 5, callback)

    def _get_motion_mode(self):
        def callback(response):
            if response and response[0] in self.VALID_MOTION_MODES:
                self.motion_mode = response[0]
            else:
                print(f"Invalid motion mode {response[0] if response else 'None'} — setting to absolute move (0x0001)")
                self.set_motion_mode(0x0001)

        self.recv(3, 0x6200, 1, callback)

    def set_motion_mode(self, mode):
        if mode in self.VALID_MOTION_MODES:
            self.send(6, 0x6200, [mode])
            self.motion_mode = mode
        else:
            raise ValueError(f"Invalid motion mode: {hex(mode)}")

    def jog_left(self):
        self.send(6, 0x1801, [0x4001])  # Jog left command

    def jog_right(self):
        self.send(6, 0x1801, [0x4002])  # Jog right command

    def abrupt_stop(self):
        self.send(6, 0x6002, [0x0040])  # Abrupt stop

    def get_current_position(self, callback):
        def handle_response(response):
            if response:
                self.curr_position = from_unsigned_16bit_regs_to_signed_32bit(response[0], response[1])
                callback(self.curr_position)
            else:
                callback(None)

        self.recv(3, 0x602C, 2, handle_response)

    def set_zero_position(self):
        self.send(6, 0x6002, [0x0021])  # Set current position to be zero

    def set_target_position(self, pos):
        self.target_position = pos
        regs = to_unsigned_16bit_regs_from_signed_32bit(pos)
        self.send(16, 0x6201, regs)

    def set_target_velocity(self, vel):
        self.target_velocity = vel
        self.send(6, 0x6203, [to_unsigned_16bit_from_signed(vel)])

    def set_target_acceleration(self, acc):
        self.target_acceleration = acc
        self.send(6, 0x6205, [to_unsigned_16bit_from_signed(acc)])

    def set_target_deceleration(self, dec):
        self.target_deceleration = dec
        self.send(6, 0x6206, [to_unsigned_16bit_from_signed(dec)])

    def move_absolute(self):
        if self.motion_mode != 0x0001:
            self.set_motion_mode(0x0001)
        self.send(6, 0x6002, [0x0010])  # Trigger move

    def move_relative(self):
        if self.motion_mode != 0x0041:
            self.set_motion_mode(0x0041)
        self.send(6, 0x6002, [0x0010])  # Trigger move

    def move_velocity(self):
        if self.motion_mode != 0x0002:
            self.set_motion_mode(0x0002)
        self.send(6, 0x6002, [0x0010])  # Trigger move
