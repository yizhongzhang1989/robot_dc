class LeadshineMotor:
    VALID_MOTION_MODES = {
        0x0001: 'absolute',
        0x0041: 'relative',
        0x0002: 'velocity'
    }

    def __init__(self, motor_id, send_request_fn, recv_request_fn):
        self.motor_id = motor_id
        self.send = send_request_fn
        self.recv = recv_request_fn

        self.motion_mode = None

        self.target_position = 0
        self.target_velocity = 0
        self.target_acceleration = 0
        self.target_deceleration = 0

        self.initialize_motor()

    @staticmethod
    def _to_unsigned_16bit_regs_from_signed_32bit(value):
        """Convert signed 32-bit int to two 16-bit unsigned integers (big-endian)."""
        if value < 0:
            value += (1 << 32)
        high = (value >> 16) & 0xFFFF
        low = value & 0xFFFF
        return [high, low]

    @staticmethod
    def _to_unsigned_16bit_from_signed(value):
        """Convert signed 16-bit int to unsigned 16-bit for Modbus."""
        return value & 0xFFFF

    @staticmethod
    def _from_unsigned_16bit_regs_to_signed_32bit(high, low):
        """Convert two 16-bit unsigned registers into signed 32-bit int."""
        raw = (high << 16) | low
        return raw - 0x100000000 if raw & 0x80000000 else raw

    @staticmethod
    def _from_unsigned_16bit_to_signed(value):
        """Convert unsigned 16-bit int to signed 16-bit."""
        return value - 0x10000 if value & 0x8000 else value

    def initialize_motor(self):
        self.send(6, 0x0001, [10000])   # Set counts per round
        self.send(6, 0x0003, [2])       # Set loop mode
        self.send(6, 0x0007, [0])       # Set direction

        self.read_motion_target()
        self.read_motion_mode()

        # print initial values for debugging
        print(f"Motor {self.motor_id} initialized with:")
        print(f"  Motion mode: {hex(self.motion_mode)} ({self.VALID_MOTION_MODES.get(self.motion_mode, 'invalid')})")
        print(f"  Target position: {self.target_position}")
        print(f"  Velocity: {self.target_velocity}")
        print(f"  Acceleration: {self.target_acceleration}")
        print(f"  Deceleration: {self.target_deceleration}")

    def read_motion_mode(self):
        response = self.recv(3, 0x6200, 1)
        if response and response[0] in self.VALID_MOTION_MODES:
            self.motion_mode = response[0]
        else:
            print(f"Invalid motion mode {response[0] if response else 'None'} — setting to absolute move (0x0001)")
            self.set_motion_mode(0x0001)

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

    def set_zero_position(self):
        self.send(6, 0x6002, [0x0021])  # Set current position to be zero

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

    def read_motion_target(self):
        response = self.recv(3, 0x6201, 5)

        if response:
            self.target_position = self._from_unsigned_16bit_regs_to_signed_32bit(response[0], response[1])
            self.target_velocity = self._from_unsigned_16bit_to_signed(response[2])
            self.target_acceleration = self._from_unsigned_16bit_to_signed(response[3])
            self.target_deceleration = self._from_unsigned_16bit_to_signed(response[4])

            return (
                self.target_position,
                self.target_velocity,
                self.target_acceleration,
                self.target_deceleration,
            )
        else:
            raise ValueError("Failed to read motion target")

    def set_target_position(self, pos):
        self.target_position = pos
        regs = self._to_unsigned_16bit_regs_from_signed_32bit(pos)
        self.send(16, 0x6201, regs)

    def set_target_velocity(self, vel):
        self.target_velocity = vel
        self.send(6, 0x6203, [self._to_unsigned_16bit_from_signed(vel)])

    def set_target_acceleration(self, acc):
        self.target_acceleration = acc
        self.send(6, 0x6205, [self._to_unsigned_16bit_from_signed(acc)])

    def set_target_deceleration(self, dec):
        self.target_deceleration = dec
        self.send(6, 0x6206, [self._to_unsigned_16bit_from_signed(dec)])