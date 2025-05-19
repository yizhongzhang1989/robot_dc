class LeadshineMotor:
    def __init__(self, motor_id, send_request_fn, recv_request_fn):
        self.motor_id = motor_id
        self.send = send_request_fn
        self.recv = recv_request_fn
        self.initialize_motor()

        self.target_position = 0
        self.target_velocity = 0
        self.target_acceleration = 0
        self.target_deceleration = 0

    def initialize_motor(self):
        self.send(6, 0x0001, [10000])   # Set counts per round
        self.send(6, 0x0003, [2])       # Set loop mode
        self.send(6, 0x0007, [0])       # Set direction

        self.read_motion_target()       # Read initial motion target

        # print initial values for debugging
        print(f"Motor {self.motor_id} initialized with target position: {self.target_position}, "
                f"velocity: {self.target_velocity}, acceleration: {self.target_acceleration}, "
                f"deceleration: {self.target_deceleration}")
        

    def jog_left(self):
        self.send(6, 0x1801, [0x4001])  # Jog left command

    def jog_right(self):
        self.send(6, 0x1801, [0x4002])  # Jog right command

    def abrupt_stop(self):
        self.send(6, 0x6002, [0x0040])  # Abrupt stop

    def set_zero_position(self):
        self.send(6, 0x6002, [0x0021])  # Set current position to be zero position

    def move_absolute(self):
        self.send(6, 0x6200, [0x0001])  # set absolute move
        self.send(6, 0x6002, [0x0010])  # trigger move

    def move_relative(self):
        self.send(6, 0x6200, [0x0041])  # set relative move
        self.send(6, 0x6002, [0x0010])  # trigger move

    def move_velocity(self):
        self.send(6, 0x6200, [0x0002])  # set velocity move
        self.send(6, 0x6002, [0x0010])  # trigger move
    
    def read_motion_target(self):
        response = self.recv(3, 0x6201, 5)

        if response:
            self.target_position = (response[0] << 16) | response[1]
            if self.target_position & 0x80000000:
                self.target_position -= 0x100000000
            self.target_velocity = response[2]
            self.target_acceleration = response[3]
            self.target_deceleration = response[4]
            return (self.target_position, self.target_velocity, self.target_acceleration, self.target_deceleration)
        else:
            raise ValueError("Failed to read motion target")