class LeadshineMotor:
    def __init__(self, motor_id, send_request_fn):
        self.motor_id = motor_id
        self.send = send_request_fn
        self.initialize_motor()

    def initialize_motor(self):
        self.send(6, 0x0001, [0x2710])  # counts per rev
        self.send(6, 0x0003, [0x0002])  # loop mode
        self.send(6, 0x0007, [0x0000])  # direction

    def jog_left(self):
        self.send(6, 0x1801, [0x4001])

    def jog_right(self):
        self.send(6, 0x1801, [0x4002])

    def abrupt_stop(self):
        self.send(6, 0x6002, [0x0040])
