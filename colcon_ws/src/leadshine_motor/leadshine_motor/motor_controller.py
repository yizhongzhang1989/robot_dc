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

    def torque_home(self, direction, stall_time=1000, output_val=50, high_speed=1000, low_speed=200, acc=100, dec=100):
        """
        direction: '+' 正向回零, '-' 反向回零
        其余参数同 torque_home_motor.py
        """
        # 力矩回零相关寄存器
        TORQUE_MODE_ADDR = 0x600A
        STALL_TIME_ADDR = 0x6013
        OUTPUT_VAL_ADDR = 0x6014
        TRIGGER_ADDR = 0x6002
        HIGH_SPEED_ADDR = 0x600F
        LOW_SPEED_ADDR = 0x6010
        ACC_ADDR = 0x6011
        DEC_ADDR = 0x6012
        # 指令值
        TORQUE_MODE_REVERSE = 0x000C  # 反向力矩回零
        TORQUE_MODE_FORWARD = 0x000D  # 正向力矩回零
        TRIGGER_TORQUE_HOME = 0x0020  # 触发力矩回零
        # 选择回零模式
        mode = TORQUE_MODE_FORWARD if direction == "+" else TORQUE_MODE_REVERSE
        # 写入回零模式
        self.send(6, TORQUE_MODE_ADDR, [mode])
        self.send(6, STALL_TIME_ADDR, [stall_time])
        self.send(6, OUTPUT_VAL_ADDR, [output_val])
        self.send(6, HIGH_SPEED_ADDR, [high_speed])
        self.send(6, LOW_SPEED_ADDR, [low_speed])
        self.send(6, ACC_ADDR, [acc])
        self.send(6, DEC_ADDR, [dec])
        # 触发力矩回零
        self.send(6, TRIGGER_ADDR, [TRIGGER_TORQUE_HOME])
        print(f"[torque_home] {('正向' if direction == '+' else '反向')}回零已触发，参数: 堵转时间={stall_time}, 出力值={output_val}, 高速={high_speed}, 低速={low_speed}, 加速度={acc}, 减速度={dec}")

    def set_software_limit(self, pos_limit, neg_limit):
        """
        设置正负软件限位
        pos_limit: int, 正限位
        neg_limit: int, 负限位
        """
        # 软件限位相关寄存器
        POS_LIMIT_HIGH_ADDR = 0x6006  # 正限位高位
        POS_LIMIT_LOW_ADDR = 0x6007   # 正限位低位
        NEG_LIMIT_HIGH_ADDR = 0x6008  # 负限位高位
        NEG_LIMIT_LOW_ADDR = 0x6009   # 负限位低位
        SET_ZERO_ADDR = 0x6002        # 设零寄存器
        SET_ZERO_CMD = 0x0021         # 设零指令
        CONTROL_SETTING_ADDR = 0x6000  # 控制设置寄存器
        CONTROL_SETTING_SOFT_LIMIT = 0x0002  # 软件限位有效
        # 1. 使能软件限位
        self.send(6, CONTROL_SETTING_ADDR, [CONTROL_SETTING_SOFT_LIMIT])
        # 2. 设零
        self.send(6, SET_ZERO_ADDR, [SET_ZERO_CMD])
        # 3. 正限位高低位
        pos_limit_high = (pos_limit >> 16) & 0xFFFF
        pos_limit_low = pos_limit & 0xFFFF
        self.send(6, POS_LIMIT_HIGH_ADDR, [pos_limit_high])
        self.send(6, POS_LIMIT_LOW_ADDR, [pos_limit_low])
        # 4. 负限位高低位
        neg_limit_high = (neg_limit >> 16) & 0xFFFF
        neg_limit_low = neg_limit & 0xFFFF
        self.send(6, NEG_LIMIT_HIGH_ADDR, [neg_limit_high])
        self.send(6, NEG_LIMIT_LOW_ADDR, [neg_limit_low])
        print(f"[set_software_limit] 软件限位已设置，正限位：{pos_limit}，负限位：{neg_limit}")

    def reset_alarm(self):
        """
        定期消除报警
        """
        ALARM_RESET_ADDR = 0x1801  # 复位报警寄存器
        ALARM_RESET_CMD = 0x1111   # 复位报警指令
        self.send(6, ALARM_RESET_ADDR, [ALARM_RESET_CMD])
