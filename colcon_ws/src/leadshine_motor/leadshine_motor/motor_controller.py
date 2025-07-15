from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *

class LeadshineMotor(ModbusDevice):
    VALID_MOTION_MODES = {
        0x0001: 'absolute',
        0x0041: 'relative',
        0x0002: 'velocity'
    }

    def __init__(self, device_id, node, use_ack_patch):
        super().__init__(device_id, node, use_ack_patch)

        self.motion_mode = None
        self.target_position = 0
        self.target_velocity = 0
        self.target_acceleration = 0
        self.target_deceleration = 0

        self.curr_position = 0

    def initialize(self, seq_id=None):
        self.send(6, 0x0001, [10000], seq_id=seq_id)   # Set counts per round
        self.send(6, 0x0003, [2], seq_id=seq_id)       # Set loop mode
        self.send(6, 0x0007, [0], seq_id=seq_id)       # Set direction
        self.set_home_params(1000, 30, 250, 250, 200, 200, seq_id=seq_id)  # Initialize homing parameters
        self.reset_software_limit(seq_id=seq_id)     # Disable software limit on startup
        self._get_motion_target(seq_id=seq_id)
        self._get_motion_mode(seq_id=seq_id)

    def _get_motion_target(self, seq_id=None):
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

        self.recv(3, 0x6201, 5, callback, seq_id=seq_id)

    def _get_motion_mode(self, seq_id=None):
        def callback(response):
            if response and response[0] in self.VALID_MOTION_MODES:
                self.motion_mode = response[0]
            else:
                print(f"Invalid motion mode {response[0] if response else 'None'} — setting to absolute move (0x0001)")
                self.set_motion_mode(0x0001, seq_id=seq_id)

        self.recv(3, 0x6200, 1, callback, seq_id=seq_id)

    def set_motion_mode(self, mode, seq_id=None):
        if mode in self.VALID_MOTION_MODES:
            self.send(6, 0x6200, [mode], seq_id=seq_id)
            self.motion_mode = mode
        else:
            raise ValueError(f"Invalid motion mode: {hex(mode)}")

    def write_control_word(self, value, seq_id=None):
        """
        Write a value to the control word register (0x1801).
        """
        self.send(6, 0x1801, [value], seq_id=seq_id)

    def reset_current_alarm(self, seq_id=None):
        self.write_control_word(0x1111, seq_id=seq_id)

    def reset_history_alarm(self, seq_id=None):
        self.write_control_word(0x1122, seq_id=seq_id)

    def save_all_params_to_eeprom(self, seq_id=None):
        self.write_control_word(0x2211, seq_id=seq_id)

    def factory_reset(self, seq_id=None):
        self.write_control_word(0x2233, seq_id=seq_id)

    def jog_left(self, seq_id=None):
        self.write_control_word(0x4001, seq_id=seq_id)

    def jog_right(self, seq_id=None):
        self.write_control_word(0x4002, seq_id=seq_id)

    def abrupt_stop(self, seq_id=None):
        self.send(6, 0x6002, [0x0040], seq_id=seq_id)  # Abrupt stop

    def get_current_position(self, callback, seq_id=None):
        def handle_response(response):
            if response:
                self.curr_position = from_unsigned_16bit_regs_to_signed_32bit(response[0], response[1])
                callback(self.curr_position)
            else:
                callback(None)

        self.recv(3, 0x602C, 2, handle_response, seq_id=seq_id)

    def get_motion_status(self, callback, seq_id=None):
        """
        Read motion status from register 0x1003
        Bit0: Fault (1=fault, 0=no fault)
        Bit1: Enabled (1=enabled, 0=disabled)
        Bit2: Running (1=running, 0=stopped)
        Bit3: Invalid (reserved)
        Bit4: Command completed (1=completed, 0=not completed)
        Bit5: Path completed (1=completed, 0=not completed)
        Bit6: Homing completed (1=completed, 0=not completed, stays 1 until next homing command)
        callback: function(dict) -> None, receives dict with status information
        """
        def handle_response(response):
            if response:
                status_register = response[0]
                fault = bool(status_register & (1 << 0))           # Bit0
                enabled = bool(status_register & (1 << 1))         # Bit1
                running = bool(status_register & (1 << 2))         # Bit2
                command_completed = bool(status_register & (1 << 4))  # Bit4
                path_completed = bool(status_register & (1 << 5))     # Bit5
                homing_completed = bool(status_register & (1 << 6))   # Bit6
                
                status_info = {
                    'fault': fault,
                    'enabled': enabled,
                    'running': running,
                    'command_completed': command_completed,
                    'path_completed': path_completed,
                    'homing_completed': homing_completed,
                    'raw_register': status_register
                }
                callback(status_info)
            else:
                callback(None)

        self.recv(3, 0x1003, 1, handle_response, seq_id=seq_id)

    def set_zero_position(self, seq_id=None):
        self.send(6, 0x6002, [0x0021], seq_id=seq_id)  # Set current position to be zero

    def set_target_position(self, pos, seq_id=None):
        self.target_position = pos
        regs = to_unsigned_16bit_regs_from_signed_32bit(pos)
        self.send(16, 0x6201, regs, seq_id=seq_id)

    def set_target_velocity(self, vel, seq_id=None):
        self.target_velocity = vel
        self.send(6, 0x6203, [to_unsigned_16bit_from_signed(vel)], seq_id=seq_id)

    def set_target_acceleration(self, acc, seq_id=None):
        self.target_acceleration = acc
        self.send(6, 0x6205, [to_unsigned_16bit_from_signed(acc)], seq_id=seq_id)

    def set_target_deceleration(self, dec, seq_id=None):
        self.target_deceleration = dec
        self.send(6, 0x6206, [to_unsigned_16bit_from_signed(dec)], seq_id=seq_id)

    def move_absolute(self, seq_id=None):
        if self.motion_mode != 0x0001:
            self.set_motion_mode(0x0001, seq_id=seq_id)
        self.send(6, 0x6002, [0x0010], seq_id=seq_id)  # Trigger move

    def move_relative(self, seq_id=None):
        if self.motion_mode != 0x0041:
            self.set_motion_mode(0x0041, seq_id=seq_id)
        self.send(6, 0x6002, [0x0010], seq_id=seq_id)  # Trigger move

    def move_velocity(self, seq_id=None):
        if self.motion_mode != 0x0002:
            self.set_motion_mode(0x0002, seq_id=seq_id)
        self.send(6, 0x6002, [0x0010], seq_id=seq_id)  # Trigger move

    def set_home_params(self, stall_time=1000, cur=50, high_speed=250, low_speed=250, acc=100, dec=100, seq_id=None):
        """
        Only set homing parameters, do not trigger homing
        """
        TORQUE_MODE_ADDR = 0x600A
        STALL_TIME_ADDR = 0x6013
        CUR_ADDR = 0x6014
        HIGH_SPEED_ADDR = 0x600F
        LOW_SPEED_ADDR = 0x6010
        ACC_ADDR = 0x6011
        DEC_ADDR = 0x6012
        # Only write parameters, do not trigger
        self.send(6, TORQUE_MODE_ADDR, [0x000D], seq_id=seq_id)  # Default: positive torque homing mode
        self.send(6, STALL_TIME_ADDR, [stall_time], seq_id=seq_id)
        self.send(6, CUR_ADDR, [cur], seq_id=seq_id)
        self.send(6, HIGH_SPEED_ADDR, [high_speed], seq_id=seq_id)
        self.send(6, LOW_SPEED_ADDR, [low_speed], seq_id=seq_id)
        self.send(6, ACC_ADDR, [acc], seq_id=seq_id)
        self.send(6, DEC_ADDR, [dec], seq_id=seq_id)
        print(f"[set_home_params] Homing parameters set: stall_time={stall_time}, current(%)={cur}, high_speed={high_speed}, low_speed={low_speed}, acc={acc}, dec={dec}")

    def torque_home(self, direction, seq_id=None):
        """
        Only trigger homing, do not set parameters
        """
        TORQUE_MODE_ADDR = 0x600A
        TRIGGER_ADDR = 0x6002
        TORQUE_MODE_REVERSE = 0x000C  # Reverse torque homing
        TORQUE_MODE_FORWARD = 0x000D  # Forward torque homing
        TRIGGER_TORQUE_HOME = 0x0020  # Trigger torque homing
        mode = TORQUE_MODE_FORWARD if direction == "home_pos" else TORQUE_MODE_REVERSE
        self.send(6, TORQUE_MODE_ADDR, [mode], seq_id=seq_id)
        self.send(6, TRIGGER_ADDR, [TRIGGER_TORQUE_HOME], seq_id=seq_id)
        print(f"[torque_home] {'Forward' if direction == 'home_pos' else 'Reverse'} homing triggered")

    def set_software_limit(self, pos_limit, neg_limit, seq_id=None):
        """
        Set positive/negative software limits
        pos_limit: int, positive limit
        neg_limit: int, negative limit
        """
        # Software limit related registers
        POS_LIMIT_HIGH_ADDR = 0x6006  # Positive limit high
        POS_LIMIT_LOW_ADDR = 0x6007   # Positive limit low
        NEG_LIMIT_HIGH_ADDR = 0x6008  # Negative limit high
        NEG_LIMIT_LOW_ADDR = 0x6009   # Negative limit low
        CONTROL_SETTING_ADDR = 0x6000  # Control setting register
        CONTROL_SETTING_SOFT_LIMIT = 0x0002  # Enable software limit
        # 1. Enable software limit
        self.send(6, CONTROL_SETTING_ADDR, [CONTROL_SETTING_SOFT_LIMIT], seq_id=seq_id)
        # 2. Positive limit high/low
        pos_limit_high = (pos_limit >> 16) & 0xFFFF
        pos_limit_low = pos_limit & 0xFFFF
        self.send(6, POS_LIMIT_HIGH_ADDR, [pos_limit_high], seq_id=seq_id)
        self.send(6, POS_LIMIT_LOW_ADDR, [pos_limit_low], seq_id=seq_id)
        # 3. Negative limit high/low
        neg_limit_high = (neg_limit >> 16) & 0xFFFF
        neg_limit_low = neg_limit & 0xFFFF
        self.send(6, NEG_LIMIT_HIGH_ADDR, [neg_limit_high], seq_id=seq_id)
        self.send(6, NEG_LIMIT_LOW_ADDR, [neg_limit_low], seq_id=seq_id)
        print(f"[set_software_limit] Software limit set, positive limit: {pos_limit}, negative limit: {neg_limit}")

    def reset_alarm(self, seq_id=None):
        """
        Periodically reset alarm
        """
        ALARM_RESET_ADDR = 0x1801  # Alarm reset register
        ALARM_RESET_CMD = 0x1111   # Alarm reset command
        self.send(6, ALARM_RESET_ADDR, [ALARM_RESET_CMD], seq_id=seq_id)

    def get_alarm_status(self, callback, seq_id=None):
        """
        Read alarm/fault status register 0x2203, callback returns register value and fault description.
        callback: function(dict) -> None, receives dict with 'fault_code', 'fault_description', 'alm_blink_count'
        """
        def handle_response(response):
            if response:
                fault_code = response[0]
                fault_info = self._get_fault_description(fault_code)
                callback({
                    'fault_code': fault_code,
                    'fault_description': fault_info['description'],
                    'alm_blink_count': fault_info['blink_count'],
                    'raw_register': fault_code
                })
            else:
                callback(None)

        ALARM_STATUS_ADDR = 0x2203
        self.recv(3, ALARM_STATUS_ADDR, 1, handle_response, seq_id=seq_id)

    def _get_fault_description(self, fault_code):
        """
        Convert fault code to description and ALM blink count
        """
        fault_descriptions = {
            0x00E0: {'description': 'Overcurrent', 'blink_count': 1},
            0x00C0: {'description': 'Overvoltage', 'blink_count': 2},
            0x00A1: {'description': 'Current sampling circuit fault', 'blink_count': 3},
            0x0152: {'description': 'Shaft lock (phase loss) fault', 'blink_count': 4},
            0x0240: {'description': 'EEPROM fault', 'blink_count': 5},
            0x05F0: {'description': 'Parameter auto-tuning fault', 'blink_count': 6},
            0x0180: {'description': 'Position error alarm', 'blink_count': 7},
            0x0150: {'description': 'Encoder disconnection detection', 'blink_count': 8},
            0x00F0: {'description': 'Overtemperature', 'blink_count': 9},
            0x0210: {'description': 'Input IO configuration duplicate', 'blink_count': 10}
        }
        
        if fault_code == 0x0000:
            return {'description': 'No fault', 'blink_count': 0}
        elif fault_code in fault_descriptions:
            return fault_descriptions[fault_code]
        else:
            return {'description': f'Unknown fault (0x{fault_code:04X})', 'blink_count': -1}

    def reset_software_limit(self, seq_id=None):
        """
        Disable software limits (both positive and negative)
        """
        CONTROL_SETTING_ADDR = 0x6000  # Control setting register
        CONTROL_SETTING_DISABLE = 0x0000  # Disable software limit
        self.send(6, CONTROL_SETTING_ADDR, [CONTROL_SETTING_DISABLE], seq_id=seq_id)
        print("[reset_software_limit] Software limit disabled.")
