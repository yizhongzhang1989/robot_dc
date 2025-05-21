import rclpy  
from rclpy.node import Node  
from modbus_driver_interfaces.srv import ModbusRequest  
from modbus_driver_interfaces.msg import MotorSimulationStatus  
  
import threading  
import time  
  
# Some mode constants  
MOTION_MODE_ABS = 0x0001  
MOTION_MODE_REL = 0x0041  
MOTION_MODE_VEL = 0x0002  
  
class MotorSimulationNode(Node):  
    def __init__(self):  
        super().__init__('motor_simulation_node')  
  
        # Get motor_id from parameter (default=1)  
        self.declare_parameter('motor_id', 1)  
        self.motor_id = self.get_parameter('motor_id').value  
        self.get_logger().info(f"Simulating motor_id = {self.motor_id}")  
  
        # Create the service server that mimics the Modbus driver  
        # (Often remapped in multi‐motor setups)  
        self.srv = self.create_service(  
            ModbusRequest,  
            'modbus_request',  
            self.handle_modbus_request  
        )  
  
        # Concurrency lock (service callbacks + simulation timer)  
        self.lock = threading.Lock()  
  
        # ─────────────────────────────────────────────────────────────  
        #  Register map: each address → 16‐bit unsigned int default  
        #  (Add more addresses as needed for your actual motor)  
        # ─────────────────────────────────────────────────────────────  
        self.registers = {  
            0x0001: 10000,    # Pulse per round  
            0x0003: 2,        # Example usage  
            0x0005: 1,  
            0x0007: 0,  
            0x0009: 1499,  
            0x000B: 65535,  
            0x000F: 0,  
            0x1801: 0,        # Jog command register  
            0x6000: 0,  
            0x6002: 0,        # Control word  
            0x6006: 32767,  
            0x6007: 65535,  
            0x6008: 65535,  
            0x6009: 65535,  
            0x600A: 0,  
            0x600C: 0,        # (Optional) Could store “actual velocity” here if desired  
            0x600D: 0,  
            0x6200: 0,        # Motion mode  
            0x6201: 0,        # Target pos (high 16)  
            0x6202: 0,        # Target pos (low 16)  
            0x6203: 0,        # Target velocity (16‐bit)  
            0x6204: 100,      # Example default acceleration, etc.  
            0x6205: 100,  
            0x6206: 0,  
            # Current position 32‐bit might live at 0x602C + 0x602D, so let’s define them:  
            0x602C: 0,        # Current position high  
            0x602D: 0,        # Current position low  
        }  
  
        # For demonstration, we keep a small “transient” state in code  
        # This is purely for sim logic: are we “in motion”? what's our “actual rpm” now?  
        self.moving = False  
        self.current_rpm = 0.0  
  
        # Track the last command (for logging/visualization)  
        self.last_command = "-"  
  
        # Publisher for motor status  
        self.status_pub = self.create_publisher(  
            MotorSimulationStatus,  
            f'/motor{self.motor_id}/sim_status',  
            10  
        )  
  
        # Create a timer to periodically update the motor's “physics” (every 10ms)  
        self.timer_period = 0.01  
        self.timer = self.create_timer(self.timer_period, self.update_simulation)  
  
        self.get_logger().info("✅ Motor simulation node is ready.")  
  
    # ─────────────────────────────────────────────────────────────  
    #  Conversion utilities (same logic as your LeadshineMotor)  
    # ─────────────────────────────────────────────────────────────  
    @staticmethod  
    def _to_unsigned_16bit_regs_from_signed_32bit(value):  
        """Convert signed 32-bit int to two 16-bit unsigned (big-endian)."""  
        if value < 0:  
            value += (1 << 32)  
        high = (value >> 16) & 0xFFFF  
        low  = value & 0xFFFF  
        return [high, low]  
  
    @staticmethod  
    def _to_unsigned_16bit_from_signed(value):  
        """Convert signed 16-bit int to unsigned 16-bit."""  
        return value & 0xFFFF  
  
    @staticmethod  
    def _from_unsigned_16bit_regs_to_signed_32bit(high, low):  
        """Convert two 16-bit unsigned registers into signed 32-bit int."""  
        raw = (high << 16) | low  
        return raw - 0x100000000 if (raw & 0x80000000) else raw  
  
    @staticmethod  
    def _from_unsigned_16bit_to_signed(value):  
        """Convert unsigned 16-bit int to signed 16-bit."""  
        return value - 0x10000 if (value & 0x8000) else value  
  
    # ─────────────────────────────────────────────────────────────  
    #  Service: fake Modbus read/write  
    # ─────────────────────────────────────────────────────────────  
    def handle_modbus_request(self, request, response):  
        """Service callback that fakes Modbus read/write by referencing internal registers."""  
        with self.lock:  
            # Only respond if the slave_id matches this motor’s ID  
            if request.slave_id != self.motor_id:  
                self.get_logger().warning(  
                    f"Received Modbus request for slave_id={request.slave_id}, "  
                    f"but this sim node is for motor_id={self.motor_id}."  
                )  
                response.success = False  
                response.response = []  
                return response  
  
            try:  
                if request.function_code == 3:  
                    # Read holding registers  
                    read_values = self._read_registers(request.address, request.count)  
                    response.success = True  
                    response.response = read_values  
                    self.log_modbus(f"FC=3 read {read_values} from addr={hex(request.address)}")  
  
                elif request.function_code == 6:  
                    # Write single register  
                    if len(request.values) != 1:  
                        raise ValueError("FC=6 expects exactly 1 value")  
                    self._write_registers(request.address, request.values)  
                    response.success = True  
                    response.response = request.values  
                    self.log_modbus(f"FC=6 wrote {request.values} to addr={hex(request.address)}")  
  
                elif request.function_code == 16:  
                    # Write multiple registers  
                    self._write_registers(request.address, request.values)  
                    response.success = True  
                    response.response = request.values  
                    self.log_modbus(f"FC=16 wrote {request.values} to addr={hex(request.address)}")  
  
                else:  
                    raise ValueError(f"Unsupported function code: {request.function_code}")  
  
            except Exception as e:  
                self.get_logger().error(f"Simulation error: {e}")  
                response.success = False  
                response.response = []  
  
        return response  
  
    def log_modbus(self, msg: str):  
        self.get_logger().info(f"[Motor {self.motor_id}] {msg}")  
        self.last_command = msg  
  
    # ─────────────────────────────────────────────────────────────  
    #  Internal: read/write registers as 16-bit chunks  
    # ─────────────────────────────────────────────────────────────  
    def _read_registers(self, start_addr, count):  
        """Return 'count' 16-bit registers from 'start_addr' onward."""  
        result = []  
        for offset in range(count):  
            addr = start_addr + offset  
            val = self.registers.get(addr, 0)  # default 0 if missing  
            result.append(val & 0xFFFF)        # ensure 16-bit  
        return result  
  
    def _write_registers(self, start_addr, values):  
        """Write a list of 16-bit values beginning at start_addr."""  
        offset = 0  
        while offset < len(values):  
            addr = start_addr + offset  
            val = values[offset] & 0xFFFF  
            self.registers[addr] = val  
            offset += 1  
            # Possibly interpret special commands (control words, jog, etc.)  
            self._interpret_register_write(addr, val)  
  
    def _interpret_register_write(self, addr, value):  
        """  
        When certain registers are written, run the needed logic:  
          - 0x1801 => jog  
          - 0x6002 => control word  
          - 0x6200 => motion mode  
          etc.  
        But we do NOT store our “physics” in self anymore; we only set register bits  
        or flags that say “start moving,” etc.  
        """  
        if addr == 0x1801:  
            # Possibly a jog command: 0x4001 (left), 0x4002 (right)  
            self._handle_jog(value)  
        elif addr == 0x6002:  
            # Control word: move, stop, set zero, etc.  
            self._handle_control_word(value)  
        elif addr == 0x6200:  
            # Motion mode  
            # e.g. 0x0001 => ABS, 0x0041 => REL, 0x0002 => VEL  
            pass  # Already stored in register; do nothing extra here  
        # ... or handle other addresses similarly.  
  
    def _handle_jog(self, cmd):  
        # 0x4001 => jog left => negative velocity  
        # 0x4002 => jog right => positive velocity  
        if cmd == 0x4001:  
            # set velocity register to -100 rpm  
            signed_val = self._to_unsigned_16bit_from_signed(-100)  
            self.registers[0x6203] = signed_val  
            # set motion mode to VEL  
            self.registers[0x6200] = MOTION_MODE_VEL  
            self.moving = True  
        elif cmd == 0x4002:  
            # set velocity register to +100 rpm  
            signed_val = self._to_unsigned_16bit_from_signed(100)  
            self.registers[0x6203] = signed_val  
            self.registers[0x6200] = MOTION_MODE_VEL  
            self.moving = True  
  
    def _handle_control_word(self, cmd):  
        """  
        Some example bits:  
          0x0010 => Trigger move  
          0x0040 => Abrupt stop  
          0x0021 => Set zero  
        """  
        if cmd == 0x0010:  
            # Trigger a move for whichever mode is in 0x6200  
            self.moving = True  
        elif cmd == 0x0040:  
            # Abrupt stop => actual velocity => 0, moving => false  
            self.current_rpm = 0.0  
            self.moving = False  
        elif cmd == 0x0021:  
            # Set current position (0x602C,0x602D) to zero  
            self.registers[0x602C] = 0  
            self.registers[0x602D] = 0  
  
    # ─────────────────────────────────────────────────────────────  
    #  Main simulation loop  
    # ─────────────────────────────────────────────────────────────  
    def update_simulation(self):  
        """  
        Called by a ROS timer (~10ms). Reads registers for  
        (pulse/round), target pos, velocity, accel, etc., updates current position  
        and velocity as needed, writes the new current position to 0x602C,0x602D,  
        and publishes status.  
        """  
        with self.lock:  
            dt = self.timer_period  
  
            # Read key parameters from registers each cycle  
            pulse_per_round = self._from_unsigned_16bit_to_signed(self.registers[0x0001])  
            motion_mode = self.registers[0x6200] & 0xFFFF  
            # Target position is split into 0x6201 (high) & 0x6202 (low):  
            target_pos = self._from_unsigned_16bit_regs_to_signed_32bit(  
                self.registers[0x6201], self.registers[0x6202]  
            )  
            # Velocity is a 16-bit signed: [rpm]  
            velocity_cmd = self._from_unsigned_16bit_to_signed(self.registers[0x6203])  
            # Acc/Dec in 16-bit signed, “ms per 1000 rpm” (optional)  
            acc = self._from_unsigned_16bit_to_signed(self.registers[0x6205])  
            dec = self._from_unsigned_16bit_to_signed(self.registers[0x6206])  
  
            # Current position in 0x602C,0x602D (signed 32)  
            curr_pos = self._from_unsigned_16bit_regs_to_signed_32bit(  
                self.registers[0x602C], self.registers[0x602D]  
            )  
  
            # For velocity ramp, we keep the “actual rpm” in self.current_rpm  
            # (You could store it in 0x600C if you want fully in registers.)  
            if not self.moving:  
                # If we’re not “moving,” ensure we track actual rpm = 0 if we want a stop  
                if motion_mode != MOTION_MODE_VEL:  
                    self.current_rpm = 0.0  
            else:  
                # We are in a “moving” state  
                if motion_mode == MOTION_MODE_ABS or motion_mode == MOTION_MODE_REL:  
                    # Simplify: move at constant velocity_cmd => pulses/sec  
                    pulses_per_sec = (velocity_cmd * pulse_per_round) / 60.0  
                    direction = 1 if (target_pos > curr_pos) else -1  
                    if abs(velocity_cmd) < 1e-3:  
                        # If velocity register says 0, pick a default or do nothing  
                        pulses_per_sec = 100 * direction  
                    step = direction * abs(pulses_per_sec) * dt  
                    next_pos = curr_pos + step  
  
                    # Check if we pass the target  
                    if direction > 0 and next_pos >= target_pos:  
                        curr_pos = target_pos  
                        self.moving = False  
                    elif direction < 0 and next_pos <= target_pos:  
                        curr_pos = target_pos  
                        self.moving = False  
                    else:  
                        curr_pos = next_pos  
                    # Actual rpm is just velocity_cmd  
                    self.current_rpm = float(velocity_cmd)  
  
                elif motion_mode == MOTION_MODE_VEL:  
                    # Ramp self.current_rpm towards velocity_cmd  
                    acc_slope = 1_000_000 / acc if acc != 0 else 9999999  
                    dec_slope = 1_000_000 / dec if dec != 0 else 9999999  
                    delta_rpm = velocity_cmd - self.current_rpm  
                    if abs(delta_rpm) > 1e-3:  
                        # choose ramp up or ramp down  
                        slope = acc_slope if delta_rpm > 0 else dec_slope  
                        dv = slope * dt  
                        if abs(delta_rpm) <= dv:  
                            self.current_rpm = velocity_cmd  
                        else:  
                            self.current_rpm += dv * (delta_rpm / abs(delta_rpm))  
                    # Convert current_rpm => pulses/sec  
                    pulses_per_sec = (self.current_rpm * pulse_per_round) / 60.0  
                    curr_pos = curr_pos + pulses_per_sec * dt  
  
            # Write updated current position back to 0x602C,0x602D:  
            [pos_high, pos_low] = self._to_unsigned_16bit_regs_from_signed_32bit(int(curr_pos))  
            self.registers[0x602C] = pos_high  
            self.registers[0x602D] = pos_low  
  
            # If you want to store the actual speed in a register, do so:  
            # self.registers[0x600C] = self._to_unsigned_16bit_from_signed(int(self.current_rpm))  
  
            # Publish status message  
            msg = MotorSimulationStatus()  
            msg.motor_id = self.motor_id  
            msg.current_position = float(curr_pos)  
            msg.target_position = float(target_pos)  
            msg.velocity = float(velocity_cmd)  # “commanded” rpm  
            msg.motion_mode = motion_mode  
            msg.moving = self.moving  
            msg.last_command = self.last_command  
            self.status_pub.publish(msg)  
  
    def destroy_node(self):  
        self.get_logger().info(f"Shutting down motor simulation for motor_id={self.motor_id}")  
        super().destroy_node()  
  
  
def main(args=None):  
    rclpy.init(args=args)  
    node = MotorSimulationNode()  
    try:  
        rclpy.spin(node)  
    finally:  
        node.destroy_node()  
        rclpy.shutdown()  
  
if __name__ == '__main__':  
    main()  