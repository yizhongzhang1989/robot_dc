import rclpy  
from rclpy.node import Node  
  
# Import your existing LeadshineMotor class.  
# Adjust the import path if your file is named or structured differently.  
from leadshine_motor.motor_controller import LeadshineMotor  
  
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
  
        # Get motor_id from ROS parameter (default=1)  
        self.declare_parameter('motor_id', 1)  
        self.motor_id = self.get_parameter('motor_id').value  
        self.get_logger().info(f"Simulating motor_id = {self.motor_id}")  
  
        # Create the service server that mimics the Modbus driver  
        self.srv = self.create_service(  
            ModbusRequest,  
            'modbus_request',  
            self.handle_modbus_request  
        )  
  
        # Concurrency lock (service callbacks + simulation timer)  
        self.lock = threading.Lock()  
  
        # ──────────────────────────────────────────────────────────  
        #  Register map: each address → 16-bit unsigned int default  
        # ──────────────────────────────────────────────────────────  
        self.registers = {  
            0x0001: 10000,    # Pulse per round  
            0x0003: 2,  
            0x0005: 1,  
            0x0007: 0,  
            0x0009: 1499,  
            0x000B: 65535,  
            0x000F: 0,  
            0x01E1: 60,       # Jog speed
            0x01E3: 100,      # Jog time ms
            0x01E5: 1,        # Jog iterations
            0x01E7: 200,      # Jog acceleration and deceleration  
            0x1801: 0,        # Jog command  
            0x6000: 0,  
            0x6002: 0,        # Control word  
            0x6006: 32767,  
            0x6007: 65535,  
            0x6008: 65535,  
            0x6009: 65535,  
            0x600A: 0,  
            0x600C: 0,  
            0x600D: 0,  
            0x6200: 0,        # Motion mode  
            0x6201: 0,        # (high word of target position)  
            0x6202: 0,        # (low word of target position)  
            0x6203: 0,        # Velocity (16-bit)  
            0x6204: 100,      # Default acceleration  
            0x6205: 100,  
            0x6206: 0,  
            0x602C: 0,        # Current position high  
            0x602D: 0,        # Current position low  
        }  
  
        # Transient simulation state  
        self.moving = False  
        self.current_rpm = 0.0  
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
  
    # ----------------------------------------------------------------  
    # Helper methods for reading/writing “physical” parameters  
    # ----------------------------------------------------------------  
    def _get_pulse_per_round(self) -> int:  
        return LeadshineMotor._from_unsigned_16bit_to_signed(self.registers[0x0001])  
  
    def _get_motion_mode(self) -> int:  
        return self.registers[0x6200] & 0xFFFF  
  
    def _get_target_pos(self) -> int:  
        return LeadshineMotor._from_unsigned_16bit_regs_to_signed_32bit(  
            self.registers[0x6201], self.registers[0x6202]  
        )  
  
    def _get_velocity_cmd(self) -> int:  
        return LeadshineMotor._from_unsigned_16bit_to_signed(self.registers[0x6203])  
  
    def _get_acc(self) -> int:  
        return LeadshineMotor._from_unsigned_16bit_to_signed(self.registers[0x6205])  
  
    def _get_dec(self) -> int:  
        return LeadshineMotor._from_unsigned_16bit_to_signed(self.registers[0x6206])  
  
    def _get_current_pos(self) -> int:  
        return LeadshineMotor._from_unsigned_16bit_regs_to_signed_32bit(  
            self.registers[0x602C], self.registers[0x602D]  
        )  
  
    def _set_current_pos(self, pos: int):  
        high, low = LeadshineMotor._to_unsigned_16bit_regs_from_signed_32bit(int(pos))  
        self.registers[0x602C] = high  
        self.registers[0x602D] = low  
  
    def _publish_status(self, curr_pos: float, target_pos: float,  
                        velocity_cmd: float, motion_mode: int):  
        msg = MotorSimulationStatus()  
        msg.motor_id = self.motor_id  
        msg.current_position = curr_pos  
        msg.target_position = target_pos  
        msg.velocity = velocity_cmd  
        msg.motion_mode = motion_mode  
        msg.moving = self.moving  
        msg.last_command = self.last_command  
        self.status_pub.publish(msg)  

    # ----------------------------------------------------------------  
    # Main simulation loop  
    # ----------------------------------------------------------------  
    def update_simulation(self):  
        with self.lock:  
            dt = self.timer_period  
  
            pulse_per_round = self._get_pulse_per_round()  
            motion_mode = self._get_motion_mode()  
            target_pos = self._get_target_pos()  
            velocity_cmd = self._get_velocity_cmd()  
            acc = self._get_acc()  
            dec = self._get_dec()  
            curr_pos = self._get_current_pos()  
  
            # If we’re not actively moving, zero the “actual rpm” in non-velocity modes  
            if not self.moving:  
                if motion_mode != MOTION_MODE_VEL:  
                    self.current_rpm = 0.0  
            else:  
                # We are in a "moving" state  
                if motion_mode == MOTION_MODE_ABS or motion_mode == MOTION_MODE_REL:  
                    # Move at constant velocity (no ramp)  
                    pulses_per_sec = (velocity_cmd * pulse_per_round) / 60.0  
                    direction = 1 if (target_pos > curr_pos) else -1  
  
                    # If commanded velocity is 0, fallback  
                    if abs(velocity_cmd) < 1e-3:  
                        pulses_per_sec = 100 * direction  
  
                    step = direction * abs(pulses_per_sec) * dt  
                    next_pos = curr_pos + step  
  
                    # Check if we’ve reached/passed the target  
                    if (direction > 0 and next_pos >= target_pos) or (direction < 0 and next_pos <= target_pos):  
                        curr_pos = target_pos  
                        self.moving = False  
                    else:  
                        curr_pos = next_pos  
  
                    # Actual RPM = commanded RPM  
                    self.current_rpm = float(velocity_cmd)  
  
                elif motion_mode == MOTION_MODE_VEL:  
                    # Accelerate or decelerate towards velocity_cmd  
                    acc_slope = 1_000_000 / acc if acc else 9999999  
                    dec_slope = 1_000_000 / dec if dec else 9999999  
                    delta_rpm = velocity_cmd - self.current_rpm  
  
                    if abs(delta_rpm) > 1e-3:  
                        slope = acc_slope if delta_rpm > 0 else dec_slope  
                        dv = slope * dt  
                        if abs(delta_rpm) <= dv:  
                            self.current_rpm = velocity_cmd  
                        else:  
                            self.current_rpm += dv * (delta_rpm / abs(delta_rpm))  
  
                    pulses_per_sec = (self.current_rpm * pulse_per_round) / 60.0  
                    curr_pos += pulses_per_sec * dt  
  
            # Save updated position to registers  
            self._set_current_pos(curr_pos)  
  
            # Optionally store current_rpm in a register, e.g. 0x600C  
            # self.registers[0x600C] = LeadshineMotor._to_unsigned_16bit_from_signed(int(self.current_rpm))  
  
            # Publish status  
            self._publish_status(float(curr_pos), float(target_pos), float(velocity_cmd), motion_mode)  
  
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
            # Possibly interpret special commands or triggers  
            self._interpret_register_write(addr, val)  
  
    def _interpret_register_write(self, addr, value):  
        if addr == 0x1801:  
            # Possibly a jog command: 0x4001 (left), 0x4002 (right)  
            self._handle_jog(value)  
        elif addr == 0x6002:  
            # Control word  
            self._handle_control_word(value)  
        elif addr == 0x6200:  
            # Motion mode  
            pass  
        # add more if needed  
  
    def _handle_jog(self, cmd):  
        # 0x4001 => jog left => negative velocity  
        # 0x4002 => jog right => positive velocity  
        if cmd == 0x4001:  
            signed_val = LeadshineMotor._to_unsigned_16bit_from_signed(-100)  
            self.registers[0x6203] = signed_val  
            self.registers[0x6200] = MOTION_MODE_VEL  
            self.moving = True  
        elif cmd == 0x4002:  
            signed_val = LeadshineMotor._to_unsigned_16bit_from_signed(100)  
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
            self.moving = True  
        elif cmd == 0x0040:  
            self.current_rpm = 0.0  
            self.moving = False  
        elif cmd == 0x0021:  
            self.registers[0x602C] = 0  
            self.registers[0x602D] = 0  
    
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