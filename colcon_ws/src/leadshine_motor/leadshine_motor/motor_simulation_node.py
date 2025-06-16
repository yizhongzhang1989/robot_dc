import rclpy  
from rclpy.node import Node  
  
# Import your existing LeadshineMotor class.    
# Adjust the import path if your file or module name differs.  
from leadshine_motor.motor_controller import LeadshineMotor  
  
from modbus_driver_interfaces.msg import ModbusPacket
from modbus_driver_interfaces.srv import ModbusRequest  
from modbus_driver_interfaces.msg import MotorSimulationStatus  
from modbus_devices.utils import *

import threading  
import time  
  
# Mode constants (used by normal non-jog logic)  
MOTION_MODE_ABS = 0x0001  
MOTION_MODE_REL = 0x0041  
MOTION_MODE_VEL = 0x0002  
  
class MotorSimulationNode(Node):  
    def __init__(self):  
        super().__init__('motor_simulation_node')  
  
        self.declare_parameter('motor_id', 1)  
        self.motor_id = self.get_parameter('motor_id').value  
        self.get_logger().info(f"Simulating motor_id = {self.motor_id}")  
    
        self.subscription = self.create_subscription(
            ModbusPacket,
            '/modbus_sim_cable',
            self.handle_modbus_packet,
            10
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
  
        # Jog-specific fields  
        self.jog_active = False  
        self.jog_speed = 0.0  
        self.jog_end_time = 0.0  
  
        # Publisher  
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
        return from_unsigned_16bit_to_signed(self.registers[0x0001])  
  
    def _get_motion_mode(self) -> int:  
        return self.registers[0x6200] & 0xFFFF  
  
    def _get_target_pos(self) -> int:  
        return from_unsigned_16bit_regs_to_signed_32bit(  
            self.registers[0x6201], self.registers[0x6202]  
        )  
  
    def _get_velocity_cmd(self) -> int:  
        return from_unsigned_16bit_to_signed(self.registers[0x6203])  
  
    def _get_acc(self) -> int:  
        return from_unsigned_16bit_to_signed(self.registers[0x6205])  
  
    def _get_dec(self) -> int:  
        return from_unsigned_16bit_to_signed(self.registers[0x6206])  
  
    def _get_current_pos(self) -> int:  
        return from_unsigned_16bit_regs_to_signed_32bit(  
            self.registers[0x602C], self.registers[0x602D]  
        )  
  
    def _set_current_pos(self, pos: int):  
        high, low = to_unsigned_16bit_regs_from_signed_32bit(int(pos))  
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
  
            if self.jog_active:  
                # Jog logic overrides all other modes  
                pulses_per_sec = (self.jog_speed * pulse_per_round) / 60.0  
                curr_pos += pulses_per_sec * dt  
                self.current_rpm = self.jog_speed  
  
                # Check if 50 ms has passed  
                if time.time() >= self.jog_end_time:  
                    self.jog_active = False  
                    self.moving = False  
                    self.jog_speed = 0.0  
                    self.current_rpm = 0.0  
  
            else:  
                # Normal (non-jog) logic  
                if not self.moving:  
                    # If motor is not moving and we're not in velocity mode, rpm=0  
                    if motion_mode != MOTION_MODE_VEL:  
                        self.current_rpm = 0.0  
                else:  
                    # We are in a "moving" state  
                    if motion_mode in (MOTION_MODE_ABS, MOTION_MODE_REL):  
                        # Simplified constant velocity  
                        pulses_per_sec = (velocity_cmd * pulse_per_round) / 60.0  
                        direction = 1 if (target_pos > curr_pos) else -1  
  
                        if abs(velocity_cmd) < 1e-3:  
                            pulses_per_sec = 100 * direction  
  
                        step = direction * abs(pulses_per_sec) * dt  
                        next_pos = curr_pos + step  
  
                        if (direction > 0 and next_pos >= target_pos) or (direction < 0 and next_pos <= target_pos):  
                            curr_pos = target_pos  
                            self.moving = False  
                        else:  
                            curr_pos = next_pos  
  
                        self.current_rpm = float(velocity_cmd)  
  
                    elif motion_mode == MOTION_MODE_VEL:  
                        # Velocity ramp  
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
  
            # Update position register  
            self._set_current_pos(curr_pos)  
  
            # Publish status  
            self._publish_status(  
                float(curr_pos), float(target_pos), float(velocity_cmd), motion_mode  
            )  
  
    # ----------------------------------------------------------------  
    # Service: fake Modbus read/write  
    # ----------------------------------------------------------------  
    def handle_modbus_packet(self, msg: ModbusPacket):
        with self.lock:
            if msg.slave_id != self.motor_id:
                return

            try:
                if msg.function_code == 3:
                    read_values = self._read_registers(msg.address, msg.count)
                    self.log_modbus(f"FC=3 read {read_values} from addr={hex(msg.address)}")
                    # Possibly publish a response if needed later

                elif msg.function_code == 6:
                    if len(msg.values) != 1:
                        raise ValueError("FC=6 expects exactly 1 value")
                    self._write_registers(msg.address, msg.values)
                    self.log_modbus(f"FC=6 wrote {msg.values} to addr={hex(msg.address)}")

                elif msg.function_code == 16:
                    self._write_registers(msg.address, msg.values)
                    self.log_modbus(f"FC=16 wrote {msg.values} to addr={hex(msg.address)}")

                else:
                    raise ValueError(f"Unsupported function code: {msg.function_code}")

            except Exception as e:
                self.get_logger().error(f"Simulation error: {e}")
    
    def log_modbus(self, msg: str):  
        self.get_logger().info(f"[Motor {self.motor_id}] {msg}")  
        self.last_command = msg  
  
    # ----------------------------------------------------------------  
    # Internal: read/write registers as 16-bit chunks  
    # ----------------------------------------------------------------  
    def _read_registers(self, start_addr, count):  
        result = []  
        for offset in range(count):  
            addr = start_addr + offset  
            val = self.registers.get(addr, 0)  
            result.append(val & 0xFFFF)  
        return result  
  
    def _write_registers(self, start_addr, values):  
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
            self._handle_jog(value)  
        elif addr == 0x6002:  
            self._handle_control_word(value)  
        # ... possibly more  
  
    def _handle_jog(self, cmd):  
        """  
        Writing:  
          0x4001 => jog left => negative jog speed  
          0x4002 => jog right => positive jog speed  
        for 50ms, and then stop automatically.  
        Does NOT change 0x6200 or 0x6203; purely internal state.  
        """  
        # Read the configured jog speed (signed RPM)  
        base_speed = from_unsigned_16bit_to_signed(self.registers[0x01E1])  
  
        if cmd == 0x4001:  
            # Negative jog  
            self.jog_speed = -abs(base_speed)  
            self.jog_active = True  
            self.moving = True  
            self.jog_end_time = time.time() + 0.05  # 50 ms  
  
        elif cmd == 0x4002:  
            # Positive jog  
            self.jog_speed = abs(base_speed)  
            self.jog_active = True  
            self.moving = True  
            self.jog_end_time = time.time() + 0.05  
  
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
            # Abrupt stop  
            self.current_rpm = 0.0  
            self.moving = False  
            # Cancel any jog in progress  
            self.jog_active = False  
            self.jog_speed = 0.0  
        elif cmd == 0x0021:  
            # Reset position to zero  
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
