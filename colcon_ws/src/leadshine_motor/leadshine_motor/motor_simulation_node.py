import rclpy  
from rclpy.node import Node  
from modbus_driver_interfaces.srv import ModbusRequest  
   
import threading  
import time  
   
# Some mode constants for readability  
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
        # IMPORTANT: in a multi-motor setup, you will typically remap /modbus_request  
        # so that each motor_simulation_node provides a unique service name, e.g.  
        #   --ros-args --remap /modbus_request:=/modbus_request_m1  (etc.)  
        self.srv = self.create_service(  
            ModbusRequest,  
            'modbus_request',  
            self.handle_modbus_request  
        )  
  
        # Concurrency lock (service callbacks + simulation timer)  
        self.lock = threading.Lock()  
  
        # Internal simulation state --------------------------------  
        # “Registers” that approximate your real motor’s register usage.  
        # Some registers hold 16 bits, some might be pairs that form 32 bits.  
        self.registers = {  
            0x0001: 10000,       # Example: counts per round  
            0x0003: 2,           # Example: loop mode  
            0x0007: 0,           # Example: direction  
            0x6200: MOTION_MODE_ABS,  # Motion mode (default: Absolute)  
            # 0x6201 will store a 32-bit target position, so we keep it as two 16-bit regs:  
            # for convenience, store them as a list [high, low].  
            0x6201: [0, 0],  
            # 0x6203: 16-bit target velocity  
            0x6203: 0,  
            # 0x6205: 16-bit acceleration  
            0x6205: 0,  
            # 0x6206: 16-bit deceleration  
            0x6206: 0,  
            # 0x6002: control word (move commands, stop, set zero, etc.)  
            0x6002: 0,  
            # 0x602C: current position 32-bit, store as [high, low].  
            0x602C: [0, 0],  
            # 0x1801: might store jog commands (0x4001, 0x4002).  
            0x1801: 0,  
        }  
  
        # Physical simulation variables (float for position, velocity, etc.)  
        # The “registers” above are just a “modbus view”; we do real physics here.  
        self.current_position = 0.0  
        self.target_position = 0.0  
        self.velocity = 0.0  
        self.acc = 0.0  
        self.dec = 0.0  
        self.motion_mode = MOTION_MODE_ABS  
        self.moving = False  # Flag to indicate a move is in progress  
        self.start_time = time.time()  
  
        # Create a timer to periodically update the motor's “physics”  
        self.timer_period = 0.01  # 10 ms  
        self.timer = self.create_timer(self.timer_period, self.update_simulation)  
  
        self.get_logger().info("✅ Motor simulation node is ready.")  
  
    def handle_modbus_request(self, request, response):  
        """Service callback that fakes Modbus read/write by referencing internal state."""  
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
                    # “address” is the start, “count” is how many 16-bit registers  
                    read_values = self._read_registers(request.address, request.count)  
                    response.success = True  
                    response.response = read_values  
  
                elif request.function_code == 6:    
                    # Write single register  
                    if len(request.values) != 1:  
                        raise ValueError("FC=6 expects exactly 1 value")  
                    self._write_registers(request.address, request.values)  
                    response.success = True  
                    response.response = request.values  
  
                elif request.function_code == 16:    
                    # Write multiple registers  
                    self._write_registers(request.address, request.values)  
                    response.success = True  
                    response.response = request.values  
  
                else:  
                    raise ValueError(f"Unsupported function code: {request.function_code}")  
  
            except Exception as e:  
                self.get_logger().error(f"Simulation error: {e}")  
                response.success = False  
                response.response = []  
  
        return response  
  
    def _read_registers(self, start_addr, count):  
        """Return 'count' 16-bit registers from 'start_addr' onward."""  
        result = []  
        for offset in range(count):  
            addr = start_addr + offset  
            val = self.registers.get(addr, 0)  
            if isinstance(val, list):  
                # If a register address is stored as a list [high, low],   
                # we return them one at a time.  For a direct read of that address   
                # it might be unusual, but we can handle carefully.  
                # Typically you read the “high” first, then the “low.”  
                # If we see a list, we read the offset from that list if we want to replicate  
                # real 16-bit registers exactly. For a simple approach:  
                #   If you do read_holding_registers(0x602C, 2),   
                #   we expect to return both high and low from that one “address pair.”   
                # So we return them in two consecutive calls.  
                index_in_list = offset  
                if index_in_list < len(val):  
                    result.append(val[index_in_list])  
                else:  
                    result.append(0)  
            else:  
                # Normal 16-bit value  
                result.append(val)  
        return result  
  
    def _write_registers(self, start_addr, values):  
        """Write a list of 16-bit registers beginning at start_addr."""  
        offset = 0  
        while offset < len(values):  
            addr = start_addr + offset  
            # If that address is stored as a list [high, low], we handle it carefully.  
            current_reg = self.registers.get(addr, 0)  
  
            if isinstance(current_reg, list):  
                # We assume we might be writing one or two 16-bit segments  
                # e.g., if the register is for a 32-bit position.  
                for i in range(len(current_reg)):  
                    if offset < len(values):  
                        current_reg[i] = values[offset]  
                        offset += 1  
                # Update the dictionary  
                self.registers[addr] = current_reg  
                # Then interpret what changed in “physics”  
                self._interpret_register_write(addr, current_reg)  
            else:  
                # Single 16-bit value case  
                val = values[offset]  
                self.registers[addr] = val  
                offset += 1  
                # Possibly interpret special commands in the simulation  
                self._interpret_register_write(addr, val)  
  
    def _interpret_register_write(self, addr, value):  
        """  
        Whenever certain registers are written, update the internal  
        simulation variables (position, velocity, mode, or triggers).  
        """  
        # If 'value' is a list, it’s the two 16-bit parts of a 32-bit number.  
        # We can convert it to a signed 32, if needed.  
        def from_regs_to_int32(hilo):  
            raw = (hilo[0] << 16) | (hilo[1] & 0xFFFF)  
            if raw & 0x80000000:  # sign bit  
                raw -= 1 << 32  
            return raw  
  
        # If 'value' is a single 16-bit, we might interpret it:  
        def from_uint16_to_int16(x):  
            return x - 0x10000 if x & 0x8000 else x  
  
        # Example addresses used in motor_controller.py  
        if addr == 0x6200:    
            # motion mode  
            mode = value if isinstance(value, int) else value[0]  
            self.motion_mode = mode  
            self.registers[0x6200] = mode  
  
        elif addr == 0x6201:  
            # target position 32-bit  
            # value is something like [high, low]  
            pos = from_regs_to_int32(value)  
            self.target_position = float(pos)  
  
        elif addr == 0x6203:  
            # target velocity  
            vel = from_uint16_to_int16(value) if isinstance(value, int) else from_uint16_to_int16(value[0])  
            self.velocity = float(vel)  
  
        elif addr == 0x6205:  
            # acceleration  
            acc = from_uint16_to_int16(value) if isinstance(value, int) else from_uint16_to_int16(value[0])  
            self.acc = float(acc)  
  
        elif addr == 0x6206:  
            # deceleration  
            dec = from_uint16_to_int16(value) if isinstance(value, int) else from_uint16_to_int16(value[0])  
            self.dec = float(dec)  
  
        elif addr == 0x1801:  
            # Possibly a jog command: 0x4001 (left) or 0x4002 (right)  
            cmd = value if isinstance(value, int) else value[0]  
            self._handle_jog(cmd)  
  
        elif addr == 0x6002:  
            # Control word: move trigger, stop, set zero, etc.  
            cmd = value if isinstance(value, int) else value[0]  
            self._handle_control_word(cmd)  
  
        # You could interpret many other addresses as needed for your real register map.  
  
    def _handle_jog(self, cmd):  
        """  
        0x4001 => jog left,  0x4002 => jog right  
        We can treat this as a velocity-based command.  
        """  
        if cmd == 0x4001:  
            # Jog left => negative velocity, just pick a value to simulate  
            self.motion_mode = MOTION_MODE_VEL  
            self.velocity = -100.0  
            self.moving = True  
        elif cmd == 0x4002:  
            # Jog right => positive velocity  
            self.motion_mode = MOTION_MODE_VEL  
            self.velocity = 100.0  
            self.moving = True  
  
    def _handle_control_word(self, cmd):  
        """  
        Bits/values we might see written to register 0x6002:  
  
        0x0010 => Trigger move    
        0x0040 => Abrupt stop    
        0x0021 => Set zero position    
        etc.  
        """  
        # For demonstration, just check particular values:  
        if cmd == 0x0010:  
            # Trigger a move (absolute, relative, or velocity)  
            self.start_time = time.time()  
            if self.motion_mode == MOTION_MODE_ABS:  
                # We'll treat this as “move from current_position to target_position”  
                self.moving = True  
            elif self.motion_mode == MOTION_MODE_REL:  
                # target_position is relative  
                self.target_position = self.current_position + self.target_position  
                self.moving = True  
            elif self.motion_mode == MOTION_MODE_VEL:  
                # we might start a velocity move  
                self.moving = True  
  
        elif cmd == 0x0040:  
            # Abrupt stop  
            self.velocity = 0.0  
            self.moving = False  
  
        elif cmd == 0x0021:  
            # Set current position to zero  
            self.current_position = 0.0  
            # Update 0x602C as well  
            self.registers[0x602C] = [0, 0]  
  
    def update_simulation(self):  
        """Called by a ROS timer ~every 10ms to step the “physics.”"""  
        with self.lock:  
            dt = self.timer_period  
  
            # If we are moving, just do a very simplistic update  
            if self.moving:  
                if self.motion_mode == MOTION_MODE_ABS:  
                    # We’ll move toward target_position at “velocity” steps per second.  
                    # If velocity is zero, pick some default or do nothing.  
                    sign = 1.0 if (self.target_position > self.current_position) else -1.0  
                    speed = abs(self.velocity) if self.velocity != 0 else 100.0  
                    step = sign * speed * dt  
                    before = self.current_position  
                    after = before + step  
  
                    # Check if we pass the target:  
                    if (sign > 0 and after >= self.target_position) or (sign < 0 and after <= self.target_position):  
                        self.current_position = self.target_position  
                        self.moving = False  
                    else:  
                        self.current_position = after  
  
                elif self.motion_mode == MOTION_MODE_REL:  
                    # Same logic, but we already adjusted target_position to absolute in _handle_control_word()  
                    sign = 1.0 if (self.target_position > self.current_position) else -1.0  
                    speed = abs(self.velocity) if self.velocity != 0 else 100.0  
                    step = sign * speed * dt  
                    before = self.current_position  
                    after = before + step  
  
                    if (sign > 0 and after >= self.target_position) or (sign < 0 and after <= self.target_position):  
                        self.current_position = self.target_position  
                        self.moving = False  
                    else:  
                        self.current_position = after  
  
                elif self.motion_mode == MOTION_MODE_VEL:  
                    # Simply update position at constant velocity  
                    self.current_position += (self.velocity * dt)  
  
            # Update the 0x602C register to reflect current_position as signed 32-bit  
            self._update_position_register()  
  
    def _update_position_register(self):  
        """Pack current_position (a float) into 602C as two 16-bit regs (signed 32-bit)."""  
        pos_int = int(self.current_position)  
        if pos_int < 0:  
            pos_int += (1 << 32)  # fill up to 32-bit unsigned  
        high = (pos_int >> 16) & 0xFFFF  
        low = pos_int & 0xFFFF  
        self.registers[0x602C] = [high, low]  
  
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
