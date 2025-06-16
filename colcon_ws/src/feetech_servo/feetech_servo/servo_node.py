import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from feetech_servo.servo_controller import FeetechServo
from modbus_driver_interfaces.srv import ModbusRequest


class ServoControlNode(Node):
    def __init__(self):
        super().__init__('feetech_servo')

        # Declare and read motor ID parameter
        self.declare_parameter('motor_id', 1)
        self.motor_id = self.get_parameter('motor_id').value
        self.get_logger().info(f"motor_id from param = {self.motor_id}")

        # Create service client
        self.cli = self.create_client(ModbusRequest, '/modbus_request')
        self.motor = None  # Will be initialized after service becomes available

        # Set up command subscriber (immediate)
        self.cmd_sub = self.create_subscription(String,  f'/servo{self.motor_id}/cmd', self.command_callback, 10)
        self.get_logger().info(f"📡 Subscription to /servo{self.motor_id}/cmd created")

        # Set up non-blocking timer to check for service availability
        self.service_check_timer = self.create_timer(1.0, self.check_service_ready)
        self.get_logger().info("⏳ Waiting for /modbus_request service...")

    def check_service_ready(self):
        if self.cli.service_is_ready():
            self.get_logger().info("✅ /modbus_request service is now available!")

            if self.motor is None:
                self.motor = FeetechServo(
                    self.motor_id,
                    self.send_modbus_request,
                    self.recv_modbus_request
                )

            self.service_check_timer.cancel()
            
    def send_modbus_request(self, func_code, addr, values):
        if not self.cli.service_is_ready():
            self.get_logger().warn("Modbus service not available. Skipping write request.")
            return

        req = ModbusRequest.Request()
        req.function_code = func_code
        req.slave_id = self.motor_id
        req.address = addr
        req.values = values

        future = self.cli.call_async(req)

        def handle_response(fut):
            if fut.result() is not None and fut.result().success:
                self.get_logger().info(
                    f"✅ Modbus write OK: fc={func_code} addr={hex(addr)} → {values} => {fut.result().response}"
                )
            else:
                self.get_logger().error("❌ Modbus request failed or timed out")

        future.add_done_callback(handle_response)

    def recv_modbus_request(self, func_code, addr, count, callback=None):
        if not self.cli.service_is_ready():
            self.get_logger().warn("Modbus service not available. Skipping read request.")
            if callback:
                callback([])
            return []
        
        # callback cannot be None if func_code is 3
        if func_code != 3 and callback is not None:
            self.get_logger().warn("Callback is required for this function code.")
            return []


        req = ModbusRequest.Request()
        req.function_code = func_code
        req.slave_id = self.motor_id
        req.address = addr
        req.count = count
        req.values = []

        future = self.cli.call_async(req)

        future.add_done_callback(
            lambda fut: callback(
                fut.result().response if fut.result() and fut.result().success else []
            )
        )
        return []

    def command_callback(self, msg):
        parts = msg.data.strip().lower().split()
        if not parts:
            return

        cmd = parts[0]
        arg = int(parts[1]) if len(parts) > 1 and parts[1].lstrip('-').isdigit() else None

        if self.motor is None:
            self.get_logger().warn("⏳ Motor not initialized yet. Command ignored.")
            return

        try:
            match cmd:
                case "set_pos":
                    if arg is not None:
                        self.motor.set_target_position(arg)
                        self.get_logger().info(f"✅ Set position to {arg}")
                case "set_vel":
                    if arg is not None:
                        self.motor.set_target_velocity(arg)
                        self.get_logger().info(f"✅ Set velocity to {arg}")
                case "set_acc":
                    if arg is not None:
                        self.motor.set_target_acceleration(arg)
                        self.get_logger().info(f"✅ Set acceleration to {arg}")
                case _:
                    self.get_logger().warn(f"Unknown command: {cmd}")
        except Exception as e:
            self.get_logger().error(f"❌ Command '{cmd}' failed: {e}")


def main():
    rclpy.init()
    node = ServoControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
