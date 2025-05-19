import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from leadshine_motor.motor_controller import LeadshineMotor
from modbus_driver_interfaces.srv import ModbusRequest


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('leadshine_motor')
        self.declare_parameter('motor_id', 1)
        self.motor_id = self.get_parameter('motor_id').value

        self.cli = self.create_client(ModbusRequest, 'modbus_request')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for modbus_request service...')

        self.motor = LeadshineMotor(
            self.motor_id, 
            self.send_modbus_request,
            self.recv_modbus_request
        )        
        self.create_subscription(String, 'motor_cmd', self.command_callback, 10)

        self.get_logger().info("✅ MotorControlNode ready!")

    def send_modbus_request(self, func_code, addr, values):
        req = ModbusRequest.Request()
        req.function_code = func_code
        req.slave_id = self.motor_id
        req.address = addr
        req.values = values

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)

        if future.done() and future.result() is not None:
            result = future.result()
            if result.success:
                self.get_logger().info(
                    f"✅ Modbus request OK: fc={func_code} addr={hex(addr)} values={values} → response={result.response}"
                )
                return result.response
            else:
                self.get_logger().error("❌ Modbus request failed: reported failure from service")
        else:
            self.get_logger().error("❌ Modbus request timed out or no result")

        return None
    
    def recv_modbus_request(self, func_code, addr, count):
        req = ModbusRequest.Request()
        req.function_code = func_code
        req.slave_id = self.motor_id
        req.address = addr
        req.count = count
        req.values = []  # Empty for read

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        result = future.result()
        if not result.success:
            self.get_logger().error(f"Modbus read failed at addr {addr}")
        return result.response

    def command_callback(self, msg):
        cmd = msg.data.strip().lower()
        now = self.get_clock().now().to_msg()
        self.get_logger().info(f"📥 Received command '{cmd}' at time: {now.sec}.{now.nanosec:09d}")

        try:
            match cmd:
                case "jog_left":
                    self.motor.jog_left()
                case "jog_right":
                    self.motor.jog_right()
                case "stop":
                    self.motor.abrupt_stop()
                case "set_zero":
                    self.motor.set_zero_position()
                case "move_absolute":
                    self.motor.move_absolute()
                case "move_relative":
                    self.motor.move_relative()
                case "move_velocity":
                    self.motor.move_velocity()
                case _:
                    self.get_logger().warn(f"Unknown command: {cmd}")
                    return
            self.get_logger().info(f"✅ Executed command: {cmd}")
        except Exception as e:
            self.get_logger().error(f"❌ Command failed: {e}")


def main():
    rclpy.init()
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
