import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from leadshine_motor.motor_controller import LeadshineMotor
from modbus_driver.srv import ModbusRequest


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('leadshine_motor')
        self.declare_parameter('motor_id', 1)
        self.motor_id = self.get_parameter('motor_id').value

        self.cli = self.create_client(ModbusRequest, 'modbus_request')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for modbus_request service...')

        self.motor = LeadshineMotor(self.motor_id, self.send_modbus_request)
        self.create_subscription(String, 'motor_cmd', self.command_callback, 10)

        self.get_logger().info("✅ MotorControlNode ready!")

    def send_modbus_request(self, func_code, addr, values):
        req = ModbusRequest.Request()
        req.function_code = func_code
        req.slave_id = self.motor_id
        req.address = addr
        req.values = values

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)
        return future.result()

    def command_callback(self, msg):
        cmd = msg.data.lower()
        try:
            if cmd == "jog_left":
                self.motor.jog_left()
            elif cmd == "jog_right":
                self.motor.jog_right()
            elif cmd == "stop":
                self.motor.abrupt_stop()
            elif cmd == "set_zero":
                self.motor.set_zero_position()
            elif cmd == "move_absolute":
                self.motor.move_absolute()
            elif cmd == "move_relative":
                self.motor.move_relative()
            elif cmd == "move_velocity":
                self.motor.move_velocity()
            else:
                self.get_logger().warn(f"Unknown command: {cmd}")
            self.get_logger().info(f"Executed command: {cmd}")
        except Exception as e:
            self.get_logger().error(f"Command failed: {e}")

    def destroy_node(self):
        self.modbus.disconnect()
        super().destroy_node()

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
