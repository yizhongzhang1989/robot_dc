import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from feetech_servo.servo_controller import FeetechServo
from modbus_driver_interfaces.srv import ModbusRequest


class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')

        # Declare and read motor ID parameter
        self.declare_parameter('device_id', 1)
        self.device_id = self.get_parameter('device_id').value
        self.get_logger().info(f"device_id from param = {self.device_id}")

        # motor instance
        self.motor = FeetechServo(self.device_id, self)

        # Set up command subscriber (immediate)
        self.cmd_sub = self.create_subscription(String,  f'/motor{self.device_id}/cmd', self.command_callback, 10)
        self.get_logger().info(f"📡 Subscription to /motor{self.device_id}/cmd created")

        # Set up non-blocking timer to check for service availability
        self.service_check_timer = self.create_timer(1.0, self.initialize_motor_params)
        self.get_logger().info("⏳ Waiting for /modbus_request service...")

    def initialize_motor_params(self):
        if self.motor.cli.service_is_ready():
            self.get_logger().info("✅ /modbus_request service is now available!")

            self.motor.initialize()
            self.get_logger().info("Motor initialized successfully.")

            self.service_check_timer.cancel()
            
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
                case "stop":
                    self.motor.stop()
                    self.get_logger().info("✅ Motor stopped")
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
