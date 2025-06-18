import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from platform_controller.platform_controller import PlatformController


class PlatformControlNode(Node):
    def __init__(self):
        super().__init__('platform_control_node')

        # Declare and read motor ID parameter
        self.declare_parameter('device_id', 1)
        self.device_id = self.get_parameter('device_id').value
        self.get_logger().info(f"device_id from param = {self.device_id}")

        # motor instance
        self.platform = PlatformController(self.device_id, self)

        # Set up command subscriber (immediate)
        self.cmd_sub = self.create_subscription(String,  f'/platform/cmd', self.command_callback, 10)
        self.get_logger().info(f"📡 Subscription to /platform/cmd created")

        # Set up non-blocking timer to check for service availability
        self.service_check_timer = self.create_timer(1.0, self.initialize_motor_params)
        self.get_logger().info("⏳ Waiting for /modbus_request service...")

    def initialize_motor_params(self):
        if self.platform.cli.service_is_ready():
            self.get_logger().info("✅ /modbus_request service is now available!")

            self.platform.initialize()
            self.get_logger().info("Platform initialized successfully.")

            self.service_check_timer.cancel()
            
    def command_callback(self, msg):
        parts = msg.data.strip().lower().split()
        if not parts:
            return

        cmd = parts[0]
        arg = int(parts[1]) if len(parts) > 1 and parts[1].lstrip('-').isdigit() else None

        if self.platform is None:
            self.get_logger().warn("⏳ Platform not initialized yet. Command ignored.")
            return

        try:
            match cmd:
                case "up":
                    if arg is not None:
                        self.platform.up(arg)
                        self.get_logger().info(f"✅ Platform moving up with flag {arg}")
                case "down":
                    if arg is not None:
                        self.platform.down(arg)
                        self.get_logger().info(f"✅ Platform moving down with flag {arg}")
                case "forward":
                    if arg is not None:
                        self.platform.forward(arg)
                        self.get_logger().info(f"✅ Platform moving forward with flag {arg}")
                case "backward":
                    if arg is not None:
                        self.platform.backward(arg)
                        self.get_logger().info(f"✅ Platform moving backward with flag {arg}")
                case _:
                    self.get_logger().warn(f"Unknown command: {cmd}")
        except Exception as e:
            self.get_logger().error(f"❌ Command '{cmd}' failed: {e}")


def main():
    rclpy.init()
    node = PlatformControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
