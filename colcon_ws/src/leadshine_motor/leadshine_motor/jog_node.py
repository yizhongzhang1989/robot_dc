import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from leadshine_motor.motor_controller import LeadshineMotor
from leadshine_motor.modbus_interface import ModbusRTUInterface

class JogNode(Node):
    def __init__(self):
        super().__init__('leadshine_motor_jog')

        self.declare_parameter('motor_id', 1)
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)

        motor_id = self.get_parameter('motor_id').value
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        self.get_logger().info(f"Connecting to motor {motor_id} on {port}...")

        self.modbus = ModbusRTUInterface(port, baudrate)
        if not self.modbus.connect():
            self.get_logger().error(f"❌ Could not connect to {port}")
            return

        self.motor = LeadshineMotor(self.modbus, motor_id)

        self.create_subscription(String, 'jog_cmd', self.jog_callback, 10)
        self.get_logger().info("✅ JogNode ready!")

    def jog_callback(self, msg):
        try:
            self.motor.jog(msg.data.lower())
            self.get_logger().info(f"Jogging {msg.data.lower()}")
        except Exception as e:
            self.get_logger().error(f"Jog failed: {e}")

    def destroy_node(self):
        self.modbus.disconnect()
        super().destroy_node()

def main():
    rclpy.init()
    node = JogNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
