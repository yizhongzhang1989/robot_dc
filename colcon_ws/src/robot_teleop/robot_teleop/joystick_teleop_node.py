import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        self.motor1_pub = self.create_publisher(String, '/motor1/motor_cmd', 10)
        self.motor2_pub = self.create_publisher(String, '/motor2/motor_cmd', 10)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.motor1_active = False
        self.motor2_active = False

    def joy_callback(self, msg):
        # Button 0 → Motor 1 jog_left
        if msg.buttons[0] and not self.motor1_active:
            self.get_logger().info('Motor 1: jog_left')
            self.send_motor_cmd(self.motor1_pub, 'jog_left')
            self.motor1_active = True
        elif not msg.buttons[0] and self.motor1_active:
            self.get_logger().info('Motor 1: stop')
            self.send_motor_cmd(self.motor1_pub, 'stop')
            self.motor1_active = False

        # Button 1 → Motor 2 jog_left
        if msg.buttons[1] and not self.motor2_active:
            self.get_logger().info('Motor 2: jog_left')
            self.send_motor_cmd(self.motor2_pub, 'jog_left')
            self.motor2_active = True
        elif not msg.buttons[1] and self.motor2_active:
            self.get_logger().info('Motor 2: stop')
            self.send_motor_cmd(self.motor2_pub, 'stop')
            self.motor2_active = False

    def send_motor_cmd(self, pub, command: str):
        msg = String()
        msg.data = command
        pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    rclpy.spin(node)
    rclpy.shutdown()
