import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import copy


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        self.motor_left_pub = self.create_publisher(String, '/motor1/cmd', 10)
        self.motor_right_pub = self.create_publisher(String, '/motor2/cmd', 10)
        self.servo_left_pub = self.create_publisher(String, '/motor17/cmd', 10)
        self.servo_right_pub = self.create_publisher(String, '/motor18/cmd', 10)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.last_joy_msg = None

    def joy_callback(self, msg: Joy):
        if self.last_joy_msg is None:
            self.last_joy_msg = copy.deepcopy(msg)

        # Axis-based control
        if msg.axes[1] != self.last_joy_msg.axes[1]:  # Left stick vertical
            self.handle_motor_axis_control(msg.axes[1], self.motor_left_pub, motor_id=1)

        if msg.axes[3] != self.last_joy_msg.axes[3]:  # Right stick vertical
            self.handle_motor_axis_control(msg.axes[3], self.motor_right_pub, motor_id=2)

        if msg.axes[0] != self.last_joy_msg.axes[0]:  # Left stick horizontal
            self.handle_servo_axis_control(msg.axes[0], self.servo_left_pub, motor_id=17)

        if msg.axes[2] != self.last_joy_msg.axes[2]:  # Right stick horizontal
            self.handle_servo_axis_control(msg.axes[2], self.servo_right_pub, motor_id=18)

        self.last_joy_msg = copy.deepcopy(msg)

    def handle_motor_axis_control(self, axis_value, publisher, motor_id):
        speed = -int(round(axis_value * 300))

        if speed == 0:
            self.get_logger().info(f'Motor {motor_id}: stop')
            self.send_motor_cmd(publisher, 'stop')
        else:
            self.get_logger().info(f'Motor {motor_id}: set_vel {speed} + move_vel')
            self.send_motor_cmd(publisher, f'set_vel {speed}')
            self.send_motor_cmd(publisher, 'move_vel')

    def handle_servo_axis_control(self, axis_value, publisher, motor_id):
        speed = -int(round(axis_value * 5))

        if speed == 0:
            self.get_logger().info(f'Motor {motor_id}: stop')
            self.send_motor_cmd(publisher, 'stop')
        else:
            self.get_logger().info(f'Motor {motor_id}: set_vel {speed}')
            self.send_motor_cmd(publisher, f'set_vel {abs(speed)}')

            if speed < 0:
                self.get_logger().info(f'Motor {motor_id}: set_pos 0')
                self.send_motor_cmd(publisher, 'set_pos 0')
            else:
                self.get_logger().info(f'Motor {motor_id}: set_pos 4095')
                self.send_motor_cmd(publisher, 'set_pos 4095')

    def send_motor_cmd(self, pub, command: str):
        msg = String()
        msg.data = command
        pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    rclpy.spin(node)
    rclpy.shutdown()
