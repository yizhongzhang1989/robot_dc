import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import copy
import datetime


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')
        self.seq_id = 0

        self.motor_left_pub = self.create_publisher(String, '/motor1/cmd', 10)
        self.motor_right_pub = self.create_publisher(String, '/motor2/cmd', 10)
        self.servo_left_pub = self.create_publisher(String, '/motor17/cmd', 10)
        self.servo_right_pub = self.create_publisher(String, '/motor18/cmd', 10)
        self.platform_pub = self.create_publisher(String, '/platform/cmd', 10)

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

        if msg.axes[4] != self.last_joy_msg.axes[4]:  # Platform control
            self.handle_platform_control(4, msg.axes[4])

        if msg.axes[5] != self.last_joy_msg.axes[5]:  # Platform control
            self.handle_platform_control(5, msg.axes[5])

        # Button-based control
        if msg.buttons[6] != self.last_joy_msg.buttons[6]:  # LT button
            self.handle_lt_button(msg.buttons[6])

        if msg.buttons[7] != self.last_joy_msg.buttons[7]:  # RT button
            self.handle_rt_button(msg.buttons[7])

        if msg.buttons[0] != self.last_joy_msg.buttons[0]:  # X button
            if msg.buttons[0] == 1:
                self.get_logger().info('X button pressed: motor1 home_back, motor2 home_back')
                self.send_motor_cmd(self.motor_left_pub, 'home_back')
                self.send_motor_cmd(self.motor_right_pub, 'home_back')

        if msg.buttons[1] != self.last_joy_msg.buttons[1]:
            if msg.buttons[1] == 1:
                self.get_logger().info('A button pressed: motor1 home_pos, motor2 home_pos')
                self.send_motor_cmd(self.motor_left_pub, 'set_home 1000 30 250 250 200 200')
                self.send_motor_cmd(self.motor_right_pub, 'set_home 1000 30 250 250 200 200')
                self.send_motor_cmd(self.motor_left_pub, 'home_pos')
                self.send_motor_cmd(self.motor_right_pub, 'home_pos')

        if msg.buttons[2] != self.last_joy_msg.buttons[2]:  # B button
            if msg.buttons[2] == 1:
                self.get_logger().info('B button pressed: motor1 home_neg, motor2 home_neg')
                self.send_motor_cmd(self.motor_left_pub, 'home_neg')
                self.send_motor_cmd(self.motor_right_pub, 'home_neg')

        if msg.buttons[3] != self.last_joy_msg.buttons[3]:  # Y button
            if msg.buttons[3] == 1:
                self.get_logger().info('Y button pressed: motor1 set_zero, motor2 set_zero')
                self.send_motor_cmd(self.motor_left_pub, 'set_zero')
                self.send_motor_cmd(self.motor_right_pub, 'set_zero')

        if msg.buttons[4] != self.last_joy_msg.buttons[4]:  # LB button
            if msg.buttons[4] == 1:
                self.get_logger().info('LB button pressed: motor1 set_limit 0 -902000, motor2 set_limit 0 -902000')
                self.send_motor_cmd(self.motor_left_pub, 'set_limit 100 -902000')
                self.send_motor_cmd(self.motor_right_pub, 'set_limit 100 -902000')

        if msg.buttons[5] != self.last_joy_msg.buttons[5]:  # RB button
            if msg.buttons[5] == 1:
                self.get_logger().info('RB button pressed: motor1 reset_limit, motor2 reset_limit')
                self.send_motor_cmd(self.motor_left_pub, 'reset_limit')
                self.send_motor_cmd(self.motor_right_pub, 'reset_limit')

        # --- New logic for back (button 8) and start (button 9) ---
        if msg.buttons[8] != self.last_joy_msg.buttons[8]:  # Back button
            if msg.buttons[8] == 1:
                self.get_logger().info('Back button pressed: set motor1 and motor2 velocity to 100 and move')
                self.send_motor_cmd(self.motor_left_pub, 'set_vel 100')
                self.send_motor_cmd(self.motor_right_pub, 'set_vel 100')
                self.send_motor_cmd(self.motor_left_pub, 'move_vel')
                self.send_motor_cmd(self.motor_right_pub, 'move_vel')
            else:
                # If released, you may want to stop motors (optional)
                pass
        if msg.buttons[9] != self.last_joy_msg.buttons[9]:  # Start button
            if msg.buttons[9] == 1:
                self.get_logger().info('Start button pressed: set motor1 and motor2 velocity to -100 and move')
                self.send_motor_cmd(self.motor_left_pub, 'set_vel -100')
                self.send_motor_cmd(self.motor_right_pub, 'set_vel -100')
                self.send_motor_cmd(self.motor_left_pub, 'move_vel')
                self.send_motor_cmd(self.motor_right_pub, 'move_vel')
            else:
                # If released, you may want to stop motors (optional)
                pass

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
                self.get_logger().info(f'Motor {motor_id}: set_pos 1')
                self.send_motor_cmd(publisher, 'set_pos 1')
            else:
                self.get_logger().info(f'Motor {motor_id}: set_pos 4090')
                self.send_motor_cmd(publisher, 'set_pos 4090')

    def handle_platform_control(self, axis_index, axis_value):
        if axis_index == 4:     # forward/backward
            if axis_value > 0.5:
                self.get_logger().info('Platform: forward')
                self.send_motor_cmd(self.platform_pub, 'forward 1')
            elif axis_value < -0.5:
                self.get_logger().info('Platform: backward')
                self.send_motor_cmd(self.platform_pub, 'backward 1')
            else:
                self.get_logger().info('Platform: stop')
                self.send_motor_cmd(self.platform_pub, 'forward 0')
        elif axis_index == 5:   # up/down
            if axis_value > 0.5:
                self.get_logger().info('Platform: up')
                self.send_motor_cmd(self.platform_pub, 'up 1')
            elif axis_value < -0.5:
                self.get_logger().info('Platform: down')
                self.send_motor_cmd(self.platform_pub, 'down 1')
            else:
                self.get_logger().info('Platform: stop')
                self.send_motor_cmd(self.platform_pub, 'up 0')

    def handle_lt_button(self, button_value):
        """Handle LT button press to stop motor1"""
        if button_value == 1:  # Button pressed
            self.get_logger().info('LT button pressed: stopping motor1')
            self.send_motor_cmd(self.motor_left_pub, 'stop')

    def handle_rt_button(self, button_value):
        """Handle RT button press to stop motor2"""
        if button_value == 1:  # Button pressed
            self.get_logger().info('RT button pressed: stopping motor2')
            self.send_motor_cmd(self.motor_right_pub, 'stop')

    def send_motor_cmd(self, pub, command: str, seq_id=None):
        if seq_id is None:
            self.seq_id += 1
            seq_id = self.seq_id
        msg = String()
        msg.data = f"seq:{seq_id}|{command}"
        pub.publish(msg)
        now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        self.get_logger().info(f"[SEQ {seq_id}] [{now}] Sent command: {command}")


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    rclpy.spin(node)
    rclpy.shutdown()
