import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String
import datetime


class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')
        self.seq_id = 0

        # Publishers
        self.motor_left_pub = self.create_publisher(String, '/motor1/cmd', 10)
        self.motor_right_pub = self.create_publisher(String, '/motor2/cmd', 10)
        self.servo_left_pub = self.create_publisher(String, '/motor17/cmd', 10)
        self.servo_right_pub = self.create_publisher(String, '/motor18/cmd', 10)
        self.platform_pub = self.create_publisher(String, '/platform/cmd', 10)

        # Subscriptions
        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        # Timer for throttled command sending
        self.timer = self.create_timer(0.05, self.command_timer_cb)

        # State
        self.target_speeds = {
            'motor1': 0,
            'motor2': 0,
            'servo17': 0,
            'servo18': 0,
            'platform_forward': 0,
            'platform_up': 0,
        }
        self.last_sent_speeds = dict(self.target_speeds)

        self.last_joy_msg = None

    def joy_callback(self, msg: Joy):
        # Cache last message
        if self.last_joy_msg is None:
            self.last_joy_msg = msg

        # Update target speeds

        # Motor1: Left stick vertical
        self.target_speeds['motor1'] = -int(round(msg.axes[1] * 300))
        # Motor2: Right stick vertical
        self.target_speeds['motor2'] = -int(round(msg.axes[3] * 300))
        # Servo17: Left stick horizontal
        self.target_speeds['servo17'] = -int(round(msg.axes[0] * 5))
        # Servo18: Right stick horizontal
        self.target_speeds['servo18'] = -int(round(msg.axes[2] * 5))

        # Platform: axes[4] forward/backward
        if msg.axes[4] > 0.5:
            self.target_speeds['platform_forward'] = 1
        elif msg.axes[4] < -0.5:
            self.target_speeds['platform_forward'] = -1
        else:
            self.target_speeds['platform_forward'] = 0

        # Platform: axes[5] up/down
        if msg.axes[5] > 0.5:
            self.target_speeds['platform_up'] = 1
        elif msg.axes[5] < -0.5:
            self.target_speeds['platform_up'] = -1
        else:
            self.target_speeds['platform_up'] = 0

        # Buttons handled immediately
        self.handle_buttons(msg)

        self.last_joy_msg = msg

    def command_timer_cb(self):
        # Motor1
        self.send_motor_velocity('motor1', self.motor_left_pub, motor_id=1)
        # Motor2
        self.send_motor_velocity('motor2', self.motor_right_pub, motor_id=2)
        # Servo17
        self.send_servo_command('servo17', self.servo_left_pub, motor_id=17)
        # Servo18
        self.send_servo_command('servo18', self.servo_right_pub, motor_id=18)
        # Platform
        self.send_platform_command()

    def send_motor_velocity(self, key, publisher, motor_id):
        target = self.target_speeds[key]
        last = self.last_sent_speeds[key]

        if target != last:
            if target == 0:
                self.get_logger().info(f'Motor {motor_id}: stop')
                self.send_motor_cmd(publisher, 'stop')
            else:
                self.get_logger().info(f'Motor {motor_id}: set_vel {target} + move_vel')
                self.send_motor_cmd(publisher, f'set_vel {target}')
                self.send_motor_cmd(publisher, 'move_vel')
            self.last_sent_speeds[key] = target

    def send_servo_command(self, key, publisher, motor_id):
        target = self.target_speeds[key]
        last = self.last_sent_speeds[key]

        if target != last:
            if target == 0:
                self.get_logger().info(f'Motor {motor_id}: stop')
                self.send_motor_cmd(publisher, 'stop')
            else:
                self.get_logger().info(f'Motor {motor_id}: set_vel {abs(target)}')
                self.send_motor_cmd(publisher, f'set_vel {abs(target)}')
                pos = '1' if target < 0 else '4090'
                self.get_logger().info(f'Motor {motor_id}: set_pos {pos}')
                self.send_motor_cmd(publisher, f'set_pos {pos}')
            self.last_sent_speeds[key] = target

    def send_platform_command(self):
        fwd = self.target_speeds['platform_forward']
        up = self.target_speeds['platform_up']

        if fwd != self.last_sent_speeds['platform_forward']:
            if fwd == 1:
                self.get_logger().info('Platform: forward')
                self.send_motor_cmd(self.platform_pub, 'forward 1')
            elif fwd == -1:
                self.get_logger().info('Platform: backward')
                self.send_motor_cmd(self.platform_pub, 'backward 1')
            else:
                self.get_logger().info('Platform: stop forward')
                self.send_motor_cmd(self.platform_pub, 'forward 0')
            self.last_sent_speeds['platform_forward'] = fwd

        if up != self.last_sent_speeds['platform_up']:
            if up == 1:
                self.get_logger().info('Platform: up')
                self.send_motor_cmd(self.platform_pub, 'up 1')
            elif up == -1:
                self.get_logger().info('Platform: down')
                self.send_motor_cmd(self.platform_pub, 'down 1')
            else:
                self.get_logger().info('Platform: stop up')
                self.send_motor_cmd(self.platform_pub, 'up 0')
            self.last_sent_speeds['platform_up'] = up

    def handle_buttons(self, msg):
        # Buttons (immediate response)
        buttons = msg.buttons

        if buttons[6] == 1:
            self.get_logger().info('LT button pressed: stopping motor1')
            self.send_motor_cmd(self.motor_left_pub, 'stop')

        if buttons[7] == 1:
            self.get_logger().info('RT button pressed: stopping motor2')
            self.send_motor_cmd(self.motor_right_pub, 'stop')

        if buttons[0] == 1:
            self.get_logger().info('X button pressed: home_back')
            self.send_motor_cmd(self.motor_left_pub, 'home_back')
            self.send_motor_cmd(self.motor_right_pub, 'home_back')

        if buttons[1] == 1:
            self.get_logger().info('A button pressed: set_home + home_pos')
            self.send_motor_cmd(self.motor_left_pub, 'set_home 1000 30 250 250 200 200')
            self.send_motor_cmd(self.motor_right_pub, 'set_home 1000 30 250 250 200 200')
            self.send_motor_cmd(self.motor_left_pub, 'home_pos')
            self.send_motor_cmd(self.motor_right_pub, 'home_pos')

        if buttons[2] == 1:
            self.get_logger().info('B button pressed: set_home + home_neg')
            self.send_motor_cmd(self.motor_left_pub, 'set_home 1000 30 250 250 200 200')
            self.send_motor_cmd(self.motor_right_pub, 'set_home 1000 30 250 250 200 200')
            self.send_motor_cmd(self.motor_left_pub, 'home_neg')
            self.send_motor_cmd(self.motor_right_pub, 'home_neg')

        if buttons[3] == 1:
            self.get_logger().info('Y button pressed: set_zero')
            self.send_motor_cmd(self.motor_left_pub, 'set_zero')
            self.send_motor_cmd(self.motor_right_pub, 'set_zero')

        if buttons[4] == 1:
            self.get_logger().info('LB button pressed: set_limit')
            self.send_motor_cmd(self.motor_left_pub, 'set_limit 100 -902000')
            self.send_motor_cmd(self.motor_right_pub, 'set_limit 100 -902000')

        if buttons[5] == 1:
            self.get_logger().info('RB button pressed: reset_limit')
            self.send_motor_cmd(self.motor_left_pub, 'reset_limit')
            self.send_motor_cmd(self.motor_right_pub, 'reset_limit')

        if buttons[8] == 1:
            self.get_logger().info('Back button pressed: set_vel + move_vel +100')
            self.send_motor_cmd(self.motor_left_pub, 'set_vel 100')
            self.send_motor_cmd(self.motor_right_pub, 'set_vel 100')
            self.send_motor_cmd(self.motor_left_pub, 'move_vel')
            self.send_motor_cmd(self.motor_right_pub, 'move_vel')

        if buttons[9] == 1:
            self.get_logger().info('Start button pressed: set_vel + move_vel -100')
            self.send_motor_cmd(self.motor_left_pub, 'set_vel -100')
            self.send_motor_cmd(self.motor_right_pub, 'set_vel -100')
            self.send_motor_cmd(self.motor_left_pub, 'move_vel')
            self.send_motor_cmd(self.motor_right_pub, 'move_vel')

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