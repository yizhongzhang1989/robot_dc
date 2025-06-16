import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Joy
from std_msgs.msg import String

class JoystickTeleop(Node):
    def __init__(self):
        super().__init__('joystick_teleop')

        self.motor1_pub = self.create_publisher(String, '/motor1/cmd', 10)
        self.motor2_pub = self.create_publisher(String, '/motor2/cmd', 10)
        self.servo1_pub = self.create_publisher(String, '/servo11/cmd', 10)
        self.servo2_pub = self.create_publisher(String, '/servo12/cmd', 10)

        self.create_subscription(Joy, '/joy', self.joy_callback, 10)

        self.motor1_active = False
        self.motor2_active = False
        self.servo1_active = False
        self.servo2_active = False

        self.last_axis1_speed = 0   # motor1
        self.last_axis3_speed = 0   # motor2
        self.last_axis2_speed = 0   # servo1
        self.last_axis4_speed = 0   # servo2
        self.axis_deadzone = 1  # minimum delta change to trigger update

    def joy_callback(self, msg):
        # Button control
        if msg.buttons[0] and not self.motor1_active:
            self.get_logger().info('Motor 1: jog_left')
            self.send_motor_cmd(self.motor1_pub, 'jog_left')
            self.motor1_active = True
        elif not msg.buttons[0] and self.motor1_active:
            self.get_logger().info('Motor 1: stop')
            self.send_motor_cmd(self.motor1_pub, 'stop')
            self.motor1_active = False

        if msg.buttons[1] and not self.motor2_active:
            self.get_logger().info('Motor 2: jog_left')
            self.send_motor_cmd(self.motor2_pub, 'jog_left')
            self.motor2_active = True
        elif not msg.buttons[1] and self.motor2_active:
            self.get_logger().info('Motor 2: stop')
            self.send_motor_cmd(self.motor2_pub, 'stop')
            self.motor2_active = False

        # Axis control
        self.handle_motor_axis_control(msg.axes[1], self.motor1_pub, motor_id=1)
        self.handle_motor_axis_control(msg.axes[3], self.motor2_pub, motor_id=2)

        self.handle_servo_axis_control(msg.axes[0], self.servo1_pub, motor_id=11)
        self.handle_servo_axis_control(msg.axes[2], self.servo2_pub, motor_id=12)

    def handle_motor_axis_control(self, axis_value, publisher, motor_id):
        speed = int(axis_value * 300)  # Scale axis value to speed

        if motor_id == 1:
            last_speed = self.last_axis1_speed
        else:
            last_speed = self.last_axis3_speed

        # If speed is zero but last speed wasn't, send stop
        if speed == 0 and last_speed != 0:
            self.update_axis_motor(publisher, speed, motor_id)
        # Only update if speed change exceeds deadzone
        elif abs(speed - last_speed) >= self.axis_deadzone:
            self.update_axis_motor(publisher, speed, motor_id)

        # Update the stored last speed
        if motor_id == 1:
            self.last_axis1_speed = speed
        else:
            self.last_axis3_speed = speed

    def update_axis_motor(self, publisher, speed, motor_id):
        if speed == 0:
            self.get_logger().info(f'Motor {motor_id}: stop')
            self.send_motor_cmd(publisher, 'stop')
        else:
            # print motor start move info (if last speed was 0)
            if (motor_id == 1 and self.last_axis1_speed == 0) or (motor_id == 2 and self.last_axis3_speed == 0):
                self.get_logger().info(f'Motor {motor_id}: start move') 

            self.send_motor_cmd(publisher, f'set_vel {speed}')
            self.send_motor_cmd(publisher, 'move_vel')

    def send_motor_cmd(self, pub, command: str):
        msg = String()
        msg.data = command
        pub.publish(msg)

    def handle_servo_axis_control(self, axis_value, publisher, motor_id):
        speed = int(axis_value * 10)  # Scale axis value to speed

        if motor_id == 11:
            last_speed = self.last_axis2_speed
        else:
            last_speed = self.last_axis4_speed

        # If speed is zero but last speed wasn't, send stop
        if speed == 0 and last_speed != 0:
            self.update_axis_servo(publisher, speed, motor_id)
        # Only update if speed change exceeds deadzone
        elif abs(speed - last_speed) >= self.axis_deadzone:
            self.update_axis_servo(publisher, speed, motor_id)

        # Update the stored last speed
        if motor_id == 11:
            self.last_axis2_speed = speed
        else:
            self.last_axis4_speed = speed

    def update_axis_servo(self, publisher, speed, motor_id):
        if speed == 0:
            self.get_logger().info(f'Motor {motor_id}: stop')
            self.send_motor_cmd(publisher, 'stop')
        else:
            # print motor start move info (if last speed was 0)
            if (motor_id == 11 and self.last_axis2_speed == 0) or (motor_id == 12 and self.last_axis4_speed == 0):
                self.get_logger().info(f'Motor {motor_id}: start move') 

            abs_speed = abs(speed)
            self.send_motor_cmd(publisher, f'set_vel {abs_speed}')

            # if speed < 0, then set pos to 0, otherwise set to 4095
            if speed < 0:
                self.get_logger().info(f'Motor {motor_id}: set pos to 0')
                self.send_motor_cmd(publisher, 'set_pos 0')
            else:
                self.get_logger().info(f'Motor {motor_id}: set pos to 4095')
                self.send_motor_cmd(publisher, 'set_pos 4095')


def main(args=None):
    rclpy.init(args=args)
    node = JoystickTeleop()
    rclpy.spin(node)
    rclpy.shutdown()
