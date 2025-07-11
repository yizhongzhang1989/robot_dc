#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32
from leadshine_motor.motor_controller import LeadshineMotor
from feetech_servo.servo_controller import FeetechServo
import threading

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        self.motor_ids = [1, 2]
        self.servo_ids = [17, 18]
        self.use_ack_patch = 1

        self.motors = [LeadshineMotor(i, self, use_ack_patch=self.use_ack_patch) for i in self.motor_ids]
        self.servos = [FeetechServo(i, self, use_ack_patch=self.use_ack_patch) for i in self.servo_ids]

        self.motor_pubs = [self.create_publisher(Int32, f'/motor{i}/position', 10) for i in self.motor_ids]
        self.servo_pos_pubs = [self.create_publisher(Int32, f'/servo{i}/position', 10) for i in self.servo_ids]
        self.servo_torque_pubs = [self.create_publisher(Float32, f'/servo{i}/pwm', 10) for i in self.servo_ids]

        self.timer = self.create_timer(0.2, self.timer_callback)
        self.lock = threading.Lock()

    def timer_callback(self):
        with self.lock:
            # Motor positions
            for idx, (motor, pub) in enumerate(zip(self.motors, self.motor_pubs)):
                def motor_cb(pos, idx=idx):
                    if pos is not None:
                        msg = Int32()
                        msg.data = pos
                        pub.publish(msg)
                    else:
                        self.get_logger().warn(f"[motor{self.motor_ids[idx]}] failed to read position")
                motor.get_current_position(motor_cb)

            # Servo positions and torque (PWM)
            for idx, (servo, pos_pub, torque_pub) in enumerate(zip(self.servos, self.servo_pos_pubs, self.servo_torque_pubs)):
                def servo_pos_cb(response, idx=idx):
                    if response:
                        pos = response[0]
                        msg = Int32()
                        msg.data = pos
                        pos_pub.publish(msg)
                    else:
                        self.get_logger().warn(f"[servo{self.servo_ids[idx]}] failed to read position")
                servo.get_position(callback=servo_pos_cb)

                def servo_torque_cb(response, idx=idx):
                    if response:
                        raw = response[0]
                        pwm_signed = raw - 0x10000 if raw >= 0x8000 else raw
                        pwm_percent = pwm_signed * 0.1
                        msg = Float32()
                        msg.data = pwm_percent
                        torque_pub.publish(msg)
                    else:
                        self.get_logger().warn(f"[servo{self.servo_ids[idx]}] failed to read PWM")
                servo.get_torque(callback=servo_torque_cb)

def main():
    rclpy.init()
    node = DetectorNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()