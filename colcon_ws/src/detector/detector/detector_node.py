#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32, Float32, String
from leadshine_motor.motor_controller import LeadshineMotor
from feetech_servo.servo_controller import FeetechServo
import threading
from typing import Optional, List
from rclpy.timer import Timer

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

        # Timers for controlling periodic reading
        self.motor_timers: List[Optional[Timer]] = [None for _ in self.motor_ids]
        self.servo_timers: List[Optional[Timer]] = [None for _ in self.servo_ids]
        self.lock = threading.Lock()

        # Subscribe to control topics
        for idx, motor_id in enumerate(self.motor_ids):
            self.create_subscription(String, f'/motor{motor_id}/read_ctrl', lambda msg, idx=idx: self.handle_motor_ctrl(msg, idx), 10)
        for idx, servo_id in enumerate(self.servo_ids):
            self.create_subscription(String, f'/servo{servo_id}/read_ctrl', lambda msg, idx=idx: self.handle_servo_ctrl(msg, idx), 10)

    def handle_motor_ctrl(self, msg, idx):
        cmd = msg.data.strip().lower()
        with self.lock:
            if cmd == 'start':
                if self.motor_timers[idx] is None:
                    self.motor_timers[idx] = self.create_timer(0.2, lambda idx=idx: self.motor_timer_callback(idx))
            elif cmd == 'stop':
                if self.motor_timers[idx] is not None:
                    self.motor_timers[idx].cancel()
                    self.motor_timers[idx] = None

    def handle_servo_ctrl(self, msg, idx):
        cmd = msg.data.strip().lower()
        with self.lock:
            if cmd == 'start':
                if self.servo_timers[idx] is None:
                    self.servo_timers[idx] = self.create_timer(0.2, lambda idx=idx: self.servo_timer_callback(idx))
            elif cmd == 'stop':
                if self.servo_timers[idx] is not None:
                    self.servo_timers[idx].cancel()
                    self.servo_timers[idx] = None

    def motor_timer_callback(self, idx):
        motor = self.motors[idx]
        pub = self.motor_pubs[idx]
        def motor_cb(pos, idx=idx):
            if pos is not None:
                msg = Int32()
                msg.data = pos
                pub.publish(msg)
            else:
                self.get_logger().error(f"[motor{self.motor_ids[idx]}] failed to read position")
        motor.get_current_position(motor_cb)

    def servo_timer_callback(self, idx):
        servo = self.servos[idx]
        pos_pub = self.servo_pos_pubs[idx]
        torque_pub = self.servo_torque_pubs[idx]
        def servo_pos_cb(response, idx=idx):
            if response:
                pos = response[0]
                msg = Int32()
                msg.data = pos
                pos_pub.publish(msg)
            else:
                self.get_logger().error(f"[servo{self.servo_ids[idx]}] failed to read position")
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
                self.get_logger().error(f"[servo{self.servo_ids[idx]}] failed to read PWM")
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