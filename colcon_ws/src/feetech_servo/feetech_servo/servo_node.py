import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from feetech_servo.servo_controller import FeetechServo
import threading
import datetime
import collections

class ServoControlNode(Node):
    def __init__(self):
        super().__init__('servo_control_node')
        self.declare_parameter('device_id', 1)
        self.device_id = self.get_parameter('device_id').value
        self.get_logger().info(f"device_id from param = {self.device_id}")
        self.declare_parameter('use_ack_patch', 1)
        use_ack_patch = self.get_parameter('use_ack_patch').value
        self.motor = FeetechServo(self.device_id, self, use_ack_patch=use_ack_patch)
        self.cmd_sub = self.create_subscription(String,  f'/motor{self.device_id}/cmd', self.command_callback, 10)
        self.get_logger().info(f"📡 Subscription to /motor{self.device_id}/cmd created")
        self.service_check_timer = self.create_timer(1.0, self.initialize_motor_params)
        self.get_logger().info("⏳ Waiting for /modbus_request service...")
        self.cmd_queue = collections.deque()
        self.waiting_for_ack = False

    def initialize_motor_params(self):
        if self.motor.cli.service_is_ready():
            self.get_logger().info("✅ /modbus_request service is now available!")
            self.motor.initialize()
            self.get_logger().info("Motor initialized successfully.")
            self.service_check_timer.cancel()
            
    def do_motion(self, cmd, arg, seq_id, parts, use_ack_patch):
        try:
            if use_ack_patch:
                # use_ack_patch=1 时，所有命令异常都被捕获，
                # 并在异常时自动清除 waiting_for_ack，递归调用 process_next_command，
                # 保证队列不会因单条命令异常而卡死，系统健壮性高。
                self.get_logger().info(f"[SEQ {seq_id}] Send {cmd}{' ' + str(arg) if arg is not None else ''}")
            else:
                self.get_logger().info(f"[SEQ {seq_id}] [use_ack_patch=0] Executing {cmd}({arg})")
            match cmd:
                case "move":
                    self.motor.move(arg, seq_id=seq_id)
                case "set_id":
                    self.motor.set_id(arg, seq_id=seq_id)
                case "set_baud":
                    self.motor.set_baud(arg, seq_id=seq_id)
                case "set_angle_limit":
                    if parts and len(parts) == 3:
                        min_angle = int(parts[1])
                        max_angle = int(parts[2])
                        self.motor.set_angle_limit(min_angle, max_angle, seq_id=seq_id)
                        self.get_logger().info(f"[SEQ {seq_id}] set_angle_limit: min={min_angle}, max={max_angle}")
                    else:
                        self.get_logger().error(f"[SEQ {seq_id}] set_angle_limit command requires two parameters: min_angle max_angle")
                        raise ValueError("set_angle_limit param error")
                case "reset":
                    self.motor.reset(seq_id=seq_id)
                case _:
                    self.get_logger().warn(f"[SEQ {seq_id}] 未知命令: {cmd}")
        except Exception as e:
            self.get_logger().error(f"[SEQ {seq_id}] ❌ Command '{cmd}' failed: {e}")
            if use_ack_patch:
                # use_ack_patch=1 时，异常兜底：
                # 1. 自动清除 waiting_for_ack
                # 2. 递归调用 process_next_command 继续队列
                # 这样即使单条命令出错，后续命令也能继续执行
                self.waiting_for_ack = False
                self.process_next_command()
            return

    def process_next_command(self):
        if not self.waiting_for_ack and self.cmd_queue:
            cmd_tuple = self.cmd_queue.popleft()
            cmd, arg, seq_id = cmd_tuple
            parts = getattr(self, 'last_cmd_parts', None)
            self.waiting_for_ack = True
            use_ack_patch = getattr(self.motor, 'use_ack_patch', 1)
            if cmd in ["move"]:
                self.try_reset_alarm_if_needed(lambda: self.do_motion(cmd, arg, seq_id, parts, use_ack_patch))
            else:
                self.do_motion(cmd, arg, seq_id, parts, use_ack_patch)

    def command_callback(self, msg):
        data = msg.data.strip()
        seq_id = None
        if data.startswith('seq:') and '|' in data:
            try:
                seq_id_str, rest = data.split('|', 1)
                seq_id = int(seq_id_str[4:])
                msg.data = rest
            except Exception:
                pass
        parts = msg.data.strip().lower().split()
        if not parts:
            return
        self.last_cmd_parts = parts
        cmd = parts[0]
        arg = int(parts[1]) if len(parts) > 1 and parts[1].lstrip('-').isdigit() else None
        use_ack_patch = getattr(self.motor, 'use_ack_patch', 1)
        if use_ack_patch:
            self.cmd_queue.append((cmd, arg, seq_id))
            self.process_next_command()
        else:
            self.do_motion(cmd, arg, seq_id, parts, use_ack_patch)

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
