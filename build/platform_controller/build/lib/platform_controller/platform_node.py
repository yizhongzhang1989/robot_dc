import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from platform_controller.platform_controller import PlatformController
import threading
import datetime
import collections

class PlatformControlNode(Node):
    def __init__(self):
        super().__init__('platform_control_node')
        self.declare_parameter('device_id', 1)
        self.device_id = self.get_parameter('device_id').value
        self.get_logger().info(f"device_id from param = {self.device_id}")
        self.declare_parameter('use_ack_patch', 1)
        use_ack_patch = self.get_parameter('use_ack_patch').value
        self.platform = PlatformController(self.device_id, self, use_ack_patch=use_ack_patch)
        self.cmd_sub = self.create_subscription(String,  f'/platform/cmd', self.command_callback, 10)
        self.get_logger().info(f"📡 Subscription to /platform/cmd created")
        self.service_check_timer = self.create_timer(1.0, self.initialize_motor_params)
        self.get_logger().info("⏳ Waiting for /modbus_request service...")
        self.cmd_queue = collections.deque()
        self.waiting_for_ack = False

    def initialize_motor_params(self):
        if self.platform.cli.service_is_ready():
            self.get_logger().info("✅ /modbus_request service is now available!")
            self.platform.initialize()
            self.get_logger().info("Platform initialized successfully.")
            self.service_check_timer.cancel()
            
    def process_next_command(self):
        if not self.waiting_for_ack and self.cmd_queue:
            cmd_tuple = self.cmd_queue.popleft()
            cmd, arg, seq_id = cmd_tuple
            now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            self.get_logger().info(f"[SEQ {seq_id}] [{now}] Send {cmd}{' ' + str(arg) if arg is not None else ''}")
            self.waiting_for_ack = True
            try:
                if cmd == "up":
                    if arg is not None:
                        self.platform.up(arg, seq_id=seq_id)
                elif cmd == "down":
                    if arg is not None:
                        self.platform.down(arg, seq_id=seq_id)
                elif cmd == "forward":
                    if arg is not None:
                        self.platform.forward(arg, seq_id=seq_id)
                elif cmd == "backward":
                    if arg is not None:
                        self.platform.backward(arg, seq_id=seq_id)
                else:
                    self.get_logger().warn(f"[SEQ {seq_id}] 未知命令: {cmd}")
            except Exception as e:
                self.get_logger().error(f"[SEQ {seq_id}] ❌ Command '{cmd}' failed: {e}")

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
        cmd = parts[0]
        arg = int(parts[1]) if len(parts) > 1 and parts[1].lstrip('-').isdigit() else None
        now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        self.get_logger().info(f"[SEQ {seq_id}] [{now}] Receive {cmd}{' ' + str(arg) if arg is not None else ''}")
        self.get_logger().info(f"[DEBUG] use_ack_patch in callback: {getattr(self.platform, 'use_ack_patch', 'NO_ATTR')}")
        if getattr(self.platform, 'use_ack_patch', 1):
            self.cmd_queue.append((cmd, arg, seq_id))
            self.process_next_command()
        else:
            try:
                self.get_logger().info(f"[SEQ {seq_id}] [use_ack_patch=0] Executing {cmd}({arg})")
                if cmd in ["up", "down", "forward", "backward"] and arg is None:
                    self.get_logger().warn(f"[SEQ {seq_id}] [use_ack_patch=0] Command '{cmd}' missing argument!")
                if cmd == "up":
                    if arg is not None:
                            self.platform.up(bool(arg), seq_id=seq_id)
                elif cmd == "down":
                    if arg is not None:
                            self.platform.down(bool(arg), seq_id=seq_id)
                elif cmd == "forward":
                    if arg is not None:
                            self.platform.forward(bool(arg), seq_id=seq_id)
                elif cmd == "backward":
                    if arg is not None:
                            self.platform.backward(bool(arg), seq_id=seq_id)
                else:
                        self.get_logger().warn(f"[SEQ {seq_id}] 未知命令: {cmd}")
            except Exception as e:
                self.get_logger().error(f"[SEQ {seq_id}] ❌ Command '{cmd}' failed: {e}")

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
