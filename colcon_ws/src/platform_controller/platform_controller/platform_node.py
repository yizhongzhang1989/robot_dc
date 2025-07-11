import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from platform_controller.platform_controller import PlatformController
import datetime
import collections

class PlatformControlNode(Node):
    def __init__(self):
        super().__init__('platform_control_node')

        # Declare parameters
        self.declare_parameter('device_id', 1)
        self.device_id = self.get_parameter('device_id').value
        self.get_logger().info(f"device_id from param = {self.device_id}")

        self.declare_parameter('use_ack_patch', 1)
        use_ack_patch = self.get_parameter('use_ack_patch').value

        # Controller instance
        self.platform = PlatformController(self.device_id, self, use_ack_patch=use_ack_patch)

        # Subscription to commands
        self.cmd_sub = self.create_subscription(String, f'/platform/cmd', self.command_callback, 10)
        self.get_logger().info(f"📡 Subscription to /platform/cmd created")

        # Wait for service availability
        self.service_check_timer = self.create_timer(1.0, self.initialize_motor_params)
        self.get_logger().info("⏳ Waiting for /modbus_request service...")

        # Command queue (FIFO)
        self.cmd_queue = collections.deque()
        self.waiting_for_ack = False

    def initialize_motor_params(self):
        if self.platform.cli.service_is_ready():
            self.get_logger().info("✅ /modbus_request service is now available!")
            self.platform.initialize()
            self.get_logger().info("Platform initialized successfully.")
            self.service_check_timer.cancel()

    def do_motion(self, cmd, arg, seq_id):
        """
        Unified method to execute a single motion command.
        """
        try:
            self.get_logger().info(f"[SEQ {seq_id}] Executing {cmd}{' ' + str(arg) if arg is not None else ''}")
            if cmd == "up":
                self.platform.up(arg, seq_id=seq_id)
            elif cmd == "down":
                self.platform.down(arg, seq_id=seq_id)
            elif cmd == "forward":
                self.platform.forward(arg, seq_id=seq_id)
            elif cmd == "backward":
                self.platform.backward(arg, seq_id=seq_id)
            else:
                self.get_logger().warn(f"[SEQ {seq_id}] Unknown command: {cmd}")
        except Exception as e:
            self.get_logger().error(f"[SEQ {seq_id}] ❌ Command '{cmd}' failed: {e}")
            if getattr(self.platform, 'use_ack_patch', 1):
                # Clear flag to allow next command
                self.waiting_for_ack = False
                self.process_next_command()

    def process_next_command(self):
        """
        Process next command from the queue.
        Only called if:
        - No command is currently waiting for ack
        - Queue has commands pending
        """
        if not self.waiting_for_ack and self.cmd_queue:
            cmd, arg, seq_id = self.cmd_queue.popleft()
            now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            self.get_logger().info(f"[SEQ {seq_id}] [{now}] Sending command {cmd}{' ' + str(arg) if arg is not None else ''}")
            self.waiting_for_ack = True
            self.do_motion(cmd, arg, seq_id)

    def command_callback(self, msg):
        """
        Receive and parse incoming command.
        If use_ack_patch:
          - Enqueue command and start processing
        Else:
          - Execute immediately
        """
        data = msg.data.strip()
        seq_id = None

        # Parse sequence ID
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
        self.get_logger().info(f"[SEQ {seq_id}] [{now}] Received command {cmd}{' ' + str(arg) if arg is not None else ''}")

        use_ack_patch = getattr(self.platform, 'use_ack_patch', 1)

        if use_ack_patch:
            # Enqueue and process serially
            self.cmd_queue.append((cmd, arg, seq_id))
            self.process_next_command()
        else:
            # Execute immediately
            self.do_motion(cmd, arg, seq_id)

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