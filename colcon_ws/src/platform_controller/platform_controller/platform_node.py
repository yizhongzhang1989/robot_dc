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

        # Command queue (FIFO) for manual commands
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
            
            # Convert argument to boolean for platform control
            # For manual control: 1 = True (start), 0 = False (stop)
            # For timed control: True/False passed directly
            move_flag = bool(arg) if arg is not None else True
            
            if cmd == "up":
                self.platform.up(move_flag, seq_id=seq_id)
            elif cmd == "down":
                self.platform.down(move_flag, seq_id=seq_id)
            elif cmd == "forward":
                self.platform.forward(move_flag, seq_id=seq_id)
            elif cmd == "backward":
                self.platform.backward(move_flag, seq_id=seq_id)
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
        
        # Handle different argument types
        arg = None
        if len(parts) > 1:
            try:
                # Try to parse as float first (for timed commands)
                arg = float(parts[1])
                # If it's a whole number, convert to int for backward compatibility
                if arg.is_integer():
                    arg = int(arg)
            except ValueError:
                # If float parsing fails, keep as string
                arg = parts[1]

        now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        self.get_logger().info(f"[SEQ {seq_id}] [{now}] Received command {cmd}{' ' + str(arg) if arg is not None else ''}")

        # Handle timed commands - delegate to controller
        if cmd.startswith('timed_'):
            direction = cmd[6:]  # Remove "timed_" prefix
            if direction in ['up', 'down', 'forward', 'backward'] and arg is not None:
                try:
                    duration = float(arg)
                    result = self.platform.move_with_timer(direction, duration)
                    self.get_logger().info(f"Timed movement result: {result}")
                except (ValueError, TypeError):
                    self.get_logger().error(f"Invalid duration for timed command: {arg}")
            else:
                self.get_logger().error(f"Invalid timed command: {cmd} with arg: {arg}")
            return

        # Handle stop timed commands - delegate to controller
        if cmd.startswith('stop_timed_'):
            direction = cmd[11:]  # Remove "stop_timed_" prefix
            if direction in ['up', 'down', 'forward', 'backward']:
                result = self.platform.stop_timed_movement(direction)
                self.get_logger().info(f"Stop timed movement result: {result}")
            elif direction == 'all':
                result = self.platform.stop_all_timed_movements()
                self.get_logger().info(f"Stop all timed movements result: {result}")
            else:
                self.get_logger().error(f"Invalid stop timed command: {cmd}")
            return

        use_ack_patch = getattr(self.platform, 'use_ack_patch', 1)

        if use_ack_patch:
            # Enqueue and process serially
            self.cmd_queue.append((cmd, arg, seq_id))
            self.process_next_command()
        else:
            # Execute immediately
            self.do_motion(cmd, arg, seq_id)

    def __del__(self):
        """Destructor to clean up resources"""
        try:
            if hasattr(self, 'platform') and self.platform:
                self.platform.cleanup()
        except Exception as e:
            pass  # Ignore errors during cleanup

def main():
    rclpy.init()
    node = PlatformControlNode()
    try:
        rclpy.spin(node)
    finally:
        # Clean up platform resources before destroying node
        if hasattr(node, 'platform') and node.platform:
            node.platform.cleanup()
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()