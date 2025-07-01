import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from leadshine_motor.motor_controller import LeadshineMotor


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Declare and read motor ID parameter
        self.declare_parameter('device_id', 1)
        self.device_id = self.get_parameter('device_id').value
        self.get_logger().info(f"device_id from param = {self.device_id}")

        # motor instance
        self.motor = LeadshineMotor(self.device_id, self)

        # Set up command subscriber (immediate)
        self.cmd_sub = self.create_subscription(String,  f'/motor{self.device_id}/cmd', self.command_callback, 10)
        self.get_logger().info(f"📡 Subscription to /motor{self.device_id}/cmd created")

        # Set up non-blocking timer to check for service availability
        self.service_check_timer = self.create_timer(1.0, self.initialize_motor_params)
        self.get_logger().info("⏳ Waiting for /modbus_request service...")

    def initialize_motor_params(self):
        if self.motor.cli.service_is_ready():
            self.get_logger().info("✅ /modbus_request service is now available!")

            self.motor.initialize()
            self.get_logger().info("Motor initialized successfully.")

            self.service_check_timer.cancel()
            
    def command_callback(self, msg):
        parts = msg.data.strip().lower().split()
        if not parts:
            return

        cmd = parts[0]
        arg = int(parts[1]) if len(parts) > 1 and parts[1].lstrip('-').isdigit() else None

        if self.motor is None:
            self.get_logger().warn("⏳ Motor not initialized yet. Command ignored.")
            return

        try:
            match cmd:
                case "jog_left":
                    self.motor.jog_left()
                case "jog_right":
                    self.motor.jog_right()
                case "stop":
                    self.motor.abrupt_stop()
                case "get_pos":
                    self.motor.get_current_position(
                        lambda pos: self.get_logger().info(f"ℹ️ Current position: {pos}") if pos is not None else
                                    self.get_logger().error("❌ Failed to read position")
                    )
                case "set_zero":
                    self.motor.set_zero_position()
                case "set_pos":
                    if arg is not None:
                        self.motor.set_target_position(arg)
                        self.get_logger().info(f"✅ Set position to {arg}")
                case "set_vel":
                    if arg is not None:
                        self.motor.set_target_velocity(arg)
                        self.get_logger().info(f"✅ Set velocity to {arg}")
                case "set_acc":
                    if arg is not None:
                        self.motor.set_target_acceleration(arg)
                        self.get_logger().info(f"✅ Set acceleration to {arg}")
                case "set_dec":
                    if arg is not None:
                        self.motor.set_target_deceleration(arg)
                        self.get_logger().info(f"✅ Set deceleration to {arg}")
                case "move_abs":
                    if arg is not None:
                        self.motor.set_target_position(arg)
                        self.get_logger().info(f"ℹ️ Updated position to {arg}")
                    self.motor.move_absolute()
                case "move_rel":
                    if arg is not None:
                        self.motor.set_target_position(arg)
                        self.get_logger().info(f"ℹ️ Updated relative offset to {arg}")
                    self.motor.move_relative()
                case "move_vel":
                    if arg is not None:
                        self.motor.set_target_velocity(arg)
                        self.get_logger().info(f"ℹ️ Updated velocity to {arg}")
                    self.motor.move_velocity()
                case "+" | "-":
                    # 回零功能，正向或反向
                    # 默认参数
                    default_params = {
                        'stall_time': 1000,
                        'output_val': 50,
                        'high_speed': 1000,
                        'low_speed': 200,
                        'acc': 100,
                        'dec': 100
                    }
                    # 检查是否有额外参数
                    if len(parts) == 7:
                        try:
                            stall_time = int(parts[1])
                            output_val = int(parts[2])
                            high_speed = int(parts[3])
                            low_speed = int(parts[4])
                            acc = int(parts[5])
                            dec = int(parts[6])
                        except Exception as e:
                            self.get_logger().error(f"❌ 回零参数解析失败: {e}")
                            return
                    else:
                        stall_time = default_params['stall_time']
                        output_val = default_params['output_val']
                        high_speed = default_params['high_speed']
                        low_speed = default_params['low_speed']
                        acc = default_params['acc']
                        dec = default_params['dec']
                    direction = "+" if cmd == "+" else "-"
                    self.motor.torque_home(direction, stall_time, output_val, high_speed, low_speed, acc, dec)
                case _:
                    self.get_logger().warn(f"Unknown command: {cmd}")
        except Exception as e:
            self.get_logger().error(f"❌ Command '{cmd}' failed: {e}")


def main():
    rclpy.init()
    node = MotorControlNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
