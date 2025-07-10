import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from leadshine_motor.motor_controller import LeadshineMotor
import threading
import datetime
import collections


class MotorControlNode(Node):
    def __init__(self):
        super().__init__('motor_control_node')

        # Declare and read motor ID parameter
        self.declare_parameter('device_id', 1)
        self.device_id = self.get_parameter('device_id').value
        self.get_logger().info(f"device_id from param = {self.device_id}")

        # Declare and read use_ack_patch parameter
        self.declare_parameter('use_ack_patch', 1)
        use_ack_patch = self.get_parameter('use_ack_patch').value
        self.get_logger().info(f"use_ack_patch from param = {use_ack_patch}")

        # motor instance
        self.motor = LeadshineMotor(self.device_id, self, use_ack_patch=use_ack_patch)

        # Set up command subscriber (immediate)
        self.cmd_sub = self.create_subscription(String,  f'/motor{self.device_id}/cmd', self.command_callback, 10)
        self.get_logger().info(f"📡 Subscription to /motor{self.device_id}/cmd created")

        # Set up non-blocking timer to check for service availability
        self.service_check_timer = self.create_timer(1.0, self.initialize_motor_params)
        self.get_logger().info("⏳ Waiting for /modbus_request service...")

        self.alarm_reset_stop_event = threading.Event()
        self.alarm_reset_thread = None
        self.last_home_offset = None
        self.last_home_speed = None
        self.cmd_queue = collections.deque()
        self.waiting_for_ack = False

        # Patch ack callback from base_device

    def initialize_motor_params(self):
        if self.motor.cli.service_is_ready():
            self.get_logger().info("✅ /modbus_request service is now available!")

            self.motor.initialize()
            self.get_logger().info("Motor initialized successfully.")

            self.service_check_timer.cancel()
            
    def try_reset_alarm_if_needed(self, done_callback=None):
        """
        Check alarm status, if alarm exists then reset it. done_callback will be called after processing.
        """
        def alarm_cb(fault_info):
            if fault_info and fault_info['fault_code'] != 0x0000:
                self.motor.reset_alarm()
                self.get_logger().info(f"Alarm detected: {fault_info['fault_description']} (0x{fault_info['fault_code']:04X}, ALM blinks {fault_info['alm_blink_count']} times), reset command sent.")
            if done_callback:
                done_callback()
        self.motor.get_alarm_status(alarm_cb)

    def process_next_command(self):
        if not self.waiting_for_ack and self.cmd_queue:
            cmd_tuple = self.cmd_queue.popleft()
            cmd, arg, seq_id = cmd_tuple
            now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
            self.get_logger().info(f"[SEQ {seq_id}] [{now}] Send {cmd}{' ' + str(arg) if arg is not None else ''}")
            self.waiting_for_ack = True
            # 调用 motor 方法
            match cmd:
                case "jog_left":
                    self.motor.jog_left(seq_id=seq_id)
                case "jog_right":
                    self.motor.jog_right(seq_id=seq_id)
                case "stop":
                    self.motor.abrupt_stop(seq_id=seq_id)
                case "set_pos":
                    self.motor.set_target_position(arg, seq_id=seq_id)
                case "set_vel":
                    self.motor.set_target_velocity(arg, seq_id=seq_id)
                case "set_acc":
                    self.motor.set_target_acceleration(arg, seq_id=seq_id)
                case "set_dec":
                    self.motor.set_target_deceleration(arg, seq_id=seq_id)
                case "move_abs":
                    self.motor.move_absolute(seq_id=seq_id)
                case "move_rel":
                    self.motor.move_relative(seq_id=seq_id)
                case "move_vel":
                    self.motor.move_velocity(seq_id=seq_id)
                case "set_zero":
                    self.motor.set_zero_position(seq_id=seq_id)
                case "reset_alarm":
                    self.motor.reset_alarm(seq_id=seq_id)
                case "set_home":
                    # 需要额外参数，略
                    pass
                case "home_pos":
                    self.motor.torque_home("home_pos", seq_id=seq_id)
                case "home_neg":
                    self.motor.torque_home("home_neg", seq_id=seq_id)
                case "set_limit":
                    # 需要两个参数，略
                    pass
                case "reset_limit":
                    self.motor.reset_software_limit(seq_id=seq_id)
                case "home_back":
                    # 需要额外参数，略
                    pass
                case "save_params":
                    self.motor.save_all_params_to_eeprom(seq_id=seq_id)
                case "factory_reset":
                    self.motor.factory_reset(seq_id=seq_id)
                case _:
                    self.get_logger().warn(f"[SEQ {seq_id}] 未知命令: {cmd}")

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
        if getattr(self.motor, 'use_ack_patch', 1):
            self.cmd_queue.append((cmd, arg, seq_id))
            self.process_next_command()
        else:
            # 直接执行命令，不用队列
            try:
                match cmd:
                    case "jog_left":
                        self.motor.jog_left(seq_id=seq_id)
                    case "jog_right":
                        self.motor.jog_right(seq_id=seq_id)
                    case "stop":
                        self.motor.abrupt_stop(seq_id=seq_id)
                    case "set_pos":
                        self.motor.set_target_position(arg, seq_id=seq_id)
                    case "set_vel":
                        self.motor.set_target_velocity(arg, seq_id=seq_id)
                    case "set_acc":
                        self.motor.set_target_acceleration(arg, seq_id=seq_id)
                    case "set_dec":
                        self.motor.set_target_deceleration(arg, seq_id=seq_id)
                    case "move_abs":
                        self.motor.move_absolute(seq_id=seq_id)
                    case "move_rel":
                        self.motor.move_relative(seq_id=seq_id)
                    case "move_vel":
                        self.motor.move_velocity(seq_id=seq_id)
                    case "set_zero":
                        self.motor.set_zero_position(seq_id=seq_id)
                    case "reset_alarm":
                        self.motor.reset_alarm(seq_id=seq_id)
                    case "set_home":
                        pass
                    case "home_pos":
                        self.motor.torque_home("home_pos", seq_id=seq_id)
                    case "home_neg":
                        self.motor.torque_home("home_neg", seq_id=seq_id)
                    case "set_limit":
                        pass
                    case "reset_limit":
                        self.motor.reset_software_limit(seq_id=seq_id)
                    case "home_back":
                        pass
                    case "save_params":
                        self.motor.save_all_params_to_eeprom(seq_id=seq_id)
                    case "factory_reset":
                        self.motor.factory_reset(seq_id=seq_id)
                    case _:
                        self.get_logger().warn(f"[SEQ {seq_id}] 未知命令: {cmd}")
            except Exception as e:
                self.get_logger().error(f"[SEQ {seq_id}] ❌ Command '{cmd}' failed: {e}")

    def destroy_node(self):
        self.alarm_reset_stop_event.set()
        if self.alarm_reset_thread:
            self.alarm_reset_thread.join()
        super().destroy_node()


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
