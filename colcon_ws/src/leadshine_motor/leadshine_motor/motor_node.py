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
                    self.get_logger().info(f"[SEQ {seq_id}] Set zero position command sent.")
                case "reset_alarm":
                    self.motor.reset_alarm(seq_id=seq_id)
                case "set_home":
                    if parts and len(parts) == 7:
                        stall_time = int(parts[1])
                        cur = int(parts[2])
                        high_speed = int(parts[3])
                        low_speed = int(parts[4])
                        acc = int(parts[5])
                        dec = int(parts[6])
                    else:
                        stall_time, cur, high_speed, low_speed, acc, dec = 1000, 30, 1000, 200, 100, 100
                    self.motor.set_home_params(stall_time, cur, high_speed, low_speed, acc, dec, seq_id=seq_id)
                    self.get_logger().info(f"[SEQ {seq_id}] set_home: stall_time={stall_time}, cur={cur}, high_speed={high_speed}, low_speed={low_speed}, acc={acc}, dec={dec}")
                case "home_pos":
                    self.last_home_offset = -10000
                    self.last_home_speed = -100
                    self.motor.torque_home("home_pos", seq_id=seq_id)
                case "home_neg":
                    self.last_home_offset = 10000
                    self.last_home_speed = 100
                    self.motor.torque_home("home_neg", seq_id=seq_id)
                case "set_limit":
                    if parts and len(parts) == 3:
                        pos_limit = int(parts[1])
                        neg_limit = int(parts[2])
                        self.motor.set_software_limit(pos_limit, neg_limit, seq_id=seq_id)
                        self.get_logger().info(f"[SEQ {seq_id}] set_limit: pos_limit={pos_limit}, neg_limit={neg_limit}")
                    else:
                        self.get_logger().error(f"[SEQ {seq_id}] set_limit command requires two parameters: positive_limit negative_limit")
                        raise ValueError("set_limit param error")
                case "reset_limit":
                    self.motor.reset_software_limit(seq_id=seq_id)
                case "home_back":
                    if getattr(self, 'last_home_offset', None) is None or getattr(self, 'last_home_speed', None) is None:
                        self.get_logger().error(f"[SEQ {seq_id}] home_back: Last homing direction not detected, please execute home_pos or home_neg first")
                        raise ValueError("home_back param error")
                    if self.last_home_offset == -10000 and self.last_home_speed == -100:
                        speed = -100
                        offset = -10000
                    elif self.last_home_offset == 10000 and self.last_home_speed == 100:
                        speed = 100
                        offset = 10000
                    else:
                        self.get_logger().error(f"[SEQ {seq_id}] home_back: last_home_offset/speed abnormal")
                        raise ValueError("home_back abnormal")
                    self.motor.set_target_velocity(speed, seq_id=seq_id)
                    self.motor.set_target_position(offset, seq_id=seq_id)
                    self.motor.move_relative(seq_id=seq_id)
                    self.get_logger().info(f"[SEQ {seq_id}] home_back (relative): move with speed {speed}, offset {offset}")
                case "save_params":
                    self.motor.save_all_params_to_eeprom(seq_id=seq_id)
                case "factory_reset":
                    self.motor.factory_reset(seq_id=seq_id)
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
            # 只对运动类命令做报警检测
            if cmd in ["jog_left", "jog_right", "move_abs", "move_rel", "move_vel", "home_pos", "home_neg"]:
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
        self.last_cmd_parts = parts  # 保存完整参数，供 set_limit/set_home 等多参数命令使用
        cmd = parts[0]
        arg = int(parts[1]) if len(parts) > 1 and parts[1].lstrip('-').isdigit() else None
        use_ack_patch = getattr(self.motor, 'use_ack_patch', 1)
        if use_ack_patch:
            self.cmd_queue.append((cmd, arg, seq_id))
            self.process_next_command()
        else:
            self.do_motion(cmd, arg, seq_id, parts, use_ack_patch)

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
