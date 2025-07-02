import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from leadshine_motor.motor_controller import LeadshineMotor
import threading


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

        self.alarm_reset_stop_event = threading.Event()
        self.alarm_reset_thread = None
        self.last_home_offset = None
        self.last_home_speed = None

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

    def command_callback(self, msg):
        parts = msg.data.strip().lower().split()
        if not parts:
            return
        cmd = parts[0]
        arg = int(parts[1]) if len(parts) > 1 and parts[1].lstrip('-').isdigit() else None
        if self.motor is None:
            self.get_logger().warn("⏳ Motor not initialized yet. Command ignored.")
            return
        def do_command():
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
                    case "get_status":
                        self.motor.get_motion_status(
                            lambda status: self.get_logger().info(f"ℹ️ Motion status: fault={status['fault']}, enabled={status['enabled']}, running={status['running']}, command_completed={status['command_completed']}, path_completed={status['path_completed']}, homing_completed={status['homing_completed']}, raw_register=0x{status['raw_register']:04X}") if status is not None else
                                        self.get_logger().error("❌ Failed to read motion status")
                        )
                    case "get_alarm":
                        self.motor.get_alarm_status(
                            lambda fault_info: self.get_logger().info(f"ℹ️ Alarm status: {fault_info['fault_description']} (0x{fault_info['fault_code']:04X}, ALM blinks {fault_info['alm_blink_count']} times)") if fault_info is not None else
                                        self.get_logger().error("❌ Failed to read alarm status")
                        )
                    case "reset_alarm":
                        self.motor.reset_alarm()
                        self.get_logger().info("✅ Alarm reset command sent")
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
                    case "set_home":
                        # Parameter order: sta cur hig low acc dec
                        default_params = {
                            'stall_time': 1000,   # sta
                            'cur': 30,            # cur
                            'high_speed': 1000,   # hig
                            'low_speed': 200,     # low
                            'acc': 100,           # acc
                            'dec': 100            # dec
                        }
                        if len(parts) == 7:
                            try:
                                stall_time = int(parts[1])   # sta
                                cur = int(parts[2])          # cur
                                high_speed = int(parts[3])   # hig
                                low_speed = int(parts[4])    # low
                                acc = int(parts[5])          # acc
                                dec = int(parts[6])          # dec
                            except Exception as e:
                                self.get_logger().error(f"❌ set_home param parse failed: {e}")
                                return
                        else:
                            stall_time = default_params['stall_time']
                            cur = default_params['cur']
                            high_speed = default_params['high_speed']
                            low_speed = default_params['low_speed']
                            acc = default_params['acc']
                            dec = default_params['dec']
                        self.motor.set_home_params(stall_time, cur, high_speed, low_speed, acc, dec)
                    case "home_pos":
                        self.last_home_offset = -10000
                        self.last_home_speed = -100
                        self.motor.torque_home("home_pos")
                    case "home_neg":
                        self.last_home_offset = 10000
                        self.last_home_speed = 100
                        self.motor.torque_home("home_neg")
                    case "set_limit":
                        if len(parts) == 3:
                            try:
                                pos_limit = int(parts[1])
                                neg_limit = int(parts[2])
                            except Exception as e:
                                self.get_logger().error(f"❌ Limit parameter parse failed: {e}")
                                return
                            self.motor.set_software_limit(pos_limit, neg_limit)
                            self.get_logger().info(f"✅ Software limit set, positive limit: {pos_limit}, negative limit: {neg_limit}")
                        else:
                            self.get_logger().error("❌ set_limit command requires two parameters: positive_limit negative_limit")
                    case "reset_limit":
                        self.motor.reset_software_limit()
                    case "home_back":
                        if self.last_home_offset is None or self.last_home_speed is None:
                            self.get_logger().error("❌ Last homing direction not detected, please execute home_pos or home_neg first")
                            return
                        # For home_pos, back is -100/-20000; for home_neg, back is 100/20000
                        if self.last_home_offset == -10000 and self.last_home_speed == -100:
                            speed = -100
                            offset = -10000
                        elif self.last_home_offset == 10000 and self.last_home_speed == 100:
                            speed = 100
                            offset = 10000
                        else:
                            self.get_logger().error("❌ last_home_offset/speed abnormal")
                            return
                        self.motor.set_target_velocity(speed)
                        self.motor.set_target_position(offset)
                        self.motor.move_relative()
                        self.get_logger().info(f"home_back (relative): move with speed {speed}, offset {offset}")
                    case "save_params":
                        self.motor.save_all_params_to_eeprom()
                        self.get_logger().info("✅ All parameters saved to EEPROM.")
                    case "factory_reset":
                        self.motor.factory_reset()
                        self.get_logger().info("✅ All parameters restored to factory defaults.")
                    case _:
                        self.get_logger().warn(f"Unknown command: {cmd}")
            except Exception as e:
                self.get_logger().error(f"❌ Command '{cmd}' failed: {e}")
        # 每次运动指令前检测报警
        if cmd in ["jog_left", "jog_right", "move_abs", "move_rel", "move_vel", "home_pos", "home_neg"]:
            self.try_reset_alarm_if_needed(do_command)
        else:
            do_command()

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
