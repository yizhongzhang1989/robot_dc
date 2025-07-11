import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
try:
    from leadshine_motor.motor_controller import LeadshineMotor
except ImportError:
    from ..leadshine_motor.motor_controller import LeadshineMotor
#import feetech_servo.servo_controller as servo_mod  # 预留servo导入
import threading

class DetectorNode(Node):
    def __init__(self):
        super().__init__('detector_node')
        # motor IDs 和 servo IDs
        self.motor_ids = [1, 2]
        self.servo_ids = [17, 18]
        self.use_ack_patch = 1  # 可根据需要设为参数

        # 初始化motor对象
        self.motors = {}
        for mid in self.motor_ids:
            self.motors[mid] = LeadshineMotor(mid, self, use_ack_patch=self.use_ack_patch)
        # 初始化servo对象（如有）
        self.servos = {}
        #for sid in self.servo_ids:
        #    self.servos[sid] = servo_mod.FeetechServo(sid, self, use_ack_patch=self.use_ack_patch)

        # 发布motor和servo位置的topic
        self.motor_pos_pubs = {mid: self.create_publisher(Float32, f'/motor{mid}/position', 10) for mid in self.motor_ids}
        self.servo_pos_pubs = {sid: self.create_publisher(Float32, f'/servo{sid}/position', 10) for sid in self.servo_ids}

        # 5Hz定时器
        self.timer = self.create_timer(0.2, self.timer_callback)
        self.get_logger().info('DetectorNode started, reading positions at 5Hz.')

    def timer_callback(self):
        # 读取motor位置
        for mid, motor in self.motors.items():
            def make_cb(mid):
                def cb(pos):
                    if pos is not None:
                        msg = Float32()
                        msg.data = float(pos)
                        self.motor_pos_pubs[mid].publish(msg)
                return cb
            try:
                motor.get_current_position(make_cb(mid))
            except Exception as e:
                self.get_logger().error(f"Error reading motor {mid} position: {e}")
        # 读取servo位置（如有实现）
        #for sid, servo in self.servos.items():
        #    pos = None  # TODO: 调用servo的实际读取方法
        #    if pos is not None:
        #        msg = Float32()
        #        msg.data = float(pos)
        #        self.servo_pos_pubs[sid].publish(msg)

    # 预留：如需单独读取motor/servo位置
    #def read_motor_position(self, mid):
    #    ...
    #def read_servo_position(self, sid):
    #    ...

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