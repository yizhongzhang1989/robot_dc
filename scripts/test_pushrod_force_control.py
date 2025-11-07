#!/usr/bin/env python3
"""简单推杆力控测试脚本

功能: 向主题 lift_robot_pushrod/command 发送一次 enable_force_control 指令
参数: target_force=145, force_threshold=5
可选: force_settle_cycles=5, increase_on_up=True

使用:
    python3 scripts/test_pushrod_force_control.py

确保已经启动 pushrod 节点 (例如 bringup 已经运行).
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json
import time

class PushrodForceTest(Node):
    def __init__(self):
        super().__init__('pushrod_force_test_sender')
        self.pub = self.create_publisher(String, 'lift_robot_pushrod/command', 10)

    def send_force_enable(self):
        msg = String()
        payload = {
            "command": "enable_force_control",
            "target_force": 145,
            "force_threshold": 5,
            "force_settle_cycles": 5,
            "increase_on_up": True
        }
        msg.data = json.dumps(payload, ensure_ascii=False)
        self.pub.publish(msg)
        self.get_logger().info(f"已发送: {msg.data}")


def main(args=None):
    rclpy.init(args=args)
    node = PushrodForceTest()
    try:
        # 发送一次后等待 1 秒再退出，确保消息送达
        node.send_force_enable()
        rclpy.spin_once(node, timeout_sec=1.0)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
