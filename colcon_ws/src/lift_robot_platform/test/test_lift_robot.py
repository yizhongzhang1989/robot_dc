#!/usr/bin/env python3
"""
测试升降平台控制器
发送各种命令测试功能
"""
import json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import time


class LiftRobotTester(Node):
    def __init__(self):
        super().__init__('lift_robot_tester')
        
        # 创建发布者
        self.command_publisher = self.create_publisher(
            String,
            'lift_robot_platform/command',
            10
        )
        
        # 订阅状态
        self.status_subscription = self.create_subscription(
            String,
            'lift_robot_platform/status',
            self.status_callback,
            10
        )
        
        self.get_logger().info("升降平台测试器启动")

    def status_callback(self, msg):
        """状态回调"""
        try:
            status = json.loads(msg.data)
            self.get_logger().info(f"状态: {status}")
        except:
            pass

    def send_command(self, command, **kwargs):
        """发送命令"""
        cmd_data = {'command': command}
        cmd_data.update(kwargs)
        
        msg = String()
        msg.data = json.dumps(cmd_data)
        
        self.command_publisher.publish(msg)
        self.get_logger().info(f"发送命令: {cmd_data}")

    def run_test(self):
        """运行测试序列"""
        self.get_logger().info("开始测试序列...")
        
        # 等待节点启动
        time.sleep(2)
        
        # 测试基本命令
        self.get_logger().info("=== 测试基本命令 ===")
        
        self.send_command('stop')
        time.sleep(1)
        
        self.send_command('up')
        time.sleep(2)
        
        self.send_command('stop')
        time.sleep(1)
        
        self.send_command('down')
        time.sleep(2)
        
        self.send_command('stop')
        time.sleep(1)
        
        # 测试定时命令
        self.get_logger().info("=== 测试定时命令 ===")
        
        self.send_command('timed_up', duration=3.0)
        time.sleep(5)  # 等待自动停止
        
        self.send_command('timed_down', duration=2.0)
        time.sleep(4)  # 等待自动停止
        
        # 测试手动停止定时命令
        self.get_logger().info("=== 测试手动停止定时命令 ===")
        
        self.send_command('timed_up', duration=10.0)
        time.sleep(2)
        self.send_command('stop_timed')  # 提前停止
        
        self.get_logger().info("测试完成!")


def main():
    rclpy.init()
    
    try:
        tester = LiftRobotTester()
        
        # 运行测试
        tester.run_test()
        
        # 继续监听状态
        rclpy.spin(tester)
        
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
