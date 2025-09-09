#!/usr/bin/env python3
"""
Lift Robot Platform ROS2 Node
使用继电器闪开命令控制升降平台
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from .lift_robot_controller import LiftRobotController
import json
import uuid
import logging

# 配置日志
logging.basicConfig(level=logging.INFO)


class LiftRobotNode(Node):
    def __init__(self):
        super().__init__('lift_robot_platform')
        
        # 参数配置
        self.declare_parameter('device_id', 1)
        self.declare_parameter('use_ack_patch', True)
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        
        # 读取参数
        self.device_id = self.get_parameter('device_id').value
        self.use_ack_patch = self.get_parameter('use_ack_patch').value
        self.baudrate = self.get_parameter('baudrate').value
        self.serial_port = self.get_parameter('serial_port').value
        
        self.get_logger().info(
            f"初始化升降平台控制器 - 设备ID: {self.device_id}, "
            f"波特率: {self.baudrate}, 串口: {self.serial_port}"
        )
        
        # 创建控制器
        self.controller = LiftRobotController(
            device_id=self.device_id,
            node=self,
            use_ack_patch=self.use_ack_patch
        )
        
        # 订阅命令主题
        self.subscription = self.create_subscription(
            String,
            'lift_robot_platform/command',
            self.command_callback,
            10
        )
        
        # 发布状态主题
        self.status_publisher = self.create_publisher(
            String,
            'lift_robot_platform/status',
            10
        )
        
        # 创建定时器发布状态
        self.status_timer = self.create_timer(1.0, self.publish_status)
        
        # 初始化升降平台
        self.controller.initialize()
        
        self.get_logger().info("升降平台控制节点启动成功")

    def command_callback(self, msg):
        """处理命令消息"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '').lower()
            seq_id_str = command_data.get('seq_id', str(uuid.uuid4())[:8])
            # 将seq_id转换为整数，使用hash来确保唯一性
            seq_id = abs(hash(seq_id_str)) % 65536  # 限制为0-65535范围
            
            self.get_logger().info(f"收到命令: {command} [SEQ {seq_id_str}]")
            
            if command == 'stop':
                self.controller.stop(seq_id=seq_id)
                
            elif command == 'up':
                self.controller.up(seq_id=seq_id)
                
            elif command == 'down':
                self.controller.down(seq_id=seq_id)
                
            elif command == 'timed_up':
                duration = command_data.get('duration', 1.0)
                self.controller.timed_up(duration, seq_id=seq_id)
                
            elif command == 'timed_down':
                duration = command_data.get('duration', 1.0)
                self.controller.timed_down(duration, seq_id=seq_id)
                
            elif command == 'stop_timed':
                self.controller.stop_timed(seq_id=seq_id)
                
            else:
                self.get_logger().warning(f"未知命令: {command}")
                
        except json.JSONDecodeError:
            self.get_logger().error(f"无法解析命令: {msg.data}")
        except Exception as e:
            self.get_logger().error(f"处理命令时出错: {e}")

    def publish_status(self):
        """发布状态信息"""
        status = {
            'node': 'lift_robot_platform',
            'device_id': self.device_id,
            'active_timers': len(self.controller.active_timers),
            'status': 'online'
        }
        
        status_msg = String()
        status_msg.data = json.dumps(status)
        self.status_publisher.publish(status_msg)

    def destroy_node(self):
        """清理资源"""
        self.get_logger().info("正在停止升降平台控制节点...")
        
        # 停止平台并清理定时器
        self.controller.cleanup()
        
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    
    try:
        node = LiftRobotNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"节点运行错误: {e}")
    finally:
        if 'node' in locals():
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
