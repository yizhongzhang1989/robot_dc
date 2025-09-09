#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import Temperature
import json
import time
import threading

from .cable_sensor_controller import CableSensorController

class CableSensorNode(Node):
    def __init__(self):
        super().__init__('cable_sensor_node')
        
        # 声明参数
        self.declare_parameter('device_id', 51)
        self.declare_parameter('use_ack_patch', True)
        self.declare_parameter('read_interval', 1.0)  # 读取间隔(秒)
        
        # 获取参数
        device_id = self.get_parameter('device_id').value
        use_ack_patch = self.get_parameter('use_ack_patch').value
        self.read_interval = self.get_parameter('read_interval').value
        
        self.get_logger().info(f"启动缆线传感器节点: 设备ID={device_id}, 读取间隔={self.read_interval}s")
        
        # 初始化控制器
        self.controller = CableSensorController(device_id, self, use_ack_patch)
        self.controller.initialize()
        
        # 创建发布者
        self.sensor_data_publisher = self.create_publisher(
            String, 
            '/cable_sensor/data', 
            10
        )
        
        # 创建服务订阅
        self.command_subscription = self.create_subscription(
            String,
            '/cable_sensor/command',
            self.command_callback,
            10
        )
        
        # 序列ID计数器
        self.seq_id = 0
        
        # 创建定时器进行周期性读取
        self.read_timer = self.create_timer(
            self.read_interval, 
            self.periodic_read_callback
        )
        
        self.get_logger().info("缆线传感器节点启动完成")
        
    def get_next_seq_id(self):
        """获取下一个序列ID"""
        self.seq_id += 1
        return self.seq_id
        
    def command_callback(self, msg):
        """处理命令回调"""
        try:
            command_data = json.loads(msg.data)
            command = command_data.get('command', '')
            seq_id = self.get_next_seq_id()
            
            self.get_logger().info(f"[SEQ {seq_id}] 收到命令: {command}")
            
            if command == 'read':
                # 立即读取传感器数据
                self.controller.read_sensor_data(seq_id=seq_id)
                
            elif command == 'get_data':
                # 获取并发布最新数据
                self.publish_sensor_data(seq_id=seq_id)
                
            else:
                self.get_logger().warn(f"[SEQ {seq_id}] 未知命令: {command}")
                
        except json.JSONDecodeError:
            self.get_logger().error("命令格式错误，应为JSON格式")
        except Exception as e:
            self.get_logger().error(f"处理命令时出错: {e}")
            
    def periodic_read_callback(self):
        """周期性读取传感器数据"""
        seq_id = self.get_next_seq_id()
        self.get_logger().debug(f"[SEQ {seq_id}] 周期性读取传感器数据")
        
        # 读取传感器数据
        self.controller.read_sensor_data(seq_id=seq_id)
        
        # 延时后发布数据
        def delayed_publish():
            time.sleep(0.1)  # 等待100ms确保数据读取完成
            self.publish_sensor_data(seq_id=seq_id)
            
        threading.Timer(0.1, delayed_publish).start()
        
    def publish_sensor_data(self, seq_id=None):
        """发布传感器数据"""
        reg0, reg1, timestamp = self.controller.get_sensor_data()
        
        # 构造数据消息
        sensor_msg = {
            'timestamp': timestamp,
            'register_0': reg0,
            'register_1': reg1,
            'device_id': 51,
            'seq_id': seq_id
        }
        
        # 发布数据
        msg = String()
        msg.data = json.dumps(sensor_msg)
        self.sensor_data_publisher.publish(msg)
        
        self.get_logger().debug(
            f"[SEQ {seq_id}] 发布传感器数据: reg0={reg0}, reg1={reg1}"
        )
        
    def destroy_node(self):
        """节点销毁时的清理"""
        self.get_logger().info("正在关闭缆线传感器节点...")
        if hasattr(self, 'controller'):
            self.controller.cleanup()
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    
    node = CableSensorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
