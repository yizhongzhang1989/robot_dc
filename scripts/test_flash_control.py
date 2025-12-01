#!/usr/bin/env python3
"""
平台硬件闪开指令控制脚本

使用硬件闪开指令控制平台升降 (继电器0-2)
延时时间: 700ms (interval_100ms=7)

使用方法:
    python3 test_flash_control.py up       # 上升
    python3 test_flash_control.py down     # 下降
    python3 test_flash_control.py stop     # 停止
"""

import rclpy
from rclpy.node import Node
from modbus_driver_interfaces.srv import ModbusRequest
import sys


class FlashControlTest(Node):
    def __init__(self):
        super().__init__('flash_control_test')
        self.client = self.create_client(ModbusRequest, '/modbus_request')
        
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('等待 modbus_request 服务...')
        
        self.get_logger().info('✅ 硬件闪开控制测试脚本已启动')
        self.seq_id = 0

    def send_flash_command(self, relay_addr, interval_100ms=7, slave_id=50):
        """
        发送硬件闪开指令
        
        参数:
            relay_addr: 继电器地址 (0=停止, 1=上升, 2=下降)
            interval_100ms: 间隔时间参数 (默认7, 实际延时 = 7 × 100ms = 700ms)
            slave_id: 设备ID (默认50)
        """
        # 构建raw_message: [功能码, 操作类型, 继电器地址, 间隔时间高字节, 间隔时间低字节]
        function_code = 0x05
        operation_type = 0x02  # 闪开
        interval_high = (interval_100ms >> 8) & 0xFF
        interval_low = interval_100ms & 0xFF
        
        raw_message = [
            function_code,      # 0x05
            operation_type,     # 0x02 (闪开)
            relay_addr,         # 继电器地址
            interval_high,      # 间隔时间高字节
            interval_low        # 间隔时间低字节
        ]
        
        request = ModbusRequest.Request()
        request.slave_id = slave_id
        request.raw_message = raw_message
        # 填充标准字段 (使用raw_message时这些不重要)
        request.function_code = 0
        request.address = 0
        request.count = 0
        request.values = []
        request.seq_id = self.seq_id
        self.seq_id += 1
        
        relay_names = {0: '停止', 1: '上升', 2: '下降'}
        relay_name = relay_names.get(relay_addr, f'继电器{relay_addr}')
        actual_delay_ms = interval_100ms * 100
        
        self.get_logger().info(f'发送硬件闪开指令:')
        self.get_logger().info(f'  动作: {relay_name}')
        self.get_logger().info(f'  设备ID: {slave_id}')
        self.get_logger().info(f'  继电器地址: {relay_addr}')
        self.get_logger().info(f'  延时: {actual_delay_ms}ms')
        self.get_logger().info(f'  raw_message: {[f"0x{b:02X}" for b in raw_message]}')
        
        future = self.client.call_async(request)
        rclpy.spin_until_future_complete(self, future)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ {relay_name}指令发送成功')
            else:
                self.get_logger().error(f'❌ {relay_name}指令发送失败')
            return response.success
        else:
            self.get_logger().error('❌ 服务调用失败')
            return False

    def stop(self):
        """发送停止指令 (继电器0, 700ms)"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('执行: 停止')
        self.get_logger().info('='*60)
        return self.send_flash_command(relay_addr=0, interval_100ms=7)

    def up(self):
        """发送上升指令 (继电器1, 700ms)"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('执行: 上升')
        self.get_logger().info('='*60)
        return self.send_flash_command(relay_addr=1, interval_100ms=7)

    def down(self):
        """发送下降指令 (继电器2, 700ms)"""
        self.get_logger().info('\n' + '='*60)
        self.get_logger().info('执行: 下降')
        self.get_logger().info('='*60)
        return self.send_flash_command(relay_addr=2, interval_100ms=7)


def main():
    rclpy.init()
    
    # 检查命令行参数
    if len(sys.argv) < 2:
        print("\n使用方法:")
        print("  python3 test_flash_control.py up       # 上升")
        print("  python3 test_flash_control.py down     # 下降")
        print("  python3 test_flash_control.py stop     # 停止")
        print()
        sys.exit(1)
    
    command = sys.argv[1].lower()
    
    if command not in ['up', 'down', 'stop']:
        print(f"\n错误: 无效的命令 '{command}'")
        print("有效命令: up, down, stop\n")
        sys.exit(1)
    
    node = FlashControlTest()
    
    try:
        if command == 'up':
            node.up()
        elif command == 'down':
            node.down()
        elif command == 'stop':
            node.stop()
        
        print()
        
    except KeyboardInterrupt:
        print("\n用户中断")
    except Exception as e:
        print(f"\n错误: {e}")
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
