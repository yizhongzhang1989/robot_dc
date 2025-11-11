#!/usr/bin/env python3
"""
Force Sensor Tare (去皮) Script

发送Modbus功能码06 (Write Single Register) 去皮指令。
帧格式: <device_id> 06 00 11 00 01 CRC(lo) CRC(hi)
  - device_id: 传感器Modbus地址 (例如 0x35=53 for force_sensor_2)
  - 功能码: 0x06 (写单个寄存器)
  - 寄存器地址: 0x0011 (去皮命令寄存器)
  - 写入值: 0x0001 (触发去皮)

使用方法:
  python3 force_sensor_tare.py --device_id 53  # force_sensor_2
  python3 force_sensor_tare.py --device_id 52  # force_sensor (主传感器)
  python3 force_sensor_tare.py --device_id 60  # force_sensor_test
"""

import rclpy
from rclpy.node import Node
from modbus_driver_interfaces.srv import ModbusRequest
import argparse
import sys


class ForceSensorTareNode(Node):
    def __init__(self, device_id):
        super().__init__('force_sensor_tare_client')
        self.device_id = device_id
        
        # 创建Modbus请求服务客户端
        self.client = self.create_client(
            ModbusRequest,
            '/modbus_request'
        )
        
        # 等待服务可用
        self.get_logger().info(f'等待Modbus服务...')
        while not self.client.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('Modbus服务未就绪，继续等待...')
        
        self.get_logger().info(f'Modbus服务已连接，准备发送去皮指令到设备 {device_id} (0x{device_id:02X})')

    def send_tare_command(self):
        """发送去皮指令 (功能码06, 寄存器0x0011, 值0x0001)"""
        request = ModbusRequest.Request()
        request.function_code = 0x06  # 功能码06: 写单个寄存器
        request.slave_id = self.device_id
        request.address = 0x0011  # 去皮命令寄存器
        request.count = 0  # 写操作不需要count
        request.values = [0x0001]  # 触发去皮
        request.seq_id = 1
        
        self.get_logger().info(
            f'发送去皮指令: device_id={self.device_id}(0x{self.device_id:02X}), '
            f'func=0x06, reg=0x0011, value=0x0001'
        )
        
        # 异步调用服务
        future = self.client.call_async(request)
        
        return future


def main():
    parser = argparse.ArgumentParser(
        description='发送Modbus去皮指令到力传感器',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 给第二个力传感器去皮 (device_id=53)
  python3 force_sensor_tare.py --device_id 53
  
  # 给主力传感器去皮 (device_id=52)
  python3 force_sensor_tare.py --device_id 52
  
  # 给测试力传感器去皮 (device_id=60)
  python3 force_sensor_tare.py --device_id 60
        """
    )
    parser.add_argument(
        '--device_id',
        type=int,
        default=53,
        help='力传感器Modbus设备ID (默认: 53 for force_sensor_2)'
    )
    
    args = parser.parse_args()
    
    print(f"\n{'='*60}")
    print(f"  力传感器去皮工具 (Force Sensor Tare)")
    print(f"{'='*60}")
    print(f"  目标设备ID: {args.device_id} (0x{args.device_id:02X})")
    print(f"  Modbus帧: {args.device_id:02X} 06 00 11 00 01 + CRC")
    print(f"{'='*60}\n")
    
    rclpy.init()
    
    try:
        node = ForceSensorTareNode(args.device_id)
        
        # 发送去皮命令
        future = node.send_tare_command()
        
        # 等待响应
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        
        if future.done():
            try:
                response = future.result()
                if response.success:
                    print(f"\n✓ 去皮命令发送成功！")
                    print(f"  响应数据: {response.response if response.response else '无'}")
                    print(f"  ACK: {response.ack if hasattr(response, 'ack') else 'N/A'}")
                else:
                    print(f"\n✗ 去皮命令失败")
                    print(f"  响应: {response}")
                    sys.exit(1)
            except Exception as e:
                print(f"\n✗ 服务调用异常: {e}")
                sys.exit(1)
        else:
            print(f"\n✗ 等待响应超时 (3秒)")
            sys.exit(1)
            
    except KeyboardInterrupt:
        print("\n用户中断")
    finally:
        node.destroy_node()
        rclpy.shutdown()
    
    print(f"\n去皮完成。传感器将重新校零当前读数。\n")


if __name__ == '__main__':
    main()
