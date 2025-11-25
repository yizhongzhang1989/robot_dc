#!/usr/bin/env python3
"""
测试脚本：pushrod继电器闪开控制 (Relay 3, 4, 5)

使用方法:
    python3 test_relay_polling.py up      # 测试 UP 继电器 (Relay 5)
    python3 test_relay_polling.py down    # 测试 DOWN 继电器 (Relay 4)
    python3 test_relay_polling.py stop    # 测试 STOP 继电器 (Relay 3)

说明:
    - 发送 ON 指令 → 立即检测，失败则重试，最多3次
    - 发送 OFF 指令 → 立即检测，失败则重试，最多3次
    - 无需50Hz轮询，每次发送后立即验证
"""

import sys
import time
import rclpy
from rclpy.node import Node
from modbus_driver_interfaces.srv import ModbusRequest

# Pushrod继电器地址映射
RELAY_MAP = {
    'stop': 3,  # Relay 3 - STOP
    'down': 4,  # Relay 4 - DOWN
    'up': 5,    # Relay 5 - UP
}

class RelayPollingTest(Node):
    def __init__(self):
        super().__init__('relay_polling_test')
        
        # Modbus 客户端
        self.cli = self.create_client(ModbusRequest, '/modbus_request')
        
        # 等待服务
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('⏳ Waiting for /modbus_request service...')
        
        self.get_logger().info('✅ /modbus_request service is ready')
        
    def read_relay_status(self, device_id=50):
        """
        读取pushrod继电器 3, 4, 5 的状态
        
        Modbus FC01 (Read Coils):
            设备ID: 50 (0x32)
            功能码: 0x01
            起始地址: 0x0000
            数量: 0x0006 (读6个继电器，取索引3,4,5)
        
        Returns:
            dict: {3: relay3_status, 4: relay4_status, 5: relay5_status}
                  True=ON, False=OFF
        """
        req = ModbusRequest.Request()
        req.slave_id = device_id
        req.function_code = 1  # FC01: Read Coils
        req.address = 0x0000   # 起始地址: Relay 0
        req.count = 6          # 读取6个继电器 (0-5)
        req.values = []
        req.seq_id = 9999
        
        self.get_logger().info(f'📤 Sending FC01 read: addr=0x{req.address:04X}, count={req.count}, slave_id={req.slave_id}')
        
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=2.0)
        
        if future.result() is not None:
            response = future.result()
            self.get_logger().info(f'📥 Response: success={response.success}, response={response.response}')
            if response.success:
                # response.response[3] = Relay3, response.response[4] = Relay4, response.response[5] = Relay5
                if len(response.response) >= 6:
                    return {
                        3: bool(response.response[3]),  # Relay 3 - STOP
                        4: bool(response.response[4]),  # Relay 4 - DOWN
                        5: bool(response.response[5]),  # Relay 5 - UP
                    }
                else:
                    self.get_logger().error(f'❌ Invalid response length: {len(response.response)}')
            else:
                self.get_logger().error(f'❌ Read failed')
        else:
            self.get_logger().error('❌ Service call timeout (2s)')
        
        return {3: False, 4: False, 5: False}
    
    def write_relay(self, relay_address, value, device_id=50):
        """
        写入pushrod继电器状态
        
        Args:
            relay_address: 3=STOP, 4=DOWN, 5=UP
            value: 0x0000=OFF, 0xFF00=ON
        """
        req = ModbusRequest.Request()
        req.slave_id = device_id
        req.function_code = 5  # FC05: Write Single Coil
        req.address = relay_address
        req.count = 1
        req.values = [value]
        req.seq_id = 8888
        
        state = "ON" if value == 0xFF00 else "OFF"
        self.get_logger().info(f'📤 Sending FC05 write: Relay {relay_address} → {state} (value=0x{value:04X})')
        
        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=1.0)
        
        if future.result() is not None:
            response = future.result()
            if response.success:
                self.get_logger().info(f'✅ Relay {relay_address} set to {state}')
                return True
            else:
                self.get_logger().error(f'❌ Write failed')
        else:
            self.get_logger().error('❌ Service call timeout')
        
        return False
    
    def flash_relay_with_polling(self, relay_address, device_id=50):
        """
        使用立即检测+重试的pushrod继电器闪开
        
        流程:
        1. 发送 ON 指令 → 立即检测，最多重试3次
        2. 发送 OFF 指令 → 立即检测，最多重试3次
        
        Args:
            relay_address: 3=STOP, 4=DOWN, 5=UP
        """
        relay_name = {3: 'STOP', 4: 'DOWN', 5: 'UP'}.get(relay_address, 'UNKNOWN')
        
        self.get_logger().info(f'\n{"="*60}')
        self.get_logger().info(f'🚀 Starting {relay_name} relay flash with retry detection')
        self.get_logger().info(f'{"="*60}')
        
        start_time = time.time()
        
        # Step 1: 发送 ON 指令并检测 (最多3次)
        self.get_logger().info(f'\n[Step 1] Sending ON command and verifying...')
        
        max_on_attempts = 3
        relay_turned_on = False
        
        for attempt in range(1, max_on_attempts + 1):
            # 发送 ON 指令
            self.get_logger().info(f'  Attempt {attempt}/{max_on_attempts}: Sending ON to Relay {relay_address}...')
            if not self.write_relay(relay_address, 0xFF00, device_id):
                self.get_logger().error(f'  ❌ Failed to send ON command')
                continue
            
            # 立即检测状态
            time.sleep(0.01)  # 短暂延迟 10ms 让硬件响应
            status = self.read_relay_status(device_id)
            relay_on = status.get(relay_address, False)
            
            elapsed_ms = (time.time() - start_time) * 1000
            
            if relay_on:
                self.get_logger().info(
                    f'  ✅ Attempt {attempt}: Relay {relay_address} is ON! '
                    f'({elapsed_ms:.2f}ms) [Status: {status}]'
                )
                relay_turned_on = True
                break
            else:
                self.get_logger().warn(
                    f'  ⚠️  Attempt {attempt}: Relay {relay_address} still OFF, retrying... '
                    f'[Status: {status}]'
                )
        
        if not relay_turned_on:
            self.get_logger().error(
                f'❌ Failed to turn ON Relay {relay_address} after {max_on_attempts} attempts'
            )
            return False
        
        # Step 2: 发送 OFF 指令并检测 (最多3次)
        self.get_logger().info(f'\n[Step 2] Sending OFF command and verifying...')
        
        max_off_attempts = 3
        relay_turned_off = False
        
        for attempt in range(1, max_off_attempts + 1):
            # 发送 OFF 指令
            self.get_logger().info(f'  Attempt {attempt}/{max_off_attempts}: Sending OFF to Relay {relay_address}...')
            if not self.write_relay(relay_address, 0x0000, device_id):
                self.get_logger().error(f'  ❌ Failed to send OFF command')
                continue
            
            # 立即检测状态
            time.sleep(0.01)  # 短暂延迟 10ms 让硬件响应
            status = self.read_relay_status(device_id)
            relay_off = not status.get(relay_address, True)
            
            elapsed_ms = (time.time() - start_time) * 1000
            
            if relay_off:
                self.get_logger().info(
                    f'  ✅ Attempt {attempt}: Relay {relay_address} is OFF! '
                    f'({elapsed_ms:.2f}ms) [Status: {status}]'
                )
                relay_turned_off = True
                break
            else:
                self.get_logger().warn(
                    f'  ⚠️  Attempt {attempt}: Relay {relay_address} still ON, retrying... '
                    f'[Status: {status}]'
                )
        
        total_time_ms = (time.time() - start_time) * 1000
        
        if relay_turned_off:
            self.get_logger().info(f'\n{"="*60}')
            self.get_logger().info(f'✅ {relay_name} relay flash completed successfully!')
            self.get_logger().info(f'   Total time: {total_time_ms:.2f}ms')
            self.get_logger().info(f'{"="*60}\n')
            return True
        else:
            self.get_logger().error(f'\n{"="*60}')
            self.get_logger().error(f'❌ {relay_name} relay flash FAILED!')
            self.get_logger().error(f'   Relay {relay_address} never turned OFF after {max_off_attempts} attempts')
            self.get_logger().error(f'   Total time: {total_time_ms:.2f}ms')
            self.get_logger().error(f'{"="*60}\n')
            return False


def main():
    if len(sys.argv) != 2:
        print("Usage: python3 test_relay_polling.py <command>")
        print("Commands:")
        print("  up    - Test UP relay (Relay 5)")
        print("  down  - Test DOWN relay (Relay 4)")
        print("  stop  - Test STOP relay (Relay 3)")
        sys.exit(1)
    
    command = sys.argv[1].lower()
    
    if command not in RELAY_MAP:
        print(f"❌ Invalid command: {command}")
        print(f"Valid commands: {list(RELAY_MAP.keys())}")
        sys.exit(1)
    
    relay_address = RELAY_MAP[command]
    
    # 初始化 ROS2
    rclpy.init()
    
    try:
        node = RelayPollingTest()
        
        # 执行继电器闪开测试
        success = node.flash_relay_with_polling(relay_address)
        
        # 清理
        node.destroy_node()
        
        sys.exit(0 if success else 1)
        
    except KeyboardInterrupt:
        print("\n⚠️  Test interrupted by user")
        sys.exit(1)
    except Exception as e:
        print(f"❌ Error: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
