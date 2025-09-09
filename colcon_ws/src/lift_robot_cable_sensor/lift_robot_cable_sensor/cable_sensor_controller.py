from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *

class CableSensorController(ModbusDevice):
    """
    缆线传感器控制器 - 读取传感器数据
    
    传感器配置：
    - 功能：读取缆线传感器数据
    - Modbus命令：33 03 00 00 00 02 +CRC (读取2个寄存器)
    
    通信参数：
    - 波特率: 115200
    - 设备ID: 51 (0x33 hex)
    - Modbus功能码03: 读取保持寄存器
    - 起始地址: 0x0000
    - 寄存器数量: 2
    """
    
    def __init__(self, device_id, node, use_ack_patch):
        super().__init__(device_id, node, use_ack_patch)
        
        # 传感器数据
        self.sensor_data = [0, 0]  # 存储2个寄存器的数据
        self.last_read_time = None
        
    def initialize(self):
        """初始化缆线传感器"""
        self.node.get_logger().info("缆线传感器初始化完成")
        
    def read_sensor_data(self, seq_id=None):
        """
        读取传感器数据
        
        Modbus命令: 33 03 00 00 00 02 +CRC
        - 设备ID: 51 (0x33)
        - 功能码: 03 (读取保持寄存器)
        - 起始地址: 0x0000
        - 寄存器数量: 2
        
        Args:
            seq_id: 序列ID
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] 读取缆线传感器数据")
        
        def handle_sensor_response(response):
            """处理传感器响应数据"""
            if response and len(response) >= 2:
                self.sensor_data = response[:2]  # 取前2个寄存器数据
                import time
                self.last_read_time = time.time()
                
                self.node.get_logger().info(
                    f"[SEQ {seq_id}] 传感器数据读取成功: 寄存器0={self.sensor_data[0]}, 寄存器1={self.sensor_data[1]}"
                )
            else:
                self.node.get_logger().error(f"[SEQ {seq_id}] 传感器数据读取失败或数据格式错误")
        
        # 发送Modbus读取命令：功能码03, 地址0x0000, 读取2个寄存器
        self.recv(3, 0x0000, 2, handle_sensor_response, seq_id=seq_id)
        
    def get_sensor_data(self):
        """
        获取最新的传感器数据
        
        Returns:
            tuple: (register0_value, register1_value, timestamp)
        """
        return self.sensor_data[0], self.sensor_data[1], self.last_read_time
        
    def cleanup(self):
        """清理资源"""
        self.node.get_logger().info("缆线传感器清理完成")
