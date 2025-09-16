from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *

class DrawWireSensorController(ModbusDevice):
    """拉线传感器控制器 (原 CableSensorController)"""
    def __init__(self, device_id, node, use_ack_patch):
        super().__init__(device_id, node, use_ack_patch)
        self.sensor_data = [0, 0]
        self.last_read_time = None

    def initialize(self):
        self.node.get_logger().info("拉线传感器初始化完成")

    def read_sensor_data(self, seq_id=None):
        self.node.get_logger().info(f"[SEQ {seq_id}] 读取拉线传感器数据")
        def handle_sensor_response(response):
            if response and len(response) >= 2:
                self.sensor_data = response[:2]
                import time
                self.last_read_time = time.time()
                self.node.get_logger().info(
                    f"[SEQ {seq_id}] 读取成功: reg0={self.sensor_data[0]}, reg1={self.sensor_data[1]}"
                )
            else:
                self.node.get_logger().error(f"[SEQ {seq_id}] 数据读取失败/格式错误")
        self.recv(3, 0x0000, 2, handle_sensor_response, seq_id=seq_id)

    def get_sensor_data(self):
        return self.sensor_data[0], self.sensor_data[1], self.last_read_time

    def cleanup(self):
        self.node.get_logger().info("拉线传感器清理完成")
