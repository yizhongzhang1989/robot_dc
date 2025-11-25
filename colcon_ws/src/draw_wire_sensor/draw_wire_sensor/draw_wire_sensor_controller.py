from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *

class DrawWireSensorController(ModbusDevice):
    """Draw-wire sensor controller (formerly CableSensorController)."""
    def __init__(self, device_id, node, use_ack_patch):
        super().__init__(device_id, node, use_ack_patch)
        self.sensor_data = [0, 0]
        self.last_read_time = None

    def initialize(self):
        self.node.get_logger().info("Draw-wire sensor initialized")

    def read_sensor_data(self, seq_id=None):
        try:
            # Removed info log to reduce output
            def handle_sensor_response(response):
                try:
                    if response and len(response) >= 2:
                        self.sensor_data = response[:2]
                        import time
                        self.last_read_time = time.time()
                        # Removed info log - data published via topic
                    else:
                        self.node.get_logger().error(f"[SEQ {seq_id}] Read failed / invalid format")
                except Exception as e:
                    self.node.get_logger().error(f"[SEQ {seq_id}] Sensor response handler error: {e}")
            self.recv(3, 0x0000, 2, handle_sensor_response, seq_id=seq_id)
        except Exception as e:
            self.node.get_logger().error(f"[SEQ {seq_id}] Read sensor data error: {e}")
            # Continue operation - sensor read failed but system should stay up

    def get_sensor_data(self):
        return self.sensor_data[0], self.sensor_data[1], self.last_read_time

    def cleanup(self):
        self.node.get_logger().info("Draw-wire sensor cleanup done")
