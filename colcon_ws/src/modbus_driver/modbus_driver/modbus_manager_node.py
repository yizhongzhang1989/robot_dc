import rclpy
from rclpy.node import Node
from modbus_driver.modbus_rtu_interface import ModbusRTUInterface
from modbus_driver.srv import ModbusRequest

class ModbusManagerNode(Node):
    def __init__(self):
        super().__init__('modbus_manager')
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)

        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        self.interface = ModbusRTUInterface(port, baudrate)

        if not self.interface.connect():
            self.get_logger().fatal(f"❌ Failed to open {port}")
            exit(1)

        self.srv = self.create_service(ModbusRequest, 'modbus_request', self.handle_modbus_request)
        self.get_logger().info("✅ Modbus Manager is running")

    def handle_modbus_request(self, request, response):
        try:
            if request.function_code == 3:
                result = self.interface.read_register(request.address, request.slave_id, len(request.values))
                if result.isError(): raise Exception(str(result))
                response.result = list(result.registers)
            elif request.function_code == 6:
                result = self.interface.write_register(request.address, request.values[0], request.slave_id)
                if result.isError(): raise Exception(str(result))
                response.result = [request.values[0]]
            elif request.function_code == 16:
                result = self.interface.write_registers(request.address, request.values, request.slave_id)
                if result.isError(): raise Exception(str(result))
                response.result = request.values
            else:
                raise ValueError("Unsupported function code")

            response.success = True
        except Exception as e:
            response.success = False
            response.error_message = str(e)
            self.get_logger().error(f"Modbus error: {e}")
        return response

def main():
    rclpy.init()
    node = ModbusManagerNode()
    rclpy.spin(node)
    node.interface.disconnect()
    rclpy.shutdown()

if __name__ == '__main__':
    main()