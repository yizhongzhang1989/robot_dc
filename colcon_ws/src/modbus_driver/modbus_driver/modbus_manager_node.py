import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
from modbus_driver_interfaces.srv import ModbusRequest
import threading


class ModbusManagerNode(Node):
    def __init__(self):
        super().__init__('modbus_manager')

        # Declare and get parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 38400)
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # Initialize Modbus RTU client
        self.client = ModbusSerialClient(port=port, baudrate=baudrate, timeout=1)
        self.lock = threading.Lock()

        if not self.client.connect():
            self.get_logger().fatal(f"❌ Failed to open serial port {port}")
            exit(1)

        # Create service
        self.srv = self.create_service(ModbusRequest, '/modbus_request', self.handle_modbus_request)
        self.get_logger().info("✅ Modbus Manager is running")

    def handle_modbus_request(self, request, response):
        with self.lock:
            try:
                # self.get_logger().info(f"Handling Modbus request: {request}")

                if request.function_code == 3:
                    result = self.client.read_holding_registers(
                        address=request.address,
                        count=request.count,
                        slave=request.slave_id
                    )
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = list(result.registers)

                elif request.function_code == 6:
                    result = self.client.write_register(
                        address=request.address,
                        value=request.values[0],
                        slave=request.slave_id
                    )
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = [request.values[0]]

                elif request.function_code == 16:
                    result = self.client.write_registers(
                        address=request.address,
                        values=request.values,
                        slave=request.slave_id
                    )
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = request.values

                else:
                    raise ValueError(f"Unsupported function code: {request.function_code}")

            except Exception as e:
                self.get_logger().error(f"Modbus error: {e}")
                response.success = False
                response.response = []

        # self.get_logger().info(f"Modbus response: {response}")

        return response

    def destroy_node(self):
        self.client.close()
        super().destroy_node()


def main():
    rclpy.init()
    node = ModbusManagerNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
