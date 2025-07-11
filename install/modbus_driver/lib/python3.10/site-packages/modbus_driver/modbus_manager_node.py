import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
from modbus_driver_interfaces.srv import ModbusRequest
import threading
import datetime


class ModbusManagerNode(Node):
    def __init__(self):
        super().__init__('modbus_manager')

        # Declare and get parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value

        # Initialize Modbus RTU client
        self.client = ModbusSerialClient(port=port, baudrate=baudrate, timeout=1)
        self.lock = threading.Lock()

        if not self.client.connect():
            self.get_logger().fatal(f"❌ Failed to open serial port {port}")
            exit(1)

        self.srv = self.create_service(ModbusRequest, '/modbus_request', self.handle_modbus_request)
        self.get_logger().info("✅ Modbus Manager is running")

    def handle_modbus_request(self, request, response):
        seq_id = getattr(request, 'seq_id', None)
        now = datetime.datetime.now().strftime('%Y-%m-%d %H:%M:%S.%f')
        self.get_logger().info(f"[SEQ {seq_id}] [{now}] Received ModbusRequest: {request}")
        response.ack = 1  # 新增：收到请求立即返回ack=1
        with self.lock:
            try:
                fc = request.function_code
                addr = request.address
                slave = request.slave_id
                values = request.values

                if fc == 1:
                    # Read coils
                    result = self.client.read_coils(address=addr, count=request.count, slave=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = [int(b) for b in result.bits]
                    self.get_logger().info(f"[SEQ {seq_id}] [{now}] Read coils: address={addr}, count={request.count}, slave={slave}, success={response.success}, response={response.response}")

                elif fc == 3:
                    # Read holding registers
                    result = self.client.read_holding_registers(address=addr, count=request.count, slave=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = list(result.registers)
                    self.get_logger().info(f"[SEQ {seq_id}] [{now}] Read holding registers: address={addr}, count={request.count}, slave={slave}, success={response.success}, response={response.response}")

                elif fc == 5:
                    # Write single coil (ON=0xFF00, OFF=0x0000)
                    result = self.client.write_coil(address=addr, value=bool(values[0]), slave=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = [int(values[0])]
                    self.get_logger().info(f"[SEQ {seq_id}] [{now}] Write single coil: address={addr}, value={values[0]}, slave={slave}, success={response.success}, response={response.response}")

                elif fc == 6:
                    # Write single register
                    result = self.client.write_register(address=addr, value=values[0], slave=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = [values[0]]
                    self.get_logger().info(f"[SEQ {seq_id}] [{now}] Write single register: address={addr}, value={values[0]}, slave={slave}, success={response.success}, response={response.response}")

                elif fc == 15:
                    # Write multiple coils
                    coil_values = [bool(v) for v in values]
                    result = self.client.write_coils(address=addr, values=coil_values, slave=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = [int(v) for v in coil_values]
                    self.get_logger().info(f"[SEQ {seq_id}] [{now}] Write multiple coils: address={addr}, values={coil_values}, slave={slave}, success={response.success}, response={response.response}")

                elif fc == 16:
                    # Write multiple registers
                    result = self.client.write_registers(address=addr, values=values, slave=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = values
                    self.get_logger().info(f"[SEQ {seq_id}] [{now}] Write multiple registers: address={addr}, values={values}, slave={slave}, success={response.success}, response={response.response}")

                else:
                    raise ValueError(f"Unsupported function code: {fc}")

            except Exception as e:
                self.get_logger().error(f"[SEQ {seq_id}] [{now}] Modbus error: {e}")
                response.success = False
                response.response = []
                self.get_logger().error(f"[SEQ {seq_id}] [{now}] Modbus error response: success={response.success}, response={response.response}")

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
