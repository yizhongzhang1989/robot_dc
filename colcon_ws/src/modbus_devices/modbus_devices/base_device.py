from abc import ABC
from modbus_driver_interfaces.srv import ModbusRequest


class ModbusDevice(ABC):
    def __init__(self, device_id, node):
        self.device_id = device_id
        self.node = node

        self.cli = node.create_client(ModbusRequest, '/modbus_request')

    def send(self, func_code, addr, values):
        if not self.cli.service_is_ready():
            self.get_logger().warn("Modbus service not available. Skipping write request.")
            return

        req = ModbusRequest.Request()
        req.function_code = func_code
        req.slave_id = self.device_id
        req.address = addr
        req.values = values

        future = self.cli.call_async(req)

        def handle_response(fut):
            if fut.result() is not None and fut.result().success:
                self.node.get_logger().info(
                    f"✅ Modbus write OK: fc={func_code} addr={hex(addr)} → {values} => {fut.result().response}"
                )
            else:
                self.node.get_logger().error("❌ Modbus request failed or timed out")

        future.add_done_callback(handle_response)

    def recv(self, func_code, addr, count, callback=None):
        if not self.cli.service_is_ready():
            self.node.get_logger().warn("Modbus service not available. Skipping read request.")
            if callback:
                callback([])
            return []
        
        # callback cannot be None if func_code is 3
        if func_code != 3 and callback is not None:
            self.node.get_logger().warn("Callback is required for this function code.")
            return []


        req = ModbusRequest.Request()
        req.function_code = func_code
        req.slave_id = self.device_id
        req.address = addr
        req.count = count
        req.values = []

        future = self.cli.call_async(req)

        future.add_done_callback(
            lambda fut: callback(
                fut.result().response if fut.result() and fut.result().success else []
            )
        )
        return []
