import rclpy
from rclpy.node import Node
from modbus_driver.srv import ModbusRequest

class ModbusClientTester(Node):
    def __init__(self):
        super().__init__('modbus_client_tester')
        self.cli = self.create_client(ModbusRequest, 'modbus_request')
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for modbus_request service...')
        self.send_test()

    def send_test(self):
        req = ModbusRequest.Request()
        req.function_code = 3
        req.slave_id = 1
        req.address = 0
        req.values = [0, 0]

        future = self.cli.call_async(req)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            result = future.result()
            self.get_logger().info(f"Response: {result.result}, Success: {result.success}")
        else:
            self.get_logger().error("Service call failed")

def main():
    rclpy.init()
    node = ModbusClientTester()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
