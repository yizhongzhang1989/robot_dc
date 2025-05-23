import rclpy
from rclpy.node import Node
from modbus_driver_interfaces.msg import ModbusPacket
from modbus_driver_interfaces.srv import ModbusRequest
import uuid

class ModbusSimulationNode(Node):
    def __init__(self):
        super().__init__('modbus_simulation_node')

        # Service interface to controllers
        self.srv = self.create_service(
            ModbusRequest,
            '/modbus_request',
            self.handle_modbus_request
        )

        # Publisher to simulate Modbus wire
        self.publisher = self.create_publisher(
            ModbusPacket,
            '/modbus_sim_cable',
            10
        )

        self.get_logger().info("✅ Modbus Simulation Node initialized (broadcast mode)")

    def handle_modbus_request(self, request, response):
        # Broadcast to all simulated devices
        msg = ModbusPacket()
        msg.function_code = request.function_code
        msg.slave_id = request.slave_id
        msg.address = request.address
        msg.count = request.count
        msg.values = request.values

        self.publisher.publish(msg)

        self.get_logger().info(
            f"📤 Broadcast: fc={request.function_code}, sid={request.slave_id}, "
            f"addr={request.address}, values={request.values}"
        )

        # Return dummy response
        response.success = True
        if request.function_code == 3:
            response.response = [0] * request.count
        elif request.function_code == 6:
            response.response = [request.values[0]]
        elif request.function_code == 16:
            response.response = request.values
        else:
            response.success = False
            response.response = []

        return response

    def destroy_node(self):
        self.get_logger().info("Shutting down ModbusSimulationNode.")
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = ModbusSimulationNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
