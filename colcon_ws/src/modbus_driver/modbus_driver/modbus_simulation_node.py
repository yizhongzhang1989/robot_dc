import rclpy  
from rclpy.node import Node  
from modbus_driver_interfaces.srv import ModbusRequest  
  
import threading  
  
class ModbusSimulationNode(Node):  
    def __init__(self):  
        super().__init__('modbus_simulation_node')  
  
        # Instead of "self.clients = []", name it something else:  
        self._modbus_clients = []  
  
        self.declare_parameter('motor_names', ['motor1', 'motor2'])  
        self.motor_names = self.get_parameter('motor_names').value  
  
        # Create the service server  
        self.srv = self.create_service(  
            ModbusRequest,  
            'modbus_request',  
            self.handle_modbus_request  
        )  
  
        self.lock = threading.Lock()  
  
        # Create a ModbusRequest client for each motor  
        for name in self.motor_names:  
            service_name = f'/{name}/modbus_request'  
            client = self.create_client(ModbusRequest, service_name)  
            self._modbus_clients.append((name, client))  
  
        # Wait for each motor service  
        for (name, client) in self._modbus_clients:  
            self.get_logger().info(f"Waiting for service {client.srv_name} for {name}...")  
            if not client.wait_for_service(timeout_sec=5.0):  
                self.get_logger().warn(f"Timed out waiting for {client.srv_name}")  
          
        self.get_logger().info("✅ Modbus Simulation Node is ready!")  
  
    def handle_modbus_request(self, request, response):  
        with self.lock:  
            got_success_response = False  
            combined_response_data = []  
  
            self.get_logger().info(  
                f"SimBus got ModbusRequest fc={request.function_code}, "  
                f"addr={request.address}, slave_id={request.slave_id}, values={request.values}"  
            )  
  
            # Broadcast to all simulated motors  
            for (name, client) in self._modbus_clients:  
                req = ModbusRequest.Request()  
                req.function_code = request.function_code  
                req.slave_id = request.slave_id  
                req.address = request.address  
                req.count = request.count  
                req.values = request.values  
  
                future = client.call_async(req)  
                rclpy.spin_until_future_complete(self, future)  
                if future.result():  
                    r = future.result()  
                    if r.success:  
                        got_success_response = True  
                        combined_response_data = r.response  
                        self.get_logger().info(  
                            f"SimBus: {name} responded success=True with {r.response}"  
                        )  
                        break  
  
            if got_success_response:  
                response.success = True  
                response.response = combined_response_data  
            else:  
                self.get_logger().warn("SimBus: No motor responded success => unknown slave_id?")  
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
