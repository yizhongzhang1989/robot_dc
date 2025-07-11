from abc import ABC
from modbus_driver_interfaces.srv import ModbusRequest


class ModbusDevice(ABC):
    def __init__(self, device_id, node, use_ack_patch=False):
        self.device_id = device_id
        self.node = node
        self.cli = node.create_client(ModbusRequest, '/modbus_request')
        self.use_ack_patch = use_ack_patch
        if use_ack_patch:
            self._orig_send = self.send
            def send_with_ack(*args, **kwargs):
                def ack_callback(fut):
                    result = fut.result()
                    if hasattr(result, 'ack') and result.ack == 1:
                        if hasattr(self.node, 'waiting_for_ack'):
                            self.node.waiting_for_ack = False
                            if hasattr(self.node, 'process_next_command'):
                                self.node.process_next_command()
                kwargs['callback'] = ack_callback
                return self._orig_send(*args, **kwargs)
            self.send = send_with_ack

    def send(self, func_code, addr, values, seq_id=None, callback=None):
        if not self.cli.service_is_ready():
            self.get_logger().warn("Modbus service not available. Skipping write request.")
            return

        req = ModbusRequest.Request()
        req.function_code = func_code
        req.slave_id = self.device_id
        req.address = addr
        req.values = values
        if seq_id is not None:
            req.seq_id = seq_id
        else:
            req.seq_id = getattr(self.node, 'seq_id', 0)

        future = self.cli.call_async(req)

        def handle_response(fut):
            result = fut.result()
            if hasattr(result, 'ack') and result.ack == 1:
                self.node.get_logger().info("收到manager确认: 1")
            if result is not None and result.success:
                self.node.get_logger().info(
                    f"[SEQ {req.seq_id}] ✅ Modbus write OK: fc={func_code} addr={hex(addr)} → {values} => {result.response}"
                )
            else:
                self.node.get_logger().error(f"[SEQ {req.seq_id}] ❌ Modbus request failed or timed out")
            if callback:
                callback(fut)

        future.add_done_callback(handle_response)

    def recv(self, func_code, addr, count, callback=None, seq_id=None):
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
        if seq_id is not None:
            req.seq_id = seq_id
        else:
            req.seq_id = getattr(self.node, 'seq_id', 0)

        future = self.cli.call_async(req)

        def handle_recv_response(fut):
            result = fut.result()
            if hasattr(result, 'ack') and result.ack == 1:
                self.node.get_logger().info("收到manager确认: 1")
            resp = result.response if result and result.success else []
            self.node.get_logger().info(f"[SEQ {req.seq_id}] ✅ Modbus read OK: fc={func_code} addr={hex(addr)} count={count} => {resp}")
            if callback:
                callback(resp)

        future.add_done_callback(handle_recv_response)
        return []
