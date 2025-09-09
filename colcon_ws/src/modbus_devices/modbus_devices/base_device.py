from abc import ABC
from modbus_driver_interfaces.srv import ModbusRequest


class ModbusDevice(ABC):
    def __init__(self, device_id, node, use_ack_patch=False):
        self.device_id = device_id
        self.node = node
        self.cli = node.create_client(ModbusRequest, '/modbus_request')
        self.use_ack_patch = use_ack_patch
        if use_ack_patch:
            # Save the original send method so we can call it later
            self._orig_send = self.send

            # Define a new send method that injects an ACK callback
            def send_with_ack(*args, **kwargs):
                # Define the ACK callback that will be triggered once the ModbusManager responds
                def ack_callback(fut):
                    result = fut.result()
                    # If manager confirms ack=1
                    if hasattr(result, 'ack') and result.ack == 1:
                        # Clear the waiting flag if the node has it
                        if hasattr(self.node, 'waiting_for_ack'):
                            self.node.waiting_for_ack = False
                            # Trigger the next command if the node supports queued execution
                            if hasattr(self.node, 'process_next_command'):
                                self.node.process_next_command()

                # Inject ack_callback into the send call
                kwargs['callback'] = ack_callback
                # Call the original send method with injected callback
                return self._orig_send(*args, **kwargs)

            # Patch the instance's send method with send_with_ack
            self.send = send_with_ack

    def send(self, func_code, addr, values, seq_id=None, callback=None):
        # Check if the Modbus service is available
        if not self.cli.service_is_ready():
            self.node.get_logger().warn("Modbus service not available. Skipping write request.")
            return

        # Build the ModbusRequest message
        req = ModbusRequest.Request()
        req.function_code = func_code              # e.g. write single register (0x06)
        req.slave_id = self.device_id              # Target device ID
        req.address = addr                         # Register address to write
        req.values = values                        # Data to be written

        # Assign sequence ID (used for tracking responses)
        if seq_id is not None:
            req.seq_id = seq_id
        else:
            req.seq_id = getattr(self.node, 'seq_id', 0)  # Default fallback if node has seq_id attr

        # Send the request asynchronously
        future = self.cli.call_async(req)

        # Define the response handler
        def handle_response(fut):
            result = fut.result()
            # Log ack if received
            if hasattr(result, 'ack') and result.ack == 1:
                self.node.get_logger().info("收到manager确认: 1")

            # Log result status
            if result is not None and result.success:
                self.node.get_logger().info(
                    f"[SEQ {req.seq_id}] ✅ Modbus write OK: fc={func_code} addr={hex(addr)} → {values} => {result.response}"
                )
            else:
                self.node.get_logger().error(f"[SEQ {req.seq_id}] ❌ Modbus request failed or timed out")

            # If a callback was given (e.g., ack_callback in send_with_ack), call it
            if callback:
                callback(fut)

        # Register the response handler to the future
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