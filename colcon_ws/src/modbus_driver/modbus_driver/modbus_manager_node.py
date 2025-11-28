import rclpy
from rclpy.node import Node
from pymodbus.client import ModbusSerialClient
from modbus_driver_interfaces.srv import ModbusRequest
import threading
import datetime
import time
import os
from .modbus_dashboard import ModbusDashboard

try:
    from common.workspace_utils import get_temp_directory
    DEFAULT_LOG_DIR = os.path.join(get_temp_directory(), 'modbus_logs')
except Exception:
    DEFAULT_LOG_DIR = 'modbus_logs'


class ModbusManagerNode(Node):
    def __init__(self):
        super().__init__('modbus_manager')

        # Declare and get parameters
        self.declare_parameter('port', '/dev/ttyUSB0')
        self.declare_parameter('baudrate', 115200)
        self.declare_parameter('enable_dashboard', True)
        self.declare_parameter('dashboard_port', 5000)
        self.declare_parameter('log_dir', DEFAULT_LOG_DIR)
        
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        enable_dashboard = self.get_parameter('enable_dashboard').value
        dashboard_port = self.get_parameter('dashboard_port').value
        log_dir = self.get_parameter('log_dir').value

        # Initialize Modbus RTU client
        self.client = ModbusSerialClient(port=port, baudrate=baudrate, timeout=1)
        self.lock = threading.Lock()

        if not self.client.connect():
            self.get_logger().fatal(f"❌ Failed to open serial port {port}")
            exit(1)

        # Initialize Dashboard
        self.dashboard = None
        if enable_dashboard:
            self.dashboard = ModbusDashboard(port=dashboard_port, log_dir=log_dir)
            self.dashboard.start()
            self.get_logger().info(f"✅ Modbus Dashboard running at http://0.0.0.0:{dashboard_port}")
            self.get_logger().info(f"✅ Logging to directory: {log_dir}")

        self.srv = self.create_service(ModbusRequest, '/modbus_request', self.handle_modbus_request)
        self.get_logger().info("✅ Modbus Manager is running")

    def calculate_crc16(self, data):
        """
        计算Modbus CRC16校验和
        """
        crc = 0xFFFF
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 0x0001:
                    crc = (crc >> 1) ^ 0xA001
                else:
                    crc >>= 1
        return crc

    def handle_raw_modbus_command(self, slave_id, raw_message, seq_id, now_str):
        """
        处理非标准Modbus协议的原始命令(如继电器闪开闪闭指令)
        
        参数:
            slave_id: 设备地址 (0x01-0xFF)
            raw_message: 原始消息字节列表 [功能码, 操作类型, 继电器地址, 间隔时间高字节, 间隔时间低字节]
                        例如: [0x05, 0x02, 0x00, 0x00, 0x07] 表示闪开指令
            seq_id: 序列号
            now_str: 时间戳字符串
        
        返回:
            (success, response_data, response_time_ms)
        """
        try:
            # 构建完整的命令: [设备地址] + raw_message
            command = [slave_id] + list(raw_message)
            
            # 计算CRC16校验和
            crc = self.calculate_crc16(command)
            crc_low = crc & 0xFF
            crc_high = (crc >> 8) & 0xFF
            
            # 完整命令包含CRC
            full_command = command + [crc_low, crc_high]
            
            # 记录发送的命令
            command_hex = ' '.join([f'{b:02X}' for b in full_command])
            self.get_logger().info(f"[SEQ {seq_id}] [{now_str}] Sending raw Modbus command: {command_hex}")
            
            # 记录发送时间
            start_time = time.time()
            
            # 发送命令
            self.client.socket.write(bytes(full_command))
            
            try:
                # 尝试读取响应 (最多8字节)
                response_bytes = self.client.socket.read(8)
                response_time_ms = (time.time() - start_time) * 1000
                if response_bytes and len(response_bytes) > 0:
                    response_hex = ' '.join([f'{b:02X}' for b in response_bytes])
                    self.get_logger().info(f"[SEQ {seq_id}] [{now_str}] Received response: {response_hex} ({response_time_ms:.2f}ms)")
                else:
                    self.get_logger().warn(f"[SEQ {seq_id}] [{now_str}] No response received from device ({response_time_ms:.2f}ms)")
            except Exception as read_error:
                response_time_ms = (time.time() - start_time) * 1000
                self.get_logger().warn(f"[SEQ {seq_id}] [{now_str}] Failed to read response: {read_error} ({response_time_ms:.2f}ms)")
            
            self.get_logger().info(f"[SEQ {seq_id}] [{now_str}] Raw Modbus command sent successfully")
            return True, full_command, response_time_ms
            
        except Exception as e:
            self.get_logger().error(f"[SEQ {seq_id}] [{now_str}] Raw Modbus command failed: {e}")
            return False, [], 0.0

    def handle_modbus_request(self, request, response):
        seq_id = getattr(request, 'seq_id', None)
        
        now_ns = time.time_ns()
        dt = datetime.datetime.fromtimestamp(now_ns / 1e9)
        ns_part = now_ns % 1_000_000_000
        now_str = dt.strftime('%Y-%m-%d %H:%M:%S') + f".{ns_part:09d}"
        now_ts = now_ns / 1e9
        
        log_entry = {
            'timestamp': now_str,
            'timestamp_raw': now_ts,
            'seq_id': seq_id,
            'function_code': request.function_code,
            'address': request.address,
            'slave_id': request.slave_id,
            'count': request.count,
            'values': list(request.values),
            'success': False,
            'response': []
        }

        self.get_logger().debug(f"[SEQ {seq_id}] [{now_str}] Received ModbusRequest: {request}")
        response.ack = 1  # Added: immediately return ack=1 upon receiving request
        
        # 检查是否为原始Modbus命令 (非标准协议)
        if request.raw_message and len(request.raw_message) > 0:
            self.get_logger().info(f"[SEQ {seq_id}] [{now_str}] Detected raw_message, using custom handler")
            log_entry['raw_message'] = list(request.raw_message)
            with self.lock:
                success, response_data, response_time_ms = self.handle_raw_modbus_command(
                    request.slave_id, 
                    request.raw_message, 
                    seq_id, 
                    now_str
                )
                response.success = success
                response.response = response_data
                log_entry['success'] = success
                log_entry['response'] = response_data
                log_entry['response_time_ms'] = response_time_ms
                # 保存完整命令(包含设备ID和CRC)到 log_entry
                if success and len(response_data) > 0:
                    log_entry['full_command'] = list(response_data)
                if self.dashboard:
                    self.dashboard.add_log(log_entry)
                return response
        
        # 原有的标准Modbus处理逻辑
        with self.lock:
            # 记录开始时间
            start_time = time.time()
            try:
                fc = request.function_code
                addr = request.address
                slave = request.slave_id
                values = request.values

                if fc == 1:
                    # Read coils
                    result = self.client.read_coils(address=addr, count=request.count, device_id=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = [int(b) for b in result.bits]
                    self.get_logger().debug(f"[SEQ {seq_id}] [{now_str}] Read coils: address={addr}, count={request.count}, slave={slave}, success={response.success}, response={response.response}")

                elif fc == 3:
                    # Read holding registers
                    result = self.client.read_holding_registers(address=addr, count=request.count, device_id=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = list(result.registers)
                    self.get_logger().debug(f"[SEQ {seq_id}] [{now_str}] Read holding registers: address={addr}, count={request.count}, slave={slave}, success={response.success}, response={response.response}")

                elif fc == 5:
                    # Write single coil (ON=0xFF00, OFF=0x0000)
                    result = self.client.write_coil(address=addr, value=bool(values[0]), device_id=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = [int(values[0])]
                    self.get_logger().debug(f"[SEQ {seq_id}] [{now_str}] Write single coil: address={addr}, value={values[0]}, slave={slave}, success={response.success}, response={response.response}")

                elif fc == 6:
                    # Write single register
                    result = self.client.write_register(address=addr, value=values[0], device_id=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = [values[0]]
                    self.get_logger().debug(f"[SEQ {seq_id}] [{now_str}] Write single register: address={addr}, value={values[0]}, slave={slave}, success={response.success}, response={response.response}")

                elif fc == 15:
                    # Write multiple coils
                    coil_values = [bool(v) for v in values]
                    result = self.client.write_coils(address=addr, values=coil_values, device_id=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = [int(v) for v in coil_values]
                    self.get_logger().debug(f"[SEQ {seq_id}] [{now_str}] Write multiple coils: address={addr}, values={coil_values}, slave={slave}, success={response.success}, response={response.response}")

                elif fc == 16:
                    # Write multiple registers
                    result = self.client.write_registers(address=addr, values=values, device_id=slave)
                    if result.isError():
                        raise Exception(str(result))
                    response.success = True
                    response.response = values
                    self.get_logger().debug(f"[SEQ {seq_id}] [{now_str}] Write multiple registers: address={addr}, values={values}, slave={slave}, success={response.success}, response={response.response}")

                else:
                    raise ValueError(f"Unsupported function code: {fc}")

            except Exception as e:
                self.get_logger().error(f"[SEQ {seq_id}] [{now_str}] Modbus error: {e}")
                response.success = False
                response.response = []
                self.get_logger().error(f"[SEQ {seq_id}] [{now_str}] Modbus error response: success={response.success}, response={response.response}")
            
            # 计算响应时间
            response_time_ms = (time.time() - start_time) * 1000

        # Update log entry
        log_entry['success'] = response.success
        log_entry['response'] = list(response.response) # Ensure it's a list for JSON serialization
        log_entry['response_time_ms'] = response_time_ms
        if self.dashboard:
            self.dashboard.add_log(log_entry)

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
