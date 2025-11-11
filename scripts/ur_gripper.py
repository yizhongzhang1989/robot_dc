#!/usr/bin/env python3
"""
Return Push Tool Script
This script returns the push tool to its storage location (reverse of Get_push_tool).
"""

import sys
import os

# Add the ur15_robot_arm module to path
current_dir = os.path.dirname(os.path.abspath(__file__))
repo_root = os.path.abspath(os.path.join(current_dir, '..'))
ur15_path = os.path.join(repo_root, 'colcon_ws/src/ur15_robot_arm/ur15_robot_arm')
sys.path.append(ur15_path)

from ur15 import UR15Robot
import socket
import time
from typing import List, Optional, Union


class RS485Client:
    """
    RS485/Modbus communication client over TCP/IP socket.
    Provides easy-to-use methods for sending commands and receiving responses.
    """
    
    def __init__(self, host: str, port: int, timeout: float = 1.0):
        """
        Initialize RS485 client.
        
        Args:
            host: IP address of the RS485 gateway
            port: Port number for RS485 communication
            timeout: Socket timeout in seconds (default: 1.0)
        """
        self.host = host
        self.port = port
        self.timeout = timeout
        self.socket: Optional[socket.socket] = None
        self._connected = False
    
    def connect(self) -> bool:
        """
        Establish connection to RS485 gateway.
        
        Returns:
            True if connection successful, False otherwise
        """
        try:
            self.socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
            self.socket.settimeout(self.timeout)
            self.socket.connect((self.host, self.port))
            self._connected = True
            print(f"Connected to RS485 gateway at {self.host}:{self.port}")
            return True
        except Exception as e:
            print(f"Failed to connect: {e}")
            self._connected = False
            return False
    
    def disconnect(self):
        """Close the socket connection."""
        if self.socket:
            self.socket.close()
            self._connected = False
            print("Disconnected from RS485 gateway")
    
    def is_connected(self) -> bool:
        """Check if client is connected."""
        return self._connected
    
    def send_command(self, command: Union[List[int], bytes], 
                     response_delay: float = 0.1) -> Optional[bytes]:
        """
        Send a command and receive response.
        
        Args:
            command: Command bytes as list of integers or bytes object
            response_delay: Delay before reading response in seconds (default: 0.1)
        
        Returns:
            Response bytes, or None if failed
        """
        if not self._connected:
            print("Error: Not connected to RS485 gateway")
            return None
        
        try:
            # Convert command to bytes if needed
            if isinstance(command, list):
                command_bytes = bytes(command)
            else:
                command_bytes = command
            
            # Send command
            self.socket.sendall(command_bytes)
            print(f"Sent command: {command_bytes.hex()}")
            
            # Wait for response
            time.sleep(response_delay)
            
            # Receive response
            response = self.socket.recv(1024)
            print(f"Received response: {response.hex()}")
            
            return response
            
        except socket.timeout:
            print("Error: Timeout waiting for response")
            return None
        except Exception as e:
            print(f"Error during communication: {e}")
            return None
    
    def send_command_no_response(self, command: Union[List[int], bytes]):
        """
        Send a command without waiting for response.
        
        Args:
            command: Command bytes as list of integers or bytes object
        """
        if not self._connected:
            print("Error: Not connected to RS485 gateway")
            return
        
        try:
            # Convert command to bytes if needed
            if isinstance(command, list):
                command_bytes = bytes(command)
            else:
                command_bytes = command
            
            # Send command
            self.socket.sendall(command_bytes)
            print(f"Sent command: {command_bytes.hex()}")
            
        except Exception as e:
            print(f"Error sending command: {e}")
    
    @staticmethod
    def _calculate_crc16(data: bytes) -> int:
        """
        Calculate Modbus CRC16.
        
        Args:
            data: Data bytes to calculate CRC for
        
        Returns:
            CRC16 value as integer
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
    
    def send_modbus_request(self, device_id: int, function_code: int, 
                           address: int, values: Union[int, List[int], None] = None,
                           count: int = 1, response_delay: float = 0.1) -> Optional[bytes]:
        """
        Send a Modbus RTU request with automatic CRC calculation.
        
        Args:
            device_id: Modbus device/slave ID (1-247)
            function_code: Modbus function code
                          1 = Read Coils
                          3 = Read Holding Registers
                          5 = Write Single Coil
                          6 = Write Single Register
                          15 = Write Multiple Coils
                          16 = Write Multiple Registers
            address: Starting address (0-65535)
            values: Value(s) to write (for write operations) or count for read operations
                   - For FC 5/6: single integer value
                   - For FC 15/16: list of integer values
                   - For FC 1/3: number of items to read (uses count parameter)
            count: Number of registers/coils to read (used for read operations)
            response_delay: Delay before reading response in seconds
        
        Returns:
            Response bytes, or None if failed
        """
        if not self._connected:
            print("Error: Not connected to RS485 gateway")
            return None
        
        try:
            # Build Modbus RTU frame
            frame = bytearray()
            frame.append(device_id)
            frame.append(function_code)
            
            # Add address (2 bytes, big-endian)
            frame.extend(address.to_bytes(2, byteorder='big'))
            
            # Add data based on function code
            if function_code in [1, 3, 4]:  # Read operations
                # Add count (2 bytes, big-endian)
                if values is not None:
                    count = values if isinstance(values, int) else count
                frame.extend(count.to_bytes(2, byteorder='big'))
                
            elif function_code in [5, 6]:  # Write single
                if values is None:
                    raise ValueError("values parameter required for write operations")
                value = values if isinstance(values, int) else values[0]
                # For FC5, convert boolean to 0xFF00 or 0x0000
                if function_code == 5:
                    value = 0xFF00 if value else 0x0000
                frame.extend(value.to_bytes(2, byteorder='big'))
                
            elif function_code in [15, 16]:  # Write multiple
                if values is None or not isinstance(values, list):
                    raise ValueError("values must be a list for write multiple operations")
                
                # Add count
                value_count = len(values)
                frame.extend(value_count.to_bytes(2, byteorder='big'))
                
                if function_code == 15:  # Write multiple coils
                    # Calculate byte count
                    byte_count = (value_count + 7) // 8
                    frame.append(byte_count)
                    # Pack coils into bytes
                    coil_bytes = bytearray((value_count + 7) // 8)
                    for i, val in enumerate(values):
                        if val:
                            coil_bytes[i // 8] |= (1 << (i % 8))
                    frame.extend(coil_bytes)
                    
                else:  # Write multiple registers (FC16)
                    byte_count = value_count * 2
                    frame.append(byte_count)
                    for val in values:
                        frame.extend(val.to_bytes(2, byteorder='big'))
            else:
                raise ValueError(f"Unsupported function code: {function_code}")
            
            # Calculate and append CRC16 (low byte first, then high byte)
            crc = self._calculate_crc16(bytes(frame))
            frame.append(crc & 0xFF)  # CRC low byte
            frame.append((crc >> 8) & 0xFF)  # CRC high byte
            
            # Send the frame
            print(f"Sending Modbus request: Device={device_id}, FC={function_code}, "
                  f"Address={address}, Frame={frame.hex()}")
            
            return self.send_command(bytes(frame), response_delay=response_delay)
            
        except Exception as e:
            print(f"Error building Modbus request: {e}")
            return None
    
    @staticmethod
    def parse_response(response: bytes, format: str = 'hex') -> Union[str, List[str], dict]:
        """
        Parse response bytes into readable format.
        
        Args:
            response: Response bytes
            format: Output format - 'hex', 'list', or 'dict'
        
        Returns:
            Formatted response data
        """
        if format == 'hex':
            return response.hex()
        elif format == 'list':
            return [hex(b) for b in response]
        elif format == 'dict':
            return {
                'hex_string': response.hex(),
                'byte_list': [hex(b) for b in response],
                'raw_bytes': list(response),
                'length': len(response)
            }
        else:
            return response.hex()
    
    @staticmethod
    def parse_modbus_response(response: bytes, function_code: int) -> dict:
        """
        Parse Modbus response and extract data.
        
        Args:
            response: Raw response bytes
            function_code: Modbus function code used in request
        
        Returns:
            Dictionary with parsed response data
        """
        if not response or len(response) < 5:
            return {'error': 'Invalid response length'}
        
        result = {
            'device_id': response[0],
            'function_code': response[1],
            'raw_hex': response.hex()
        }
        
        # Check if error response (function code has bit 7 set)
        if response[1] & 0x80:
            result['error'] = True
            result['exception_code'] = response[2] if len(response) > 2 else None
            return result
        
        result['error'] = False
        
        # Parse based on function code
        if function_code in [1, 3, 4]:  # Read responses
            byte_count = response[2]
            data_bytes = response[3:3+byte_count]
            
            if function_code == 1:  # Coils (bits)
                result['byte_count'] = byte_count
                result['coils'] = []
                for byte in data_bytes:
                    for bit in range(8):
                        result['coils'].append(bool(byte & (1 << bit)))
            else:  # Registers (16-bit values)
                result['byte_count'] = byte_count
                result['registers'] = []
                for i in range(0, len(data_bytes), 2):
                    if i + 1 < len(data_bytes):
                        value = (data_bytes[i] << 8) | data_bytes[i+1]
                        result['registers'].append(value)
        
        elif function_code in [5, 6, 15, 16]:  # Write responses
            result['address'] = (response[2] << 8) | response[3]
            result['value'] = (response[4] << 8) | response[5]
        
        return result
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()



def main():
    # Robot connection
    robot = UR15Robot("192.168.1.15", 30002)
    
    if robot.open() != 0:
        print("Failed to connect to robot")
        return
    
    # RS485 connection using the new RS485Client class
    with RS485Client("192.168.1.15", 54321) as rs485:
        
        # ==== Example using send_modbus_request() ====
        print("\n=== Using send_modbus_request() ===")


        # activate
        response = rs485.send_modbus_request(
            device_id=9, 
            function_code=16,  # Write Multiple Registers
            address=0x03E8,   # Starting address
            values=[0x0000, 0x0000, 0x0000]
        )
        
        if response:
            parsed = RS485Client.parse_modbus_response(response, function_code=3)
            print(f"Read registers response: {parsed}")

        time.sleep(3)      

        # check status
        response = rs485.send_modbus_request(
            device_id=9, 
            function_code=4,  # Read Holding Registers
            address=0x07D0,   # Starting address
            values=2          # Number of registers to read
        )
        
        if response:
            parsed = RS485Client.parse_modbus_response(response, function_code=3)
            print(f"Read registers response: {parsed}")

        time.sleep(1)      

        # close gripper
        response = rs485.send_modbus_request(
            device_id=9, 
            function_code=16,  # Write Multiple Registers
            address=0x03E8,   # Starting address
            values=[0x0900, 0x00FF, 0xFFFF]
        )
        
        if response:
            parsed = RS485Client.parse_modbus_response(response, function_code=3)
            print(f"Read registers response: {parsed}")

        time.sleep(3)      


        # open gripper
        response = rs485.send_modbus_request(
            device_id=9, 
            function_code=16,  # Write Multiple Registers
            address=0x03E8,   # Starting address
            values=[0x0900, 0x0000, 0xFFFF]
        )
        
        if response:
            parsed = RS485Client.parse_modbus_response(response, function_code=3)
            print(f"Read registers response: {parsed}")

        time.sleep(3)      


        # response = rs485.send_modbus_request(
        #     device_id=1, 
        #     function_code=3,  # Read Holding Registers
        #     address=0x1010,   # Starting address
        #     values=2          # Number of registers to read
        # )
        
        # if response:
        #     parsed = RS485Client.parse_modbus_response(response, function_code=3)
        #     print(f"Read registers response: {parsed}")


        # response = rs485.send_modbus_request(
        #     device_id=1, 
        #     function_code=0x10,  # write register
        #     address=0x1004,   # Starting address
        #     values=[0, 0]
        # )
        
        # if response:
        #     parsed = RS485Client.parse_modbus_response(response, function_code=3)
        #     print(f"Read registers response: {parsed}")

        # time.sleep(1)


        # Example 2: Write single register (FC06)
        # response = rs485.send_modbus_request(
        #     device_id=1,
        #     function_code=6,  # Write Single Register
        #     address=0x1000,
        #     values=1234       # Value to write
        # )
        # if response:
        #     parsed = RS485Client.parse_modbus_response(response, function_code=6)
        #     print(f"Write register response: {parsed}")
        
        # Example 3: Write multiple registers (FC16)
        # response = rs485.send_modbus_request(
        #     device_id=1,
        #     function_code=16,  # Write Multiple Registers
        #     address=0x1000,
        #     values=[100, 200, 300]  # List of values
        # )
        # if response:
        #     parsed = RS485Client.parse_modbus_response(response, function_code=16)
        #     print(f"Write multiple registers response: {parsed}")
        
    
    # Close robot connection
    robot.close()


def main_RM_EGB():
    # Robot connection
    robot = UR15Robot("192.168.1.15", 30002)
    
    if robot.open() != 0:
        print("Failed to connect to robot")
        return
    
    # RS485 connection using the new RS485Client class
    with RS485Client("192.168.1.15", 54321) as rs485:
        
        # ==== Example using send_modbus_request() ====
        print("\n=== Using send_modbus_request() ===")
        
        # # Example 1: Read holding registers (FC03)
        # # Read 2 registers starting from address 0x1008 (4104 decimal)
        # response = rs485.send_modbus_request(
        #     device_id=1, 
        #     function_code=3,  # Read Holding Registers
        #     address=0x1008,   # Starting address
        #     values=2          # Number of registers to read
        # )
        
        # if response:
        #     parsed = RS485Client.parse_modbus_response(response, function_code=3)
        #     print(f"Read registers response: {parsed}")
        

        # response = rs485.send_modbus_request(
        #     device_id=1, 
        #     function_code=3,  # Read Holding Registers
        #     address=0x1010,   # Starting address
        #     values=2          # Number of registers to read
        # )
        
        # if response:
        #     parsed = RS485Client.parse_modbus_response(response, function_code=3)
        #     print(f"Read registers response: {parsed}")


        for i in range(11):    
            response = rs485.send_modbus_request(
                device_id=1, 
                function_code=0x10,  # write register
                address=0x1004,   # Starting address
                values=[i*10, 0]
            )
            
            if response:
                parsed = RS485Client.parse_modbus_response(response, function_code=3)
                print(f"Read registers response: {parsed}")

            time.sleep(1)


        response = rs485.send_modbus_request(
            device_id=1, 
            function_code=0x10,  # write register
            address=0x1004,   # Starting address
            values=[0, 0]
        )
        
        if response:
            parsed = RS485Client.parse_modbus_response(response, function_code=3)
            print(f"Read registers response: {parsed}")

        time.sleep(1)


        # Example 2: Write single register (FC06)
        # response = rs485.send_modbus_request(
        #     device_id=1,
        #     function_code=6,  # Write Single Register
        #     address=0x1000,
        #     values=1234       # Value to write
        # )
        # if response:
        #     parsed = RS485Client.parse_modbus_response(response, function_code=6)
        #     print(f"Write register response: {parsed}")
        
        # Example 3: Write multiple registers (FC16)
        # response = rs485.send_modbus_request(
        #     device_id=1,
        #     function_code=16,  # Write Multiple Registers
        #     address=0x1000,
        #     values=[100, 200, 300]  # List of values
        # )
        # if response:
        #     parsed = RS485Client.parse_modbus_response(response, function_code=16)
        #     print(f"Write multiple registers response: {parsed}")
        
        # ==== Old raw command method (for comparison) ====
        print("\n=== Using raw send_command() ===")
        query_status_command = [0x01, 0x03, 0x10, 0x08, 0x00, 0x02, 0x41, 0x09]
        response = rs485.send_command(query_status_command, response_delay=0.1)
        
        if response:
            print("\n--- Response Parsing ---")
            print(f"Hex string: {RS485Client.parse_response(response, 'hex')}")
            print(f"Modbus parsed: {RS485Client.parse_modbus_response(response, 3)}")
    
    # Close robot connection
    robot.close()


# Alternative usage without context manager
def main_alternative():
    """Example showing manual connection management."""
    robot = UR15Robot("192.168.1.15", 30002)
    
    if robot.open() != 0:
        print("Failed to connect to robot")
        return
    
    # Create RS485 client
    rs485 = RS485Client("192.168.1.15", 54321, timeout=2.0)
    
    # Connect manually
    if not rs485.connect():
        robot.close()
        return
    
    # Send commands
    query_status_command = [0x01, 0x03, 0x10, 0x08, 0x00, 0x02, 0x41, 0x09]
    response = rs485.send_command(query_status_command)
    
    if response:
        parsed = RS485Client.parse_response(response, format='dict')
        print(f"Response details: {parsed}")
    
    # Disconnect manually
    rs485.disconnect()
    robot.close()



def main_mock():
    """
    Mock/test function to validate RS485Client Modbus frame building
    without requiring actual robot or socket connections.
    """
    print("=== Mock Testing RS485Client ===\n")
    
    # Create a mock RS485 client (won't connect)
    class MockRS485Client(RS485Client):
        """Mock client that simulates frame building without network connection."""
        
        def __init__(self):
            super().__init__("mock_host", 12345)
            self._connected = True  # Pretend we're connected
            self.last_frame = None
        
        def send_command(self, command: Union[List[int], bytes], 
                        response_delay: float = 0.1) -> Optional[bytes]:
            """Override to capture frame without sending."""
            if isinstance(command, list):
                command_bytes = bytes(command)
            else:
                command_bytes = command
            
            self.last_frame = command_bytes
            print(f"Mock send: {command_bytes.hex()}")
            print(f"Frame breakdown: {[f'0x{b:02x}' for b in command_bytes]}")
            
            # Simulate responses based on function code
            fc = command_bytes[1] if len(command_bytes) > 1 else 0
            
            if fc == 3:  # Read holding registers
                # Mock response: device_id + fc + byte_count + data + crc
                device_id = command_bytes[0]
                # Simulate 2 registers: 0x1234, 0x5678
                mock_response = bytearray([device_id, fc, 0x04, 0x12, 0x34, 0x56, 0x78])
                crc = self._calculate_crc16(bytes(mock_response))
                mock_response.extend([crc & 0xFF, (crc >> 8) & 0xFF])
                print(f"Mock response: {mock_response.hex()}\n")
                return bytes(mock_response)
            
            elif fc == 6:  # Write single register
                # Echo back the request (typical Modbus response)
                print(f"Mock response: {command_bytes.hex()}\n")
                return command_bytes
            
            elif fc == 16:  # Write multiple registers
                # Response: device_id + fc + address + count + crc
                device_id = command_bytes[0]
                address_bytes = command_bytes[2:4]
                count_bytes = command_bytes[4:6]
                mock_response = bytearray([device_id, fc] + list(address_bytes) + list(count_bytes))
                crc = self._calculate_crc16(bytes(mock_response))
                mock_response.extend([crc & 0xFF, (crc >> 8) & 0xFF])
                print(f"Mock response: {mock_response.hex()}\n")
                return bytes(mock_response)
            
            else:
                print("Mock response: (no mock response for this FC)\n")
                return None
    
    # Create mock client
    mock_client = MockRS485Client()
    
    # Test 1: Read holding registers
    print("--- Test 1: Read Holding Registers (FC03) ---")
    print("Command: Read 2 registers starting at address 0x1008")
    response = mock_client.send_modbus_request(
        device_id=1,
        function_code=3,
        address=0x1008,
        values=2
    )
    if response:
        parsed = RS485Client.parse_modbus_response(response, function_code=3)
        print(f"Parsed response: {parsed}\n")
    
    # Verify CRC
    if mock_client.last_frame:
        frame_without_crc = mock_client.last_frame[:-2]
        expected_crc = mock_client._calculate_crc16(frame_without_crc)
        actual_crc = (mock_client.last_frame[-1] << 8) | mock_client.last_frame[-2]
        print(f"CRC verification: Expected=0x{expected_crc:04x}, Actual=0x{actual_crc:04x}, "
              f"Valid={expected_crc == actual_crc}\n")
    
    # Test 2: Write single register
    print("--- Test 2: Write Single Register (FC06) ---")
    print("Command: Write value 1234 to address 0x1000")
    response = mock_client.send_modbus_request(
        device_id=1,
        function_code=6,
        address=0x1000,
        values=1234
    )
    if response:
        parsed = RS485Client.parse_modbus_response(response, function_code=6)
        print(f"Parsed response: {parsed}\n")
    
    # Test 3: Write multiple registers
    print("--- Test 3: Write Multiple Registers (FC16) ---")
    print("Command: Write [100, 200, 300] to address 0x2000")
    response = mock_client.send_modbus_request(
        device_id=1,
        function_code=16,
        address=0x2000,
        values=[100, 200, 300]
    )
    if response:
        parsed = RS485Client.parse_modbus_response(response, function_code=16)
        print(f"Parsed response: {parsed}\n")
    
    # Test 4: Frame format validation
    print("--- Test 4: Manual Frame Comparison ---")
    print("Building frame for: Device=1, FC=3, Address=0x1008, Count=2")
    
    # Expected frame breakdown:
    # Device ID: 0x01
    # Function Code: 0x03
    # Address High: 0x10
    # Address Low: 0x08
    # Count High: 0x00
    # Count Low: 0x02
    # CRC Low: calculated
    # CRC High: calculated
    
    expected_frame_no_crc = bytes([0x01, 0x03, 0x10, 0x08, 0x00, 0x02])
    expected_crc = mock_client._calculate_crc16(expected_frame_no_crc)
    expected_frame = expected_frame_no_crc + bytes([expected_crc & 0xFF, (expected_crc >> 8) & 0xFF])
    
    print(f"Expected frame: {expected_frame.hex()}")
    print(f"Expected breakdown: {[f'0x{b:02x}' for b in expected_frame]}")
    
    # Compare with what was sent in Test 1
    response = mock_client.send_modbus_request(
        device_id=1,
        function_code=3,
        address=0x1008,
        values=2
    )
    
    if mock_client.last_frame == expected_frame:
        print("✅ Frame matches expected format!\n")
    else:
        print(f"❌ Frame mismatch!")
        print(f"   Expected: {expected_frame.hex()}")
        print(f"   Actual:   {mock_client.last_frame.hex()}\n")
    
    # Test 5: CRC calculation test
    print("--- Test 5: CRC16 Calculation Test ---")
    test_data = bytes([0x01, 0x03, 0x10, 0x08, 0x00, 0x02])
    calculated_crc = RS485Client._calculate_crc16(test_data)
    print(f"Test data: {test_data.hex()}")
    print(f"Calculated CRC16: 0x{calculated_crc:04x}")
    print(f"CRC bytes (low, high): 0x{calculated_crc & 0xFF:02x}, 0x{(calculated_crc >> 8) & 0xFF:02x}")
    
    # Known good test case (from Modbus spec)
    # Frame: 01 03 00 00 00 02 should have CRC: C4 0B (low byte first)
    # Which equals 0x0BC4 as a 16-bit value
    spec_test = bytes([0x01, 0x03, 0x00, 0x00, 0x00, 0x02])
    spec_crc = RS485Client._calculate_crc16(spec_test)
    print(f"\nSpec test data: {spec_test.hex()}")
    print(f"Calculated CRC16: 0x{spec_crc:04x}")
    print(f"CRC bytes (low, high): 0x{spec_crc & 0xFF:02x}, 0x{(spec_crc >> 8) & 0xFF:02x}")
    print("Expected CRC16: 0x0bc4 (bytes: 0xc4, 0x0b from Modbus spec)")
    if spec_crc == 0x0bc4:
        print("✅ CRC calculation is correct!\n")
    else:
        print(f"⚠️  CRC mismatch - expected 0x0bc4, got 0x{spec_crc:04x}\n")
    
    print("=== Mock Testing Complete ===")


if __name__ == "__main__":
    # Uncomment to run mock tests
    # main_mock()
    
    # Run normal main
    main()
