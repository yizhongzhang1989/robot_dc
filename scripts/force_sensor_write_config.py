#!/usr/bin/env python3
"""
Force Sensor Configuration Writer
Write configuration parameters to force sensor via Modbus RTU (RS485)
Uses Modbus Function Code 0x06 (Write Single Register)
"""

import serial
import time
import glob
import sys
from typing import Optional

def crc16_modbus(data: bytearray) -> int:
    """Calculate Modbus CRC16"""
    crc = 0xFFFF
    for byte in data:
        crc ^= byte
        for _ in range(8):
            if crc & 0x0001:
                crc = (crc >> 1) ^ 0xA001
            else:
                crc >>= 1
    return crc

def find_serial_port() -> Optional[str]:
    """Auto-detect USB serial port"""
    ports = glob.glob('/dev/ttyUSB*')
    return ports[0] if ports else None

def write_single_register(ser, device_id: int, reg_addr: int, value: int) -> bool:
    """
    Write a single register using Modbus Function Code 0x06
    
    Args:
        ser: Serial port object
        device_id: Modbus device ID (1-247)
        reg_addr: Register address (0-based, 0=40001)
        value: Value to write (0-65535)
    
    Returns:
        True if successful, False otherwise
    """
    # Build Modbus request: [device_id, func, addr_hi, addr_lo, val_hi, val_lo]
    request = bytearray([
        device_id,
        0x06,  # Function Code: Write Single Register
        (reg_addr >> 8) & 0xFF,
        reg_addr & 0xFF,
        (value >> 8) & 0xFF,
        value & 0xFF
    ])
    
    # Calculate and append CRC
    crc = crc16_modbus(request)
    request.append(crc & 0xFF)
    request.append((crc >> 8) & 0xFF)
    
    print(f"📤 Writing to register 40{reg_addr+1:03d}: value={value} (0x{value:04X})")
    print(f"   Request: {' '.join(f'{b:02X}' for b in request)}")
    
    # Send request
    ser.reset_input_buffer()
    ser.write(request)
    time.sleep(0.1)
    
    # Read response (should echo the request for successful write)
    response = ser.read(8)
    
    if len(response) < 8:
        print(f"❌ Error: Response too short ({len(response)} bytes)")
        return False
    
    print(f"📥 Response: {' '.join(f'{b:02X}' for b in response)}")
    
    # Verify response matches request (Modbus 0x06 echoes the request on success)
    if response[:6] != request[:6]:
        print(f"❌ Error: Response does not match request")
        return False
    
    # Verify CRC
    data_with_header = response[:6]
    received_crc = (response[7] << 8) | response[6]
    calculated_crc = crc16_modbus(data_with_header)
    
    if received_crc != calculated_crc:
        print(f"❌ Error: CRC mismatch (received {received_crc:04X}, calculated {calculated_crc:04X})")
        return False
    
    print(f"✅ Successfully wrote to register 40{reg_addr+1:03d}")
    return True

# Parameter mapping: name -> (register_address_0based, description, value_parser, safety_check)
PARAMETERS = {
    # Basic settings
    "decimal_point": (1, "Decimal point position (0=integer, 1=0.0, 2=0.00, 3=0.000)", 
                     lambda x: int(x), lambda v: 0 <= v <= 3),
    "unit": (2, "Unit selection (1=MPa, 2=Kg, 3=T, 4=g, 5=N, 6=KN)", 
            lambda x: int(x), lambda v: 1 <= v <= 6),
    "data_format": (4, "Data format (0=unsigned, 1=signed)", 
                   lambda x: int(x), lambda v: v in [0, 1]),
    
    # Analog output range
    "analog_low": (9, "Analog output low value (signed 16-bit)", 
                  lambda x: int(x) if int(x) >= 0 else int(x) + 65536, lambda v: True),
    "analog_high": (10, "Analog output high value (signed 16-bit)", 
                   lambda x: int(x) if int(x) >= 0 else int(x) + 65536, lambda v: True),
    "analog_low_corr": (11, "Analog output low correction value", 
                       lambda x: int(x, 0), lambda v: 0 <= v <= 65535),
    "analog_high_corr": (12, "Analog output high correction value", 
                        lambda x: int(x, 0), lambda v: 0 <= v <= 65535),
    
    # Mask and sampling
    "mask_value": (13, "Mask value (0=disabled, or mask threshold)", 
                  lambda x: int(x, 0), lambda v: True),
    "sampling_freq": (14, "Sampling frequency (0=10Hz, 1=20Hz, 2=50Hz, 3=100Hz, 4=200Hz, 5=500Hz, 6=1000Hz)", 
                     lambda x: int(x), lambda v: 0 <= v <= 6),
    "rc_filter": (15, "RC filter level (0=none, 1-5=level 1-5)", 
                 lambda x: int(x), lambda v: 0 <= v <= 5),
    
    # Zero tracking and tare
    "tare": (17, "Tare operation (0=no operation, 1=set tare, 2=clear tare)", 
            lambda x: int(x), lambda v: 0 <= v <= 2),
    "zero_track_range": (18, "Zero tracking range", 
                        lambda x: int(x, 0), lambda v: True),
    "zero_track_time": (19, "Zero tracking time (0=disabled, 1=0.5s, 2=1s, 3=2s, 4=5s, 5=10s)", 
                       lambda x: int(x), lambda v: 0 <= v <= 5),
    "clear_zero": (20, "Clear zero (0=inactive, 1=clear)", 
                  lambda x: int(x), lambda v: v in [0, 1]),
    
    # Polarity
    "polarity": (23, "Polarity mode (0=reserved, 1=unipolar, 2=bipolar)", 
                lambda x: int(x), lambda v: v in [0, 1, 2]),
    
    # Communication settings
    "rs485_addr": (30, "RS485 device address (1-247)", 
                  lambda x: int(x), lambda v: 1 <= v <= 247),
    "baudrate": (31, "Baud rate (0=1200, 1=2400, 2=4800, 3=9600, 4=19200, 5=38400, 6=115200)", 
                lambda x: int(x), lambda v: 0 <= v <= 6),
    "parity": (32, "Parity setting (0=N.8.1, 1=E.8.1, 2=O.8.1)", 
              lambda x: int(x), lambda v: 0 <= v <= 2),
}

def parse_value(value_str: str, parser) -> Optional[int]:
    """
    Parse value string using the provided parser function
    
    Args:
        value_str: String representation of value (can be decimal, hex, or negative)
        parser: Parser function to convert string to int
    
    Returns:
        Parsed integer value, or None if parsing failed
    """
    try:
        # Handle hex values (0x prefix or just hex digits)
        if value_str.startswith('0x') or value_str.startswith('0X'):
            return int(value_str, 16)
        
        # Handle negative numbers
        val = parser(value_str)
        
        # Convert negative to unsigned 16-bit
        if val < 0:
            val = val + 65536
        
        # Ensure within 16-bit range
        return val & 0xFFFF
        
    except Exception as e:
        print(f"❌ Error parsing value '{value_str}': {e}")
        return None

def write_parameter(device_id: int, param_name: str, value_str: str, 
                   port: str = None, baudrate: int = 115200) -> bool:
    """
    Write a configuration parameter to the force sensor
    
    Args:
        device_id: Modbus device ID (1-247)
        param_name: Parameter name (from PARAMETERS dict)
        value_str: Value to write (as string, can be decimal or hex)
        port: Serial port (auto-detect if None)
        baudrate: Serial baudrate (default: 115200)
    
    Returns:
        True if successful, False otherwise
    """
    # Check if parameter exists
    if param_name not in PARAMETERS:
        print(f"❌ Error: Unknown parameter '{param_name}'")
        print(f"\n📋 Available parameters:")
        for name, (addr, desc, _, _) in sorted(PARAMETERS.items()):
            print(f"  • {name:20s} - {desc}")
        return False
    
    reg_addr, description, parser, validator = PARAMETERS[param_name]
    
    # Parse value
    value = parse_value(value_str, parser)
    if value is None:
        return False
    
    # Validate value
    if not validator(value):
        print(f"❌ Error: Invalid value {value} for parameter '{param_name}'")
        print(f"   {description}")
        return False
    
    # Safety confirmation for critical parameters
    critical_params = ['polarity', 'tare', 'clear_zero', 'rs485_addr', 'baudrate', 'parity']
    if param_name in critical_params:
        print(f"\n⚠️  WARNING: You are about to modify a critical parameter!")
        print(f"   Parameter: {param_name}")
        print(f"   Description: {description}")
        print(f"   New value: {value} (0x{value:04X})")
        confirm = input("   Type 'yes' to confirm: ")
        if confirm.lower() != 'yes':
            print("❌ Operation cancelled")
            return False
    
    # Auto-detect port if not specified
    if port is None:
        port = find_serial_port()
        if port is None:
            print("❌ Error: No serial port found")
            return False
        print(f"📡 Auto-detected serial port: {port}")
    
    # Open serial port
    try:
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
        
        print(f"\n🔧 Writing parameter '{param_name}' to device {device_id}")
        print(f"   Register: 40{reg_addr+1:03d}")
        print(f"   Value: {value} (0x{value:04X})")
        
        # Write the register
        success = write_single_register(ser, device_id, reg_addr, value)
        
        ser.close()
        
        if success:
            print(f"\n✅ Parameter '{param_name}' updated successfully!")
            if param_name in ['rs485_addr', 'baudrate', 'parity']:
                print(f"⚠️  Note: Communication settings changed. You may need to reconnect with new settings.")
        
        return success
        
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        return False
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return False

def main():
    if len(sys.argv) < 4:
        print("=" * 70)
        print("Force Sensor Configuration Writer")
        print("=" * 70)
        print("\nUsage:")
        print("  python3 force_sensor_write_config.py <device_id> <parameter> <value> [port] [baudrate]")
        print("\nExamples:")
        print("  python3 force_sensor_write_config.py 52 decimal_point 1")
        print("  python3 force_sensor_write_config.py 52 polarity 2")
        print("  python3 force_sensor_write_config.py 52 sampling_freq 6")
        print("  python3 force_sensor_write_config.py 52 unit 0")
        print("  python3 force_sensor_write_config.py 52 mask_value 0x0100")
        print("  python3 force_sensor_write_config.py 52 polarity 2 /dev/ttyUSB0 115200")
        print("\n📋 Available parameters:")
        for name, (addr, desc, _, _) in sorted(PARAMETERS.items()):
            print(f"  • {name:20s} (40{addr+1:03d}) - {desc}")
        print("=" * 70)
        sys.exit(1)
    
    device_id = int(sys.argv[1])
    param_name = sys.argv[2]
    value_str = sys.argv[3]
    port = sys.argv[4] if len(sys.argv) > 4 else None
    baudrate = int(sys.argv[5]) if len(sys.argv) > 5 else 115200
    
    success = write_parameter(device_id, param_name, value_str, port, baudrate)
    sys.exit(0 if success else 1)

if __name__ == "__main__":
    main()
