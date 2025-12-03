#!/usr/bin/env python3
"""
Force Sensor Configuration Reader
Reads all configuration registers from force sensor via Modbus and displays parsed values.

Usage:
    python3 force_sensor_read_config.py <device_id>
    
Examples:
    python3 force_sensor_read_config.py 52    # Read from right sensor
    python3 force_sensor_read_config.py 53    # Read from left sensor
"""

import sys
import struct
import serial
import time
from pathlib import Path


def crc16_modbus(data):
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


def find_serial_port():
    """Auto-detect serial port for Modbus (RS485 converter)"""
    import glob
    
    # Common patterns for USB-RS485 converters
    patterns = [
        '/dev/ttyUSB*',
        '/dev/ttyACM*',
        '/dev/serial/by-id/*USB*',
    ]
    
    for pattern in patterns:
        ports = glob.glob(pattern)
        if ports:
            return sorted(ports)[0]
    
    return None


def read_force_sensor_config(device_id, port=None, baudrate=115200, num_registers=30):
    """
    Read configuration registers from force sensor
    
    Args:
        device_id: Modbus device ID (52 or 53)
        port: Serial port path (auto-detect if None)
        baudrate: Serial baudrate (default: 115200)
        num_registers: Number of registers to read (default: 30, reads 40001-40030)
    
    Returns:
        dict: Parsed configuration parameters, or None if failed
    """
    
    # Auto-detect port if not specified
    if port is None:
        port = find_serial_port()
        if port is None:
            print("❌ Error: No serial port found. Please specify manually.")
            return None
        print(f"📡 Auto-detected serial port: {port}")
    
    try:
        # Open serial port
        ser = serial.Serial(
            port=port,
            baudrate=baudrate,
            bytesize=serial.EIGHTBITS,
            parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE,
            timeout=1.0
        )
        
        # Modbus Read Holding Registers (Function Code 0x03)
        # Starting address: 0x0000 (register 40001)
        # Number of registers: num_registers
        start_addr = 0x0000
        
        # Build Modbus request frame
        request = bytearray([
            device_id,              # Device address
            0x03,                   # Function code (Read Holding Registers)
            (start_addr >> 8) & 0xFF,  # Start address high byte
            start_addr & 0xFF,         # Start address low byte
            (num_registers >> 8) & 0xFF,  # Number of registers high byte
            num_registers & 0xFF          # Number of registers low byte
        ])
        
        # Calculate and append CRC
        crc = crc16_modbus(request)
        request.append(crc & 0xFF)         # CRC low byte
        request.append((crc >> 8) & 0xFF)  # CRC high byte
        
        print(f"📤 Sending request: {' '.join(f'{b:02X}' for b in request)}")
        
        # Clear input buffer and send request
        ser.reset_input_buffer()
        ser.write(request)
        time.sleep(0.1)  # Wait for response
        
        # Read response
        # Expected: device_id + 0x03 + byte_count + (num_registers*2) bytes + CRC(2)
        expected_bytes = 3 + (num_registers * 2) + 2
        response = ser.read(expected_bytes)
        
        if len(response) < 5:
            print(f"❌ Error: Response too short ({len(response)} bytes)")
            return None
        
        print(f"📥 Received response ({len(response)} bytes): {' '.join(f'{b:02X}' for b in response)}")
        
        # Verify response
        if response[0] != device_id:
            print(f"❌ Error: Wrong device ID in response (expected {device_id}, got {response[0]})")
            return None
        
        if response[1] != 0x03:
            if response[1] == 0x83:  # Error response
                error_code = response[2]
                error_msg = {
                    0x01: "Illegal function",
                    0x02: "Illegal data address",
                    0x03: "Illegal data value",
                    0x04: "Slave device failure"
                }.get(error_code, f"Unknown error {error_code}")
                print(f"❌ Modbus error: {error_msg}")
            else:
                print(f"❌ Error: Wrong function code (expected 0x03, got 0x{response[1]:02X})")
            return None
        
        byte_count = response[2]
        expected_byte_count = num_registers * 2
        
        if byte_count != expected_byte_count:
            print(f"⚠️  Warning: Byte count mismatch (expected {expected_byte_count}, got {byte_count})")
        
        # Verify CRC
        response_data = response[:-2]
        response_crc = response[-2] | (response[-1] << 8)
        calculated_crc = crc16_modbus(response_data)
        
        if response_crc != calculated_crc:
            print(f"⚠️  Warning: CRC mismatch (received 0x{response_crc:04X}, calculated 0x{calculated_crc:04X})")
        
        # Parse register values
        registers = []
        data_start = 3
        for i in range(min(byte_count // 2, num_registers)):
            offset = data_start + (i * 2)
            if offset + 1 < len(response):
                reg_value = (response[offset] << 8) | response[offset + 1]
                registers.append(reg_value)
        
        # Parse configuration
        config = parse_force_sensor_config(registers)
        config['device_id'] = device_id
        config['raw_registers'] = registers
        
        ser.close()
        return config
        
    except serial.SerialException as e:
        print(f"❌ Serial error: {e}")
        return None
    except Exception as e:
        print(f"❌ Unexpected error: {e}")
        import traceback
        traceback.print_exc()
        return None


def parse_force_sensor_config(registers):
    """
    Parse force sensor configuration registers
    
    Complete register mapping (based on official documentation):
        40001 (0): Measurement display value (unipolar: 0~65535, bipolar: -32768~32767)
        40002 (1): Decimal point (0=none, 1=0.0, 2=0.00, 3=0.000)
        40003 (2): Unit selection (1=MPa, 2=Kg, 3=T, 4=g, 5=N, 6=KN)
        40004 (3): Mask value (outputs 0 if below this value, range: 0-50)
        40005 (4): Sampling frequency (1:600pcs, 2:300pcs, 3:150pcs, 4:75pcs, 5:37.5pcs, 6:18.75pcs, 7:10pcs)
        40006 (5): RC filter time constant (1:none, 2:0.8, 3:0.6, 4:0.4, 5:0.2, 6:0.1)
        40007 (6): RS485 device address (1-250)
        40008 (7): Baud rate (0:1200, 1:2400, 2:4800, 3:9600, 4:19200, 5:38400, 6:115200)
        40009 (8): Parity setting (0:N.8.1, 1:N.8.2, 2:O.8.1, 3:E.8.1)
        40010 (9): Analog output low range
        40011 (10): Analog output high range
        40012 (11): Calibration point 1 expected value
        40013 (12): Calibration point 2 expected value
        40014 (13): Calibration point 3 expected value
        40015 (14): Calibration point 4 expected value
        40016 (15): Calibration point 5 expected value
        40017 (16): Number of calibration points (2-5)
        40018 (17): Tare operation (1=tare, 2=clear tare)
        40019 (18): Analog output low correction value (0~6000)
        40020 (19): Analog output high correction value (0~6000)
        40021 (20): Zero tracking range (0-100)
        40022 (21): Zero tracking time (1:5s, 2:10s, 3:15s, 4:20s)
        40023 (22): Clear zero (send 0x0011 to clear)
        40024 (23): Input polarity setting (1=unipolar, 2=bipolar)
        40025 (24): Output signal type (0:none, 1:0-5V/0-10V, 2:±5V, 3:4-20mA, 4:12±8mA)
        40033 (32): Analog output calibration value selection (1=low, 2=high)
        40034 (33): Analog output calibration confirm (send 0xa0a0 to confirm)
    
    Args:
        registers: List of 16-bit register values
    
    Returns:
        dict: Parsed configuration
    """
    config = {}
    
    # Helper to safely get register value
    def get_reg(index, default=0):
        return registers[index] if index < len(registers) else default
    
    # Helper to convert to signed 16-bit
    def to_signed(val):
        return val if val < 32768 else val - 65536
    
    # 40001: Measurement display value (current force reading)
    display_value = get_reg(0)
    display_value_signed = to_signed(display_value)
    config['current_measurement'] = display_value
    config['current_measurement_signed'] = display_value_signed
    
    # 40002: Decimal point
    decimal_point = get_reg(1)
    config['decimal_point'] = decimal_point
    config['decimal_point_desc'] = {
        0: 'None (integer)',
        1: 'One decimal (0.0)',
        2: 'Two decimals (0.00)',
        3: 'Three decimals (0.000)'
    }.get(decimal_point, f'Unknown ({decimal_point})')
    
    # 40003: Unit selection
    unit_code = get_reg(2)
    config['unit_code'] = unit_code
    config['unit'] = {
        1: 'MPa (Megapascal)',
        2: 'Kg (Kilogram)',
        3: 'T (Ton)',
        4: 'g (Gram)',
        5: 'N (Newton)',
        6: 'KN (Kilonewton)'
    }.get(unit_code, f'Unknown unit ({unit_code})')
    
    # 40004: Mask value (shield value)
    mask_value = get_reg(3)
    config['mask_value'] = mask_value
    config['mask_value_desc'] = f'Output 0 if reading < {mask_value}' if mask_value > 0 else 'Disabled'
    
    # 40005: Sampling frequency
    sampling_freq_code = get_reg(4)
    config['sampling_freq_code'] = sampling_freq_code
    config['sampling_frequency'] = {
        1: '600 samples/sec',
        2: '300 samples/sec',
        3: '150 samples/sec',
        4: '75 samples/sec',
        5: '37.5 samples/sec (default)',
        6: '18.75 samples/sec',
        7: '10 samples/sec'
    }.get(sampling_freq_code, f'Unknown ({sampling_freq_code})')
    
    # 40006: RC filter time constant
    rc_filter_code = get_reg(5)
    config['rc_filter_code'] = rc_filter_code
    config['rc_filter'] = {
        1: 'None (no filter)',
        2: '0.8s (default)',
        3: '0.6s',
        4: '0.4s',
        5: '0.2s',
        6: '0.1s'
    }.get(rc_filter_code, f'Unknown ({rc_filter_code})')
    
    # 40007: RS485 device address
    rs485_addr = get_reg(6)
    config['rs485_address'] = rs485_addr
    
    # 40008: Baud rate
    baudrate_code = get_reg(7)
    config['baudrate_code'] = baudrate_code
    config['baudrate'] = {
        0: '1200',
        1: '2400',
        2: '4800',
        3: '9600',
        4: '19200',
        5: '38400',
        6: '115200'
    }.get(baudrate_code, f'Unknown ({baudrate_code})')
    
    # 40009: Parity setting
    parity_code = get_reg(8)
    config['parity_code'] = parity_code
    config['parity'] = {
        0: 'N.8.1 (None, 8 data bits, 1 stop bit)',
        1: 'N.8.2 (None, 8 data bits, 2 stop bits)',
        2: 'O.8.1 (Odd, 8 data bits, 1 stop bit)',
        3: 'E.8.1 (Even, 8 data bits, 1 stop bit)'
    }.get(parity_code, f'Unknown ({parity_code})')
    
    # 40010: Analog output low range
    analog_low = get_reg(9)
    analog_low_signed = to_signed(analog_low)
    config['analog_output_low'] = analog_low
    config['analog_output_low_signed'] = analog_low_signed
    
    # 40011: Analog output high range
    analog_high = get_reg(10)
    analog_high_signed = to_signed(analog_high)
    config['analog_output_high'] = analog_high
    config['analog_output_high_signed'] = analog_high_signed
    
    # 40012-40016: Calibration points
    config['calibration_points'] = []
    for i in range(5):
        cal_point = get_reg(11 + i)
        cal_point_signed = to_signed(cal_point)
        config['calibration_points'].append({
            'point_num': i + 1,
            'expected_value': cal_point,
            'expected_value_signed': cal_point_signed
        })
    
    # 40017: Number of calibration points
    num_cal_points = get_reg(16)
    config['num_calibration_points'] = num_cal_points
    
    # 40018: Tare operation
    tare_op = get_reg(17)
    config['tare_operation'] = tare_op
    config['tare_operation_desc'] = {
        1: 'Tare (zero current reading)',
        2: 'Clear tare'
    }.get(tare_op, 'No operation')
    
    # 40019: Analog output low correction value
    analog_low_corr = get_reg(18)
    config['analog_low_correction'] = analog_low_corr
    
    # 40020: Analog output high correction value
    analog_high_corr = get_reg(19)
    config['analog_high_correction'] = analog_high_corr
    
    # 40021: Zero tracking range
    zero_track_range = get_reg(20)
    config['zero_tracking_range'] = zero_track_range
    
    # 40022: Zero tracking time
    zero_track_time = get_reg(21)
    config['zero_tracking_time_code'] = zero_track_time
    config['zero_tracking_time'] = {
        1: '5 seconds',
        2: '10 seconds',
        3: '15 seconds',
        4: '20 seconds'
    }.get(zero_track_time, f'Unknown ({zero_track_time})')
    
    # 40023: Clear zero
    clear_zero = get_reg(22)
    config['clear_zero'] = clear_zero
    config['clear_zero_desc'] = 'Active (0x0011)' if clear_zero == 0x0011 else 'Inactive'
    
    # 40024: Input polarity setting
    polarity = get_reg(23)
    config['input_polarity'] = polarity
    config['polarity_mode'] = {
        1: 'Unipolar (0 to positive)',
        2: 'Bipolar (negative to positive)'
    }.get(polarity, f'Unknown ({polarity})')
    
    # 40025: Output signal type
    output_type = get_reg(24)
    config['output_signal_type'] = output_type
    config['output_signal_desc'] = {
        0: 'No analog output',
        1: '0-5V or 0-10V',
        2: '±5V',
        3: '4-20mA',
        4: '12±8mA'
    }.get(output_type, f'Unknown ({output_type})')
    
    # 40033: Analog output calibration value selection (if available)
    if len(registers) > 32:
        cal_select = get_reg(32)
        config['analog_cal_selection'] = cal_select
        config['analog_cal_selection_desc'] = {
            1: 'Low calibration',
            2: 'High calibration'
        }.get(cal_select, 'Not selected')
    
    # 40034: Analog output calibration confirm (if available)
    if len(registers) > 33:
        cal_confirm = get_reg(33)
        config['analog_cal_confirm'] = cal_confirm
        config['analog_cal_confirm_desc'] = 'Confirmed (0xa0a0)' if cal_confirm == 0xa0a0 else 'Not confirmed'
    
    # Determine display range based on polarity and analog output settings
    if polarity == 1:  # Unipolar
        config['display_range'] = f"{analog_low} to {analog_high}"
        config['data_format'] = f"Unsigned 16-bit (0 to 65535)"
    elif polarity == 2:  # Bipolar
        config['display_range'] = f"{analog_low_signed} to {analog_high_signed}"
        config['data_format'] = f"Signed 16-bit (-32768 to +32767)"
    else:
        config['display_range'] = f"{analog_low} to {analog_high} (polarity unknown)"
        config['data_format'] = "Unknown"
    
    return config


def parse_status_code(status):
    """Parse device status/fault code"""
    if status == 0:
        return "Normal operation"
    
    # Common status bits (may vary by manufacturer)
    status_bits = []
    if status & 0x0001:
        status_bits.append("Overload")
    if status & 0x0002:
        status_bits.append("Underload")
    if status & 0x0004:
        status_bits.append("Sensor error")
    if status & 0x0008:
        status_bits.append("Calibration error")
    if status & 0x0010:
        status_bits.append("Communication error")
    
    return ', '.join(status_bits) if status_bits else f"Unknown status (0x{status:04X})"


def print_config(config):
    """Pretty print configuration"""
    if config is None:
        return
    
    print("\n" + "="*70)
    print(f"🔧 Force Sensor Configuration (Device ID: {config['device_id']})")
    print("="*70)
    
    print(f"\n📊 Current Measurement:")
    print(f"  Raw Value:        {config['current_measurement']} (unsigned)")
    print(f"  Signed Value:     {config['current_measurement_signed']}")
    print(f"  Display Format:   {config['decimal_point_desc']}")
    print(f"  Unit:             {config['unit']}")
    
    print(f"\n⚙️  Sensor Settings:")
    print(f"  Polarity Mode:    {config['polarity_mode']} (code: {config['input_polarity']})")
    print(f"  Display Range:    {config['display_range']}")
    print(f"  Data Format:      {config['data_format']}")
    print(f"  Mask Value:       {config['mask_value_desc']}")
    
    print(f"\n📈 Analog Output Range:")
    print(f"  Low Value:        {config['analog_output_low']} (signed: {config['analog_output_low_signed']})")
    print(f"  High Value:       {config['analog_output_high']} (signed: {config['analog_output_high_signed']})")
    print(f"  Low Correction:   {config['analog_low_correction']}")
    print(f"  High Correction:  {config['analog_high_correction']}")
    print(f"  Output Type:      {config['output_signal_desc']}")
    
    print(f"\n🔬 Sampling & Filtering:")
    print(f"  Sampling Freq:    {config['sampling_frequency']}")
    print(f"  RC Filter:        {config['rc_filter']}")
    
    print(f"\n🔧 Calibration:")
    print(f"  Num Cal Points:   {config['num_calibration_points']}")
    for point in config['calibration_points'][:config['num_calibration_points']]:
        print(f"    Point {point['point_num']}:        {point['expected_value']} (signed: {point['expected_value_signed']})")
    
    print(f"\n⚖️  Zero Tracking & Tare:")
    print(f"  Tare Operation:   {config['tare_operation_desc']}")
    print(f"  Zero Track Range: {config['zero_tracking_range']}")
    print(f"  Zero Track Time:  {config['zero_tracking_time']}")
    print(f"  Clear Zero:       {config['clear_zero_desc']}")
    
    print(f"\n📡 Communication Settings:")
    print(f"  RS485 Address:    {config['rs485_address']}")
    print(f"  Baud Rate:        {config['baudrate']} bps")
    print(f"  Parity:           {config['parity']}")
    
    if 'analog_cal_selection' in config:
        print(f"\n🎯 Calibration Status:")
        print(f"  Cal Selection:    {config['analog_cal_selection_desc']}")
        if 'analog_cal_confirm' in config:
            print(f"  Cal Confirmed:    {config['analog_cal_confirm_desc']}")
    
    print(f"\n🔢 Raw Register Dump:")
    regs = config['raw_registers']
    for i in range(0, len(regs), 8):
        chunk = regs[i:i+8]
        reg_nums = ' '.join(f'{40001+i+j:05d}' for j in range(len(chunk)))
        values = ' '.join(f'{v:5d}' for v in chunk)
        hex_vals = ' '.join(f'{v:04X}' for v in chunk)
        print(f"  Regs {reg_nums}")
        print(f"       {values}")
        print(f"       {hex_vals}")
    
    print("="*70 + "\n")


def main():
    """Main entry point"""
    if len(sys.argv) < 2:
        print("Usage: python3 force_sensor_read_config.py <device_id> [serial_port]")
        print("\nExamples:")
        print("  python3 force_sensor_read_config.py 52")
        print("  python3 force_sensor_read_config.py 53")
        print("  python3 force_sensor_read_config.py 52 /dev/ttyUSB0")
        sys.exit(1)
    
    try:
        device_id = int(sys.argv[1])
        if device_id not in [52, 53]:
            print(f"⚠️  Warning: Device ID {device_id} is unusual (expected 52 or 53)")
    except ValueError:
        print(f"❌ Error: Invalid device ID '{sys.argv[1]}' (must be integer)")
        sys.exit(1)
    
    port = sys.argv[2] if len(sys.argv) > 2 else None
    
    print(f"🔍 Reading configuration from force sensor (Device ID: {device_id})...")
    config = read_force_sensor_config(device_id, port=port)
    
    if config:
        print_config(config)
    else:
        print("\n❌ Failed to read configuration")
        sys.exit(1)


if __name__ == "__main__":
    main()
