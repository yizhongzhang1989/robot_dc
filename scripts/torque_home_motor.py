import time
import sys
import argparse
from pymodbus.client import ModbusSerialClient

# Torque homing related register addresses
TORQUE_MODE_ADDR = 0x600A
STALL_TIME_ADDR = 0x6013
OUTPUT_VAL_ADDR = 0x6014
TRIGGER_ADDR = 0x6002
HIGH_SPEED_ADDR = 0x600F
LOW_SPEED_ADDR = 0x6010
ACC_ADDR = 0x6011
DEC_ADDR = 0x6012

# Instruction values
TORQUE_MODE_REVERSE = 0x000C  # Reverse torque homing
TORQUE_MODE_FORWARD = 0x000D  # Forward torque homing
TRIGGER_TORQUE_HOME = 0x0020  # Trigger torque homing
TRIGGER_STOP = 0x0040         # Emergency stop

# Modbus parameters
BAUDRATE = 115200
SLAVE_ID = 1

def get_default_port():
    if sys.platform.startswith('win'):
        return 'COM4'
    elif sys.platform.startswith('linux'):
        return '/dev/ttyUSB0'
    else:
        raise EnvironmentError('Unsupported platform')

def torque_home_motor(port):
    client = ModbusSerialClient(
        port=port,
        baudrate=BAUDRATE,
        stopbits=1,
        parity='N',
        bytesize=8,
        timeout=1
    )

    if not client.connect():
        print(f"❌ Failed to connect to {port}")
        return

    print(f"✅ Connected to Modbus motor at {port}")
    print("Please input + (forward homing) or - (reverse homing), t for emergency stop, q to quit:")

    try:
        while True:
            user_input = input("Command (+/-/t/q): ").strip().lower()
            if user_input == 'q':
                print("Exit torque homing control.")
                break
            elif user_input == 't':
                # Emergency stop
                result = client.write_register(address=TRIGGER_ADDR, value=TRIGGER_STOP, slave=SLAVE_ID)
                if result.isError():
                    print("❌ Emergency stop failed", result)
                else:
                    print("🛑 Emergency stop command sent")
                continue
            elif user_input not in ['+', '-']:
                print("Invalid input, please enter + (forward), - (reverse), t (emergency stop), or q (quit).")
                continue

            # Select homing mode
            if user_input == '+':
                mode = TORQUE_MODE_FORWARD
                mode_str = "Forward torque homing"
            else:
                mode = TORQUE_MODE_REVERSE
                mode_str = "Reverse torque homing"
            print(f"Selected mode: {mode_str}")

            # Interactive parameter input
            try:
                param_input = input("Please input parameters: stall_time(ms) output_val(%) high_speed(rpm) low_speed(rpm) acc(ms/1000rpm) dec(ms/1000rpm), separated by space: ").strip()
                stall_time, output_val, high_speed, low_speed, acc, dec = map(int, param_input.split())
            except Exception:
                print("Parameter input error, please input: stall_time output_val high_speed low_speed acc dec (space separated)!")
                continue

            # Write homing mode
            result = client.write_register(address=TORQUE_MODE_ADDR, value=mode, slave=SLAVE_ID)
            if result.isError():
                print("❌ Set homing mode failed", result)
                continue
            # Stall time
            result = client.write_register(address=STALL_TIME_ADDR, value=stall_time, slave=SLAVE_ID)
            if result.isError():
                print("❌ Set stall time failed", result)
                continue
            # Output value
            result = client.write_register(address=OUTPUT_VAL_ADDR, value=output_val, slave=SLAVE_ID)
            if result.isError():
                print("❌ Set output value failed", result)
                continue
            # High speed
            result = client.write_register(address=HIGH_SPEED_ADDR, value=high_speed, slave=SLAVE_ID)
            if result.isError():
                print("❌ Set high speed failed", result)
                continue
            # Low speed
            result = client.write_register(address=LOW_SPEED_ADDR, value=low_speed, slave=SLAVE_ID)
            if result.isError():
                print("❌ Set low speed failed", result)
                continue
            # Acceleration
            result = client.write_register(address=ACC_ADDR, value=acc, slave=SLAVE_ID)
            if result.isError():
                print("❌ Set acceleration failed", result)
                continue
            # Deceleration
            result = client.write_register(address=DEC_ADDR, value=dec, slave=SLAVE_ID)
            if result.isError():
                print("❌ Set deceleration failed", result)
                continue
            # Trigger torque homing
            result = client.write_register(address=TRIGGER_ADDR, value=TRIGGER_TORQUE_HOME, slave=SLAVE_ID)
            if result.isError():
                print("❌ Trigger torque homing failed", result)
            else:
                print(f"✅ {mode_str} triggered")
            time.sleep(0.5)
    finally:
        client.close()
        print("🔌 Disconnected.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Torque homing control via Modbus RTU")
    parser.add_argument(
        "--port",
        type=str,
        default=get_default_port(),
        help="Serial port (e.g., COM4 or /dev/ttyUSB0)"
    )
    args = parser.parse_args()
    torque_home_motor(args.port) 