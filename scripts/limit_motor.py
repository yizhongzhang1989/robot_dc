import time
import sys
import argparse
from pymodbus.client import ModbusSerialClient

# Modbus control parameters
BAUDRATE = 115200
SLAVE_ID = 1

# Software limit related register addresses
POS_LIMIT_HIGH_ADDR = 0x6006  # Positive limit high
POS_LIMIT_LOW_ADDR = 0x6007   # Positive limit low
NEG_LIMIT_HIGH_ADDR = 0x6008  # Negative limit high
NEG_LIMIT_LOW_ADDR = 0x6009   # Negative limit low
SET_ZERO_ADDR = 0x6002        # Set zero register
SET_ZERO_CMD = 0x0021         # Set zero command
CONTROL_SETTING_ADDR = 0x6000  # Control setting register
CONTROL_SETTING_SOFT_LIMIT = 0x0002  # Enable software limit

def get_default_port():
    if sys.platform.startswith('win'):
        return 'COM4'
    elif sys.platform.startswith('linux'):
        return '/dev/ttyUSB0'
    else:
        raise EnvironmentError('Unsupported platform')

def set_control_setting(client, enable_soft_limit):
    value = CONTROL_SETTING_SOFT_LIMIT if enable_soft_limit else 0x0000
    result = client.write_register(address=CONTROL_SETTING_ADDR, value=value, slave=SLAVE_ID)
    if result.isError():
        print(f"❌ Write to control setting register failed, software limit {'not enabled' if enable_soft_limit else 'not disabled'}", result)
        return False
    else:
        print(f"✅ Control setting written, software limit {'enabled' if enable_soft_limit else 'disabled'}")
    time.sleep(0.2)
    return True

def set_software_limit(client):
    # First set control setting register, enable software limit
    result = client.write_register(address=CONTROL_SETTING_ADDR, value=CONTROL_SETTING_SOFT_LIMIT, slave=SLAVE_ID)
    if result.isError():
        print("❌ Write to control setting register failed, software limit not enabled", result)
        return
    else:
        print("✅ Control setting written, software limit enabled")
    time.sleep(0.2)
    try:
        pos_limit = int(input("Enter positive software limit value (e.g. 100000), or q to quit: ").strip())
        neg_limit = int(input("Enter negative software limit value (e.g. -100000), or q to quit: ").strip())
    except ValueError:
        print("Parameter input error, please re-enter!")
        return
    # Set zero manually first
    result = client.write_register(address=SET_ZERO_ADDR, value=SET_ZERO_CMD, slave=SLAVE_ID)
    if result.isError():
        print("❌ Set zero failed", result)
        return
    else:
        print("✅ Set zero command sent, current position cleared to zero")
    time.sleep(0.5)
    # Positive limit high/low
    pos_limit_high = (pos_limit >> 16) & 0xFFFF
    pos_limit_low = pos_limit & 0xFFFF
    result = client.write_register(address=POS_LIMIT_HIGH_ADDR, value=pos_limit_high, slave=SLAVE_ID)
    if result.isError():
        print("❌ Set positive limit high failed", result)
        return
    result = client.write_register(address=POS_LIMIT_LOW_ADDR, value=pos_limit_low, slave=SLAVE_ID)
    if result.isError():
        print("❌ Set positive limit low failed", result)
        return
    # Negative limit high/low
    neg_limit_high = (neg_limit >> 16) & 0xFFFF
    neg_limit_low = neg_limit & 0xFFFF
    result = client.write_register(address=NEG_LIMIT_HIGH_ADDR, value=neg_limit_high, slave=SLAVE_ID)
    if result.isError():
        print("❌ Set negative limit high failed", result)
        return
    result = client.write_register(address=NEG_LIMIT_LOW_ADDR, value=neg_limit_low, slave=SLAVE_ID)
    if result.isError():
        print("❌ Set negative limit low failed", result)
        return
    print(f"✅ Software limit set, positive limit: {pos_limit}, negative limit: {neg_limit}")

def main(port):
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
    # 新增：选择是否开启软件限位
    print("Select whether to enable software limit? (y/n): ")
    while True:
        choice = input("Enable software limit? (y/n): ").strip().lower()
        if choice == 'y':
            if not set_control_setting(client, True):
                return
            break
        elif choice == 'n':
            if not set_control_setting(client, False):
                return
            break
        else:
            print("Invalid input, please enter y or n.")
    try:
        while True:
            user_input = input("Enter 's' to set software limit, 'q' to quit: ").strip().lower()
            if user_input == 'q':
                print("Exit software limit setting."); break
            elif user_input == 's':
                set_software_limit(client)
            else:
                print("Invalid input, please enter 's' or 'q'.")
    finally:
        client.close()
        print("🔌 Disconnected.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Software limit setting via Modbus RTU")
    parser.add_argument(
        "--port",
        type=str,
        default=get_default_port(),
        help="Serial port (e.g., COM4 or /dev/ttyUSB0)"
    )
    args = parser.parse_args()
    main(args.port) 