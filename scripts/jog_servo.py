import time
import sys
import argparse
from pymodbus.client import ModbusSerialClient

# Modbus control parameters for SMSMD
BAUDRATE = 115200
SLAVE_ID = 11
REGISTER_POSITION = 0x80  # DEC 128
REGISTER_TORQUE = 0x81  # DEC 129
REGISTER_ACC = 0x82  # DEC 130 (optional)
REGISTER_SPEED = 0x83  # DEC 131 (optional)

# Example positions (you can adjust as needed)
POSITION_LEFT = 0   # ~ -87.9 degrees
POSITION_RIGHT = 4095   # ~ +87.9 degrees
SPEED = 100             # ~ 73.2 RPM

def get_default_port():
    if sys.platform.startswith('win'):
        return 'COM4'
    elif sys.platform.startswith('linux'):
        return '/dev/ttyUSB0'
    else:
        raise EnvironmentError('Unsupported platform')

def jog_smsmd(port):
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

    print(f"✅ Connected to SMSMD motor at {port}")

    try:
        # Enable torque
        res = client.write_register(REGISTER_TORQUE, 1, slave=SLAVE_ID)
        if res.isError():
            print("❌ Failed to enable torque")
            return
        print("🔧 Torque enabled")

        # Set acceleration (optional)
        res = client.write_register(REGISTER_ACC, 100, slave=SLAVE_ID)
        if res.isError():
            print("⚠️ Failed to set acceleration, proceeding anyway")

        # Set speed (optional)
        res = client.write_register(REGISTER_SPEED, SPEED, slave=SLAVE_ID)
        if res.isError():
            print("⚠️ Failed to set speed, proceeding anyway")

        # Jog back and forth
        for i in range(10):
            pos = POSITION_LEFT if i % 2 == 0 else POSITION_RIGHT
            print(f"[{i+1}/10] Move to position {pos}")
            res = client.write_register(REGISTER_POSITION, pos, slave=SLAVE_ID)
            if res.isError():
                print(f"❌ Failed to set position: {res}")
            time.sleep(1.5)  # Allow movement time
    finally:
        client.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Jog SMSMD motor via Modbus RTU")
    parser.add_argument(
        "--port",
        type=str,
        default=get_default_port(),
        help="Serial port (e.g., COM4 or /dev/ttyUSB0)"
    )
    args = parser.parse_args()
    jog_smsmd(args.port)
