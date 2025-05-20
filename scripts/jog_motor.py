import time
import sys
import argparse
from pymodbus.client import ModbusSerialClient

# Jog commands
JOG_LEFT = 0x4001
JOG_RIGHT = 0x4002

# Modbus control parameters
BAUDRATE = 38400
SLAVE_ID = 1
REGISTER_ADDR = 0x1801

def get_default_port():
    if sys.platform.startswith('win'):
        return 'COM4'
    elif sys.platform.startswith('linux'):
        return '/dev/ttyUSB0'
    else:
        raise EnvironmentError('Unsupported platform')

def jog_motor(port):
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

    try:
        for i in range(10):
            direction = JOG_LEFT if i % 2 == 0 else JOG_RIGHT
            cmd = "LEFT" if direction == JOG_LEFT else "RIGHT"

            print(f"[{i+1}/10] Jog {cmd}")
            result = client.write_register(address=REGISTER_ADDR, value=direction, slave=SLAVE_ID)

            if result.isError():
                print(f"❌ Failed to write jog command: {result}")
            time.sleep(1)
    finally:
        client.close()
        print("🔌 Disconnected.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="Jog motor left/right via Modbus RTU")
    parser.add_argument(
        "--port",
        type=str,
        default=get_default_port(),
        help="Serial port (e.g., COM4 or /dev/ttyUSB0)"
    )
    args = parser.parse_args()
    jog_motor(args.port)
