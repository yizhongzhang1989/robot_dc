import time
import sys
import argparse
import threading
from pymodbus.client import ModbusSerialClient

# Modbus control parameters
BAUDRATE = 115200
SLAVE_ID = 1

# PR related register addresses
PR_MODE_ADDR = 0x6200
PR_POS_HIGH_ADDR = 0x6201
PR_POS_LOW_ADDR = 0x6202
PR_SPEED_ADDR = 0x6203
PR_ACC_ADDR = 0x6204
PR_DEC_ADDR = 0x6205
PR_TRIGGER_ADDR = 0x6002
ALARM_RESET_ADDR = 0x1801  # Alarm reset register
ALARM_RESET_CMD = 0x1111   # Alarm reset command

# PR instruction values
PR_MODE_ABS = 0x0001  # Absolute position
PR_MODE_REL = 0x0041  # Relative position
PR_TRIGGER_RUN = 0x0010
PR_TRIGGER_STOP = 0x0040


def get_default_port():
    if sys.platform.startswith('win'):
        return 'COM4'
    elif sys.platform.startswith('linux'):
        return '/dev/ttyUSB0'
    else:
        raise EnvironmentError('Unsupported platform')

def pr_set_zero(client):
    # Manual set zero command
    result = client.write_register(address=PR_TRIGGER_ADDR, value=0x0021, slave=SLAVE_ID)
    if result.isError():
        print("❌ Set zero failed", result)
    else:
        print("✅ Set zero command sent, current position cleared to zero")
    time.sleep(0.5)

def alarm_reset_periodic(client, interval, stop_event):
    def reset_loop():
        while not stop_event.is_set():
            result = client.write_register(address=ALARM_RESET_ADDR, value=ALARM_RESET_CMD, slave=SLAVE_ID)
            # No output
            stop_event.wait(interval)
    t = threading.Thread(target=reset_loop, daemon=True)
    t.start()
    return t

def pr_move(client, position, speed, acc, dec):
    # Set zero before motion
    pr_set_zero(client)
    # 1. Set PR0 mode to absolute-position
    result = client.write_register(address=PR_MODE_ADDR, value=PR_MODE_ABS, slave=SLAVE_ID)
    if result.isError():
        print("❌ Set PR mode failed", result)
        return False
    # 2. Set PR0 position high/low
    pos = int(position)
    pos_high = (pos >> 16) & 0xFFFF
    pos_low = pos & 0xFFFF
    result = client.write_register(address=PR_POS_HIGH_ADDR, value=pos_high, slave=SLAVE_ID)
    if result.isError():
        print("❌ Set position high failed", result)
        return False
    result = client.write_register(address=PR_POS_LOW_ADDR, value=pos_low, slave=SLAVE_ID)
    if result.isError():
        print("❌ Set position low failed", result)
        return False
    # 3. Set speed, acceleration, deceleration
    result = client.write_register(address=PR_SPEED_ADDR, value=int(speed), slave=SLAVE_ID)
    if result.isError():
        print("❌ Set speed failed", result)
        return False
    result = client.write_register(address=PR_ACC_ADDR, value=int(acc), slave=SLAVE_ID)
    if result.isError():
        print("❌ Set acceleration failed", result)
        return False
    result = client.write_register(address=PR_DEC_ADDR, value=int(dec), slave=SLAVE_ID)
    if result.isError():
        print("❌ Set deceleration failed", result)
        return False
    # 4. Trigger PR0 run
    result = client.write_register(address=PR_TRIGGER_ADDR, value=PR_TRIGGER_RUN, slave=SLAVE_ID)
    if result.isError():
        print("❌ Trigger run failed", result)
        return False
    print("✅ PR motion triggered")
    return True

def pr_stop(client):
    # Emergency stop command
    result = client.write_register(address=PR_TRIGGER_ADDR, value=PR_TRIGGER_STOP, slave=SLAVE_ID)
    if result.isError():
        print("❌ Emergency stop failed", result)
    else:
        print("🛑 Emergency stop command sent")

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
    # 新增：输入消除警报的时间间隔
    print("Please input the interval (seconds, can be decimal) for automatic alarm reset:")
    while True:
        try:
            alarm_interval = float(input("Enter alarm reset interval (seconds, can be decimal): ").strip())
            if alarm_interval <= 0:
                print("Interval must be greater than 0, please re-enter.")
                continue
            break
        except Exception:
            print("Invalid input, please enter a positive number!")
    stop_event = threading.Event()
    alarm_reset_periodic(client, alarm_interval, stop_event)
    print("Please input target position, speed, acceleration, deceleration, or input t for emergency stop, q to quit:")
    try:
        while True:
            user_input = input("Input format: position speed acc dec | t(emergency stop) | q(quit): ").strip().lower()
            if user_input == 'q':
                print("Exit PR control.")
                break
            elif user_input == 't':
                pr_stop(client)
                continue
            else:
                try:
                    pos, speed, acc, dec = map(int, user_input.split())
                except Exception:
                    print("Input format error, please input: position speed acc dec, or t/q")
                    continue
                pr_move(client, pos, speed, acc, dec)
            time.sleep(0.5)
    finally:
        stop_event.set()
        client.close()
        print("🔌 Disconnected.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PR motion control via Modbus RTU")
    parser.add_argument(
        "--port",
        type=str,
        default=get_default_port(),
        help="Serial port (e.g., COM4 or /dev/ttyUSB0)"
    )
    args = parser.parse_args()
    main(args.port) 