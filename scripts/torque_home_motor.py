import time
import sys
import argparse
from pymodbus.client import ModbusSerialClient

# 力矩回零相关寄存器
TORQUE_MODE_ADDR = 0x600A
STALL_TIME_ADDR = 0x6013
OUTPUT_VAL_ADDR = 0x6014
TRIGGER_ADDR = 0x6002
HIGH_SPEED_ADDR = 0x600F
LOW_SPEED_ADDR = 0x6010
ACC_ADDR = 0x6011
DEC_ADDR = 0x6012

# 指令值
TORQUE_MODE_REVERSE = 0x000C  # 反向力矩回零
TORQUE_MODE_FORWARD = 0x000D  # 正向力矩回零
TRIGGER_TORQUE_HOME = 0x0020  # 触发力矩回零
TRIGGER_STOP = 0x0040         # 急停

# Modbus参数
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
    print("请输入 +（正向回零）或 -（反向回零），t 急停，q 退出：")

    try:
        while True:
            user_input = input("指令 (+/-/t/q): ").strip().lower()
            if user_input == 'q':
                print("退出力矩回零控制。")
                break
            elif user_input == 't':
                # 急停
                result = client.write_register(address=TRIGGER_ADDR, value=TRIGGER_STOP, slave=SLAVE_ID)
                if result.isError():
                    print("❌ 急停失败", result)
                else:
                    print("🛑 已发送急停指令")
                continue
            elif user_input not in ['+', '-']:
                print("无效输入，请输入 +（正向）、-（反向）、t（急停）、q（退出）。")
                continue

            # 选择回零模式
            if user_input == '+':
                mode = TORQUE_MODE_FORWARD
                mode_str = "正向力矩回零"
            else:
                mode = TORQUE_MODE_REVERSE
                mode_str = "反向力矩回零"
            print(f"选择模式：{mode_str}")

            # 交互输入参数
            try:
                param_input = input("请输入参数：堵转时间(ms) 出力值(%) 回零高速(rpm) 回零低速(rpm) 回零加速度(ms/1000rpm) 回零减速度(ms/1000rpm)，用空格分隔: ").strip()
                stall_time, output_val, high_speed, low_speed, acc, dec = map(int, param_input.split())
            except Exception:
                print("参数输入有误，请按格式输入：堵转时间 出力值 高速 低速 加速度 减速度（空格分隔）！")
                continue

            # 写入回零模式
            result = client.write_register(address=TORQUE_MODE_ADDR, value=mode, slave=SLAVE_ID)
            if result.isError():
                print("❌ 设置回零模式失败", result)
                continue
            # 堵转时间
            result = client.write_register(address=STALL_TIME_ADDR, value=stall_time, slave=SLAVE_ID)
            if result.isError():
                print("❌ 设置堵转时间失败", result)
                continue
            # 出力值
            result = client.write_register(address=OUTPUT_VAL_ADDR, value=output_val, slave=SLAVE_ID)
            if result.isError():
                print("❌ 设置出力值失败", result)
                continue
            # 回零高速
            result = client.write_register(address=HIGH_SPEED_ADDR, value=high_speed, slave=SLAVE_ID)
            if result.isError():
                print("❌ 设置回零高速失败", result)
                continue
            # 回零低速
            result = client.write_register(address=LOW_SPEED_ADDR, value=low_speed, slave=SLAVE_ID)
            if result.isError():
                print("❌ 设置回零低速失败", result)
                continue
            # 回零加速度
            result = client.write_register(address=ACC_ADDR, value=acc, slave=SLAVE_ID)
            if result.isError():
                print("❌ 设置回零加速度失败", result)
                continue
            # 回零减速度
            result = client.write_register(address=DEC_ADDR, value=dec, slave=SLAVE_ID)
            if result.isError():
                print("❌ 设置回零减速度失败", result)
                continue

            # 触发力矩回零
            result = client.write_register(address=TRIGGER_ADDR, value=TRIGGER_TORQUE_HOME, slave=SLAVE_ID)
            if result.isError():
                print("❌ 触发力矩回零失败", result)
            else:
                print(f"✅ {mode_str} 已触发")
            time.sleep(0.5)
    finally:
        client.close()
        print("🔌 Disconnected.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="力矩回零控制 via Modbus RTU")
    parser.add_argument(
        "--port",
        type=str,
        default=get_default_port(),
        help="Serial port (e.g., COM4 or /dev/ttyUSB0)"
    )
    args = parser.parse_args()
    torque_home_motor(args.port) 