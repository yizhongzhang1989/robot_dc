import time
import sys
import argparse
from pymodbus.client import ModbusSerialClient

# Modbus control parameters
BAUDRATE = 115200
SLAVE_ID = 1

# 软件限位相关寄存器
POS_LIMIT_HIGH_ADDR = 0x6006  # 正限位高位
POS_LIMIT_LOW_ADDR = 0x6007   # 正限位低位
NEG_LIMIT_HIGH_ADDR = 0x6008  # 负限位高位
NEG_LIMIT_LOW_ADDR = 0x6009   # 负限位低位
SET_ZERO_ADDR = 0x6002        # 设零寄存器
SET_ZERO_CMD = 0x0021         # 设零指令
CONTROL_SETTING_ADDR = 0x6000  # 控制设置寄存器
CONTROL_SETTING_SOFT_LIMIT = 0x0002  # 软件限位有效

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
        print(f"❌ 控制设置寄存器写入失败，软件限位{'未使能' if enable_soft_limit else '未关闭'}", result)
        return False
    else:
        print(f"✅ 控制设置已写入，软件限位{'已使能' if enable_soft_limit else '已关闭'}")
    time.sleep(0.2)
    return True

def set_software_limit(client):
    # 先设置控制设置寄存器，软件限位有效
    result = client.write_register(address=CONTROL_SETTING_ADDR, value=CONTROL_SETTING_SOFT_LIMIT, slave=SLAVE_ID)
    if result.isError():
        print("❌ 控制设置寄存器写入失败，软件限位未使能", result)
        return
    else:
        print("✅ 控制设置已写入，软件限位已使能")
    time.sleep(0.2)
    try:
        pos_limit = int(input("请输入正软件限位值（如100000），输入 q 退出: ").strip())
        neg_limit = int(input("请输入负软件限位值（如-100000），输入 q 退出: ").strip())
    except ValueError:
        print("参数输入有误，请重新输入！")
        return
    # 先手动设零
    result = client.write_register(address=SET_ZERO_ADDR, value=SET_ZERO_CMD, slave=SLAVE_ID)
    if result.isError():
        print("❌ 设零失败", result)
        return
    else:
        print("✅ 已发送设零指令，当前位置已清零")
    time.sleep(0.5)
    # 正限位高低位
    pos_limit_high = (pos_limit >> 16) & 0xFFFF
    pos_limit_low = pos_limit & 0xFFFF
    result = client.write_register(address=POS_LIMIT_HIGH_ADDR, value=pos_limit_high, slave=SLAVE_ID)
    if result.isError():
        print("❌ 正限位高位设置失败", result)
        return
    result = client.write_register(address=POS_LIMIT_LOW_ADDR, value=pos_limit_low, slave=SLAVE_ID)
    if result.isError():
        print("❌ 正限位低位设置失败", result)
        return
    # 负限位高低位
    neg_limit_high = (neg_limit >> 16) & 0xFFFF
    neg_limit_low = neg_limit & 0xFFFF
    result = client.write_register(address=NEG_LIMIT_HIGH_ADDR, value=neg_limit_high, slave=SLAVE_ID)
    if result.isError():
        print("❌ 负限位高位设置失败", result)
        return
    result = client.write_register(address=NEG_LIMIT_LOW_ADDR, value=neg_limit_low, slave=SLAVE_ID)
    if result.isError():
        print("❌ 负限位低位设置失败", result)
        return
    print(f"✅ 软件限位已设置，正限位：{pos_limit}，负限位：{neg_limit}")

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
    while True:
        choice = input("是否开启软件限位？(y/n): ").strip().lower()
        if choice == 'y':
            if not set_control_setting(client, True):
                return
            break
        elif choice == 'n':
            if not set_control_setting(client, False):
                return
            break
        else:
            print("无效输入，请输入 y 或 n。")
    try:
        while True:
            user_input = input("输入 s 设置软件限位，q 退出: ").strip().lower()
            if user_input == 'q':
                print("退出软件限位设置。"); break
            elif user_input == 's':
                set_software_limit(client)
            else:
                print("无效输入，请输入 's' 或 'q'.")
    finally:
        client.close()
        print("🔌 Disconnected.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="软件限位设置 via Modbus RTU")
    parser.add_argument(
        "--port",
        type=str,
        default=get_default_port(),
        help="Serial port (e.g., COM4 or /dev/ttyUSB0)"
    )
    args = parser.parse_args()
    main(args.port) 