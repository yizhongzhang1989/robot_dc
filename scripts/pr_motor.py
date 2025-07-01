import time
import sys
import argparse
import threading
from pymodbus.client import ModbusSerialClient

# Modbus control parameters
BAUDRATE = 115200
SLAVE_ID = 1

# PR相关寄存器地址
PR_MODE_ADDR = 0x6200
PR_POS_HIGH_ADDR = 0x6201
PR_POS_LOW_ADDR = 0x6202
PR_SPEED_ADDR = 0x6203
PR_ACC_ADDR = 0x6204
PR_DEC_ADDR = 0x6205
PR_TRIGGER_ADDR = 0x6002
ALARM_RESET_ADDR = 0x1801  # 复位报警寄存器
ALARM_RESET_CMD = 0x1111   # 复位报警指令

# PR指令值
PR_MODE_ABS = 0x0001  # 绝对位置
PR_MODE_REL = 0x0041  # 相对位置
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
    # 手动设零命令
    result = client.write_register(address=PR_TRIGGER_ADDR, value=0x0021, slave=SLAVE_ID)
    if result.isError():
        print("❌ 设零失败", result)
    else:
        print("✅ 已发送设零指令，当前位置已清零")
    time.sleep(0.5)

def alarm_reset_periodic(client, interval, stop_event):
    def reset_loop():
        while not stop_event.is_set():
            result = client.write_register(address=ALARM_RESET_ADDR, value=ALARM_RESET_CMD, slave=SLAVE_ID)
            # 不输出任何内容
            stop_event.wait(interval)
    t = threading.Thread(target=reset_loop, daemon=True)
    t.start()
    return t

def pr_move(client, position, speed, acc, dec):
    # 运动前先设零
    pr_set_zero(client)
    # 1. 设定PR0模式为绝对-位置
    result = client.write_register(address=PR_MODE_ADDR, value=PR_MODE_ABS, slave=SLAVE_ID)
    if result.isError():
        print("❌ 设定PR模式失败", result)
        return False
    # 2. 设定PR0位置高位、低位
    pos = int(position)
    pos_high = (pos >> 16) & 0xFFFF
    pos_low = pos & 0xFFFF
    result = client.write_register(address=PR_POS_HIGH_ADDR, value=pos_high, slave=SLAVE_ID)
    if result.isError():
        print("❌ 设定位置高位失败", result)
        return False
    result = client.write_register(address=PR_POS_LOW_ADDR, value=pos_low, slave=SLAVE_ID)
    if result.isError():
        print("❌ 设定位置低位失败", result)
        return False
    # 3. 设定速度、加速度、减速度
    result = client.write_register(address=PR_SPEED_ADDR, value=int(speed), slave=SLAVE_ID)
    if result.isError():
        print("❌ 设定速度失败", result)
        return False
    result = client.write_register(address=PR_ACC_ADDR, value=int(acc), slave=SLAVE_ID)
    if result.isError():
        print("❌ 设定加速度失败", result)
        return False
    result = client.write_register(address=PR_DEC_ADDR, value=int(dec), slave=SLAVE_ID)
    if result.isError():
        print("❌ 设定减速度失败", result)
        return False
    # 4. 触发PR0运行
    result = client.write_register(address=PR_TRIGGER_ADDR, value=PR_TRIGGER_RUN, slave=SLAVE_ID)
    if result.isError():
        print("❌ 触发运行失败", result)
        return False
    print("✅ PR运动已触发")
    return True

def pr_stop(client):
    # 急停命令
    result = client.write_register(address=PR_TRIGGER_ADDR, value=PR_TRIGGER_STOP, slave=SLAVE_ID)
    if result.isError():
        print("❌ 急停失败", result)
    else:
        print("🛑 已发送急停指令")

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
    while True:
        try:
            alarm_interval = float(input("请输入自动消除警报的时间间隔（秒，可为小数）：").strip())
            if alarm_interval <= 0:
                print("时间间隔需大于0，请重新输入。")
                continue
            break
        except Exception:
            print("输入有误，请输入正数！")
    stop_event = threading.Event()
    alarm_reset_periodic(client, alarm_interval, stop_event)
    print("请输入目标位置、速度、加速度、减速度，或输入 t 急停，q 退出：")
    try:
        while True:
            user_input = input("输入格式: 位置 速度 加速度 减速度 | t(急停) | q(退出): ").strip().lower()
            if user_input == 'q':
                print("退出PR控制。")
                break
            elif user_input == 't':
                pr_stop(client)
                continue
            else:
                try:
                    pos, speed, acc, dec = map(int, user_input.split())
                except Exception:
                    print("输入格式错误，请输入: 位置 速度 加速度 减速度，或 t/q")
                    continue
                pr_move(client, pos, speed, acc, dec)
            time.sleep(0.5)
    finally:
        stop_event.set()
        client.close()
        print("🔌 Disconnected.")

if __name__ == "__main__":
    parser = argparse.ArgumentParser(description="PR运动控制 via Modbus RTU")
    parser.add_argument(
        "--port",
        type=str,
        default=get_default_port(),
        help="Serial port (e.g., COM4 or /dev/ttyUSB0)"
    )
    args = parser.parse_args()
    main(args.port) 