from modbus_devices.base_device import ModbusDevice
from modbus_devices.utils import *
import threading
import time
from collections import deque

class LiftRobotController(ModbusDevice):
    """
    升降机器人控制器 - 使用标准继电器开关命令实现闪开效果
    
    继电器配置：
    - 0号继电器：停止 (stop)
    - 1号继电器：上升 (up) 
    - 2号继电器：下降 (down)
    
    通信参数：
    - 波特率: 115200
    - 设备ID: 50
    - 标准Modbus功能码05控制继电器
    
    闪开实现方式：
    - 开启继电器 (0xFF00) -> 延时100ms -> 关闭继电器 (0x0000)
    """
    
    def __init__(self, device_id, node, use_ack_patch):
        super().__init__(device_id, node, use_ack_patch)
        
        # Timer-based movement attributes
        self.active_timers = {}  # Dictionary to store active timers
        self.timer_lock = threading.Lock()  # Lock for thread safety
        
        # Command queue for timed operations
        self.timed_cmd_queue = deque()
        self.waiting_for_timed_ack = False

    def initialize(self):
        """初始化升降平台，复位所有继电器"""
        # 等待Modbus服务准备就绪，然后复位所有继电器
        self.node.get_logger().info("等待Modbus服务准备就绪...")
        
        # 创建一个定时器来重试初始化，直到Modbus服务可用
        def retry_reset():
            if hasattr(self, 'cli') and self.cli.service_is_ready():
                self.reset_all_relays()
                self.node.get_logger().info("升降平台初始化完成，已复位所有继电器")
            else:
                self.node.get_logger().warn("Modbus服务仍未准备就绪，1秒后重试...")
                # 1秒后重试
                timer = threading.Timer(1.0, retry_reset)
                timer.start()
        
        # 立即尝试一次，如果失败则启动重试机制
        if hasattr(self, 'cli') and self.cli.service_is_ready():
            self.reset_all_relays()
            self.node.get_logger().info("升降平台初始化完成，已复位所有继电器")
        else:
            retry_reset()

    def reset_all_relays(self, seq_id=None):
        """
        复位所有继电器
        发送485命令: 01 05 00 FF 00 00 FD FA
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] 复位所有继电器")
        
        # 根据协议: 地址=0x00FF, 值=0x0000
        reset_address = 0x00FF
        reset_value = 0x0000
        
        self.node.get_logger().info(
            f"[SEQ {seq_id}] 发送复位命令: 地址=0x{reset_address:04X}, 值=0x{reset_value:04X}"
        )
        
        self.send(5, reset_address, [reset_value], seq_id=seq_id)

    def stop(self, seq_id=None):
        """
        停止升降平台
        开启0号继电器 -> 延时100ms -> 关闭0号继电器 (闪开效果)
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] 发送停止命令 (0号继电器闪开)")
        self.flash_relay(relay_address=0, duration_ms=100, seq_id=seq_id)

    def up(self, seq_id=None):
        """
        升降平台上升
        开启1号继电器 -> 延时100ms -> 关闭1号继电器 (闪开效果)
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] 发送上升命令 (1号继电器闪开)")
        self.flash_relay(relay_address=1, duration_ms=100, seq_id=seq_id)

    def down(self, seq_id=None):
        """
        升降平台下降
        开启2号继电器 -> 延时100ms -> 关闭2号继电器 (闪开效果)
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] 发送下降命令 (2号继电器闪开)")
        self.flash_relay(relay_address=2, duration_ms=100, seq_id=seq_id)

    def open_relay(self, relay_address, seq_id=None):
        """
        开启继电器
        
        标准Modbus协议:
        - 0号继电器开启: 01 05 00 00 FF 00 8C 3A
        - 1号继电器开启: 01 05 00 01 FF 00 DD FA  
        - 2号继电器开启: 01 05 00 02 FF 00 2D FA
        
        Args:
            relay_address: 继电器地址 (0, 1, 2)
            seq_id: 序列ID
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] 开启继电器{relay_address}")
        
        # 地址: 继电器寄存器地址 0x0000-0x000F
        modbus_address = relay_address
        
        # 值: 0xFF00 = 继电器开启
        open_value = 0xFF00
        
        self.node.get_logger().info(
            f"[SEQ {seq_id}] 继电器开启: 地址=0x{modbus_address:04X}, 值=0x{open_value:04X}"
        )
        
        self.send(5, modbus_address, [open_value], seq_id=seq_id)

    def close_relay(self, relay_address, seq_id=None):
        """
        关闭继电器
        
        标准Modbus协议:
        - 0号继电器关闭: 01 05 00 00 00 00 CD CA
        - 1号继电器关闭: 01 05 00 01 00 00 9C 0A
        - 2号继电器关闭: 01 05 00 02 00 00 6C 0A
        
        Args:
            relay_address: 继电器地址 (0, 1, 2)
            seq_id: 序列ID
        """
        self.node.get_logger().info(f"[SEQ {seq_id}] 关闭继电器{relay_address}")
        
        # 地址: 继电器寄存器地址 0x0000-0x000F
        modbus_address = relay_address
        
        # 值: 0x0000 = 继电器关闭
        close_value = 0x0000
        
        self.node.get_logger().info(
            f"[SEQ {seq_id}] 继电器关闭: 地址=0x{modbus_address:04X}, 值=0x{close_value:04X}"
        )
        
        self.send(5, modbus_address, [close_value], seq_id=seq_id)

    def flash_relay(self, relay_address, duration_ms=100, seq_id=None):
        """
        继电器闪开: 开启 -> 延时 -> 关闭
        
        Args:
            relay_address: 继电器地址 (0=停止, 1=上升, 2=下降)
            duration_ms: 延时时间(毫秒)
            seq_id: 序列ID
        """
        self.node.get_logger().info(
            f"[SEQ {seq_id}] 继电器{relay_address}闪开: 延时{duration_ms}ms"
        )
        
        # 1. 开启继电器
        self.open_relay(relay_address, seq_id=seq_id)
        
        # 2. 延时后关闭继电器
        def close_after_delay():
            time.sleep(duration_ms / 1000.0)  # 转换为秒
            self.close_relay(relay_address, seq_id=seq_id)
        
        # 启动延时关闭线程
        flash_timer = threading.Timer(duration_ms / 1000.0, self.close_relay, 
                                     args=[relay_address], kwargs={'seq_id': seq_id})
        flash_timer.start()
        
        # 存储定时器以便清理
        with self.timer_lock:
            timer_name = f'flash_relay_{relay_address}'
            if timer_name in self.active_timers:
                self.active_timers[timer_name].cancel()
            self.active_timers[timer_name] = flash_timer

    def timed_up(self, duration, seq_id=None):
        """
        定时上升
        
        Args:
            duration: 上升时间(秒)
            seq_id: 序列ID
        """
        with self.timer_lock:
            # 取消之前的定时器
            self.cancel_all_timers()
            
            self.node.get_logger().info(f"[SEQ {seq_id}] 开始定时上升 {duration}秒")
            
            # 发送上升命令
            self.up(seq_id=seq_id)
            
            # 设置定时器，时间到后自动停止
            timer = threading.Timer(duration, self.stop)
            timer.start()
            self.active_timers['timed_up'] = timer

    def timed_down(self, duration, seq_id=None):
        """
        定时下降
        
        Args:
            duration: 下降时间(秒)
            seq_id: 序列ID
        """
        with self.timer_lock:
            # 取消之前的定时器
            self.cancel_all_timers()
            
            self.node.get_logger().info(f"[SEQ {seq_id}] 开始定时下降 {duration}秒")
            
            # 发送下降命令
            self.down(seq_id=seq_id)
            
            # 设置定时器，时间到后自动停止
            timer = threading.Timer(duration, self.stop)
            timer.start()
            self.active_timers['timed_down'] = timer

    def stop_timed(self, seq_id=None):
        """停止所有定时运动"""
        with self.timer_lock:
            self.node.get_logger().info(f"[SEQ {seq_id}] 停止所有定时运动")
            self.cancel_all_timers()
            self.stop(seq_id=seq_id)

    def cancel_all_timers(self):
        """取消所有活动的定时器"""
        for timer_name, timer in self.active_timers.items():
            if timer.is_alive():
                timer.cancel()
                self.node.get_logger().info(f"已取消定时器: {timer_name}")
        self.active_timers.clear()

    def cleanup(self):
        """清理资源"""
        with self.timer_lock:
            self.cancel_all_timers()
            self.node.get_logger().info("所有定时器清理成功")
