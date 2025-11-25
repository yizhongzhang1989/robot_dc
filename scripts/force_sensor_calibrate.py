#!/usr/bin/env python3
"""
Interactive calibration tool for force sensor.
(类似拉绳传感器标定工具的力传感器版本)

Workflow:
1. source install/setup.bash
2. python3 scripts/force_sensor_calibrate.py --device_id 53  # 或52, 60
3. 输入'd'记录当前传感器读数
4. 输入实际施加的标准力值(单位: N)
5. 重复步骤3-4收集多组数据
6. 输入'cal'计算线性拟合参数 (强制过原点: actual_force = sensor_reading * scale)
7. 输入'q'退出并显示最终标定结果
8. 输入'list', 'help'等命令查看

注意: 
  - 标定前请先对传感器去皮(force_sensor_tare.py)
  - 拟合算法强制曲线过原点(offset=0)，仅计算scale参数

支持的传感器:
  - force_sensor (device_id=52, topic=/force_sensor)
  - force_sensor_2 (device_id=53, topic=/force_sensor_2)
  - force_sensor_test (device_id=60, topic=/force_sensor_test)
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import json
import threading
import time
import argparse
import sys
from dataclasses import dataclass
from typing import List, Optional, Tuple


@dataclass
class Sample:
    """标定样本: 传感器原始读数 -> 实际施加力值"""
    sensor_reading: float  # 传感器原始读数
    actual_force: float    # 实际施加的标准力 (N)
    timestamp: float


class CalibrationState:
    """标定状态管理"""
    def __init__(self):
        self.latest_force: Optional[float] = None  # 最新传感器读数
        self.samples: List[Sample] = []
        self.lock = threading.Lock()
        self.scale: Optional[float] = None
        self.offset: Optional[float] = None
        self.pending_sensor_reading: Optional[float] = None  # 等待输入实际力值的传感器读数
    
    def record_sensor_reading(self) -> Optional[float]:
        """记录当前传感器读数，等待输入实际力值"""
        with self.lock:
            if self.latest_force is None:
                return None
            self.pending_sensor_reading = self.latest_force
            return self.pending_sensor_reading
    
    def add_sample(self, actual_force: float) -> bool:
        """添加完整样本 (传感器读数 + 实际力值)"""
        with self.lock:
            if self.pending_sensor_reading is None:
                return False
            self.samples.append(Sample(
                sensor_reading=self.pending_sensor_reading,
                actual_force=actual_force,
                timestamp=time.time()
            ))
            self.pending_sensor_reading = None
            return True
    
    def fit(self) -> Tuple[float, float]:
        """线性拟合: actual_force = sensor_reading * scale + offset
        
        由于传感器已去皮，强制拟合曲线过原点 (offset=0)
        即: actual_force = sensor_reading * scale
        """
        with self.lock:
            if len(self.samples) < 1:
                raise ValueError('需要至少1组样本数据')
            xs = [s.sensor_reading for s in self.samples]
            ys = [s.actual_force for s in self.samples]
        
        # 强制过原点的线性拟合: y = k*x
        # 最小二乘法: k = Σ(x*y) / Σ(x²)
        numerator = sum(x * y for x, y in zip(xs, ys))
        denominator = sum(x * x for x in xs)
        
        if denominator == 0:
            raise ValueError('传感器读数全为零，无法拟合')
        
        scale = numerator / denominator
        offset = 0.0  # 强制过原点
        
        with self.lock:
            self.scale, self.offset = scale, offset
        
        return scale, offset
    
    def get_calibration_summary(self, device_id: int, topic: str) -> str:
        """生成标定结果摘要字符串"""
        with self.lock:
            if not self.samples:
                return "没有标定数据"
            
            lines = []
            lines.append('\n' + '='*70)
            lines.append('  力传感器标定结果摘要')
            lines.append('='*70)
            lines.append(f'  设备ID:  {device_id}')
            lines.append(f'  话题:    {topic}')
            lines.append(f'  样本数:  {len(self.samples)}')
            lines.append('='*70)
            
            if self.scale is not None and self.offset is not None:
                lines.append('\n  线性拟合参数:')
                lines.append(f'    scale  = {self.scale:.6f}')
                lines.append(f'    offset = {self.offset:.6f} (强制过原点)')
                lines.append(f'\n  拟合公式:')
                lines.append(f'    actual_force = sensor_reading × {self.scale:.6f}')
                lines.append(f'  或简写为:')
                lines.append(f'    F_actual = F_sensor × {self.scale:.6f}')
                
                # 计算拟合误差
                errors = []
                for s in self.samples:
                    predicted = s.sensor_reading * self.scale + self.offset
                    error = predicted - s.actual_force
                    errors.append(error)
                
                if errors:
                    import statistics
                    lines.append(f'\n  拟合误差统计:')
                    lines.append(f'    平均误差: {statistics.mean(errors):+.3f} N')
                    lines.append(f'    标准差:   {statistics.stdev(errors) if len(errors) > 1 else 0:.3f} N')
                    lines.append(f'    最大误差: {max(errors, key=abs):+.3f} N')
            else:
                lines.append('\n  (未计算拟合参数，请先执行 cal 命令)')
            
            lines.append('\n  样本数据明细:')
            lines.append(f"  {'#':<4} {'传感器读数':<15} {'实际力值(N)':<15} {'时间':<12}")
            lines.append('  ' + '-' * 50)
            for i, s in enumerate(self.samples, 1):
                ts_str = time.strftime('%H:%M:%S', time.localtime(s.timestamp))
                lines.append(f"  {i:<4} {s.sensor_reading:<15.3f} {s.actual_force:<15.3f} {ts_str:<12}")
            
            lines.append('='*70)
            
            return '\n'.join(lines)
    
    def save(self, device_id: int, topic: str) -> str:
        """保存标定结果到JSON文件"""
        filename = f'calibration_force_sensor_{device_id}.json'
        with self.lock:
            data = {
                'device_id': device_id,
                'topic': topic,
                'samples': [
                    {
                        'sensor_reading': s.sensor_reading,
                        'actual_force': s.actual_force,
                        'timestamp': s.timestamp
                    }
                    for s in self.samples
                ],
                'scale': self.scale,
                'offset': self.offset,
                'formula': 'actual_force = sensor_reading * scale + offset',
                'generated_at': time.time(),
                'timestamp_str': time.strftime('%Y-%m-%d %H:%M:%S')
            }
        
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(data, f, indent=2, ensure_ascii=False)
        
        return filename


class ForceCalibrationNode(Node):
    """力传感器标定节点"""
    def __init__(self, state: CalibrationState, topic: str):
        super().__init__('force_sensor_calibration_helper')
        self.state = state
        self.topic = topic
        
        # 订阅力传感器话题
        self.sub = self.create_subscription(
            Float32,
            topic,
            self.force_callback,
            10
        )
        
        self.get_logger().info(f'已订阅话题: {topic}')
        self.get_logger().info('输入命令: d (记录传感器) | 数字 (实际力值) | cal | list | save | help | q')
    
    def force_callback(self, msg: Float32):
        """接收传感器数据"""
        with self.state.lock:
            self.state.latest_force = msg.data


def interactive_loop(state: CalibrationState, node: ForceCalibrationNode, device_id: int, topic: str):
    """交互式标定循环"""
    print('\n' + '='*70)
    print('  力传感器标定工具 (Force Sensor Calibration)')
    print('='*70)
    print(f'  设备ID: {device_id}')
    print(f'  话题:   {topic}')
    print('='*70)
    print('\n命令说明:')
    print('  d       - 记录当前传感器读数(Data)')
    print('  <数字>  - 输入实际施加的标准力值(N)')
    print('  cal     - 计算线性拟合参数')
    print('  list    - 列出所有已记录样本')
    print('  help    - 显示帮助信息')
    print('  q/quit  - 退出并显示标定结果')
    print('\n标定流程:')
    print('  1. 施加已知标准力(如砝码)')
    print('  2. 输入 d 记录当前传感器读数')
    print('  3. 输入实际力值(单位: N)')
    print('  4. 重复1-3收集多组数据(建议≥3组)')
    print('  5. 输入 cal 查看拟合结果')
    print('  6. 输入 q 退出并显示标定摘要')
    print('\n重要提示:')
    print('  - 标定前请先使用 force_sensor_tare.py 对传感器去皮')
    print('  - 拟合算法强制过原点 (offset=0)，仅计算 scale 系数')
    print('  - 公式: actual_force = sensor_reading × scale')
    print('='*70 + '\n')
    
    while rclpy.ok():
        try:
            line = input('> ').strip()
        except EOFError:
            break
        
        if not line:
            continue
        
        low = line.lower()
        
        # 退出
        if low in ('quit', 'q', 'exit'):
            # 显示最终标定结果
            summary = state.get_calibration_summary(device_id, topic)
            print(summary)
            print('\n退出标定程序。')
            break
        
        # 帮助
        if low in ('help', 'h', '?'):
            print('\n标定步骤:')
            print('  1. 输入 d → 记录当前传感器读数')
            print('  2. 输入实际力值 → 完成一组样本')
            print('  3. 重复收集多组数据')
            print('  4. 输入 cal → 计算拟合参数')
            print('  5. 输入 q → 退出并显示结果摘要\n')
            continue
        
        # 列出样本
        if low == 'list':
            with state.lock:
                if not state.samples:
                    print('  还没有样本数据')
                    continue
                print(f'\n已记录 {len(state.samples)} 组样本:')
                print(f"{'#':<4} {'传感器读数':<15} {'实际力值(N)':<15} {'时间':<12}")
                print('-' * 50)
                for i, s in enumerate(state.samples, 1):
                    ts_str = time.strftime('%H:%M:%S', time.localtime(s.timestamp))
                    print(f"{i:<4} {s.sensor_reading:<15.3f} {s.actual_force:<15.3f} {ts_str:<12}")
                print()
            continue
        
        # 计算拟合
        if low == 'cal':
            try:
                scale, offset = state.fit()
                print('\n' + '='*60)
                print('  线性拟合结果 (强制过原点):')
                print('='*60)
                print(f'  actual_force = sensor_reading × {scale:.6f}')
                print(f'  (offset = 0, 已去皮)')
                print('='*60)
                
                # 显示当前传感器读数的估计值
                with state.lock:
                    current = state.latest_force
                if current is not None:
                    estimated = current * scale + offset
                    print(f'\n  当前传感器读数: {current:.3f}')
                    print(f'  估计实际力值:   {estimated:.3f} N')
                
                # 计算拟合误差
                with state.lock:
                    samples = state.samples.copy()
                print(f'\n  拟合误差统计 (共{len(samples)}组):')
                errors = []
                for s in samples:
                    predicted = s.sensor_reading * scale + offset
                    error = predicted - s.actual_force
                    errors.append(error)
                
                if errors:
                    import statistics
                    print(f'    平均误差: {statistics.mean(errors):.3f} N')
                    print(f'    标准差:   {statistics.stdev(errors) if len(errors) > 1 else 0:.3f} N')
                    print(f'    最大误差: {max(errors, key=abs):.3f} N')
                print()
                
            except Exception as e:
                print(f'  ✗ 拟合失败: {e}\n')
            continue
        
        # 记录传感器读数
        if low == 'd' or low == 'data':
            sensor_val = state.record_sensor_reading()
            if sensor_val is None:
                print('  ✗ 还没有接收到传感器数据，请稍候重试\n')
            else:
                print(f'  ✓ 已记录传感器读数: {sensor_val:.3f}')
                print(f'  → 请输入实际施加的标准力值 (单位: N): ', end='')
            continue
        
        # 尝试解析为数字 (实际力值)
        try:
            actual_force = float(line)
            if state.pending_sensor_reading is None:
                print('  ✗ 请先输入 d 记录传感器读数\n')
            else:
                success = state.add_sample(actual_force)
                if success:
                    print(f'  ✓ 样本已添加: 传感器={state.samples[-1].sensor_reading:.3f}, 实际力={actual_force:.3f} N')
                    print(f'  当前共 {len(state.samples)} 组样本\n')
                else:
                    print('  ✗ 添加样本失败\n')
        except ValueError:
            print(f'  ✗ 未知命令: {line}')
            print('  提示: 输入 help 查看帮助\n')


def main():
    parser = argparse.ArgumentParser(
        description='力传感器交互式标定工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
示例:
  # 标定 force_sensor_2 (device_id=53)
  python3 scripts/force_sensor_calibrate.py 53
  
  # 标定主力传感器 (device_id=52)
  python3 scripts/force_sensor_calibrate.py 52
  
  # 标定测试传感器 (device_id=60)
  python3 scripts/force_sensor_calibrate.py 60
  
  # 或使用完整参数形式
  python3 scripts/force_sensor_calibrate.py --device_id 53
        """
    )
    parser.add_argument(
        'device_id',
        type=int,
        nargs='?',
        help='力传感器设备ID (52, 53, 或 60)'
    )
    parser.add_argument(
        '--device_id',
        dest='device_id_flag',
        type=int,
        help='力传感器设备ID (替代位置参数)'
    )
    
    args = parser.parse_args()
    
    # 优先使用位置参数，其次使用--device_id标志
    device_id = args.device_id if args.device_id is not None else args.device_id_flag
    
    if device_id is None:
        print('\n错误: 缺少必需参数 device_id')
        print('\n用法:')
        print('  python3 scripts/force_sensor_calibrate.py <device_id>')
        print('\n示例:')
        print('  python3 scripts/force_sensor_calibrate.py 53  # force_sensor_2')
        print('  python3 scripts/force_sensor_calibrate.py 52  # force_sensor')
        print('  python3 scripts/force_sensor_calibrate.py 60  # force_sensor_test')
        print('\n支持的设备ID:')
        print('  52 - 主力传感器 (/force_sensor)')
        print('  53 - 第二力传感器 (/force_sensor_2)')
        print('  60 - 测试力传感器 (/force_sensor_test)')
        print()
        sys.exit(1)
    
    # 根据device_id确定话题
    topic_map = {
        52: '/force_sensor',
        53: '/force_sensor_2',
        60: '/force_sensor_test'
    }
    
    topic = topic_map.get(device_id, f'/force_sensor_{device_id}')
    
    rclpy.init()
    
    state = CalibrationState()
    node = ForceCalibrationNode(state, topic)
    
    # 在后台线程运行ROS2 spin
    spin_thread = threading.Thread(target=lambda: rclpy.spin(node), daemon=True)
    spin_thread.start()
    
    # 等待一下确保订阅建立
    time.sleep(0.5)
    
    try:
        interactive_loop(state, node, device_id, topic)
    except KeyboardInterrupt:
        print('\n\n用户中断')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
