#!/usr/bin/env python3
"""
ROS2 Timer Accuracy Test Script
测试 ROS2 create_timer 调度的实际频率与稳定性。

原理:
- 使用 ROS2 自身的定时器 (create_timer) 按设定间隔调用回调, 在回调中自增计数并记录时间戳。
- 外层用标准 Python 的 time.perf_counter() + while 循环 + sleep 控制测试总时长 (非 ROS2 机制), 模拟“标准延时开关”。
- 结束后统计: 实际调用次数、预期调用次数、丢失/超额、每次回调间隔的最小/最大/平均/标准差、偏移漂移。

使用示例:
    python ros2_timer_accuracy_test.py --interval 0.02 --duration 10

可选参数:
    --interval   定时器间隔 (秒), 默认 0.02 (≈50Hz)
    --duration   测试总时长 (秒), 默认 5
    --warmup     忽略开头若干次回调 (避免初始化抖动), 默认 2
    --print-intervals  是否打印前若干和后若干间隔样本
    --head       打印前 N 个间隔 (默认 5)
    --tail       打印后 N 个间隔 (默认 5)
    --csv        输出全部间隔到指定 CSV 文件 (两列: index, interval_sec)
    --no-color   关闭彩色输出

注意事项:
1. 建议在已 source 的 ROS2 工作环境下运行 (例如: source install/local_setup.bash)。
2. 实际精度会受系统负载、调度延迟、Python GIL、ROS2 Executor 行为影响。
3. 对于更严格的实时需求请使用 rclcpp + RT 内核或 DDS QoS 调优。

结果解读:
- expected_count = duration / interval (理论上限, 不一定是整数, 用 round 或 floor 比较)。
- drift = (最后一次时间戳 - 第一次时间戳) - ( (count-1) * interval )。
  drift 接近 0 表示整体累积误差小; 正值说明总体慢了 (周期略大), 负值说明总体快了。
- jitter/std: 越小定时器越稳定。可与 interval 比值 (std/interval) 评估相对抖动。
"""
import rclpy
from rclpy.node import Node
import time
import argparse
import statistics
import sys
from typing import List

try:
    from colorama import Fore, Style, init as colorama_init
    COLORAMA_AVAILABLE = True
except ImportError:
    COLORAMA_AVAILABLE = False

class TimerAccuracyNode(Node):
    def __init__(self, interval_sec: float):
        super().__init__('timer_accuracy_test')
        self.interval = interval_sec
        self.count = 0
        self.timestamps: List[int] = []  # nanoseconds from ROS clock
        self.monotonic_times: List[float] = []  # perf_counter for cross-check
        self.timer = self.create_timer(self.interval, self._on_timer)

    def _on_timer(self):
        now_ros = self.get_clock().now().nanoseconds
        now_mono = time.perf_counter()
        self.count += 1
        self.timestamps.append(now_ros)
        self.monotonic_times.append(now_mono)

    def get_intervals(self) -> List[float]:
        if len(self.timestamps) < 2:
            return []
        intervals = []
        prev = self.timestamps[0]
        for ts in self.timestamps[1:]:
            intervals.append((ts - prev) / 1e9)
            prev = ts
        return intervals

    def get_monotonic_intervals(self) -> List[float]:
        if len(self.monotonic_times) < 2:
            return []
        intervals = []
        prev = self.monotonic_times[0]
        for t in self.monotonic_times[1:]:
            intervals.append(t - prev)
            prev = t
        return intervals


def fmt_color(text: str, color: str, use_color: bool) -> str:
    if not use_color or not COLORAMA_AVAILABLE:
        return text
    return getattr(Fore, color.upper(), '') + text + Style.RESET_ALL


def compute_stats(intervals: List[float]):
    if not intervals:
        return {
            'count': 0,
            'min': None,
            'max': None,
            'mean': None,
            'stdev': None
        }
    return {
        'count': len(intervals),
        'min': min(intervals),
        'max': max(intervals),
        'mean': statistics.mean(intervals),
        'stdev': statistics.pstdev(intervals)  # population stdev
    }


def main():
    parser = argparse.ArgumentParser(description='ROS2 Timer Accuracy Test')
    parser.add_argument('--interval', type=float, default=0.02, help='Timer interval seconds (default 0.02)')
    parser.add_argument('--duration', type=float, default=5.0, help='Total test duration seconds (default 5)')
    parser.add_argument('--warmup', type=int, default=2, help='Ignore first N callbacks for stats (default 2)')
    parser.add_argument('--print-intervals', action='store_true', help='Print head/tail interval samples')
    parser.add_argument('--head', type=int, default=5, help='Number of head intervals to print (default 5)')
    parser.add_argument('--tail', type=int, default=5, help='Number of tail intervals to print (default 5)')
    parser.add_argument('--csv', type=str, default='', help='Write all intervals to CSV file')
    parser.add_argument('--no-color', action='store_true', help='Disable colored output')
    args = parser.parse_args()

    use_color = (not args.no_color) and COLORAMA_AVAILABLE
    if COLORAMA_AVAILABLE:
        colorama_init()

    rclpy.init()
    node = TimerAccuracyNode(args.interval)

    start_perf = time.perf_counter()
    # 主循环: 使用 spin_once 驱动 ROS 回调 + 外层时间裁剪
    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=min(args.interval, 0.005))
            if time.perf_counter() - start_perf >= args.duration:
                break
    except KeyboardInterrupt:
        print(fmt_color('Interrupted by user.', 'yellow', use_color))
    finally:
        rclpy.shutdown()

    total_callbacks = node.count
    raw_intervals_ros = node.get_intervals()
    raw_intervals_mono = node.get_monotonic_intervals()

    # 应用 warmup 忽略前 N 个间隔 (注意间隔数量比回调次数少 1)
    intervals_ros = raw_intervals_ros[args.warmup:] if len(raw_intervals_ros) > args.warmup else raw_intervals_ros
    intervals_mono = raw_intervals_mono[args.warmup:] if len(raw_intervals_mono) > args.warmup else raw_intervals_mono

    stats_ros = compute_stats(intervals_ros)
    stats_mono = compute_stats(intervals_mono)

    expected_count_float = args.duration / args.interval
    expected_count_round = round(expected_count_float)
    expected_count_floor = int(expected_count_float)

    # 漂移 (ROS 时间)
    if len(node.timestamps) >= 2:
        total_elapsed_ros = (node.timestamps[-1] - node.timestamps[0]) / 1e9
        ideal_elapsed_ros = (total_callbacks - 1) * args.interval
        drift_ros = total_elapsed_ros - ideal_elapsed_ros
    else:
        total_elapsed_ros = 0.0
        drift_ros = 0.0

    # 漂移 (Monotonic 系统时间)
    if len(node.monotonic_times) >= 2:
        total_elapsed_mono = node.monotonic_times[-1] - node.monotonic_times[0]
        ideal_elapsed_mono = (total_callbacks - 1) * args.interval
        drift_mono = total_elapsed_mono - ideal_elapsed_mono
    else:
        total_elapsed_mono = 0.0
        drift_mono = 0.0

    print('\n' + fmt_color('=== ROS2 Timer Accuracy Report ===', 'cyan', use_color))
    print(f"Timer interval (requested): {args.interval:.6f} s")
    print(f"Test duration (requested): {args.duration:.3f} s")
    print(f"Warmup ignored intervals: {args.warmup}")

    print(fmt_color('\nCounts:', 'green', use_color))
    print(f"  Total callbacks: {total_callbacks}")
    print(f"  Expected (float): {expected_count_float:.2f}")
    print(f"  Expected (round): {expected_count_round}")
    print(f"  Expected (floor): {expected_count_floor}")
    print(f"  Difference (callbacks - round): {total_callbacks - expected_count_round}")
    print(f"  Difference (callbacks - floor): {total_callbacks - expected_count_floor}")

    def print_stats(label: str, stats: dict, drift: float, total_elapsed: float):
        print(fmt_color(f"\n{label} intervals stats:", 'magenta', use_color))
        if stats['count'] == 0:
            print('  (insufficient data)')
            return
        print(f"  Samples: {stats['count']}")
        print(f"  Min: {stats['min']:.6f} s")
        print(f"  Max: {stats['max']:.6f} s")
        print(f"  Mean: {stats['mean']:.6f} s (diff from target: {stats['mean'] - args.interval:+.6f} s)")
        print(f"  Std Dev: {stats['stdev']:.6f} s (relative: {(stats['stdev']/args.interval) if args.interval else 0:.3%})")
        print(f"  Total elapsed (first->last): {total_elapsed:.6f} s")
        print(f"  Ideal elapsed ((n-1)*interval): {( (stats['count'] + args.warmup) * args.interval ):.6f} s")
        print(f"  Drift: {drift:+.6f} s")

    print_stats('ROS Clock', stats_ros, drift_ros, total_elapsed_ros)
    print_stats('Monotonic Clock', stats_mono, drift_mono, total_elapsed_mono)

    if args.print_intervals and intervals_ros:
        print(fmt_color('\nSample intervals (ROS clock):', 'yellow', use_color))
        head_list = intervals_ros[:args.head]
        tail_list = intervals_ros[-args.tail:] if args.tail > 0 else []
        for i, v in enumerate(head_list):
            print(f"  [HEAD {i}] {v:.6f} s")
        if tail_list:
            for i, v in enumerate(tail_list):
                print(f"  [TAIL {-len(tail_list)+i}] {v:.6f} s")

    if args.csv and intervals_ros:
        try:
            with open(args.csv, 'w', encoding='utf-8') as f:
                f.write('index,interval_ros_sec\n')
                for i, v in enumerate(intervals_ros):
                    f.write(f'{i},{v:.9f}\n')
            print(fmt_color(f"\nCSV written: {args.csv}", 'cyan', use_color))
        except Exception as e:
            print(fmt_color(f"Failed to write CSV: {e}", 'red', use_color))

    print('\n' + fmt_color('Done.', 'green', use_color))

if __name__ == '__main__':
    main()
