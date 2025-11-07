#!/usr/bin/env python3
"""力 / 拉升传感器 / 测试力传感器实时可视化脚本（仅绝对时间模式）

精简功能:
    - 订阅 /force_sensor (Float32)、/draw_wire_sensor/data (String JSON) 和 /force_sensor_test (Float32)。
    - X 轴直接使用真实时间戳 time.time()，显示最近 --x-span 秒的数据。
    - 只用折线连接，不再提供样条/动态跨度/simple-x 其他模式。
    - 可选 4 点去极值滤波 (--filter 默认启用, --no-filter 关闭)。
      * 滤波在数据采集时完成：维护最近4个原始值队列，新数据到达时立即滤波后存储
      * 仅对模拟力传感器进行滤波，拉绳传感器（数字信号）不滤波
      * 已显示的曲线不会再变化，保证稳定性
    - 支持自定义力/高度/测试力显示范围与横轴刻度。
    - 三条曲线：力传感器(黄色)、高度(绿色)、测试力传感器(紫红色)

示例:
    source install/setup.bash
    # 默认高度范围 1184 ~ 1186，测试力范围 45 ~ 120N
    python3 scripts/force_sensor_visualizer.py --x-span 3 --x-tick 0.1 --force-min 450 --force-max 900

关闭滤波:
    python3 scripts/force_sensor_visualizer.py --no-filter

按键: q 退出
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32, String
import json
import argparse, time
try:
    import cv2
    import numpy as np
except Exception:
    cv2 = None
    np = None

class ForceVisualizer(Node):
    def __init__(self, args):
        super().__init__('force_sensor_visualizer')
        self.args = args
        self.force_value = None
        self.height_value = None
        self.force_test_value = None  # Test force sensor value
        self.force_history = []  # (t, force_filtered) - 存储滤波后的值
        self.height_history = [] # (t, height_raw) - 拉绳传感器数字信号，直接存储原始值
        self.force_test_history = []  # (t, force_test_filtered) - 存储滤波后的值
        # 滤波用的原始值队列（最近4个）- 仅用于模拟力传感器
        self.force_raw_queue = []  # 最近4个原始力值
        self.force_test_raw_queue = []  # 最近4个原始测试力值
        self.height_sample_intervals = []  # for height rate estimation
        self.force_test_sample_intervals = []  # for test force rate estimation
        self.window_sec = args.window_sec
        self.intervals = []
        self.last_sample_time = None
        self.last_test_sample_time = None
        self.width = int(args.width)
        self.panel_h = int(args.plot_height)
        self.margin_left = 65
        self.margin_right = 120  # 增加右侧边距以容纳两组刻度
        self.margin_top = 40
        self.margin_bottom = 45
        self.height = self.margin_top + self.margin_bottom + self.panel_h
        # 显示范围（可由参数覆盖）
        self.force_min = args.force_min
        self.force_max = args.force_max
        self.lift_min = args.lift_min
        self.lift_max = args.lift_max
        self.force_test_min = args.force_test_min
        self.force_test_max = args.force_test_max
        # 仅保留绝对时间窗口模式
        self.x_span = args.x_span
        # Subscriptions
        self.force_sub = self.create_subscription(Float32, '/force_sensor', self.force_cb, 50)
        self.height_sub = self.create_subscription(String, '/draw_wire_sensor/data', self.height_cb, 50)
        self.force_test_sub = self.create_subscription(Float32, '/force_sensor_test', self.force_test_cb, 50)
        # Draw at 50 Hz for lower perceived latency
        self.draw_timer = self.create_timer(1.0/50.0, self.draw)

        if cv2 is None:
            self.get_logger().error('OpenCV 不可用，无法显示窗口。请安装 opencv-python')
        else:
            self.get_logger().info('ForceVisualizer 已启动, 按 q 退出')

    def _filter_value(self, raw_queue, new_value):
        """使用最近4个值进行去极值滤波，返回滤波后的值。
        
        Args:
            raw_queue: 原始值队列（会被修改）
            new_value: 新到达的原始值
            
        Returns:
            滤波后的值（如果队列不足4个，返回原始值）
        """
        if not self.args.filter:
            # 不开启滤波，直接返回原始值
            return new_value
        
        # 添加新值到队列
        raw_queue.append(new_value)
        # 保持队列最多4个元素
        if len(raw_queue) > 4:
            raw_queue.pop(0)
        
        # 如果少于4个值，返回原始值
        if len(raw_queue) < 4:
            return new_value
        
        # 4点去极值滤波：排序后取中间两个的平均
        sorted_vals = sorted(raw_queue)
        filtered = (sorted_vals[1] + sorted_vals[2]) / 2.0
        return filtered

    def force_cb(self, msg: Float32):
        self.force_value = float(msg.data)
        self._append_force()

    def height_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            h = data.get('height')
            if h is not None:
                self.height_value = float(h)
                self._append_height()
        except Exception:
            pass

    def force_test_cb(self, msg: Float32):
        self.force_test_value = float(msg.data)
        self._append_force_test()

    def _append_force(self):
        if self.force_value is None:
            return
        now = time.time()
        # 使用滤波函数处理新值
        filtered_value = self._filter_value(self.force_raw_queue, self.force_value)
        # 存储滤波后的值
        self.force_history.append((now, filtered_value))
        cutoff = now - self.window_sec
        while self.force_history and self.force_history[0][0] < cutoff:
            self.force_history.pop(0)
        if self.last_sample_time is not None:
            dt = now - self.last_sample_time
            if 0 < dt < 1.0:
                self.intervals.append(dt)
                if len(self.intervals) > 200:
                    self.intervals.pop(0)
        self.last_sample_time = now

    def _append_height(self):
        if self.height_value is None:
            return
        now = time.time()
        if self.height_history:
            dt = now - self.height_history[-1][0]
            if 0 < dt < 1.0:
                self.height_sample_intervals.append(dt)
                if len(self.height_sample_intervals) > 200:
                    self.height_sample_intervals.pop(0)
        # 拉绳传感器是数字信号，不需要滤波，直接使用原始值
        self.height_history.append((now, self.height_value))
        cutoff = now - self.window_sec
        while self.height_history and self.height_history[0][0] < cutoff:
            self.height_history.pop(0)

    def _append_force_test(self):
        if self.force_test_value is None:
            return
        now = time.time()
        if self.last_test_sample_time is not None:
            dt = now - self.last_test_sample_time
            if 0 < dt < 1.0:
                self.force_test_sample_intervals.append(dt)
                if len(self.force_test_sample_intervals) > 200:
                    self.force_test_sample_intervals.pop(0)
        self.last_test_sample_time = now
        # 使用滤波函数处理新值
        filtered_value = self._filter_value(self.force_test_raw_queue, self.force_test_value)
        # 存储滤波后的值
        self.force_test_history.append((now, filtered_value))
        cutoff = now - self.window_sec
        while self.force_test_history and self.force_test_history[0][0] < cutoff:
            self.force_test_history.pop(0)

    def _rate(self):
        if not self.intervals:
            return 0.0
        avg = sum(self.intervals)/len(self.intervals)
        return 0.0 if avg <= 0 else 1.0/avg

    def draw(self):
        if cv2 is None or np is None:
            return
        if len(self.force_history) < 5:
            return
        w, h = self.width, self.height
        img = np.zeros((h,w,3), dtype=np.uint8); img[:] = (25,25,25)

        # Force samples (已经是滤波后的值)
        times_full = np.array([t for t,_ in self.force_history])
        force_vals_full = np.array([v for _,v in self.force_history])
        height_times_full = np.array([t for t,_ in self.height_history]) if self.height_history else np.array([])
        height_vals_full = np.array([v for _,v in self.height_history]) if self.height_history else np.array([])
        force_test_times_full = np.array([t for t,_ in self.force_test_history]) if self.force_test_history else np.array([])
        force_test_vals_full = np.array([v for _,v in self.force_test_history]) if self.force_test_history else np.array([])

        # 绝对时间模式：最近 self.x_span 秒
        now = time.time()
        span = self.x_span if (self.x_span and self.x_span>0) else 2.0
        t_min = now - span
        mask_force = (times_full >= t_min)
        times = times_full[mask_force]
        force_series = force_vals_full[mask_force]  # 已经是滤波后的数据
        mask_height = (height_times_full >= t_min) if len(height_times_full)>0 else np.array([])
        height_times = height_times_full[mask_height] if len(height_times_full)>0 else np.array([])
        height_series = height_vals_full[mask_height] if len(height_vals_full)>0 else np.array([])  # 已经是滤波后的数据
        mask_force_test = (force_test_times_full >= t_min) if len(force_test_times_full)>0 else np.array([])
        force_test_times = force_test_times_full[mask_force_test] if len(force_test_times_full)>0 else np.array([])
        force_test_series = force_test_vals_full[mask_force_test] if len(force_test_vals_full)>0 else np.array([])  # 已经是滤波后的数据
        if len(times) < 2:
            return
        usable_w = (w - self.margin_left - self.margin_right)
        x = ((times - t_min)/span)*usable_w + self.margin_left
        x_h_base = ((height_times - t_min)/span)*usable_w + self.margin_left if len(height_times)>0 else np.array([])
        x_test_base = ((force_test_times - t_min)/span)*usable_w + self.margin_left if len(force_test_times)>0 else np.array([])
        # 使用固定力范围，不再依据自动最大值
        max_force = self.force_max
        min_force = self.force_min
        panel_top = self.margin_top; panel_bottom = panel_top + self.panel_h
        cv2.rectangle(img,(self.margin_left-50,panel_top),(w-self.margin_right,panel_bottom),(50,50,50),1)
        # grid
        for g in range(6):
            gy = int(panel_top + 10 + g*(self.panel_h-40)/5)
            cv2.line(img,(self.margin_left-49,gy),(w-self.margin_right,gy),(45,45,45),1)
        # y ticks
        y_ticks = self.args.y_ticks if (hasattr(self.args,'y_ticks') and self.args.y_ticks and self.args.y_ticks>0) else 5
        for i in range(y_ticks+1):
            val = min_force + (i/y_ticks)*(max_force - min_force)
            norm = (val - min_force)/(max_force - min_force)
            y = int(panel_bottom - 20 - norm*(self.panel_h-40))
            cv2.line(img,(self.margin_left-52,y),(self.margin_left-48,y),(180,180,180),1)
            cv2.putText(img,f"{val:.1f}N",(self.margin_left-62,y+4),cv2.FONT_HERSHEY_SIMPLEX,0.35,(180,180,180),1)
        # 数据已经在采集时滤波，直接使用
        force_clipped = np.clip(force_series, min_force, max_force)
        y_force = (panel_bottom - 20) - ((force_clipped - min_force)/(max_force - min_force))*(self.panel_h-40)
        # 高度曲线映射到同一 panel 使用右侧刻度
        if len(height_series) > 0:
            height_clipped = np.clip(height_series, self.lift_min, self.lift_max)
            y_height = (panel_bottom - 20) - ((height_clipped - self.lift_min)/(self.lift_max - self.lift_min))*(self.panel_h-40)
        # 测试力传感器曲线映射（使用独立范围 45-120N，映射到相同 panel）
        if len(force_test_series) > 0:
            force_test_clipped = np.clip(force_test_series, self.force_test_min, self.force_test_max)
            y_force_test = (panel_bottom - 20) - ((force_test_clipped - self.force_test_min)/(self.force_test_max - self.force_test_min))*(self.panel_h-40)
        # 曲线绘制：绝对时间模式 & simple_x 模式下不做样条，仅使用原始折线
        if len(x) >= 2:
            for i in range(1,len(x)):
                cv2.line(img,(int(x[i-1]),int(y_force[i-1])),(int(x[i]),int(y_force[i])),(255,255,0),2)
        # 高度曲线 (简单折线，不做样条)
        if len(height_series) > 1:
            x_h = x_h_base
            height_clipped_plot = np.clip(height_series, self.lift_min, self.lift_max)
            y_h_plot = (panel_bottom - 20) - ((height_clipped_plot - self.lift_min)/(self.lift_max - self.lift_min))*(self.panel_h-40)
            for i in range(1,len(x_h)):
                cv2.line(img,(int(x_h[i-1]),int(y_h_plot[i-1])),(int(x_h[i]),int(y_h_plot[i])),(0,200,100),2)
        # 测试力传感器曲线 (简单折线，紫红色)
        if len(force_test_series) > 1:
            x_test = x_test_base
            for i in range(1,len(x_test)):
                cv2.line(img,(int(x_test[i-1]),int(y_force_test[i-1])),(int(x_test[i]),int(y_force_test[i])),(255,100,255),2)
        # 原始测量点（用小圆点标记）
        for i in range(len(x)):
            color = (0,255,255) if (force_series[i] < min_force or force_series[i] > max_force) else (200,200,50)
            cv2.circle(img,(int(x[i]),int(y_force[i])),3,color,-1)
        # 测试力传感器测量点
        if len(force_test_series) > 0:
            for i in range(len(x_test_base)):
                color_test = (255,0,255) if (force_test_series[i] < self.force_test_min or force_test_series[i] > self.force_test_max) else (200,100,200)
                cv2.circle(img,(int(x_test_base[i]),int(y_force_test[i])),3,color_test,-1)
        if len(height_series)>0:
            # 高度当前值标记
            cv2.putText(img,f"Lift: {height_series[-1]:.2f}",(self.margin_left+220,panel_top+18),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,200,100),2)
        cv2.putText(img,f"Force: {force_series[-1]:.2f} N",(self.margin_left,panel_top+18),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,0),2)
        if len(force_test_series) > 0:
            cv2.putText(img,f"Test: {force_test_series[-1]:.2f} N",(self.margin_left+450,panel_top+18),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,100,255),2)
        rate = self._rate()
        if self.height_sample_intervals:
            h_avg = sum(self.height_sample_intervals)/len(self.height_sample_intervals)
            h_rate = 0 if h_avg<=0 else 1.0/h_avg
        else:
            h_rate = 0.0
        if self.force_test_sample_intervals:
            ft_avg = sum(self.force_test_sample_intervals)/len(self.force_test_sample_intervals)
            ft_rate = 0 if ft_avg<=0 else 1.0/ft_avg
        else:
            ft_rate = 0.0
        cv2.putText(img,f"span={span:.1f}s f≈{rate:.1f}Hz h≈{h_rate:.1f}Hz ft≈{ft_rate:.1f}Hz",(self.margin_left,25),cv2.FONT_HERSHEY_SIMPLEX,0.6,(220,220,220),1)
        cv2.putText(img,f"Force {min_force:.0f}-{max_force:.0f}N | Lift {self.lift_min:.0f}-{self.lift_max:.0f} | Test {self.force_test_min:.0f}-{self.force_test_max:.0f}N",(self.margin_left,h-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(170,170,170),1)
        cv2.putText(img,"Force (N)",(self.margin_left-55,panel_top-10),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,0),1)
        # 右侧高度刻度（绿色，靠右）
        for i in range(6):
            val = self.lift_min + (i/5)*(self.lift_max - self.lift_min)
            norm = (val - self.lift_min)/(self.lift_max - self.lift_min)
            y = int(panel_bottom - 20 - norm*(self.panel_h-40))
            cv2.putText(img,f"{val:.0f}",(w-self.margin_right-40,y+4),cv2.FONT_HERSHEY_SIMPLEX,0.35,(0,200,100),1)
        cv2.putText(img,"Lift",(w-self.margin_right-50,panel_top-10),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,200,100),1)
        # 右侧测试力刻度（紫色，靠右上方第二列）
        for i in range(6):
            val = self.force_test_min + (i/5)*(self.force_test_max - self.force_test_min)
            norm = (val - self.force_test_min)/(self.force_test_max - self.force_test_min)
            y = int(panel_bottom - 20 - norm*(self.panel_h-40))
            cv2.putText(img,f"{val:.0f}",(w-self.margin_right-95,y+4),cv2.FONT_HERSHEY_SIMPLEX,0.35,(255,100,255),1)
        cv2.putText(img,"Test(N)",(w-self.margin_right-110,panel_top-10),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,100,255),1)
        cv2.putText(img,"Time (s)",(w//2,panel_bottom+25),cv2.FONT_HERSHEY_SIMPLEX,0.5,(200,200,200),1)
        # x ticks
        tick = self.args.x_tick if (self.args.x_tick and self.args.x_tick>0) else 1.0
        span_ax = span if span>0 else 1.0
        n_ticks = int(span_ax/tick)+1
        for i in range(n_ticks):
            sec_abs = t_min + i*tick
            if sec_abs>now: break
            rel = (sec_abs - t_min)/span_ax
            xt = int(self.margin_left + rel*(w - self.margin_left - self.margin_right))
            cv2.line(img,(xt,panel_top-4),(xt,panel_top-1),(90,90,90),1)
            lab = f"{sec_abs%1000:.2f}" if tick<1 else f"{sec_abs%1000:.0f}"
            cv2.putText(img,lab,(xt-12,panel_top-8),cv2.FONT_HERSHEY_SIMPLEX,0.4,(160,160,160),1)

        cv2.imshow('ForceSensorVisualizer', img)
        k = cv2.waitKey(1)&0xFF
        if k == ord('q'):
            self.get_logger().info('关闭可视化窗口')
            cv2.destroyWindow('ForceSensorVisualizer')
            rclpy.shutdown()

    # 动态跨度与其他模式已移除

    def destroy_node(self):
        if cv2 is not None:
            try:
                cv2.destroyWindow('ForceSensorVisualizer')
            except Exception:
                pass
        super().destroy_node()

def parse_args():
    ap = argparse.ArgumentParser(description='Force + Lift + Test Force Triple Visualizer (Absolute Only)')
    ap.add_argument('--window-sec', type=float, default=60.0, help='保留历史窗口秒数 (默认60)')
    ap.add_argument('--width', type=int, default=1200, help='窗口宽度（默认1200以容纳更多刻度）')
    ap.add_argument('--plot-height', type=int, default=350, help='曲线区域高度')
    ap.add_argument('--x-span', type=float, default=2.0, help='横轴显示跨度秒(默认2)')
    ap.add_argument('--x-tick', type=float, default=0.05, help='X轴刻度间隔秒 (默认0.05)')
    ap.add_argument('--y-ticks', type=int, default=5, help='左侧力刻度数量 (默认5)')
    ap.add_argument('--force-min', type=float, default=450.0, help='力显示下限 (默认450)')
    ap.add_argument('--force-max', type=float, default=900.0, help='力显示上限 (默认900)')
    ap.add_argument('--lift-min', type=float, default=1184.0, help='拉升传感器显示下限 (默认1184)')
    ap.add_argument('--lift-max', type=float, default=1186.0, help='拉升传感器显示上限 (默认1186)')
    ap.add_argument('--force-test-min', type=float, default=45.0, help='测试力传感器显示下限 (默认45)')
    ap.add_argument('--force-test-max', type=float, default=120.0, help='测试力传感器显示上限 (默认120)')
    ap.add_argument('--filter', action='store_true', default=True, help='启用4点去极值滤波 (默认启用)')
    ap.add_argument('--no-filter', action='store_false', dest='filter', help='禁用滤波，显示原始值')
    # 移除其他模式参数，仅保留核心参数
    return ap.parse_args()

def main():
    args = parse_args()
    rclpy.init()
    node = ForceVisualizer(args)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
