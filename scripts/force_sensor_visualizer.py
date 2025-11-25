#!/usr/bin/env python3
"""Dual Force Sensor + Draw-Wire Height Sensor Real-time Visualization Script (Absolute Time Mode, Line Plot Only)

Features:
    - Subscribe to /force_sensor (primary), /force_sensor_2 (secondary), /draw_wire_sensor/data (height)
    - Use real timestamps, display recent --x-span seconds window
    - Both force sensors support 4-point outlier filtering (--filter / --no-filter)
    - Three curves: Force1(yellow), Force2(magenta), Height(green)
    - Force display range default 35~160N, auto-select integer or decimal scale

Example:
    source install/setup.bash
    python3 scripts/force_sensor_visualizer.py --x-span 3 --x-tick 0.1

Disable filtering:
    python3 scripts/force_sensor_visualizer.py --no-filter

Hotkey: q to exit
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
        # Current values
        self.force_value = None          # Primary force sensor
        self.force2_value = None         # Secondary force sensor
        self.height_value = None         # Height sensor
        # History (t, filtered)
        self.force_history = []          # Primary force
        self.force2_history = []         # Secondary force
        self.height_history = []         # Height
        # 4-point outlier filtering queue
        self.force_raw_queue = []        # Primary force
        self.force2_raw_queue = []       # Secondary force
        # Sample rate estimation intervals
        self.force_sample_intervals = []
        self.force2_sample_intervals = []
        self.height_sample_intervals = []
        self.window_sec = args.window_sec
        self.last_primary_sample_time = None
        self.last_secondary_sample_time = None
        # Layout dimensions
        self.width = int(args.width)
        self.panel_h = int(args.plot_height)
        self.margin_left = 65
        self.margin_right = 70  # Right side for height scale only
        self.margin_top = 40
        self.margin_bottom = 45
        self.height = self.margin_top + self.margin_bottom + self.panel_h
        # Display range (can be overridden by parameters)
        self.force_min = args.force_min
        self.force_max = args.force_max
        self.lift_min = args.lift_min
        self.lift_max = args.lift_max
        # Absolute time window mode
        self.x_span = args.x_span
        # Subscriptions
        self.force_sub = self.create_subscription(Float32, '/force_sensor', self.force_cb, 50)
        self.force2_sub = self.create_subscription(Float32, '/force_sensor_2', self.force2_cb, 50)
        self.height_sub = self.create_subscription(String, '/draw_wire_sensor/data', self.height_cb, 50)
        # Draw at 50 Hz for lower perceived latency
        self.draw_timer = self.create_timer(1.0/50.0, self.draw)

        if cv2 is None:
            self.get_logger().error('OpenCV not available, cannot display window. Please install opencv-python')
        else:
            self.get_logger().info('ForceVisualizer started, press q to exit')

    def _filter_value(self, raw_queue, new_value):
        """Filter using the most recent 4 values with outlier removal, return filtered value.
        
        Args:
            raw_queue: Raw value queue (will be modified)
            new_value: Newly arrived raw value
            
        Returns:
            Filtered value (if queue has less than 4 values, return raw value)
        """
        if not self.args.filter:
            # Filter disabled, return raw value directly
            return new_value
        
        # Add new value to queue
        raw_queue.append(new_value)
        # Keep queue at most 4 elements
        if len(raw_queue) > 4:
            raw_queue.pop(0)
        
        # If less than 4 values, return raw value
        if len(raw_queue) < 4:
            return new_value
        
        # 4-point outlier filtering: sort and average middle two values
        sorted_vals = sorted(raw_queue)
        filtered = (sorted_vals[1] + sorted_vals[2]) / 2.0
        return filtered

    def height_cb(self, msg: String):
        try:
            data = json.loads(msg.data)
            h = data.get('height')
            if h is not None:
                self.height_value = float(h)
                self._append_height()
        except Exception:
            pass

    def force_cb(self, msg: Float32):
        self.force_value = float(msg.data)
        self._append_force(primary=True)

    def force2_cb(self, msg: Float32):
        self.force2_value = float(msg.data)
        self._append_force(primary=False)

    def _append_force(self, primary: bool):
        value = self.force_value if primary else self.force2_value
        if value is None:
            return
        now = time.time()
        queue = self.force_raw_queue if primary else self.force2_raw_queue
        filtered = self._filter_value(queue, value)
        history = self.force_history if primary else self.force2_history
        history.append((now, filtered))
        cutoff = now - self.window_sec
        while history and history[0][0] < cutoff:
            history.pop(0)
        # Sample rate estimation
        if primary:
            if self.last_primary_sample_time is not None:
                dt = now - self.last_primary_sample_time
                if 0 < dt < 1.0:
                    self.force_sample_intervals.append(dt)
                    if len(self.force_sample_intervals) > 200:
                        self.force_sample_intervals.pop(0)
            self.last_primary_sample_time = now
        else:
            if self.last_secondary_sample_time is not None:
                dt = now - self.last_secondary_sample_time
                if 0 < dt < 1.0:
                    self.force2_sample_intervals.append(dt)
                    if len(self.force2_sample_intervals) > 200:
                        self.force2_sample_intervals.pop(0)
            self.last_secondary_sample_time = now

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
        # Draw-wire sensor is digital signal, no filtering needed, use raw value directly
        self.height_history.append((now, self.height_value))
        cutoff = now - self.window_sec
        while self.height_history and self.height_history[0][0] < cutoff:
            self.height_history.pop(0)

    def _rate(self, intervals):
        if not intervals:
            return 0.0
        avg = sum(intervals)/len(intervals)
        return 0.0 if avg <= 0 else 1.0/avg

    def draw(self):
        if cv2 is None or np is None:
            return
        if len(self.force_history) < 5:
            return
        w, h = self.width, self.height
        img = np.zeros((h,w,3), dtype=np.uint8); img[:] = (25,25,25)

        # Force samples (already filtered values)
        times_full = np.array([t for t,_ in self.force_history])
        force_vals_full = np.array([v for _,v in self.force_history])
        height_times_full = np.array([t for t,_ in self.height_history]) if self.height_history else np.array([])
        height_vals_full = np.array([v for _,v in self.height_history]) if self.height_history else np.array([])
        force2_times_full = np.array([t for t,_ in self.force2_history]) if self.force2_history else np.array([])
        force2_vals_full = np.array([v for _,v in self.force2_history]) if self.force2_history else np.array([])

        # Absolute time mode: recent self.x_span seconds
        now = time.time()
        span = self.x_span if (self.x_span and self.x_span>0) else 2.0
        t_min = now - span
        mask_force = (times_full >= t_min)
        times = times_full[mask_force]
        force_series = force_vals_full[mask_force]  # Already filtered data
        mask_height = (height_times_full >= t_min) if len(height_times_full)>0 else np.array([])
        height_times = height_times_full[mask_height] if len(height_times_full)>0 else np.array([])
        height_series = height_vals_full[mask_height] if len(height_vals_full)>0 else np.array([])  # Already filtered data
        mask_force2 = (force2_times_full >= t_min) if len(force2_times_full)>0 else np.array([])
        force2_times = force2_times_full[mask_force2] if len(force2_times_full)>0 else np.array([])
        force2_series = force2_vals_full[mask_force2] if len(force2_vals_full)>0 else np.array([])
        if len(times) < 2:
            return
        usable_w = (w - self.margin_left - self.margin_right)
        x = ((times - t_min)/span)*usable_w + self.margin_left
        x_h_base = ((height_times - t_min)/span)*usable_w + self.margin_left if len(height_times)>0 else np.array([])
        x2_base = ((force2_times - t_min)/span)*usable_w + self.margin_left if len(force2_times)>0 else np.array([])
        # Use fixed force range, no longer based on auto-max
        max_force = self.force_max
        min_force = self.force_min
        panel_top = self.margin_top; panel_bottom = panel_top + self.panel_h
        cv2.rectangle(img,(self.margin_left-50,panel_top),(w-self.margin_right,panel_bottom),(50,50,50),1)
        # grid
        for g in range(6):
            gy = int(panel_top + 10 + g*(self.panel_h-40)/5)
            cv2.line(img,(self.margin_left-49,gy),(w-self.margin_right,gy),(45,45,45),1)
        # Y-axis ticks (auto choose integer vs one decimal based on step size)
        y_ticks = self.args.y_ticks if (hasattr(self.args,'y_ticks') and self.args.y_ticks and self.args.y_ticks>0) else 5
        step = (max_force - min_force)/y_ticks if y_ticks>0 else (max_force - min_force)
        for i in range(y_ticks+1):
            val = min_force + (i/y_ticks)*(max_force - min_force) if y_ticks>0 else min_force
            norm = (val - min_force)/(max_force - min_force) if (max_force - min_force)!=0 else 0
            y = int(panel_bottom - 20 - norm*(self.panel_h-40))
            cv2.line(img,(self.margin_left-52,y),(self.margin_left-48,y),(180,180,180),1)
            label = f"{val:.0f}N" if step >= 5 else f"{val:.1f}N"
            cv2.putText(img,label,(self.margin_left-62,y+4),cv2.FONT_HERSHEY_SIMPLEX,0.35,(180,180,180),1)
        # Data already filtered during acquisition, use directly
        force_clipped = np.clip(force_series, min_force, max_force)
        y_force = (panel_bottom - 20) - ((force_clipped - min_force)/(max_force - min_force))*(self.panel_h-40)
        # Height curve mapped to same panel using right-side scale
        if len(height_series) > 0:
            height_clipped = np.clip(height_series, self.lift_min, self.lift_max)
            y_height = (panel_bottom - 20) - ((height_clipped - self.lift_min)/(self.lift_max - self.lift_min))*(self.panel_h-40)
        # Secondary force sensor curve mapping (uses independent range 45-120N, mapped to same panel)
        if len(force2_series) > 0:
            force2_clipped = np.clip(force2_series, self.force_min, self.force_max)
            y_force2 = (panel_bottom - 20) - ((force2_clipped - self.force_min)/(self.force_max - self.force_min))*(self.panel_h-40)
        # Curve drawing: absolute time mode & simple_x mode without spline, use raw polyline only
        if len(x) >= 2:
            for i in range(1,len(x)):
                cv2.line(img,(int(x[i-1]),int(y_force[i-1])),(int(x[i]),int(y_force[i])),(255,255,0),2)
        # Height curve (simple polyline, no spline)
        if len(height_series) > 1:
            x_h = x_h_base
            height_clipped_plot = np.clip(height_series, self.lift_min, self.lift_max)
            y_h_plot = (panel_bottom - 20) - ((height_clipped_plot - self.lift_min)/(self.lift_max - self.lift_min))*(self.panel_h-40)
            for i in range(1,len(x_h)):
                cv2.line(img,(int(x_h[i-1]),int(y_h_plot[i-1])),(int(x_h[i]),int(y_h_plot[i])),(0,200,100),2)
        # Secondary force sensor curve (simple polyline, magenta)
        if len(force2_series) > 1:
            x2 = x2_base
            for i in range(1,len(x2)):
                cv2.line(img,(int(x2[i-1]),int(y_force2[i-1])),(int(x2[i]),int(y_force2[i])),(255,100,255),2)
        # Raw measurement points (marked with small circles)
        for i in range(len(x)):
            color = (0,255,255) if (force_series[i] < min_force or force_series[i] > max_force) else (200,200,50)
            cv2.circle(img,(int(x[i]),int(y_force[i])),3,color,-1)
        # Secondary force sensor measurement points
        if len(force2_series) > 0:
            for i in range(len(x2_base)):
                color2 = (255,0,255) if (force2_series[i] < self.force_min or force2_series[i] > self.force_max) else (200,100,200)
                cv2.circle(img,(int(x2_base[i]),int(y_force2[i])),3,color2,-1)
        if len(height_series)>0:
            # Height current value marker
            cv2.putText(img,f"Lift: {height_series[-1]:.2f}",(self.margin_left+220,panel_top+18),cv2.FONT_HERSHEY_SIMPLEX,0.7,(0,200,100),2)
        cv2.putText(img,f"Force: {force_series[-1]:.2f} N",(self.margin_left,panel_top+18),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,255,0),2)
        if len(force2_series) > 0:
            cv2.putText(img,f"Force2: {force2_series[-1]:.2f} N",(self.margin_left+320,panel_top+18),cv2.FONT_HERSHEY_SIMPLEX,0.7,(255,100,255),2)
        f1_rate = self._rate(self.force_sample_intervals)
        f2_rate = self._rate(self.force2_sample_intervals)
        if self.height_sample_intervals:
            h_avg = sum(self.height_sample_intervals)/len(self.height_sample_intervals)
            h_rate = 0 if h_avg<=0 else 1.0/h_avg
        else:
            h_rate = 0.0
        cv2.putText(img,f"span={span:.1f}s f1≈{f1_rate:.1f}Hz f2≈{f2_rate:.1f}Hz h≈{h_rate:.1f}Hz",(self.margin_left,25),cv2.FONT_HERSHEY_SIMPLEX,0.6,(220,220,220),1)
        cv2.putText(img,f"Force(1/2) {min_force:.0f}-{max_force:.0f}N | Lift {self.lift_min:.0f}-{self.lift_max:.0f}",(self.margin_left,h-15),cv2.FONT_HERSHEY_SIMPLEX,0.5,(170,170,170),1)
        cv2.putText(img,"Force (N)",(self.margin_left-55,panel_top-10),cv2.FONT_HERSHEY_SIMPLEX,0.45,(255,255,0),1)
        # Right-side height scale (green, right-aligned)
        for i in range(6):
            val = self.lift_min + (i/5)*(self.lift_max - self.lift_min)
            norm = (val - self.lift_min)/(self.lift_max - self.lift_min)
            y = int(panel_bottom - 20 - norm*(self.panel_h-40))
            cv2.putText(img,f"{val:.0f}",(w-self.margin_right-40,y+4),cv2.FONT_HERSHEY_SIMPLEX,0.35,(0,200,100),1)
        cv2.putText(img,"Lift",(w-self.margin_right-50,panel_top-10),cv2.FONT_HERSHEY_SIMPLEX,0.45,(0,200,100),1)
        # Remove secondary force scale; secondary force shares left scale with primary force
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
            self.get_logger().info('Closing visualization window')
            cv2.destroyWindow('ForceSensorVisualizer')
            rclpy.shutdown()

    # Dynamic span and other modes have been removed

    def destroy_node(self):
        if cv2 is not None:
            try:
                cv2.destroyWindow('ForceSensorVisualizer')
            except Exception:
                pass
        super().destroy_node()

def parse_args():
    ap = argparse.ArgumentParser(description='Dual Force Sensor + Height Visualization (Absolute Time Window)')
    ap.add_argument('--window-sec', type=float, default=60.0, help='History window seconds (default 60)')
    ap.add_argument('--width', type=int, default=1200, help='Window width (default 1200)')
    ap.add_argument('--plot-height', type=int, default=350, help='Plot area height (default 350)')
    ap.add_argument('--x-span', type=float, default=2.0, help='X-axis display span seconds (default 2)')
    ap.add_argument('--x-tick', type=float, default=0.05, help='X-axis tick interval seconds (default 0.05)')
    ap.add_argument('--y-ticks', type=int, default=5, help='Force tick count (default 5)')
    ap.add_argument('--force-min', type=float, default=35.0, help='Force display lower limit (default 35)')
    ap.add_argument('--force-max', type=float, default=160.0, help='Force display upper limit (default 160)')
    ap.add_argument('--lift-min', type=float, default=1184.0, help='Height display lower limit (default 1184)')
    ap.add_argument('--lift-max', type=float, default=1186.0, help='Height display upper limit (default 1186)')
    ap.add_argument('--filter', action='store_true', default=True, help='Enable 4-point outlier filtering (default enabled)')
    ap.add_argument('--no-filter', action='store_false', dest='filter', help='Disable filtering, show raw values')
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
