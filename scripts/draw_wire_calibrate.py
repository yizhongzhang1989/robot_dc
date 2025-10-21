#!/usr/bin/env python3
"""
Interactive calibration tool for draw-wire sensor.
(Repository-level scripts folder version)

Workflow:
1. source install/setup.bash
2. python3 scripts/draw_wire_calibrate.py
3. Enter actual platform heights as you move it.
4. Type 'cal' to compute linear fit (height = sensor * scale + offset).
5. Type 'save' to persist calibration_draw_wire.json.
6. Type 'list', 'help', 'quit' as needed.
"""
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, threading, time
from dataclasses import dataclass
from typing import List, Optional, Tuple

RAW_FIELD = 'register_1'

@dataclass
class Sample:
    sensor: float
    height: float
    timestamp: float

class CalibrationState:
    def __init__(self):
        self.latest_msg = None
        self.samples: List[Sample] = []
        self.lock = threading.Lock()
        self.scale: Optional[float] = None
        self.offset: Optional[float] = None
    def add(self, sensor_val: float, height: float):
        with self.lock:
            self.samples.append(Sample(sensor=sensor_val, height=height, timestamp=time.time()))
    def fit(self) -> Tuple[float,float]:
        with self.lock:
            if len(self.samples) < 2:
                raise ValueError('Need >=2 samples')
            xs = [s.sensor for s in self.samples]
            ys = [s.height for s in self.samples]
        n = len(xs); mx = sum(xs)/n; my = sum(ys)/n
        num = sum((x-mx)*(y-my) for x,y in zip(xs,ys))
        den = sum((x-mx)**2 for x in xs)
        if den == 0: raise ValueError('Zero variance in sensor values')
        scale = num/den; offset = my - scale*mx
        with self.lock: self.scale, self.offset = scale, offset
        return scale, offset
    def save(self, path='calibration_draw_wire.json'):
        with self.lock:
            data = {
                'raw_field': RAW_FIELD,
                'samples': [s.__dict__ for s in self.samples],
                'scale': self.scale,
                'offset': self.offset,
                'generated_at': time.time()
            }
        with open(path,'w') as f: json.dump(data,f,indent=2)
        return path

class CalibNode(Node):
    def __init__(self,state:CalibrationState):
        super().__init__('draw_wire_calibration_helper')
        self.state = state
        self.sub = self.create_subscription(String,'/draw_wire_sensor/data',self.cb,10)
        self.get_logger().info('Subscribed to /draw_wire_sensor/data. Enter height, cal, save, list.')
    def cb(self,msg:String):
        try: self.state.latest_msg = json.loads(msg.data)
        except Exception: pass


def loop(state:CalibrationState,node:CalibNode):
    print('Commands: number height | cal | list | save | help | quit')
    while rclpy.ok():
        try: line = input('> ').strip()
        except EOFError: break
        if not line: continue
        low = line.lower()
        if low in ('quit','q','exit'): print('Bye'); break
        if low in ('help','h','?'):
            print('Enter numeric height samples, then cal for linear fit; save to write JSON; list to view.')
            continue
        if low == 'list':
            with state.lock:
                if not state.samples: print('No samples'); continue
                for i,s in enumerate(state.samples):
                    ts_str = time.strftime('%H:%M:%S', time.localtime(s.timestamp))
                    print(f"#{i+1} sensor={s.sensor} height={s.height} ts={ts_str}")
            continue
        if low == 'cal':
            try:
                scale, offset = state.fit()
                print(f'Fit: height ≈ sensor * {scale:.6f} + {offset:.6f}')
                if state.latest_msg and RAW_FIELD in state.latest_msg:
                    sv = state.latest_msg[RAW_FIELD]; est = sv*scale+offset
                    print(f'Current sensor={sv} -> estimated height={est:.3f}')
            except Exception as e: print('Calibration failed:',e)
            continue
        if low == 'save':
            path = state.save(); print('Saved to',path); continue
        # numeric
        try: h = float(line)
        except ValueError: print('Not number or command'); continue
        obj = state.latest_msg
        if not obj or RAW_FIELD not in obj:
            print('No sensor data yet'); continue
        sv = obj[RAW_FIELD]; state.add(sv,h)
        print(f'Added sample: sensor={sv} height={h} total={len(state.samples)}')


def main():
    rclpy.init()
    state = CalibrationState(); node = CalibNode(state)
    th = threading.Thread(target=lambda: rclpy.spin(node),daemon=True); th.start()
    try: loop(state,node)
    finally: node.destroy_node(); rclpy.shutdown()

if __name__ == '__main__':
    main()
