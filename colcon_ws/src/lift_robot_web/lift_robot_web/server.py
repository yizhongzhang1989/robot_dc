#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import json, asyncio, threading, os, time
from ament_index_python.packages import get_package_share_directory

try:
    from fastapi import FastAPI, WebSocket, WebSocketDisconnect
    from fastapi.responses import FileResponse, JSONResponse
    from fastapi.staticfiles import StaticFiles
    import uvicorn
    FASTAPI = True
except ImportError:
    FASTAPI = False

class LiftRobotWeb(Node):
    def __init__(self):
        super().__init__('lift_robot_web_node')
        self.declare_parameter('port', 8090)
        self.declare_parameter('sensor_topic', '/draw_wire_sensor/data')
        self.port = self.get_parameter('port').value
        self.sensor_topic = self.get_parameter('sensor_topic').value

        # State holders
        self.latest_raw = None
        self.latest_obj = None
        self.connections = []
        self.loop = None
        # 双通道力值：右侧力传感器 (device_id=52) 和左侧力传感器 (device_id=53)
        self.right_force_sensor = None  # /force_sensor (device_id=52)
        self.left_force_sensor = None   # /force_sensor_2 (device_id=53)
        self.combined_force_sensor = None  # 合力 (两个力传感器相加，缺失时退化为单个存在的值)
        self.last_force_update = None  # 最近力传感器更新时间戳（任一传感器）
        self.platform_status = None
        self.pushrod_status = None

        # Calibration state (draw-wire sensor)
        self.calib_samples = []  # List of {'sensor': float, 'height': float, 'timestamp': float}
        self.calib_scale = None
        self.calib_offset = None
        self.calib_lock = threading.Lock()

        # Platform overshoot calibration state
        self.overshoot_samples_up = []  # List of {'target': float, 'actual': float, 'overshoot': float, 'timestamp': float}
        self.overshoot_samples_down = []
        self.overshoot_up = None
        self.overshoot_down = None
        self.overshoot_lock = threading.Lock()

        # Force sensor calibration state (dual-channel)
        self.force_calib_samples_right = []  # device_id=52, List of {'sensor': float, 'force': float, 'timestamp': float}
        self.force_calib_samples_left = []   # device_id=53, List of {'sensor': float, 'force': float, 'timestamp': float}
        self.force_calib_scale_right = None  # Scale for right sensor (force = sensor * scale)
        self.force_calib_scale_left = None   # Scale for left sensor
        self.force_calib_lock = threading.Lock()

        # ROS interfaces
        self.sub = self.create_subscription(String, self.sensor_topic, self.sensor_cb, 10)
        # Force sensor subscriptions (Float32)
        try:
            from std_msgs.msg import Float32
            # 订阅两个力传感器话题
            self.force_sub_right = self.create_subscription(Float32, '/force_sensor', self.force_cb_right, 10)
            self.force_sub_left = self.create_subscription(Float32, '/force_sensor_2', self.force_cb_left, 10)
            self.get_logger().info("Subscribed to /force_sensor (right) and /force_sensor_2 (left)")
        except Exception as e:
            self.get_logger().warn(f"Failed to create force sensor subscriptions: {e}")
        # Status subscriptions
        self.platform_status_sub = self.create_subscription(String, '/lift_robot_platform/status', self.platform_status_cb, 10)
        self.pushrod_status_sub = self.create_subscription(String, '/lift_robot_pushrod/status', self.pushrod_status_cb, 10)
        
        self.cmd_pub = self.create_publisher(String, '/lift_robot_platform/command', 10)
        self.pushrod_cmd_pub = self.create_publisher(String, '/lift_robot_pushrod/command', 10)

        self.get_logger().info(f"Web server subscribing: {self.sensor_topic}")
        self.start_server()

    def sensor_cb(self, msg: String):
        try:
            self.latest_raw = msg.data
            try:
                self.latest_obj = json.loads(msg.data)
            except Exception:
                self.latest_obj = None
            if self.loop and self.connections:
                # Merge force values and status if available
                outbound = msg.data
                if self.latest_obj is not None:
                    try:
                        merged = dict(self.latest_obj)
                        # 添加双通道力值
                        if self.right_force_sensor is not None:
                            merged['right_force_sensor'] = self.right_force_sensor
                        if self.left_force_sensor is not None:
                            merged['left_force_sensor'] = self.left_force_sensor
                        if self.combined_force_sensor is not None:
                            merged['combined_force_sensor'] = self.combined_force_sensor
                        # 力传感器数据陈旧检测（超过2s未更新）
                        if self.last_force_update is not None:
                            if (time.time() - self.last_force_update) > 2.0:
                                merged['force_stale'] = True
                        if self.platform_status is not None:
                            merged['platform_status'] = self.platform_status
                        if self.pushrod_status is not None:
                            merged['pushrod_status'] = self.pushrod_status
                        outbound = json.dumps(merged)
                    except Exception as e:
                        self.get_logger().warn(f"Sensor data merge error: {e}")
                        outbound = msg.data  # Fallback to raw data
                asyncio.run_coroutine_threadsafe(self.broadcast(outbound), self.loop)
        except Exception as e:
            self.get_logger().error(f"Sensor callback error: {e}")
            # Continue operation

    def force_cb_right(self, msg):
        """右侧力传感器回调 (device_id=52, /force_sensor)"""
        try:
            if 0 <= msg.data <= 2000:
                self.right_force_sensor = msg.data
                self.last_force_update = time.time()
                self._update_combined_force()
            else:
                self.get_logger().warn(f"Right force out of range: {msg.data}")
        except Exception as e:
            self.get_logger().warn(f"Right force callback error: {e}")
    
    def force_cb_left(self, msg):
        """左侧力传感器回调 (device_id=53, /force_sensor_2)"""
        try:
            if 0 <= msg.data <= 2000:
                self.left_force_sensor = msg.data
                self.last_force_update = time.time()
                self._update_combined_force()
            else:
                self.get_logger().warn(f"Left force out of range: {msg.data}")
        except Exception as e:
            self.get_logger().warn(f"Left force callback error: {e}")

    def _update_combined_force(self):
        """更新合力：两个力都存在则求和；只存在一个则等于该值；都不存在为 None；防止 inf 溢出"""
        try:
            if self.right_force_sensor is not None and self.left_force_sensor is not None:
                combined = self.right_force_sensor + self.left_force_sensor
                # 防止溢出到 inf
                if combined > 4000 or combined < -1000:
                    self.get_logger().warn(f"Combined force overflow detected: {combined} (right={self.right_force_sensor}, left={self.left_force_sensor})")
                    self.combined_force_sensor = None
                else:
                    self.combined_force_sensor = combined
            elif self.right_force_sensor is not None:
                self.combined_force_sensor = self.right_force_sensor
            elif self.left_force_sensor is not None:
                self.combined_force_sensor = self.left_force_sensor
            else:
                self.combined_force_sensor = None
        except Exception as e:
            self.get_logger().warn(f"Combined force update error: {e}")
    
    def platform_status_cb(self, msg: String):
        try:
            self.platform_status = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Platform status parse error: {e}")
    
    def pushrod_status_cb(self, msg: String):
        try:
            self.pushrod_status = json.loads(msg.data)
        except Exception as e:
            self.get_logger().warn(f"Pushrod status parse error: {e}")

    async def broadcast(self, text):
        drop = []
        for ws in self.connections:
            try:
                await ws.send_text(text)
            except Exception:
                drop.append(ws)
        for ws in drop:
            if ws in self.connections:
                self.connections.remove(ws)

    def start_server(self):
        if not FASTAPI:
            self.get_logger().error('FastAPI / uvicorn not installed')
            return

        def run():
            app = FastAPI(title='Lift Robot Web')
            try:
                web_dir = os.path.join(get_package_share_directory('lift_robot_web'), 'web')
            except Exception:
                web_dir = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', 'web'))

            @app.get('/')
            def index():
                return FileResponse(os.path.join(web_dir, 'index.html'))

            @app.get('/api/latest')
            def latest():
                if self.latest_obj:
                    return self.latest_obj
                return JSONResponse({'error': 'no data'}, status_code=404)

            @app.post('/api/cmd')
            async def send_cmd(payload: dict):
                cmd = payload.get('command')
                target = payload.get('target','platform')
                duration = payload.get('duration')
                allowed = {'up','down','stop','timed_up','timed_down','stop_timed','goto_point','goto_height','force_up','force_down','reset'}
                if cmd not in allowed:
                    return JSONResponse({'error':'invalid command'}, status_code=400)
                
                # CRITICAL: Reset command sends to BOTH platform and pushrod
                if cmd == 'reset':
                    if target != 'platform':
                        return JSONResponse({'error':'reset command only valid for platform target (auto-sends to pushrod too)'}, status_code=400)
                    # Send reset to both platform and pushrod
                    reset_msg = String()
                    reset_msg.data = json.dumps({'command': 'reset'})
                    self.cmd_pub.publish(reset_msg)  # Platform
                    self.pushrod_cmd_pub.publish(reset_msg)  # Pushrod
                    return {'status':'ok','command':'reset','target':'both (platform + pushrod)'}
                
                # Timed commands only meaningful for pushrod target currently
                if cmd.startswith('timed') and target != 'pushrod':
                    return JSONResponse({'error':'timed commands only supported for pushrod target'}, status_code=400)
                if cmd == 'goto_point':
                    if target != 'pushrod':
                        return JSONResponse({'error':'goto_point only valid for pushrod target'}, status_code=400)
                    point = payload.get('point')
                    if not point:
                        return JSONResponse({'error':'point field required for goto_point'}, status_code=400)
                if cmd == 'goto_height':
                    # goto_height now supports both platform and pushrod targets
                    target_height = payload.get('target_height')
                    if target_height is None:
                        return JSONResponse({'error':'target_height field required for goto_height'}, status_code=400)
                if cmd in ('force_up','force_down'):
                    if target != 'platform':
                        return JSONResponse({'error':'force_up/force_down only valid for platform target'}, status_code=400)
                    target_force = payload.get('target_force')
                    if target_force is None:
                        return JSONResponse({'error':'target_force field required for force_up/force_down'}, status_code=400)
                    try:
                        tf = float(target_force)
                        if tf <= 0:
                            return JSONResponse({'error':'target_force must be > 0'}, status_code=400)
                    except Exception:
                        return JSONResponse({'error':'invalid target_force'}, status_code=400)
                # Auto inject 5s for timed_up if not provided
                if cmd == 'timed_up' and (duration is None):
                    duration = 3.5
                if cmd == 'timed_down' and (duration is None):
                    # Provide a default if user omits (optional design choice)
                    duration = 3.5
                if duration is not None:
                    try:
                        duration = float(duration)
                        if duration <= 0:
                            return JSONResponse({'error':'duration must be > 0'}, status_code=400)
                    except Exception:
                        return JSONResponse({'error':'invalid duration'}, status_code=400)
                body = {'command': cmd}
                if cmd == 'goto_point':
                    body['point'] = payload.get('point')
                if cmd == 'goto_height':
                    body['target_height'] = payload.get('target_height')
                if cmd in ('force_up','force_down'):
                    body['target_force'] = float(payload.get('target_force'))
                if duration is not None:
                    body['duration'] = duration
                msg = String(); msg.data = json.dumps(body)
                if target == 'pushrod':
                    self.pushrod_cmd_pub.publish(msg)
                else:
                    self.cmd_pub.publish(msg)
                return {'status':'ok','command':cmd,'target':target,'duration':duration}
            
            @app.get('/api/status')
            def get_status():
                """Get current platform and pushrod task status"""
                response = {}
                if self.platform_status:
                    response['platform'] = {
                        'task_state': self.platform_status.get('task_state', 'unknown'),
                        'task_type': self.platform_status.get('task_type'),
                        'task_start_time': self.platform_status.get('task_start_time'),
                        'task_end_time': self.platform_status.get('task_end_time'),
                        'task_duration': self.platform_status.get('task_duration'),
                        'completion_reason': self.platform_status.get('completion_reason'),
                        'control_mode': self.platform_status.get('control_mode'),
                        'movement_state': self.platform_status.get('movement_state'),
                        'current_height': self.platform_status.get('current_height'),
                        'target_height': self.platform_status.get('target_height'),
                        'limit_exceeded': self.platform_status.get('limit_exceeded', False),
                    }
                if self.pushrod_status:
                    response['pushrod'] = {
                        'task_state': self.pushrod_status.get('task_state', 'unknown'),
                        'task_type': self.pushrod_status.get('task_type'),
                        'task_start_time': self.pushrod_status.get('task_start_time'),
                        'task_end_time': self.pushrod_status.get('task_end_time'),
                        'task_duration': self.pushrod_status.get('task_duration'),
                        'completion_reason': self.pushrod_status.get('completion_reason'),
                        'control_mode': self.pushrod_status.get('control_mode'),
                        'movement_state': self.pushrod_status.get('movement_state'),
                        'current_height': self.pushrod_status.get('current_height'),
                        'target_height': self.pushrod_status.get('target_height'),
                    }
                if not response:
                    return JSONResponse({'error': 'no status data available'}, status_code=503)
                return response
            
            # Calibration API endpoints
            @app.post('/api/calibration/add_sample')
            async def add_calibration_sample(payload: dict):
                """Add calibration sample using latest sensor value"""
                height = payload.get('height')
                if height is None:
                    return JSONResponse({'success': False, 'error': 'Missing height field'})
                
                if self.latest_obj is None:
                    return JSONResponse({'success': False, 'error': 'No sensor data available'})
                
                # Check for register_1 (full mode) or height (compact mode as fallback)
                sensor_val = None
                if 'register_1' in self.latest_obj:
                    sensor_val = self.latest_obj['register_1']
                elif 'height' in self.latest_obj:
                    # Compact mode - using calibrated height as "raw" value
                    # This is not ideal for calibration but allows functionality
                    sensor_val = self.latest_obj['height']
                    self.get_logger().warn('Using calibrated height as sensor value - consider disabling publish_compact for proper calibration')
                else:
                    return JSONResponse({'success': False, 'error': 'No sensor raw value (register_1) or height field available'})
                
                if sensor_val is None:
                    return JSONResponse({'success': False, 'error': 'Sensor value is null'})
                
                with self.calib_lock:
                    sample = {
                        'sensor': sensor_val,
                        'height': float(height),
                        'timestamp': time.time()
                    }
                    self.calib_samples.append(sample)
                    total = len(self.calib_samples)
                
                return JSONResponse({
                    'success': True,
                    'sample': sample,
                    'total_samples': total
                })
            
            @app.get('/api/calibration/samples')
            async def get_calibration_samples():
                """Get all calibration samples"""
                with self.calib_lock:
                    return JSONResponse({
                        'success': True,
                        'samples': list(self.calib_samples)
                    })
            
            @app.delete('/api/calibration/samples/{index}')
            async def delete_calibration_sample(index: int):
                """Delete sample by index"""
                with self.calib_lock:
                    if 0 <= index < len(self.calib_samples):
                        removed = self.calib_samples.pop(index)
                        return JSONResponse({
                            'success': True,
                            'removed': removed,
                            'total_samples': len(self.calib_samples)
                        })
                    else:
                        return JSONResponse({
                            'success': False,
                            'error': f'Invalid index: {index}'
                        })
            
            @app.delete('/api/calibration/samples')
            async def clear_calibration_samples():
                """Clear all samples"""
                with self.calib_lock:
                    count = len(self.calib_samples)
                    self.calib_samples.clear()
                    self.calib_scale = None
                    self.calib_offset = None
                
                return JSONResponse({
                    'success': True,
                    'cleared': count
                })
            
            @app.post('/api/calibration/calculate')
            async def calculate_calibration():
                """Calculate linear calibration: height = sensor * scale + offset"""
                with self.calib_lock:
                    if len(self.calib_samples) < 2:
                        return JSONResponse({
                            'success': False,
                            'error': 'Need at least 2 samples for calibration'
                        })
                    
                    xs = [s['sensor'] for s in self.calib_samples]
                    ys = [s['height'] for s in self.calib_samples]
                
                # Linear regression
                n = len(xs)
                mx = sum(xs) / n
                my = sum(ys) / n
                
                num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
                den = sum((x - mx) ** 2 for x in xs)
                
                if den == 0:
                    return JSONResponse({
                        'success': False,
                        'error': 'Zero variance in sensor values'
                    })
                
                scale = num / den
                offset = my - scale * mx
                
                # Calculate R²
                y_pred = [scale * x + offset for x in xs]
                ss_tot = sum((y - my) ** 2 for y in ys)
                ss_res = sum((y - yp) ** 2 for y, yp in zip(ys, y_pred))
                r_squared = 1 - (ss_res / ss_tot) if ss_tot != 0 else 0
                
                # Calculate errors
                errors = [abs(y - yp) for y, yp in zip(ys, y_pred)]
                max_error = max(errors) if errors else 0
                avg_error = sum(errors) / len(errors) if errors else 0
                
                with self.calib_lock:
                    self.calib_scale = scale
                    self.calib_offset = offset
                
                return JSONResponse({
                    'success': True,
                    'scale': scale,
                    'offset': offset,
                    'r_squared': r_squared,
                    'max_error_mm': max_error,
                    'avg_error_mm': avg_error,
                    'num_samples': n,
                    'formula': f'height = {scale:.6f} * sensor + {offset:.6f}'
                })
            
            @app.get('/api/calibration/status')
            async def get_calibration_status():
                """Get current calibration status"""
                with self.calib_lock:
                    # Get sensor value (prefer register_1, fallback to height)
                    sensor_val = None
                    if self.latest_obj:
                        if 'register_1' in self.latest_obj:
                            sensor_val = self.latest_obj['register_1']
                        elif 'height' in self.latest_obj:
                            sensor_val = self.latest_obj['height']
                    
                    # Calculate estimated height if calibrated
                    estimated_height = None
                    if sensor_val is not None and self.calib_scale is not None and self.calib_offset is not None:
                        estimated_height = sensor_val * self.calib_scale + self.calib_offset
                    
                    # Load timestamp from config file if exists
                    calibrated_at = None
                    config_path = '/home/robot/Documents/robot_dc/colcon_ws/config/draw_wire_calibration.json'
                    if os.path.exists(config_path):
                        try:
                            with open(config_path, 'r') as f:
                                config_data = json.load(f)
                                calibrated_at = config_data.get('generated_at_iso')
                        except Exception as e:
                            self.get_logger().error(f"Failed to read calibration timestamp: {e}")
                    
                    return JSONResponse({
                        'num_samples': len(self.calib_samples),
                        'calibrated': self.calib_scale is not None and self.calib_offset is not None,
                        'scale': self.calib_scale,
                        'offset': self.calib_offset,
                        'latest_sensor': sensor_val,
                        'estimated_height': estimated_height,
                        'calibrated_at': calibrated_at
                    })
            
            @app.post('/api/calibration/save')
            async def save_calibration():
                """Save calibration to JSON config file"""
                with self.calib_lock:
                    if self.calib_scale is None or self.calib_offset is None:
                        return JSONResponse({
                            'success': False,
                            'error': 'No calibration calculated yet'
                        })
                    
                    scale = self.calib_scale
                    offset = self.calib_offset
                
                # Save to colcon_ws/config directory
                config_dir = '/home/robot/Documents/robot_dc/colcon_ws/config'
                config_path = os.path.join(config_dir, 'draw_wire_calibration.json')
                
                try:
                    # Create config directory if it doesn't exist
                    if not os.path.exists(config_dir):
                        os.makedirs(config_dir)
                        self.get_logger().info(f"Created config directory: {config_dir}")
                    
                    # Prepare calibration data (no samples stored)
                    config_data = {
                        'scale': scale,
                        'offset': offset,
                        'enable': True,
                        'formula': f'height = register_1 * {scale} + {offset}',
                        'generated_at': time.time(),
                        'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S')
                    }
                    
                    # Write JSON file
                    with open(config_path, 'w') as f:
                        json.dump(config_data, f, indent=2)
                    
                    self.get_logger().info(f"Calibration saved: scale={scale}, offset={offset} -> {config_path}")
                    
                    return JSONResponse({
                        'success': True,
                        'filepath': config_path,
                        'scale': scale,
                        'offset': offset,
                        'message': 'Config saved. Restart draw_wire_sensor to apply (no rebuild needed).'
                    })
                except Exception as e:
                    self.get_logger().error(f"Failed to save calibration: {e}")
                    return JSONResponse({
                        'success': False,
                        'error': str(e)
                    })

            # Platform overshoot calibration API endpoints
            @app.post('/api/overshoot/add_sample')
            async def add_overshoot_sample(request: dict):
                """Add a platform overshoot sample"""
                try:
                    direction = request.get('direction')  # 'up' or 'down'
                    target = request.get('target')
                    actual = request.get('actual')
                    stop_height = request.get('stop_height')  # Height when stop command issued
                    
                    if direction not in ['up', 'down']:
                        return JSONResponse({'success': False, 'error': 'Direction must be up or down'})
                    if actual is None:
                        return JSONResponse({'success': False, 'error': 'Missing actual height'})
                    
                    # Calculate RESIDUAL overshoot (drift after stop command)
                    # This is what the EMA should learn, NOT the total error from target
                    if stop_height is not None:
                        # Correct calculation: residual drift after stop
                        if direction == 'up':
                            overshoot = actual - stop_height  # How much drifted up after stop
                        else:
                            overshoot = stop_height - actual  # How much drifted down after stop
                    else:
                        # Fallback: if stop_height not provided, use target (legacy behavior)
                        # This is INCORRECT for EMA learning but maintains backward compatibility
                        overshoot = actual - target if target is not None else 0.0
                        self.get_logger().warn(f"No stop_height provided for {direction} sample - using target (incorrect)")
                    
                    sample = {
                        'target': float(target) if target is not None else None,
                        'actual': float(actual),
                        'stop_height': float(stop_height) if stop_height is not None else None,
                        'overshoot': overshoot,
                        'timestamp': time.time()
                    }
                    
                    with self.overshoot_lock:
                        if direction == 'up':
                            self.overshoot_samples_up.append(sample)
                        else:
                            self.overshoot_samples_down.append(sample)
                    
                    return JSONResponse({
                        'success': True,
                        'sample': sample,
                        'direction': direction,
                        'count_up': len(self.overshoot_samples_up),
                        'count_down': len(self.overshoot_samples_down)
                    })
                except Exception as e:
                    self.get_logger().error(f"Add overshoot sample error: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.get('/api/overshoot/samples')
            async def get_overshoot_samples():
                """Get all overshoot samples"""
                with self.overshoot_lock:
                    return JSONResponse({
                        'samples_up': self.overshoot_samples_up.copy(),
                        'samples_down': self.overshoot_samples_down.copy()
                    })
            
            @app.delete('/api/overshoot/samples/{direction}/{index}')
            async def delete_overshoot_sample(direction: str, index: int):
                """Delete a specific overshoot sample"""
                try:
                    if direction not in ['up', 'down']:
                        return JSONResponse({'success': False, 'error': 'Invalid direction'})
                    
                    with self.overshoot_lock:
                        samples = self.overshoot_samples_up if direction == 'up' else self.overshoot_samples_down
                        if 0 <= index < len(samples):
                            samples.pop(index)
                            return JSONResponse({
                                'success': True,
                                'count_up': len(self.overshoot_samples_up),
                                'count_down': len(self.overshoot_samples_down)
                            })
                        return JSONResponse({'success': False, 'error': 'Index out of range'})
                except Exception as e:
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.delete('/api/overshoot/samples')
            async def clear_overshoot_samples():
                """Clear all overshoot samples (both up and down)"""
                try:
                    with self.overshoot_lock:
                        self.overshoot_samples_up.clear()
                        self.overshoot_samples_down.clear()
                    return JSONResponse({
                        'success': True,
                        'count_up': len(self.overshoot_samples_up),
                        'count_down': len(self.overshoot_samples_down)
                    })
                except Exception as e:
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.post('/api/overshoot/calculate')
            async def calculate_overshoot():
                """Calculate overshoot using EMA (Exponential Moving Average) for fast convergence"""
                try:
                    with self.overshoot_lock:
                        samples_up = self.overshoot_samples_up.copy()
                        samples_down = self.overshoot_samples_down.copy()
                    
                    result = {
                        'success': True,
                        'overshoot_up': None,
                        'overshoot_down': None,
                        'count_up': len(samples_up),
                        'count_down': len(samples_down),
                        'avg_up': None,
                        'avg_down': None,
                        'std_up': None,
                        'std_down': None,
                        'ema_up': None,
                        'ema_down': None
                    }
                    
                    # EMA parameters
                    ALPHA = 0.3  # Weight for new samples (0.3 = 30% new, 70% old)
                    BOOTSTRAP_COUNT = 3  # Use median of first N samples as EMA seed
                    
                    # Calculate EMA for upward movement
                    if len(samples_up) >= 2:
                        overshoots = [s['overshoot'] for s in samples_up]
                        
                        # Calculate simple average and std for reference
                        avg_up = sum(overshoots) / len(overshoots)
                        std_up = (sum((x - avg_up)**2 for x in overshoots) / len(overshoots)) ** 0.5
                        
                        # EMA calculation with median bootstrap
                        if len(overshoots) <= BOOTSTRAP_COUNT:
                            # Bootstrap phase: use median as initial EMA
                            sorted_vals = sorted(overshoots)
                            ema_up = sorted_vals[len(sorted_vals) // 2]
                        else:
                            # Apply EMA: start with median of first BOOTSTRAP_COUNT samples
                            bootstrap_samples = overshoots[:BOOTSTRAP_COUNT]
                            sorted_bootstrap = sorted(bootstrap_samples)
                            ema_up = sorted_bootstrap[len(sorted_bootstrap) // 2]
                            
                            # Apply EMA to remaining samples
                            for val in overshoots[BOOTSTRAP_COUNT:]:
                                ema_up = (1 - ALPHA) * ema_up + ALPHA * val
                        
                        with self.overshoot_lock:
                            self.overshoot_up = ema_up
                        
                        result['overshoot_up'] = ema_up
                        result['ema_up'] = ema_up
                        result['avg_up'] = avg_up
                        result['std_up'] = std_up
                    
                    # Calculate EMA for downward movement
                    if len(samples_down) >= 2:
                        overshoots = [abs(s['overshoot']) for s in samples_down]  # Use absolute value for down
                        
                        # Calculate simple average and std for reference
                        avg_down = sum(overshoots) / len(overshoots)
                        std_down = (sum((x - avg_down)**2 for x in overshoots) / len(overshoots)) ** 0.5
                        
                        # EMA calculation with median bootstrap
                        if len(overshoots) <= BOOTSTRAP_COUNT:
                            # Bootstrap phase: use median as initial EMA
                            sorted_vals = sorted(overshoots)
                            ema_down = sorted_vals[len(sorted_vals) // 2]
                        else:
                            # Apply EMA: start with median of first BOOTSTRAP_COUNT samples
                            bootstrap_samples = overshoots[:BOOTSTRAP_COUNT]
                            sorted_bootstrap = sorted(bootstrap_samples)
                            ema_down = sorted_bootstrap[len(sorted_bootstrap) // 2]
                            
                            # Apply EMA to remaining samples
                            for val in overshoots[BOOTSTRAP_COUNT:]:
                                ema_down = (1 - ALPHA) * ema_down + ALPHA * val
                        
                        with self.overshoot_lock:
                            self.overshoot_down = ema_down
                        
                        result['overshoot_down'] = ema_down
                        result['ema_down'] = ema_down
                        result['avg_down'] = avg_down
                        result['std_down'] = std_down
                    
                    return JSONResponse(result)
                    
                except Exception as e:
                    self.get_logger().error(f"Overshoot calculation error: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.get('/api/overshoot/status')
            async def overshoot_status():
                """Get current overshoot calibration status"""
                try:
                    with self.overshoot_lock:
                        result = {
                            'overshoot_up': self.overshoot_up,
                            'overshoot_down': self.overshoot_down,
                            'count_up': len(self.overshoot_samples_up),
                            'count_down': len(self.overshoot_samples_down),
                            'calibrated_up': self.overshoot_up is not None,
                            'calibrated_down': self.overshoot_down is not None
                        }
                        
                        # Add last goto_height measurement from platform status
                        if self.platform_status:
                            if 'last_goto_target' in self.platform_status:
                                result['last_goto_target'] = self.platform_status['last_goto_target']
                            if 'last_goto_actual' in self.platform_status:
                                result['last_goto_actual'] = self.platform_status['last_goto_actual']
                                result['last_goto_direction'] = self.platform_status.get('last_goto_direction')
                                result['last_goto_timestamp'] = self.platform_status.get('last_goto_timestamp')
                            if 'last_goto_stop_height' in self.platform_status:
                                result['last_goto_stop_height'] = self.platform_status['last_goto_stop_height']
                        
                        # Load timestamp from config file if exists
                        config_path = '/home/robot/Documents/robot_dc/colcon_ws/config/platform_overshoot_calibration.json'
                        if os.path.exists(config_path):
                            try:
                                with open(config_path, 'r') as f:
                                    config_data = json.load(f)
                                    result['calibrated_at'] = config_data.get('generated_at_iso')
                            except Exception as e:
                                self.get_logger().error(f"Failed to read overshoot timestamp: {e}")
                        
                        return JSONResponse(result)
                except Exception as e:
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.post('/api/overshoot/save')
            async def save_overshoot():
                """Save overshoot calibration to JSON config file"""
                try:
                    with self.overshoot_lock:
                        overshoot_up = self.overshoot_up
                        overshoot_down = self.overshoot_down
                    
                    if overshoot_up is None and overshoot_down is None:
                        return JSONResponse({
                            'success': False,
                            'error': 'No calibration calculated yet. Please calculate first.'
                        })
                    
                    # Config file path
                    config_path = '/home/robot/Documents/robot_dc/colcon_ws/config/platform_overshoot_calibration.json'
                    
                    # Prepare config data (no samples stored)
                    config_data = {
                        'overshoot_up': overshoot_up if overshoot_up is not None else 2.067,
                        'overshoot_down': overshoot_down if overshoot_down is not None else 2.699,
                        'enable': True,
                        'formula': 'Predictive early stop to reduce overshoot',
                        'generated_at': time.time(),
                        'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
                    }
                    
                    # Create directory if needed
                    config_dir = os.path.dirname(config_path)
                    if not os.path.exists(config_dir):
                        os.makedirs(config_dir)
                        self.get_logger().info(f"Created config directory: {config_dir}")
                    
                    # Save to file
                    with open(config_path, 'w') as f:
                        json.dump(config_data, f, indent=2)
                    
                    self.get_logger().info(f"Saved overshoot calibration: up={overshoot_up}, down={overshoot_down}")
                    
                    return JSONResponse({
                        'success': True,
                        'filepath': config_path,
                        'overshoot_up': overshoot_up,
                        'overshoot_down': overshoot_down,
                        'message': 'Config saved. Restart lift_robot_platform to apply (no rebuild needed).'
                    })
                except Exception as e:
                    self.get_logger().error(f"Failed to save overshoot calibration: {e}")
                    return JSONResponse({
                        'success': False,
                        'error': str(e)
                    })

            # ═══════════════════════════════════════════════════════════════
            # Force Sensor Calibration API Endpoints (Dual-Channel)
            # ═══════════════════════════════════════════════════════════════
            @app.post('/api/force_calib/add_sample')
            async def add_force_calib_sample(request: dict):
                """Add a force sensor calibration sample (right or left channel)"""
                try:
                    channel = request.get('channel')  # 'right' or 'left'
                    actual_force = request.get('force')  # Actual applied force (N)
                    
                    if channel not in ['right', 'left']:
                        return JSONResponse({'success': False, 'error': 'Channel must be right or left'})
                    if actual_force is None:
                        return JSONResponse({'success': False, 'error': 'Missing force value'})
                    
                    # Get current sensor reading
                    sensor_reading = self.right_force_sensor if channel == 'right' else self.left_force_sensor
                    if sensor_reading is None:
                        return JSONResponse({'success': False, 'error': f'No sensor data for {channel} channel'})
                    
                    sample = {
                        'sensor': float(sensor_reading),
                        'force': float(actual_force),
                        'timestamp': time.time()
                    }
                    
                    with self.force_calib_lock:
                        if channel == 'right':
                            self.force_calib_samples_right.append(sample)
                        else:
                            self.force_calib_samples_left.append(sample)
                    
                    return JSONResponse({
                        'success': True,
                        'sample': sample,
                        'channel': channel,
                        'count_right': len(self.force_calib_samples_right),
                        'count_left': len(self.force_calib_samples_left)
                    })
                except Exception as e:
                    self.get_logger().error(f"Add force calib sample error: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.get('/api/force_calib/samples')
            async def get_force_calib_samples():
                """Get all force calibration samples"""
                with self.force_calib_lock:
                    return JSONResponse({
                        'samples_right': self.force_calib_samples_right.copy(),
                        'samples_left': self.force_calib_samples_left.copy()
                    })
            
            @app.delete('/api/force_calib/samples/{channel}/{index}')
            async def delete_force_calib_sample(channel: str, index: int):
                """Delete a specific force calibration sample"""
                try:
                    if channel not in ['right', 'left']:
                        return JSONResponse({'success': False, 'error': 'Invalid channel'})
                    
                    with self.force_calib_lock:
                        samples = self.force_calib_samples_right if channel == 'right' else self.force_calib_samples_left
                        if 0 <= index < len(samples):
                            removed = samples.pop(index)
                            return JSONResponse({
                                'success': True,
                                'removed': removed,
                                'count_right': len(self.force_calib_samples_right),
                                'count_left': len(self.force_calib_samples_left)
                            })
                        else:
                            return JSONResponse({'success': False, 'error': f'Invalid index: {index}'})
                except Exception as e:
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.delete('/api/force_calib/samples')
            async def clear_force_calib_samples():
                """Clear all force calibration samples"""
                with self.force_calib_lock:
                    count_right = len(self.force_calib_samples_right)
                    count_left = len(self.force_calib_samples_left)
                    self.force_calib_samples_right.clear()
                    self.force_calib_samples_left.clear()
                    self.force_calib_scale_right = None
                    self.force_calib_scale_left = None
                
                return JSONResponse({
                    'success': True,
                    'cleared_right': count_right,
                    'cleared_left': count_left
                })
            
            @app.post('/api/force_calib/calculate')
            async def calculate_force_calibration():
                """Calculate force sensor calibration (zero-intercept linear fit)
                Formula: actual_force = sensor_reading * scale
                """
                try:
                    result = {'success': True}
                    
                    # Calculate right channel
                    with self.force_calib_lock:
                        samples_right = self.force_calib_samples_right.copy()
                    
                    if len(samples_right) >= 1:
                        xs = [s['sensor'] for s in samples_right]
                        ys = [s['force'] for s in samples_right]
                        
                        # Zero-intercept linear fit: y = k*x, k = Σ(x*y) / Σ(x²)
                        numerator = sum(x * y for x, y in zip(xs, ys))
                        denominator = sum(x * x for x in xs)
                        
                        if denominator == 0:
                            result['error_right'] = 'Zero variance in sensor values'
                        else:
                            scale_right = numerator / denominator
                            
                            # Calculate errors
                            errors = [abs(y - x * scale_right) for x, y in zip(xs, ys)]
                            max_error = max(errors) if errors else 0
                            avg_error = sum(errors) / len(errors) if errors else 0
                            
                            with self.force_calib_lock:
                                self.force_calib_scale_right = scale_right
                            
                            result['scale_right'] = scale_right
                            result['max_error_right'] = max_error
                            result['avg_error_right'] = avg_error
                            result['num_samples_right'] = len(samples_right)
                    
                    # Calculate left channel
                    with self.force_calib_lock:
                        samples_left = self.force_calib_samples_left.copy()
                    
                    if len(samples_left) >= 1:
                        xs = [s['sensor'] for s in samples_left]
                        ys = [s['force'] for s in samples_left]
                        
                        numerator = sum(x * y for x, y in zip(xs, ys))
                        denominator = sum(x * x for x in xs)
                        
                        if denominator == 0:
                            result['error_left'] = 'Zero variance in sensor values'
                        else:
                            scale_left = numerator / denominator
                            
                            errors = [abs(y - x * scale_left) for x, y in zip(xs, ys)]
                            max_error = max(errors) if errors else 0
                            avg_error = sum(errors) / len(errors) if errors else 0
                            
                            with self.force_calib_lock:
                                self.force_calib_scale_left = scale_left
                            
                            result['scale_left'] = scale_left
                            result['max_error_left'] = max_error
                            result['avg_error_left'] = avg_error
                            result['num_samples_left'] = len(samples_left)
                    
                    if 'scale_right' not in result and 'scale_left' not in result:
                        return JSONResponse({'success': False, 'error': 'Need at least 1 sample per channel'})
                    
                    return JSONResponse(result)
                    
                except Exception as e:
                    self.get_logger().error(f"Force calibration calculation error: {e}")
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.get('/api/force_calib/status')
            async def force_calib_status():
                """Get current force calibration status"""
                try:
                    with self.force_calib_lock:
                        result = {
                            'scale_right': self.force_calib_scale_right,
                            'scale_left': self.force_calib_scale_left,
                            'count_right': len(self.force_calib_samples_right),
                            'count_left': len(self.force_calib_samples_left),
                            'calibrated_right': self.force_calib_scale_right is not None,
                            'calibrated_left': self.force_calib_scale_left is not None,
                            'current_sensor_right': self.right_force_sensor,
                            'current_sensor_left': self.left_force_sensor
                        }
                        
                        # Load timestamp from config files if exist
                        config_dir = '/home/robot/Documents/robot_dc/colcon_ws/config'
                        config_path_right = os.path.join(config_dir, 'force_sensor_calibration_52.json')
                        config_path_left = os.path.join(config_dir, 'force_sensor_calibration_53.json')
                        
                        # Load right channel timestamp
                        if os.path.exists(config_path_right):
                            try:
                                with open(config_path_right, 'r') as f:
                                    config_data = json.load(f)
                                    result['calibrated_at_right'] = config_data.get('generated_at_iso')
                            except Exception as e:
                                self.get_logger().error(f"Failed to read right force timestamp: {e}")
                        
                        # Load left channel timestamp
                        if os.path.exists(config_path_left):
                            try:
                                with open(config_path_left, 'r') as f:
                                    config_data = json.load(f)
                                    result['calibrated_at_left'] = config_data.get('generated_at_iso')
                            except Exception as e:
                                self.get_logger().error(f"Failed to read left force timestamp: {e}")
                        
                        return JSONResponse(result)
                except Exception as e:
                    return JSONResponse({'success': False, 'error': str(e)})
            
            @app.post('/api/force_calib/save')
            async def save_force_calibration():
                """Save force sensor calibration to JSON config files"""
                try:
                    with self.force_calib_lock:
                        scale_right = self.force_calib_scale_right
                        scale_left = self.force_calib_scale_left
                    
                    if scale_right is None and scale_left is None:
                        return JSONResponse({
                            'success': False,
                            'error': 'No calibration calculated yet. Please calculate first.'
                        })
                    
                    config_dir = '/home/robot/Documents/robot_dc/colcon_ws/config'
                    saved_files = []
                    
                    # Save right channel (device_id=52)
                    if scale_right is not None:
                        config_path_right = os.path.join(config_dir, 'force_sensor_calibration_52.json')
                        config_data_right = {
                            'device_id': 52,
                            'topic': '/force_sensor',
                            'scale': scale_right,
                            'offset': 0.0,  # Zero-intercept
                            'formula': f'actual_force = sensor_reading × {scale_right}',
                            'generated_at': time.time(),
                            'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
                        }
                        
                        if not os.path.exists(config_dir):
                            os.makedirs(config_dir)
                        
                        with open(config_path_right, 'w') as f:
                            json.dump(config_data_right, f, indent=2)
                        
                        saved_files.append(config_path_right)
                        self.get_logger().info(f"Saved right force calibration: scale={scale_right}")
                    
                    # Save left channel (device_id=53)
                    if scale_left is not None:
                        config_path_left = os.path.join(config_dir, 'force_sensor_calibration_53.json')
                        config_data_left = {
                            'device_id': 53,
                            'topic': '/force_sensor_2',
                            'scale': scale_left,
                            'offset': 0.0,
                            'formula': f'actual_force = sensor_reading × {scale_left}',
                            'generated_at': time.time(),
                            'generated_at_iso': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime())
                        }
                        
                        if not os.path.exists(config_dir):
                            os.makedirs(config_dir)
                        
                        with open(config_path_left, 'w') as f:
                            json.dump(config_data_left, f, indent=2)
                        
                        saved_files.append(config_path_left)
                        self.get_logger().info(f"Saved left force calibration: scale={scale_left}")
                    
                    return JSONResponse({
                        'success': True,
                        'saved_files': saved_files,
                        'scale_right': scale_right,
                        'scale_left': scale_left,
                        'message': 'Config saved. Restart force sensor nodes to apply.'
                    })
                except Exception as e:
                    self.get_logger().error(f"Failed to save force calibration: {e}")
                    return JSONResponse({
                        'success': False,
                        'error': str(e)
                    })

            @app.post('/api/force_calib/tare')
            async def tare_force_sensor(request: dict):
                """
                Tare (zero) force sensor by sending Modbus command.
                
                Request body:
                {
                    "channel": "right" or "left",
                    "device_id": 52 or 53
                }
                
                Sends Modbus function code 06 (Write Single Register) to address 0x0011 with value 0x0001.
                """
                try:
                    from modbus_driver_interfaces.srv import ModbusRequest
                    
                    body = await request.json()
                    channel = body.get('channel', 'right')
                    device_id = body.get('device_id', 52)
                    
                    # Validate device_id matches channel
                    if channel == 'right' and device_id != 52:
                        return JSONResponse({
                            'success': False,
                            'error': f'Channel "right" requires device_id=52, got {device_id}'
                        })
                    if channel == 'left' and device_id != 53:
                        return JSONResponse({
                            'success': False,
                            'error': f'Channel "left" requires device_id=53, got {device_id}'
                        })
                    
                    # Create Modbus request client
                    tare_client = self.create_client(ModbusRequest, '/modbus_request')
                    
                    # Wait for service with timeout
                    if not tare_client.wait_for_service(timeout_sec=2.0):
                        return JSONResponse({
                            'success': False,
                            'error': 'Modbus service not available'
                        })
                    
                    # Prepare tare command (FC06, register 0x0011, value 0x0001)
                    request_msg = ModbusRequest.Request()
                    request_msg.function_code = 0x06  # Write Single Register
                    request_msg.slave_id = device_id
                    request_msg.address = 0x0011  # Tare command register
                    request_msg.count = 0  # Not used for write operations
                    request_msg.values = [0x0001]  # Trigger tare
                    request_msg.seq_id = int(time.time() * 1000) % 65536
                    
                    self.get_logger().info(
                        f'Sending tare command: device_id={device_id} (0x{device_id:02X}), '
                        f'func=0x06, reg=0x0011, value=0x0001'
                    )
                    
                    # Send request
                    future = tare_client.call_async(request_msg)
                    
                    # Wait for response with timeout
                    start_time = time.time()
                    while not future.done():
                        if time.time() - start_time > 3.0:
                            return JSONResponse({
                                'success': False,
                                'error': 'Tare command timeout (3s)'
                            })
                        await asyncio.sleep(0.05)
                    
                    response = future.result()
                    
                    if response.success:
                        self.get_logger().info(f'Tare successful for device_id={device_id}')
                        return JSONResponse({
                            'success': True,
                            'message': f'Tared {channel} sensor (device_id={device_id}). Zero point reset.',
                            'device_id': device_id,
                            'channel': channel
                        })
                    else:
                        return JSONResponse({
                            'success': False,
                            'error': f'Modbus tare command failed: {response.message}'
                        })
                        
                except Exception as e:
                    self.get_logger().error(f"Tare force sensor error: {e}")
                    import traceback
                    traceback.print_exc()
                    return JSONResponse({
                        'success': False,
                        'error': f'Exception: {str(e)}'
                    })

            @app.websocket('/ws')
            async def ws_endpoint(ws: WebSocket):
                await ws.accept()
                self.connections.append(ws)
                if self.latest_raw:
                    await ws.send_text(self.latest_raw)
                try:
                    while True:
                        try:
                            _ = await asyncio.wait_for(ws.receive_text(), timeout=30)
                        except asyncio.TimeoutError:
                            await ws.send_text('ping')
                except WebSocketDisconnect:
                    pass
                finally:
                    if ws in self.connections:
                        self.connections.remove(ws)

            app.mount('/static', StaticFiles(directory=web_dir), name='static')

            # Create and own event loop explicitly; run uvicorn server inside
            self.loop = asyncio.new_event_loop()
            asyncio.set_event_loop(self.loop)

            # Use 'warning' log level to suppress access logs (INFO level logs every request)
            config = uvicorn.Config(app, host='0.0.0.0', port=self.port, loop='asyncio', log_level='warning')
            server = uvicorn.Server(config)

            async def serve():
                try:
                    self.get_logger().info(f"Web server started on 0.0.0.0:{self.port}")
                    await server.serve()
                except Exception as e:
                    self.get_logger().error(f"Web server serve error: {e}")

            self.loop.create_task(serve())
            try:
                self.loop.run_forever()
            except Exception as e:
                self.get_logger().error(f"Web server loop error: {e}")
            finally:
                try:
                    self.loop.close()
                except Exception as e:
                    self.get_logger().error(f"Web server loop close error: {e}")

        threading.Thread(target=run, daemon=True).start()

def main(args=None):
    if not FASTAPI:
        print('Install fastapi uvicorn first: pip install fastapi uvicorn')
        return
    rclpy.init(args=args)
    node = LiftRobotWeb()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
