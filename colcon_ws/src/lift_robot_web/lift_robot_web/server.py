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

            config = uvicorn.Config(app, host='0.0.0.0', port=self.port, loop='asyncio', log_level='info')
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
