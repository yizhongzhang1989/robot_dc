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
        self.right_force = None
        self.left_force = None
        self.platform_status = None
        self.pushrod_status = None

        # ROS interfaces
        self.sub = self.create_subscription(String, self.sensor_topic, self.sensor_cb, 10)
        # Force sensor subscriptions (Float32)
        try:
            from std_msgs.msg import Float32
            self.right_sub = self.create_subscription(Float32, '/right_force_sensor', self.right_cb, 10)
            self.left_sub = self.create_subscription(Float32, '/left_force_sensor', self.left_cb, 10)
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
                        if self.right_force is not None:
                            merged['right_force_sensor'] = self.right_force
                        if self.left_force is not None:
                            merged['left_force_sensor'] = self.left_force
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

    def right_cb(self, msg):
        try:
            self.right_force = msg.data
        except Exception as e:
            self.get_logger().warn(f"Right force callback error: {e}")

    def left_cb(self, msg):
        try:
            self.left_force = msg.data
        except Exception as e:
            self.get_logger().warn(f"Left force callback error: {e}")
    
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
                allowed = {'up','down','stop','timed_up','timed_down','stop_timed','goto_point','goto_height'}
                if cmd not in allowed:
                    return JSONResponse({'error':'invalid command'}, status_code=400)
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
                if duration is not None:
                    body['duration'] = duration
                msg = String(); msg.data = json.dumps(body)
                if target == 'pushrod':
                    self.pushrod_cmd_pub.publish(msg)
                else:
                    self.cmd_pub.publish(msg)
                return {'status':'ok','command':cmd,'target':target,'duration':duration}

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
