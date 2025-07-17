import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import uvicorn
from fastapi import FastAPI, Request, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse
import os
from ament_index_python.packages import get_package_share_directory
import threading
import json
import asyncio
from typing import List

# WebSocket connection manager
class ConnectionManager:
    def __init__(self):
        self.active_connections: List[WebSocket] = []

    async def connect(self, websocket: WebSocket):
        await websocket.accept()
        self.active_connections.append(websocket)

    def disconnect(self, websocket: WebSocket):
        if websocket in self.active_connections:
            self.active_connections.remove(websocket)

    async def broadcast(self, message: dict):
        """Send message to all connected clients"""
        disconnected_clients = []
        for connection in self.active_connections:
            try:
                await connection.send_json(message)
            except RuntimeError:
                # Mark connection for removal if it's closed
                disconnected_clients.append(connection)
        
        # Clean up disconnected clients
        for client in disconnected_clients:
            self.disconnect(client)


class RobotArmWebServer(Node):
    def __init__(self):
        super().__init__('robot_arm_web_server')
        
        # Declare parameters
        self.declare_parameter('device_id', 1)
        self.declare_parameter('port', 8080)
        
        self.device_id = self.get_parameter('device_id').value
        self.port = self.get_parameter('port').value
        
        # WebSocket connection manager
        self.connection_manager = ConnectionManager()
        
        # Create event for notifying the websocket broadcaster
        self.new_state_event = threading.Event()
        self.broadcast_task = None
        
        # Create publisher for robot arm commands
        self.cmd_publisher = self.create_publisher(
            String, 
            f'/arm{self.device_id}/cmd', 
            10
        )
        
        # Create subscription for robot state
        self.robot_state_subscription = self.create_subscription(
            String,
            f'/arm{self.device_id}/robot_state',
            self.robot_state_callback,
            10
        )
        
        # Store latest robot state
        self.latest_robot_state = None
        self.robot_state_lock = threading.Lock()
        
        self.get_logger().info(f'Robot arm web server initialized for device {self.device_id}')
        self.get_logger().info(f'Publishing to topic: /arm{self.device_id}/cmd')
        self.get_logger().info(f'Subscribing to topic: /arm{self.device_id}/robot_state')
        
        # Start FastAPI server in a separate thread
        self.start_web_server()
    
    def robot_state_callback(self, msg):
        """Handle incoming robot state messages"""
        with self.robot_state_lock:
            try:
                self.latest_robot_state = json.loads(msg.data)
                # Signal that new state is available
                self.new_state_event.set()
            except json.JSONDecodeError as e:
                self.get_logger().error(f'Error parsing robot state JSON: {e}')
    
    def get_latest_robot_state(self):
        """Get the latest robot state data"""
        with self.robot_state_lock:
            return self.latest_robot_state
    
    def send_command(self, command):
        """Send command to robot arm"""
        msg = String()
        msg.data = command
        self.cmd_publisher.publish(msg)
        self.get_logger().info(f'Sent command: {command}')
        return {"status": "success", "command": command}
    
    def start_web_server(self):
        """Start the FastAPI web server"""
        def run_server():
            # Create FastAPI app
            app = FastAPI(title="Robot Arm Web Interface")
            
            # Get web directory
            try:
                web_path = os.path.join(get_package_share_directory('robot_arm_web'), 'web')
                STATIC_DIR = os.path.abspath(web_path)
            except:
                # Fallback for development
                STATIC_DIR = os.path.join(os.path.dirname(__file__), '..', 'web')
            
            self.get_logger().info(f"Serving web from: {STATIC_DIR}")
            
            # Routes
            @app.get("/")
            def serve_index():
                return FileResponse(os.path.join(STATIC_DIR, "index.html"))
            
            @app.post("/api/robot_arm/cmd")
            async def send_robot_arm_command(request: Request):
                data = await request.json()
                command = data.get("command")
                if command:
                    return self.send_command(command)
                else:
                    return JSONResponse(content={"error": "No command provided"}, status_code=400)
            
            @app.get("/api/robot_arm/info")
            def get_robot_arm_info():
                return {
                    "device_id": self.device_id,
                    "topic": f"/arm{self.device_id}/cmd",
                    "state_topic": f"/arm{self.device_id}/robot_state",
                    "available_commands": ["power_on", "power_off", "enable", "disable"]
                }
            
            @app.get("/api/robot_arm/state")
            def get_robot_arm_state():
                """Get current robot state"""
                state = self.get_latest_robot_state()
                if state is None:
                    return JSONResponse(content={"error": "No robot state data available"}, status_code=503)
                return JSONResponse(content=state)
            
            # Mount static files
            app.mount("/web", StaticFiles(directory=STATIC_DIR), name="web")
            
            # WebSocket endpoint for real-time updates
            @app.websocket("/ws/robot_state")
            async def websocket_endpoint(websocket: WebSocket):
                await self.connection_manager.connect(websocket)
                try:
                    while True:
                        # Keep connection alive with periodic pings
                        await asyncio.sleep(30)
                        await websocket.send_text("ping")
                except WebSocketDisconnect:
                    self.connection_manager.disconnect(websocket)

            # Run server
            uvicorn.run(app, host="0.0.0.0", port=self.port)
        
        # Start server in daemon thread
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        
        # Start the state broadcaster thread
        broadcaster_thread = threading.Thread(target=self.run_state_broadcaster, daemon=True)
        broadcaster_thread.start()
        
        self.get_logger().info(f'Web server started on port {self.port}')
        
    def run_state_broadcaster(self):
        """Run the state broadcaster loop in a separate thread"""
        import asyncio
        
        # Create a new event loop for this thread
        loop = asyncio.new_event_loop()
        asyncio.set_event_loop(loop)
        
        async def broadcast_state():
            while True:
                # Wait for new state event
                self.new_state_event.wait()
                self.new_state_event.clear()
                
                # Get the latest state and broadcast it
                with self.robot_state_lock:
                    if self.latest_robot_state:
                        await self.connection_manager.broadcast(self.latest_robot_state)
                
                # Small sleep to avoid CPU overuse
                await asyncio.sleep(0.01)
        
        # Run the broadcast loop
        loop.run_until_complete(broadcast_state())

def main(args=None):
    rclpy.init(args=args)
    
    node = RobotArmWebServer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Shutting down robot arm web server...')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
