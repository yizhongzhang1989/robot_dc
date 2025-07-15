import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import uvicorn
from fastapi import FastAPI, Request
from fastapi.responses import FileResponse
from fastapi.staticfiles import StaticFiles
from fastapi.responses import JSONResponse
import os
from ament_index_python.packages import get_package_share_directory
import threading

class RobotArmWebServer(Node):
    def __init__(self):
        super().__init__('robot_arm_web_server')
        
        # Declare parameters
        self.declare_parameter('device_id', 1)
        self.declare_parameter('port', 8080)
        
        self.device_id = self.get_parameter('device_id').value
        self.port = self.get_parameter('port').value
        
        # Create publisher for robot arm commands
        self.cmd_publisher = self.create_publisher(
            String, 
            f'/arm{self.device_id}/cmd', 
            10
        )
        
        self.get_logger().info(f'Robot arm web server initialized for device {self.device_id}')
        self.get_logger().info(f'Publishing to topic: /arm{self.device_id}/cmd')
        
        # Start FastAPI server in a separate thread
        self.start_web_server()
    
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
                    "available_commands": ["power_on", "power_off", "enable", "disable"]
                }
            
            # Mount static files
            app.mount("/web", StaticFiles(directory=STATIC_DIR), name="web")
            
            # Run server
            uvicorn.run(app, host="0.0.0.0", port=self.port)
        
        # Start server in daemon thread
        server_thread = threading.Thread(target=run_server, daemon=True)
        server_thread.start()
        self.get_logger().info(f'Web server started on port {self.port}')

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
