import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Int32, Float32
from std_srvs.srv import Trigger
from modbus_driver_interfaces.msg import MotorSimulationStatus
from threading import Thread, Lock

class WebROSClient:
    def __init__(self, motor_list):
        self.motor_list = motor_list
        self.node = None
        self.publishers = {}
        self.status_map = {}
        self.lock = Lock()
        self.obs_ctrl_publishers = {}  # Ensure this is an instance attribute
        
        # Snapshot service clients for each camera
        self.snapshot_clients = {}
        
        # Camera restart service clients
        self.restart_clients = {}
        
        self._start_ros_node()

    def _start_ros_node(self):
        def ros_spin():
            rclpy.init()
            self.node = Node("web_ros_client")

            # Observation control Publisher
            for motor_id in [1, 2]:
                ctrl_topic = f"/motor{motor_id}/read_ctrl"
                self.obs_ctrl_publishers[f"motor{motor_id}"] = self.node.create_publisher(String, ctrl_topic, 10)
            for servo_id in [17, 18]:
                ctrl_topic = f"/servo{servo_id}/read_ctrl"
                self.obs_ctrl_publishers[f"servo{servo_id}"] = self.node.create_publisher(String, ctrl_topic, 10)

            # Subscribe to motor position published by detector node
            for motor_id in [1, 2]:
                topic = f"/motor{motor_id}/position"
                self.node.create_subscription(
                    Int32, topic,
                    lambda msg, m=motor_id: self._update_motor_position(f"motor{m}", msg),
                    10
                )

            # Subscribe to servo position and torque published by detector node
            for servo_id in [17, 18]:
                pos_topic = f"/servo{servo_id}/position"
                pwm_topic = f"/servo{servo_id}/pwm"
                self.node.create_subscription(
                    Int32, pos_topic,
                    lambda msg, s=servo_id: self._update_servo_position(f"servo{s}", msg),
                    10
                )
                self.node.create_subscription(
                    Float32, pwm_topic,
                    lambda msg, s=servo_id: self._update_servo_torque(f"servo{s}", msg),
                    10
                )

            # Keep original functionality
            for motor in self.motor_list:
                cmd_topic = f"/{motor}/cmd"
                status_topic = f"/{motor}/sim_status"
                self.publishers[motor] = self.node.create_publisher(String, cmd_topic, 10)
                self.node.create_subscription(
                    MotorSimulationStatus,
                    status_topic,
                    lambda msg, m=motor: self._update_status(m, msg),
                    10
                )

            # Create snapshot clients for each camera
            camera_names = ['cam100', 'cam101']
            for camera_name in camera_names:
                snapshot_service = f'{camera_name}_snapshot'
                restart_service = f'restart_{camera_name}_node'
                
                self.snapshot_clients[camera_name] = self.node.create_client(Trigger, snapshot_service)
                self.restart_clients[camera_name] = self.node.create_client(Trigger, restart_service)
                
                self.node.get_logger().info(f"Created snapshot client for {camera_name}: {snapshot_service}")
                self.node.get_logger().info(f"Created restart client for {camera_name}: {restart_service}")
            
            # Create platform command publisher
            self.platform_publisher = self.node.create_publisher(String, '/platform/cmd', 10)
            
            # Check snapshot service availability without blocking
            for camera_name, client in self.snapshot_clients.items():
                if not client.wait_for_service(timeout_sec=1.0):  # Short timeout
                    self.node.get_logger().warn(f"Snapshot service for {camera_name} not available - will retry when needed")
                else:
                    self.node.get_logger().info(f"Snapshot service for {camera_name} is available")

            rclpy.spin(self.node)

        Thread(target=ros_spin, daemon=True).start()

    def send_command(self, motor_id, command, value=None):
        if motor_id not in self.publishers:
            return {"error": f"Unknown motor: {motor_id}"}
        cmd_str = f"{command} {value}" if value is not None else command
        msg = String()
        msg.data = cmd_str
        self.publishers[motor_id].publish(msg)
        return {"motor": motor_id, "command": cmd_str}

    def control_observation(self, target, action):
        # target: 'motor1', 'motor2', 'servo17', 'servo18'
        # action: 'start' or 'stop'
        pub = self.obs_ctrl_publishers.get(target)
        if pub is None:
            return {"error": f"Unknown target: {target}"}
        msg = String()
        msg.data = action
        pub.publish(msg)
        return {"target": target, "action": action, "result": "sent"}

    def _update_status(self, motor_id, msg: MotorSimulationStatus):
        with self.lock:
            self.status_map[motor_id] = {
                "motor_id": msg.motor_id,
                "current_position": msg.current_position,
                "target_position": msg.target_position,
                "velocity": msg.velocity,
                "motion_mode": msg.motion_mode,
                "moving": msg.moving,
                "last_command": msg.last_command,
            }

    def _update_motor_position(self, motor_key, msg):
        with self.lock:
            if motor_key not in self.status_map:
                self.status_map[motor_key] = {}
            self.status_map[motor_key]["position"] = msg.data

    def _update_servo_position(self, servo_key, msg):
        with self.lock:
            if servo_key not in self.status_map:
                self.status_map[servo_key] = {}
            self.status_map[servo_key]["position"] = msg.data

    def _update_servo_torque(self, servo_key, msg):
        with self.lock:
            if servo_key not in self.status_map:
                self.status_map[servo_key] = {}
            self.status_map[servo_key]["torque"] = msg.data

    def get_motor_status(self, motor_id):
        with self.lock:
            return self.status_map.get(motor_id, None)

    def get_all_status(self):
        with self.lock:
            return self.status_map.copy()

    def get_snapshot(self, camera_name=None):
        """Get snapshot from specific camera or all cameras."""
        if camera_name:
            # Get snapshot from specific camera
            return self._get_single_snapshot(camera_name)
        else:
            # Get snapshots from all cameras
            results = {}
            for cam_name in self.snapshot_clients.keys():
                result = self._get_single_snapshot(cam_name)
                if 'error' not in result:
                    results[cam_name] = result
                else:
                    results[cam_name] = {"error": result['error']}
            
            # Map to IP-based format for frontend
            mapped_results = {}
            if 'cam100' in results:
                mapped_results['100'] = results['cam100']
            if 'cam101' in results:
                mapped_results['101'] = results['cam101']
            
            return mapped_results
    
    def _get_single_snapshot(self, camera_name):
        """Get snapshot from a single camera."""
        if camera_name not in self.snapshot_clients:
            return {"error": f"Unknown camera: {camera_name}"}
        
        snapshot_client = self.snapshot_clients[camera_name]
        if snapshot_client is None:
            return {"error": f"Snapshot client for {camera_name} not initialized"}
        
        # Check if service is available, recreate client if needed
        if not snapshot_client.service_is_ready():
            self.node.get_logger().warning(f"Snapshot service for {camera_name} not ready, recreating client...")
            service_name = f'{camera_name}_snapshot'
            snapshot_client = self.node.create_client(Trigger, service_name)
            self.snapshot_clients[camera_name] = snapshot_client
            
            # Wait a bit for service to become available
            import time
            time.sleep(1)
            
            if not snapshot_client.service_is_ready():
                return {"error": f"Snapshot service for {camera_name} not available after reconnection attempt"}
        
        try:
            request = Trigger.Request()
            future = snapshot_client.call_async(request)
            
            # Wait for the service call to complete with timeout
            import time
            timeout = 10.0
            start_time = time.time()
            
            while not future.done() and (time.time() - start_time) < timeout:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if future.done():
                response = future.result()
                if response.success:
                    try:
                        # Parse the response message (it's a string representation of a dict)
                        import ast
                        result = ast.literal_eval(response.message)
                        # Validate result format and add timestamp if missing
                        if isinstance(result, dict) and 'img' in result:
                            self.node.get_logger().info(f"Snapshot received for {camera_name}")
                            return result
                        else:
                            return {"error": "Invalid response format - missing image data"}
                    except (ValueError, SyntaxError) as e:
                        self.node.get_logger().error(f"Failed to parse snapshot response for {camera_name}: {e}")
                        return {"error": f"Failed to parse response: {str(e)}"}
                else:
                    return {"error": f"Snapshot failed for {camera_name}: {response.message}"}
            else:
                return {"error": f"Snapshot service call timed out for {camera_name}"}
                
        except Exception as e:
            self.node.get_logger().error(f"Exception during snapshot for {camera_name}: {e}")
            return {"error": f"Exception during snapshot for {camera_name}: {str(e)}"}

    def restart_camera_node(self, camera_name=None):
        """Restart specific camera node or all camera nodes."""
        if camera_name:
            # Restart specific camera
            return self._restart_single_camera(camera_name)
        else:
            # Restart all cameras
            results = {}
            for cam_name in self.restart_clients.keys():
                results[cam_name] = self._restart_single_camera(cam_name)
            return results
    
    def _restart_single_camera(self, camera_name):
        """Restart a single camera node."""
        if camera_name not in self.restart_clients:
            return {"error": f"Unknown camera: {camera_name}"}
        
        restart_client = self.restart_clients[camera_name]
        if restart_client is None:
            return {"error": f"Restart client for {camera_name} not initialized"}
        
        # Check if service is available, recreate client if needed
        if not restart_client.service_is_ready():
            self.node.get_logger().warning(f"Restart service for {camera_name} not ready, recreating client...")
            service_name = f'restart_{camera_name}_node'
            restart_client = self.node.create_client(Trigger, service_name)
            self.restart_clients[camera_name] = restart_client
            
            # Wait a bit for service to become available
            import time
            time.sleep(1)
            
            if not restart_client.service_is_ready():
                return {"success": False, "message": f"Camera restart service for {camera_name} not available"}
        
        try:
            request = Trigger.Request()
            future = restart_client.call_async(request)
            
            # Wait for the service call to complete
            import time
            timeout = 5.0
            start_time = time.time()
            
            while not future.done() and (time.time() - start_time) < timeout:
                rclpy.spin_once(self.node, timeout_sec=0.1)
            
            if future.done():
                response = future.result()
                return {"success": response.success, "message": response.message}
            else:
                return {"success": False, "message": f"Restart service call timed out for {camera_name}"}
                
        except Exception as e:
            self.node.get_logger().error(f"Exception during camera restart for {camera_name}: {e}")
            return {"success": False, "message": f"Exception during restart for {camera_name}: {str(e)}"}

    def send_platform_command(self, command, value=None):
        """Send command to platform controller."""
        if self.platform_publisher is None:
            return {"error": "Platform publisher not initialized"}
        
        try:
            cmd_str = f"{command} {value}" if value is not None else command
            msg = String()
            msg.data = cmd_str
            self.platform_publisher.publish(msg)
            
            self.node.get_logger().info(f"Sent platform command: {cmd_str}")
            return {"success": True, "command": cmd_str}
            
        except Exception as e:
            self.node.get_logger().error(f"Error sending platform command: {e}")
            return {"error": str(e)}
