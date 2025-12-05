import os
import time
import numpy as np
import argparse
import threading
from scipy.spatial.transform import Rotation as R
from ur_operate_wobj import UROperateWobj
from ur_positioning import URPositioning
from courier_robot_webapi import CourierRobotWebAPI
import rclpy
from rclpy.executors import MultiThreadedExecutor


class URWobjExtractServer(UROperateWobj):
    def __init__(self, robot_ip=None, robot_port=None, server_index=14):
        """
        Initialize URWobjExtractServer instance
        Args:
            robot_ip: IP address of the UR15 robot. If None, loads from config file
            robot_port: Port number of the UR15 robot. If None, loads from config file
            server_index: Server index for operation (default: 14)
        """      
        # Call parent class constructor
        super().__init__(robot_ip=robot_ip, robot_port=robot_port, server_index=server_index)
        
        # Store operation name and template points before parent init
        self.operation_name = "extract_server"
        self.template_points = [
            {"name": "Handle_Top_Left_Corner",  "x": -0.074, "y": -0.036, "z": 0.007},
            {"name": "Handle_Top_Right_Corner", "x": 0.074, "y": -0.036, "z": 0.007},
            {"name": "Handle_Bottom_Left_Corner", "x": -0.074, "y": -0.056, "z": 0.002},
            {"name": "Handle_Bottom_Right_Corner", "x": 0.074, "y": -0.056, "z": 0.002}
        ]

        # Initialize URPositioning variables before parent init
        self.ur_positioning = None
        self.executor = None
        self.spin_thread = None
        
        # Initialize CourierRobotWebAPI variable
        self.courier_robot = None

        # Load tool parameters from config file
        self._load_tool_extract_parameters_from_config()
        
        # Initialize URPositioning instance and ROS2 executor after parent init
        self._initialize_ur_positioning()
        
        # Initialize CourierRobotWebAPI instance
        self._initialize_courier_robot()

        self._calculate_server2base(self.server_index)
        
        print(f"URWobjExtractServer initialized for server index: {server_index}")

    def _load_tool_extract_parameters_from_config(self):
        """
        Load tool_extract parameters from robot_config.yaml
        
        Loads:
            - tool_length: Length of the tool from flange to tip (meters)
            - tool_angle_z: Rotation angle around Z axis (degrees)
        """
        # Default values
        defaults = {
            'tool_length': 0.2265,
            'tool_angle_z': -31
        }
        
        try:
            import yaml
            from common.workspace_utils import get_workspace_root
            
            # Get workspace root
            workspace_root = get_workspace_root()
            if workspace_root is None:
                workspace_root = os.path.abspath(os.path.join(os.path.dirname(os.path.abspath(__file__)), '..'))
            
            # Path to config file
            config_path = os.path.join(workspace_root, 'config', 'robot_config.yaml')
            
            if not os.path.exists(config_path):
                print(f"Config file not found: {config_path}")
                print(f"Using default tool_extract values: length={defaults['tool_length']}, angle_z={defaults['tool_angle_z']}")
                self.tool_length = defaults['tool_length']
                self.tool_angle_z = defaults['tool_angle_z']
                return
            
            # Load config file
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Get tool_extract parameters from ur15.tool.tool_extract
            tool_extract = config.get('ur15', {}).get('tool', {}).get('tool_extract', {})
            
            self.tool_length = tool_extract.get('length', defaults['tool_length'])
            self.tool_angle_z = tool_extract.get('angle_z', defaults['tool_angle_z'])
            
            print(f"✓ Loaded tool_extract parameters: length={self.tool_length}m, angle_z={self.tool_angle_z}°")
            
        except Exception as e:
            print(f"Error loading tool_extract parameters: {e}")
            print(f"Using default values: length={defaults['tool_length']}, angle_z={defaults['tool_angle_z']}")
            self.tool_length = defaults['tool_length']
            self.tool_angle_z = defaults['tool_angle_z']

    def _initialize_ur_positioning(self):
        """Initialize URPositioning instance for camera capture and positioning"""
        try:
            print(f'Initializing URPositioning for operation: {self.operation_name}...')
            
            # Initialize ROS2 if not already initialized
            if not rclpy.ok():
                rclpy.init()
                print('✓ ROS2 initialized')
            
            # Create URPositioning instance with robot connection parameters
            self.ur_positioning = URPositioning(
                robot_ip=self.robot_ip,
                robot_port=self.robot_port,
                operation_name=self.operation_name,
            )
            
            self.ur_positioning.template_points = self.template_points

            # Create executor and start spinning in a separate thread
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.ur_positioning)
            
            # Start spinning in background thread
            self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.spin_thread.start()
            
            print('✓ URPositioning initialized and spinning in background')
            
            # Wait a moment for camera subscription to be ready
            time.sleep(2.0)
            
        except Exception as e:
            print(f'Error initializing URPositioning: {e}')
            import traceback
            traceback.print_exc()
    
    def _initialize_courier_robot(self):
        """
        Initialize CourierRobotWebAPI instance for lift platform control
        
        This private function initializes the courier robot web API client
        to enable communication with the lift platform system.
        """
        try:
            print('Initializing CourierRobotWebAPI...')
            
            # Initialize with default URL and verbose mode
            self.courier_robot = CourierRobotWebAPI(
                base_url="http://192.168.1.3:8090", 
                verbose=True
            )
            
            # Test connection by getting status
            status_result = self.courier_robot.get_status()
            if status_result.get('success', False):
                print('✓ CourierRobotWebAPI initialized successfully')
                print(f'  Platform state: {status_result.get("platform", {}).get("task_state", "unknown")}')
                print(f'  Pushrod state: {status_result.get("pushrod", {}).get("task_state", "unknown")}')
            else:
                print(f'⚠️  CourierRobotWebAPI initialized but connection test failed: {status_result.get("error", "unknown")}')
                
        except Exception as e:
            print(f'Error initializing CourierRobotWebAPI: {e}')
            self.courier_robot = None
            import traceback
            traceback.print_exc()

    def update_server2base_by_positioning(self):
        """
        Update server2base_matrix using vision positioning.
        
        This method:
        1. Calls auto_positioning() to perform vision-based positioning
        2. Extracts the local2world transformation matrix from the result
        3. Updates self.server2base_matrix with the vision-corrected coordinate system
        
        Returns:
            int: 0 if successful, -1 if failed
        """
        if self.ur_positioning is None:
            print("[ERROR] URPositioning is not initialized")
            return -1
        
        print("Updating server coordinate system using vision positioning...")
        
        # Execute auto positioning to get updated server coordinate system
        positioning_result = self.ur_positioning.auto_positioning()
        
        # Check if positioning was successful
        if not isinstance(positioning_result, dict) or not positioning_result.get('success', False):
            print(f"[ERROR] Auto positioning failed")
            if isinstance(positioning_result, dict):
                error_msg = positioning_result.get('result', {}).get('error_message', 'Unknown error')
                print(f"  Error: {error_msg}")
            return -1
        
        # Read updated server2base_matrix from positioning result
        local2world = positioning_result.get('result', {}).get('local2world', None)
        if local2world is None:
            print("[ERROR] No transformation matrix in positioning result")
            return -1
        
        # Update self.server2base_matrix with vision-corrected coordinate system
        self.server2base_matrix = np.array(local2world)
        print("✓ server2base_matrix updated from vision positioning result")
        
        return 0
    def cleanup(self):
        """Cleanup resources before exit"""
        print("\nCleaning up resources...")
        
        if self.courier_robot is not None:
            # Perform emergency reset to stop any ongoing operations
            try:
                self.courier_robot.emergency_reset()
                print("✓ CourierRobot emergency reset")
            except Exception as e:
                print(f"Warning: Failed to reset courier robot: {e}")
            self.courier_robot = None
        
        if self.executor is not None:
            self.executor.shutdown()
            print("✓ Executor shutdown")
        
        if self.ur_positioning is not None:
            self.ur_positioning.destroy_node()
            print("✓ URPositioning node destroyed")
        
        if rclpy.ok():
            rclpy.shutdown()
            print("✓ ROS2 shutdown")

    # ================================ Force Control Functions ================================
    def force_task_extract_server(self, distance):
        """
        Execute force control task to extract the server out of rack.
        
        Args:
            distance: Distance to extract in meters (e.g., 0.60)
        
        Returns:
            int: Result code from force control task
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.server2base_matrix is None:
            print("Server coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")

        # Extract rotation matrix from server transformation matrix and convert to rotation vector
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame using current position with server orientation
        task_frame = [
            tcp_pose[0],               # x (current position)
            tcp_pose[1],               # y
            tcp_pose[2],               # z
            server_rotation_vector[0], # rx (server orientation)
            server_rotation_vector[1], # ry
            server_rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (server coordinate system): {task_frame}")
        
        # Set force mode parameters
        # In server coordinate system: Y- is pulling direction (away from rack)
        selection_vector = [1, 1, 0, 0, 0, 0]  # Enable force control in X and Y directions
        wrench = [0, -70, 0, 0, 0, 0]  # -70N in server Y direction = pulling away from rack
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print(f"[INFO] Starting force control task to extract server {distance}m...")
        
        # Execute force control task with distance-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=3,  # Force-based termination
            end_distance=[0.05, distance, 0, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    def force_task_touch_handle(self,distance):
        """
        Execute force control task to touch handle based on server coordinate system.
        
        Returns:
            int: Result code from force control task
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.server2base_matrix is None:
            print("Server coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")

        # Extract rotation matrix from server transformation matrix and convert to rotation vector
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame using current position with server orientation
        task_frame = [
            tcp_pose[0],               # x (current position)
            tcp_pose[1],               # y
            tcp_pose[2],               # z
            server_rotation_vector[0], # rx (server orientation)
            server_rotation_vector[1], # ry
            server_rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (server coordinate system): {task_frame}")
        
        # Set force mode parameters
        # In server coordinate system: Z- is downward (toward handle)
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable force control only in Z direction
        wrench = [0, 0, -25, 0, 0, 0]  # -25N in server Z direction = downward (toward handle)
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task to touch handle...")
        
        # Execute force control task with force-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=3,  # Force-based termination
            end_distance=[0, 0, distance+0.01, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    # ================================ Main Sequence ================================
    def execute_extract_server_sequence(self):
        """
        Execute the complete extract server sequence.
        
        Returns:
            int: 0 on success, error code otherwise
        """
        print("\n" + "="*70)
        print("STARTING EXTRACT SERVER SEQUENCE")
        print("="*70)
        
        # Step 1: Correct TCP pose
        print("\n" + "="*50)
        print("Step 1: Correcting TCP pose...")
        print("="*50)
        result = self.movel_to_correct_tcp_pose(
            tcp_x_to_rack=[1, 0, 0],
            tcp_y_to_rack=[0, -1, 0],
            tcp_z_to_rack=[0, 0, -1],
            angle_deg=0,
        )
        if result != 0:
            print(f"[ERROR] Failed to correct TCP pose (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 2: Move to target position to position before extraction
        print("\n" + "="*50)
        print("Step 2: Moving to target position before extraction...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0, -0.40, 0.20+self.tool_length]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to target position (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 3: Update server2base_matrix by vision positioning
        print("\n" + "="*50)
        print("Step 3: Updating server coordinate system...")
        print("="*50)
        
        result = self.update_server2base_by_positioning()
        if result != 0:
            print(f"[ERROR] Failed to update server coordinate system")
            return result
        time.sleep(0.5)

        # Step 4: Correct TCP pose again before extraction
        print("\n" + "="*50)
        print("Step 4: Correcting TCP pose...")
        print("="*50)
        result = self.movel_to_correct_tcp_pose(
            tcp_x_to_rack=[1, 0, 0],
            tcp_y_to_rack=[0, -1, 0],
            tcp_z_to_rack=[0, 0, -1],
            angle_deg=-self.tool_angle_z,
        )
        if result != 0:
            print(f"[ERROR] Failed to correct TCP pose (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 5: move to extract serevr position
        print("\n" + "="*50)
        print("Step 5: Extracting server from rack...")
        print("="*50)
        print("\n" + "="*50)
        print("Step 2: Moving to target position before extraction...")
        print("="*50)

        distance =0.06

        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0, -0.045, distance+self.tool_length]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to target position (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 6: Courier robot lift platform up to touch the server before extraction
        print("\n" + "="*50)
        print("Step 6: Lifting platform to touch server...")
        print("="*50)
        if self.courier_robot is None:
            print("[ERROR] CourierRobotWebAPI is not initialized")
            return -1
        lift_result = self.courier_robot.platform_hybrid_control(target_height=800, target_force=125)
        if not lift_result.get('success', False):
            print(f"[ERROR] Failed to lift platform: {lift_result.get('error', 'Unknown error')}")
            return -1

        # Step 7: Execute force control task to touch handle
        print("\n" + "="*50)
        print("Step 7: Touching handle with force control...")
        print("="*50)
        result = self.force_task_touch_handle(distance)
        if result != 0:
            print(f"[ERROR] Failed to touch handle (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 8: Execute force control task to extract server
        print("\n" + "="*50)
        print("Step 8: Extracting server with force control...")
        print("="*50)
        result = self.force_task_extract_server(distance=0.50)
        if result != 0:
            print(f"[ERROR] Failed to extract server (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 9: Slightly adjust height of end effector
        print("\n" + "="*50)
        print("Step 9: Adjusting end effector height...")
        print("="*50)
        result = self.movel_in_server_frame([[0, 0, 0.01]])
        if result != 0:
            print(f"[ERROR] Failed to adjust end effector height (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 10: Courier robot up to transfer center of mass of server
        print("\n" + "="*50)
        print("Step 10: Lifting platform to transfer center of mass...")
        print("="*50)
        if self.courier_robot is None:
            print("[ERROR] CourierRobotWebAPI is not initialized")
            return -1
        lift_result = self.courier_robot.platform_hybrid_control(target_height=800, target_force=350)
        if not lift_result.get('success', False):
            print(f"[ERROR] Failed to lift platform: {lift_result.get('error', 'Unknown error')}")
            return -1

        # Step 11: Execute force control task to extract server completely
        print("\n" + "="*50)
        print("Step 11: Extracting server with force control...")
        print("="*50)
        result = self.force_task_extract_server(distance=0.30)
        if result != 0:
            print(f"[ERROR] Failed to extract server (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 12: Move away from the server after extraction
        print("\n" + "="*50)
        print("Step 12: Moving away from the server...")
        print("="*50)
        result = self.movel_in_rack_frame([[0, 0, 0.20]])
        if result != 0:
            print(f"[ERROR] Failed to move away from server (error code: {result})")
            return result
        time.sleep(0.5)

        print("\n" + "="*70)
        print("EXTRACT SERVER SEQUENCE FINISHED SUCCESSFULLY")
        print("="*70)
        return 0


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='URWobjExtractServer - Extract server operation using server coordinate system')
    parser.add_argument('--robot-ip', type=str, default=None,
                       help='Robot IP address (default: from config file)')
    parser.add_argument('--robot-port', type=int, default=None,
                       help='Robot port (default: from config file)')
    parser.add_argument('--server-index', type=int, default=14,
                       help='Server index for operation (default: 14)')
    
    args = parser.parse_args()
    
    # Create URWobjExtractServer instance
    ur_extract_server = URWobjExtractServer(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        server_index=args.server_index
    )
    
    # ==============Display automatically loaded parameters=================
    print("\n" + "="*70)
    print("AUTOMATICALLY LOADED PARAMETERS")
    print("="*70)
    
    if ur_extract_server.camera_matrix is not None:
        print("\n✓ Camera Matrix loaded")
        print(ur_extract_server.camera_matrix)
        print("\n✓ Distortion Coefficients loaded")
        print(ur_extract_server.distortion_coefficients)
    
    if ur_extract_server.cam2end_matrix is not None:
        print("\n✓ Camera to End-effector Matrix loaded")
        print(ur_extract_server.cam2end_matrix)
    
    if ur_extract_server.rack_transformation_matrix_in_base is not None:
        print("\n✓ Rack Coordinate System loaded")
        print(f"  Origin: {ur_extract_server.rack_origin_in_base}")
    
    if ur_extract_server.server2base_matrix is not None:
        print("\n✓ Server Coordinate System loaded")
        server_origin = ur_extract_server.server2base_matrix[:3, 3]
        print(f"  Origin: x={server_origin[0]:.6f}, y={server_origin[1]:.6f}, z={server_origin[2]:.6f}")
        print(f"  Server Index: {ur_extract_server.server_index}")

    # ==============Execute the extract server task=================
    print("\n" + "="*70)
    print("STARTING TASK EXECUTION")
    print("="*70)
    
    try:
        result = ur_extract_server.execute_extract_server_sequence()
        
        if result == 0:
            print("\n✓ Task completed successfully!")
        else:
            print(f"\n✗ Task failed with error code: {result}")
            
    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] Task interrupted by user")
    except Exception as e:
        print(f"\n✗ Error during task execution: {e}")
        import traceback
        traceback.print_exc()
    finally:
        # Cleanup resources
        ur_extract_server.cleanup()
