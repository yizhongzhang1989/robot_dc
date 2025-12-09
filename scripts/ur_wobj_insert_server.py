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


class URWobjInsertServer(UROperateWobj):
    def __init__(self, robot_ip=None, robot_port=None, server_index=14):
        """
        Initialize URWobjInsertServer instance
        Args:
            robot_ip: IP address of the UR15 robot. If None, loads from config file
            robot_port: Port number of the UR15 robot. If None, loads from config file
            server_index: Server index for operation (default: 14)
        """      
        # Call parent class constructor
        super().__init__(robot_ip=robot_ip, robot_port=robot_port, server_index=server_index)
        
        # Store operation name and template points before parent init
        self.operation_name = "insert_server"
        self.template_points = [
            {"name": "Handle_Top_Left_Corner",  "x": -0.074, "y": -0.036, "z": 0.002},
            {"name": "Handle_Top_Right_Corner", "x": 0.074, "y": -0.036, "z": 0.002},
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
        self._load_tool_insert_parameters_from_config()
        
        # Initialize URPositioning instance and ROS2 executor after parent init
        self._initialize_ur_positioning()
        
        # Initialize CourierRobotWebAPI instance
        self._initialize_courier_robot()

        self._calculate_server2base(self.server_index)
        
        print(f"URWobjInsertServer initialized for server index: {server_index}")

    def _load_tool_insert_parameters_from_config(self):
        """
        Load tool_insert parameters from robot_config.yaml
        
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
                print(f"Using default tool_insert values: length={defaults['tool_length']}, angle_z={defaults['tool_angle_z']}")
                self.tool_length = defaults['tool_length']
                self.tool_angle_z = defaults['tool_angle_z']
                return
            
            # Load config file
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Get tool_insert parameters from ur15.tool.tool_insert
            tool_insert = config.get('ur15', {}).get('tool', {}).get('tool_insert', {})
            
            self.tool_length = tool_insert.get('length', defaults['tool_length'])
            self.tool_angle_z = tool_insert.get('angle_z', defaults['tool_angle_z'])
            
            print(f"✓ Loaded tool_insert parameters: length={self.tool_length}m, angle_z={self.tool_angle_z}°")
            
        except Exception as e:
            print(f"Error loading tool_insert parameters: {e}")
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
            
            # Initialize with automatic config loading and verbose mode
            # CourierRobotWebAPI will automatically load URL from config file
            # or use default fallback if config not found
            self.courier_robot = CourierRobotWebAPI(verbose=True)
            
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
    def force_task_insert_server(self, distance):
        """
        Execute force control task to insert the server into rack.
        
        Args:
            distance: Distance to insert in meters (e.g., 0.60)
        
        Returns:
            int: Result code from force control task
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.rack_transformation_matrix_in_base is None:
            print("Rack coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")

        # Extract rotation matrix from rack transformation matrix and convert to rotation vector
        rack_rotation_matrix = self.rack_transformation_matrix_in_base[:3, :3]
        rack_rotation = R.from_matrix(rack_rotation_matrix)
        rack_rotation_vector = rack_rotation.as_rotvec()
        
        # Create task frame using TCP position with rack orientation
        task_frame = [
            tcp_pose[0],               # x (current TCP position)
            tcp_pose[1],               # y
            tcp_pose[2],               # z
            rack_rotation_vector[0],   # rx (rack orientation)
            rack_rotation_vector[1],   # ry
            rack_rotation_vector[2]    # rz
        ]
        
        print(f"[INFO] Task frame (rack coordinate system): {task_frame}")
        
        # Set force mode parameters
        # In rack coordinate system: Y+ is pushing direction (into rack)
        selection_vector = [1, 1, 0, 0, 0, 0]  # Enable force control in X and Y directions
        wrench = [0, 70, 0, 0, 0, 0]  # +70N in rack Y direction = pushing into rack
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print(f"[INFO] Starting force control task to insert server {distance}m...")
        
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

    def force_task_touch_handle(self, distance):
        """
        Execute force control task to align handle based on rack coordinate system.
        
        Returns:
            int: Result code from force control task
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.rack_transformation_matrix_in_base is None:
            print("Rack coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")

        # Extract rotation matrix from rack transformation matrix and convert to rotation vector
        rack_rotation_matrix = self.rack_transformation_matrix_in_base[:3, :3]
        rack_rotation = R.from_matrix(rack_rotation_matrix)
        rack_rotation_vector = rack_rotation.as_rotvec()
        
        # Create task frame using TCP position with rack orientation
        task_frame = [
            tcp_pose[0],               # x (current TCP position)
            tcp_pose[1],               # y
            tcp_pose[2],               # z
            rack_rotation_vector[0],   # rx (rack orientation)
            rack_rotation_vector[1],   # ry
            rack_rotation_vector[2]    # rz
        ]
        
        print(f"[INFO] Task frame (rack coordinate system): {task_frame}")
        
        # Set force mode parameters
        # In rack coordinate system: Z- is downward (toward handle)
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable force control only in Z direction
        wrench = [0, 0, -25, 0, 0, 0]  # -25N in rack Z direction = downward (toward handle)
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task to align handle...")
        
        # Execute force control task with force-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=3,  # Force-based termination
            end_distance=[0, 0, distance-0.03, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    # ================================ Main Sequence ================================
    def execute_insert_server_sequence(self):
        """
        Execute the complete insert server sequence.
        
        Returns:
            int: 0 on success, error code otherwise
        """
        print("\n" + "="*70)
        print("STARTING INSERT SERVER SEQUENCE")
        print("="*70)
        
        # Step 1: Initialize height of courier robot
        print("\n" + "="*50)
        print("Step 1: Initializing courier robot platform height...")
        print("="*50)
        if self.courier_robot is None:
            print("[ERROR] CourierRobotWebAPI is not initialized")
            return -1
        time.sleep(0.5)
        
        result = self.courier_robot.pushrod_down()
        if not result.get('success', False):
            print(f"[ERROR] Failed to lower pushrod: {result.get('error', 'Unknown error')}")
            return -1
        time.sleep(7)
        
        result = self.courier_robot.platform_down()
        if not result.get('success', False):
            print(f"[ERROR] Failed to lower platform: {result.get('error', 'Unknown error')}")
            return -1
        time.sleep(10)
        
        result = self.courier_robot.pushrod_goto_height(target_height=20, mode='relative')
        if not result.get('success', False):
            print(f"[ERROR] Failed to set pushrod height: {result.get('error', 'Unknown error')}")
            return -1
        time.sleep(0.5)

        result = self.courier_robot.platform_hybrid_control(target_height=798, target_force=300)
        if not result.get('success', False):
            print(f"[ERROR] Failed to set platform hybrid control: {result.get('error', 'Unknown error')}")
            return -1
        time.sleep(0.5)

        # Step 2: Correct TCP pose
        print("\n" + "="*50)
        print("Step 2: Correcting TCP pose...")
        print("="*50)
        result = self.movel_to_correct_tcp_pose(
            tcp_x_to_rack=[-1, 0, 0],
            tcp_y_to_rack=[0, 1, 0],
            tcp_z_to_rack=[0, 0, -1],
            angle_deg=0,
        )
        if result != 0:
            print(f"[ERROR] Failed to correct TCP pose (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 3: Move to target position to position before insertion
        print("\n" + "="*50)
        print("Step 3: Moving to target position before insertion...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 2, 3],
            offset_in_rack=[0, -1.25, 0.20+self.tool_length]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to target position (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 4: Update server2base_matrix by vision positioning
        print("\n" + "="*50)
        print("Step 4: Updating server coordinate system...")
        print("="*50)
        
        result = self.update_server2base_by_positioning()
        if result != 0:
            print(f"[ERROR] Failed to update server coordinate system")
            return result
        time.sleep(0.5)

        # Step 5: Correct TCP pose again before insertion
        print("\n" + "="*50)
        print("Step 5: Correcting TCP pose...")
        print("="*50)
        result = self.movel_to_correct_tcp_pose(
            tcp_x_to_rack=[-1, 0, 0],
            tcp_y_to_rack=[0, 1, 0],
            tcp_z_to_rack=[0, 0, -1],
            angle_deg=-self.tool_angle_z,
        )
        if result != 0:
            print(f"[ERROR] Failed to correct TCP pose (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 6: Move to insert server position
        print("\n" + "="*50)
        print("Step 6: Moving to insert server position...")
        print("="*50)
        
        distance = 0.06

        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 2, 3],
            offset_in_rack=[0, -0.045, distance+self.tool_length]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to insert position (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 7: Execute force control task to align handle
        print("\n" + "="*50)
        print("Step 7: Aligning handle with force control...")
        print("="*50)
        result = self.force_task_touch_handle(distance)
        if result != 0:
            print(f"[ERROR] Failed to align handle (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 8: Execute force control task to insert server partially
        print("\n" + "="*50)
        print("Step 8: Inserting server partially with force control...")
        print("="*50)
        result = self.force_task_insert_server(distance=0.52)
        if result != 0:
            print(f"[ERROR] Failed to insert server partially (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 9: Courier robot down to transfer server weight to rack
        print("\n" + "="*50)
        print("Step 9: Lowering platform to transfer server weight...")
        print("="*50)
        if self.courier_robot is None:
            print("[ERROR] CourierRobotWebAPI is not initialized")
            return -1
        lift_result = self.courier_robot.platform_hybrid_control(target_height=780, target_force=350)
        if not lift_result.get('success', False):
            print(f"[ERROR] Failed to lower platform: {lift_result.get('error', 'Unknown error')}")
            return -1

        # Step 10: Slightly adjust height of end effector
        print("\n" + "="*50)
        print("Step 10: Adjusting end effector height...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.01, -0.01])
        if result != 0:
            print(f"[ERROR] Failed to adjust end effector height (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 11: Execute force control task to insert server completely
        print("\n" + "="*50)
        print("Step 11: Inserting server completely with force control...")
        print("="*50)
        result = self.force_task_insert_server(distance=0.60)
        if result != 0:
            print(f"[ERROR] Failed to insert server completely (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 12: Move away from the server after insertion
        print("\n" + "="*50)
        print("Step 12: Moving away from the server...")
        print("="*50)
        result = self.movel_in_rack_frame([0, 0, 0.20])
        if result != 0:
            print(f"[ERROR] Failed to move away from server (error code: {result})")
            return result
        time.sleep(0.5)

        # # Step 13: Final platform down to default position
        # print("\n" + "="*50)
        # print("Step 13: Lowering platform to default position...")
        # print("="*50)
        # if self.courier_robot is None:
        #     print("[ERROR] CourierRobotWebAPI is not initialized")
        #     return -1
        # lift_result = self.courier_robot.platform_hybrid_control(target_height=500, target_force=0)
        # if not lift_result.get('success', False):
        #     print(f"[ERROR] Failed to lower platform to default: {lift_result.get('error', 'Unknown error')}")
        #     return -1

        print("\n" + "="*70)
        print("INSERT SERVER SEQUENCE FINISHED SUCCESSFULLY")
        print("="*70)
        return 0


if __name__ == "__main__":
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='URWobjInsertServer - Insert server operation using server coordinate system')
    parser.add_argument('--robot-ip', type=str, default=None,
                       help='Robot IP address (default: from config file)')
    parser.add_argument('--robot-port', type=int, default=None,
                       help='Robot port (default: from config file)')
    parser.add_argument('--server-index', type=int, default=14,
                       help='Server index for operation (default: 14)')
    
    args = parser.parse_args()
    
    # Create URWobjInsertServer instance
    ur_insert_server = URWobjInsertServer(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        server_index=args.server_index
    )
    
    # ==============Display automatically loaded parameters=================
    print("\n" + "="*70)
    print("AUTOMATICALLY LOADED PARAMETERS")
    print("="*70)
    
    if ur_insert_server.camera_matrix is not None:
        print("\n✓ Camera Matrix loaded")
        print(ur_insert_server.camera_matrix)
        print("\n✓ Distortion Coefficients loaded")
        print(ur_insert_server.distortion_coefficients)
    
    if ur_insert_server.cam2end_matrix is not None:
        print("\n✓ Camera to End-effector Matrix loaded")
        print(ur_insert_server.cam2end_matrix)
    
    if ur_insert_server.rack_transformation_matrix_in_base is not None:
        print("\n✓ Rack Coordinate System loaded")
        print(f"  Origin: {ur_insert_server.rack_origin_in_base}")
    
    if ur_insert_server.server2base_matrix is not None:
        print("\n✓ Server Coordinate System loaded")
        server_origin = ur_insert_server.server2base_matrix[:3, 3]
        print(f"  Origin: x={server_origin[0]:.6f}, y={server_origin[1]:.6f}, z={server_origin[2]:.6f}")
        print(f"  Server Index: {ur_insert_server.server_index}")

    # ==============Execute the insert server task=================
    print("\n" + "="*70)
    print("STARTING TASK EXECUTION")
    print("="*70)
    
    try:
        result = ur_insert_server.execute_insert_server_sequence()
        
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
        ur_insert_server.cleanup()