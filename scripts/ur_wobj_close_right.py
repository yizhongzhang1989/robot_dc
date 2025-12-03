import os
import time
import numpy as np
import threading
from scipy.spatial.transform import Rotation as R
from ur_operate_wobj import UROperateWobj
from ur_positioning import URPositioning
import rclpy
from rclpy.executors import MultiThreadedExecutor


class URWobjCloseRight(UROperateWobj):
    def __init__(self, robot_ip=None, robot_port=None, server_index=14):
        """
        Initialize URWobjCloseRight instance
        Args:
            robot_ip: IP address of the UR15 robot. If None, loads from config file
            robot_port: Port number of the UR15 robot. If None, loads from config file
            server_index: Server index for operation (default: 14)
        """
        # Call parent class constructor
        super().__init__(robot_ip=robot_ip, robot_port=robot_port, server_index=server_index)
        
        # Load tool parameters from config file
        self._load_tool_pushpull_parameters_from_config()
        
        # Store operation name
        self.operation_name = "close_right"
        
        # Initialize URPositioning instance and ROS2 executor
        self.ur_positioning = None
        self.executor = None
        self.spin_thread = None
        self._initialize_ur_positioning()

        self._calculate_server2base(self.server_index)
        
        print(f"URWobjCloseRight initialized for server index: {server_index}")

    def _load_tool_pushpull_parameters_from_config(self):
        """
        Load tool_pushpull parameters from robot_config.yaml
        
        Loads:
            - tool_length: Length of the tool from flange to tip (meters)
            - tool_angle_z: Rotation angle around Z axis (degrees)
        """
        # Default values
        defaults = {
            'tool_length': 0.325,
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
                print(f"Using default tool_pushpull values: length={defaults['tool_length']}, angle_z={defaults['tool_angle_z']}")
                self.tool_length = defaults['tool_length']
                self.tool_angle_z = defaults['tool_angle_z']
                return
            
            # Load config file
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Get tool_pushpull parameters from ur15.tool.tool_pushpull
            tool_pushpull = config.get('ur15', {}).get('tool', {}).get('tool_pushpull', {})
            
            self.tool_length = tool_pushpull.get('length', defaults['tool_length'])
            self.tool_angle_z = tool_pushpull.get('angle_z', defaults['tool_angle_z'])
            
            print(f"✓ Loaded tool_pushpull parameters: length={self.tool_length}m, angle_z={self.tool_angle_z}°")
            
        except Exception as e:
            print(f"Error loading tool_pushpull parameters: {e}")
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
            
            self.ur_positioning = URPositioning(
                robot_ip=self.robot_ip,
                robot_port=self.robot_port,
                operation_name=self.operation_name
            )
            
            # Create executor and start spinning in separate thread
            self.executor = MultiThreadedExecutor()
            self.executor.add_node(self.ur_positioning)
            
            self.spin_thread = threading.Thread(target=self.executor.spin, daemon=True)
            self.spin_thread.start()
            
            print('✓ URPositioning initialized successfully')
            print('✓ ROS2 executor started in background thread')
            
            # Wait a moment for camera subscription to be ready
            time.sleep(2.0)
            
        except Exception as e:
            print(f'✗ Failed to initialize URPositioning: {e}')
            import traceback
            traceback.print_exc()
            self.ur_positioning = None

    def cleanup(self):
        """Cleanup resources"""
        try:
            if self.executor is not None:
                self.executor.shutdown()
                print('✓ ROS2 executor shutdown')
            
            if rclpy.ok():
                rclpy.shutdown()
                print('✓ ROS2 shutdown')
        except Exception as e:
            print(f'Warning: Error during cleanup: {e}')

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

    # ================================ Force Control Functions ================================
    def force_task_close_right_handle(self):
        """
        Execute force control task to close the right handle using server coordinate system.
        This function includes all 6 motions (0-5) in sequence.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.server2base_matrix is None:
            print("Server coordinate system transformation matrix not loaded")
            return -1

        # ==============Motion 1: push to close the handle (first push)=================
        print("\n[Motion 1] Pushing to close the handle (first push)...")

        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Extract rotation matrix from server transformation matrix and convert to rotation vector
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame using current position with server orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            server_rotation_vector[0], # rx (server orientation)
            server_rotation_vector[1], # ry
            server_rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (server coordinate system): {task_frame}")
        
        # Set force mode parameters for first push
        selection_vector = [1, 0, 0, 0, 0, 0]  # Enable force control in X direction only
        wrench = [-15, 0, 0, 0, 0, -1.0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - first push...")
        
        # Execute force control with distance-based termination
        result1 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=3,
            end_distance=[0.05, 0, 0, 0, 0, 0]
        )
        
        if result1 != 0:
            print(f"[ERROR] Motion 1 failed with code: {result1}")
            return result1
        
        print("[INFO] Motion 1 completed successfully")
        time.sleep(0.5)
        
        # ==============Motion 2: push to close the handle (main push)=================
        print("\n[Motion 2] Pushing to close the handle (main push)...")

        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Extract rotation matrix from server transformation matrix and convert to rotation vector
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame using current position with server orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            server_rotation_vector[0], # rx (server orientation)
            server_rotation_vector[1], # ry
            server_rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (server coordinate system): {task_frame}")
        
        # Set force mode parameters for main push
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in X, Y, Z directions
        wrench = [-20, 15, 0, 0, 0, -1.0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - main push...")
        
        # Execute force control with distance-based termination
        result2 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=3,
            end_distance=[0.11, 0.10, 0.05, 0, 0, 0]
        )
        
        if result2 != 0:
            print(f"[ERROR] Motion 2 failed with code: {result2}")
            return result2
        
        print("[INFO] Motion 2 completed successfully")
        time.sleep(0.5)

        # ==============Motion 3: Move along Z direction up=================
        print("\n[Motion 3] Moving along Z positive direction by 1cm...")
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Create target pose: move 1cm (0.01m) along Z positive direction
        target_pose = [
            tcp_pose[0],        # x (keep current)
            tcp_pose[1],        # y (keep current)
            tcp_pose[2] + 0.01, # z (move 1cm up)
            tcp_pose[3],        # rx (keep current orientation)
            tcp_pose[4],        # ry
            tcp_pose[5]         # rz
        ]
        
        print(f"[INFO] Target pose: {target_pose}")
        
        # Execute movel movement
        result3 = self.robot.movel(target_pose, a=0.1, v=0.05)
        
        if result3 != 0:
            print(f"[ERROR] Motion 3 failed with code: {result3}")
            return result3
        
        print("[INFO] Motion 3 completed successfully")
        time.sleep(0.5)

        # ==============Motion 4: push handle to completely close=================
        print("\n[Motion 4] Pushing handle to completely close...")
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Extract rotation matrix from server transformation matrix and convert to rotation vector
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame using current position with server orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            server_rotation_vector[0], # rx (server orientation)
            server_rotation_vector[1], # ry
            server_rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (server coordinate system): {task_frame}")
        
        # Set force mode parameters for complete close
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in X, Y, Z directions
        wrench = [0, 20, 0, 0, 0, -1.0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - completing close...")
        
        # Execute force control with force-based termination
        result4 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=2,
            end_force=[10, 20, 10, 0, 0, 0]
        )
        
        if result4 != 0:
            print(f"[ERROR] Motion 4 failed with code: {result4}")
            return result4
        
        print("[INFO] Motion 4 completed successfully")
        time.sleep(0.5)

        # ==============Motion 5: Move along Z direction down=================
        print("\n[Motion 5] Moving along Z negative direction by 1cm...")
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Create target pose: move 1cm (0.01m) along Z negative direction
        target_pose = [
            tcp_pose[0],        # x (keep current)
            tcp_pose[1],        # y (keep current)
            tcp_pose[2] - 0.01, # z (move 1cm down)
            tcp_pose[3],        # rx (keep current orientation)
            tcp_pose[4],        # ry
            tcp_pose[5]         # rz
        ]
        
        print(f"[INFO] Target pose: {target_pose}")
        
        # Execute movel movement
        result5 = self.robot.movel(target_pose, a=0.1, v=0.05)
        
        if result5 != 0:
            print(f"[ERROR] Motion 5 failed with code: {result5}")
            return result5
        
        print("[INFO] Motion 5 completed successfully")
        time.sleep(0.5)

        # ==============Motion 6: push server to end=================
        print("\n[Motion 6] Pushing to server to end...")
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Extract rotation matrix from server transformation matrix and convert to rotation vector
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame using current position with server orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            server_rotation_vector[0], # rx (server orientation)
            server_rotation_vector[1], # ry
            server_rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (server coordinate system): {task_frame}")
        
        # Set force mode parameters for locking
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Y direction
        wrench = [0, 60, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - locking knob...")
        
        # Execute force control with distance-based termination
        result6 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=1,
            end_time=3.0
        )
        
        if result6 != 0:
            print(f"[ERROR] Motion 6 failed with code: {result6}")
            return result6
        
        print("[INFO] Motion 6 completed successfully")
        time.sleep(0.5)

        # ==============Motion 7: push to lock the knob=================
        print("\n[Motion 7] Pushing to lock the knob...")
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Extract rotation matrix from server transformation matrix and convert to rotation vector
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame using current position with server orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            server_rotation_vector[0], # rx (server orientation)
            server_rotation_vector[1], # ry
            server_rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (server coordinate system): {task_frame}")
        
        # Set force mode parameters for locking
        selection_vector = [1, 1, 0, 0, 0, 0]  # Enable force control in X, Y, Z directions
        wrench = [25, 25, 0, 0, 0, -1.0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - locking knob...")
        
        # Execute force control with distance-based termination
        result7 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=3,
            end_distance=[0.07, 0.10, 0, 0, 0, 0]
        )
        
        if result7 != 0:
            print(f"[ERROR] Motion 7 failed with code: {result7}")
            return result7
        
        print("[INFO] Motion 7 completed successfully")
        time.sleep(0.5)
        
        print("[INFO] CloseRight handle sequence completed successfully")
        return 0

    def force_task_touch_server(self):
        """
        Execute force control task to touch the server using server coordinate system.
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
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            server_rotation_vector[0], # rx (server orientation)
            server_rotation_vector[1], # ry
            server_rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (server coordinate system): {task_frame}")
        
        # Set force mode parameters
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable force control in Z direction
        wrench = [0, 0, 15, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - touching server...")
        
        # Execute force control task with force-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=3,
            end_distance=[0, 0, 0.015, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    def force_task_pull_server(self):
        """
        Execute force control task to pull the server using server coordinate system.
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
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            server_rotation_vector[0], # rx (server orientation)
            server_rotation_vector[1], # ry
            server_rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (server coordinate system): {task_frame}")
        
        # Set force mode parameters
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Y direction
        wrench = [0, -50, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - pulling server...")
        
        # Execute force control task with distance-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=3,
            end_distance=[0, 0.15, 0, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    # ================================ Complete Sequence ================================
    def execute_close_right_sequence(self):
        """
        Execute the complete sequence for closing right handle operation.
        This includes:
        1. Correct TCP pose
        2. Move to target server position
        3. Update server2base by vision positioning
        4. Touch right handle
        5. Close right handle (6 motions)
        6. Move away and up
        7. Touch server
        8. Pull server
        9. Move away from server
        """
        print("\n" + "="*70)
        print("STARTING COMPLETE CLOSE RIGHT HANDLE SEQUENCE")
        print("="*70)

        # Step 1: Correct TCP pose
        print("\n" + "="*50)
        print("Step 1: Correcting TCP pose...")
        print("="*50)
        result = self.movel_to_correct_tcp_pose(
            tcp_x_to_rack=[1, 0, 0],
            tcp_y_to_rack=[0, 0, -1],
            tcp_z_to_rack=[0, 1, 0],
            angle_deg=0
        )
        if result != 0:
            print(f"[ERROR] Failed to correct TCP pose (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 2: Move to target position with offset
        print("\n" + "="*50)
        print("Step 2: Moving to target server position...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0, -0.70, 0]
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

        # Step 4: Move to target server position with offset
        print("\n" + "="*50)
        print("Step 4: Moving to target server position...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0, -0.65, 0]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to target position (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 5: Correct TCP pose
        print("\n" + "="*50)
        print("Step 5: Correcting TCP pose...")
        print("="*50)
        result = self.movel_to_correct_tcp_pose(
            tcp_x_to_rack=[1, 0, 0],
            tcp_y_to_rack=[0, 0, -1],
            tcp_z_to_rack=[0, 1, 0],
            angle_deg=-self.tool_angle_z
        )
        if result != 0:
            print(f"[ERROR] Failed to correct TCP pose (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 6: Move to close left handle position
        print("\n" + "="*50)
        print("Step 6: Moving to close left handle position...")
        print("="*50)
        result = self.movel_in_server_frame([0.24, 0, 0])
        if result != 0:
            print(f"[ERROR] Failed to move to close left handle position (error code: {result})")
            return result
        time.sleep(0.5)

        result = self.movel_in_server_frame([0, 0.13, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away from right knob")
            return result
        time.sleep(0.5)

        # Step 7: Close right handle (complete sequence with 6 motions)
        print("\n" + "="*50)
        print("Step 7: Closing right handle...")
        print("="*50)
        result = self.force_task_close_right_handle()
        if result != 0:
            print(f"[ERROR] Failed to close right handle (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 8: Move away from the server
        print("\n" + "="*50)
        print("Step 8: Moving away from the server...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.10, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 9: Move to leave the server and prepare for pull out
        print("\n" + "="*50)
        print("Step 9: Moving to leave the server and prepare for pull out...")
        print("="*50)
        result = self.movel_in_server_frame([0, 0, -0.025])
        if result != 0:
            return result
        time.sleep(0.5)

        result = self.movel_in_server_frame([-0.11, 0.09, 0])
        if result != 0:
            return result
        time.sleep(0.5)

        # Step 10: Touch server
        print("\n" + "="*50)
        print("Step 10: Touching server...")
        print("="*50)
        result = self.force_task_touch_server()
        if result != 0:
            print(f"[ERROR] Failed to touch server (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 11: Pull server
        print("\n" + "="*50)
        print("Step 11: Pulling server...")
        print("="*50)
        result = self.force_task_pull_server()
        if result != 0:
            print(f"[ERROR] Failed to pull server (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 12: Move to leave the server
        print("\n" + "="*50)
        print("Step 12: Moving to leave the server...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.015, -0.03])
        if result != 0:
            print(f"[ERROR] Failed to leave server (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 13: Move away from the server
        print("\n" + "="*50)
        print("Step 13: Moving away from the server...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.15, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away (error code: {result})")
            return result
        time.sleep(0.5)
        
        print("\n" + "="*70)
        print("COMPLETE CLOSE RIGHT HANDLE SEQUENCE FINISHED SUCCESSFULLY")
        print("="*70)
        return 0


if __name__ == "__main__":
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='URWobjCloseRight - Close right handle operation using server coordinate system')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot (default: 192.168.1.15)')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot (default: 30002)')
    parser.add_argument('--server-index', type=int, default=14,
                       help='Server index for operation (default: 14)')
    
    args = parser.parse_args()
    
    # Create URWobjCloseRight instance
    ur_wobj_close_right = URWobjCloseRight(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        server_index=args.server_index
    )
    
    # ==============Display automatically loaded parameters=================
    print("\n" + "="*70)
    print("AUTOMATICALLY LOADED PARAMETERS")
    print("="*70)
    
    if ur_wobj_close_right.camera_matrix is not None:
        print("\n✓ Camera Matrix loaded")
        print(ur_wobj_close_right.camera_matrix)
        print("\n✓ Distortion Coefficients loaded")
        print(ur_wobj_close_right.distortion_coefficients)
    
    if ur_wobj_close_right.cam2end_matrix is not None:
        print("\n✓ Camera to End-effector Matrix loaded")
        print(ur_wobj_close_right.cam2end_matrix)
    
    if ur_wobj_close_right.rack_transformation_matrix_in_base is not None:
        print("\n✓ Rack Coordinate System loaded")
        print(f"  Origin: {ur_wobj_close_right.rack_origin_in_base}")
    
    if ur_wobj_close_right.server2base_matrix is not None:
        print("\n✓ Server Coordinate System loaded")
        server_origin = ur_wobj_close_right.server2base_matrix[:3, 3]
        print(f"  Origin: x={server_origin[0]:.6f}, y={server_origin[1]:.6f}, z={server_origin[2]:.6f}")
        print(f"  Server Index: {ur_wobj_close_right.server_index}")

    # ==============Execute the close right handle task=================
    print("\n" + "="*70)
    print("STARTING TASK EXECUTION")
    print("="*70)
    
    try:
        result = ur_wobj_close_right.execute_close_right_sequence()
        
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
        ur_wobj_close_right.cleanup()
