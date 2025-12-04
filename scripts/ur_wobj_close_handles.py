import os
import time
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R
from ur_operate_wobj import UROperateWobj


class URWobjCloseHandles(UROperateWobj):
    def __init__(self, robot_ip=None, robot_port=None, server_index=14):
        """
        Initialize URWobjCloseHandles instance
        Args:
            robot_ip: IP address of the UR15 robot. If None, loads from config file
            robot_port: Port number of the UR15 robot. If None, loads from config file
            server_index: Server index for operation (default: 14)
        """
        # Call parent class constructor
        super().__init__(robot_ip=robot_ip, robot_port=robot_port, server_index=server_index)
        
        # Load tool parameters from config file
        self._load_tool_pushpull_parameters_from_config()

        self._calculate_server2base(self.server_index)
        
        print(f"URWobjCloseHandles initialized for server index: {server_index}")

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

    # ================================ Force Control Functions ================================
    def force_task_touch_left_handle(self):
        """
        Execute force control task to touch the left handle using TCP coordinate system.
        The robot will apply a downward force (relative to TCP) to make contact with the left handle.
        Returns: result from force_control_task
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        # Get current TCP pose to use as task frame (for force control in TCP coordinate system)
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Set force mode parameters
        task_frame = tcp_pose  # Use TCP coordinate system
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in XYZ directions
        wrench = [0, 0, 25, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - touching left handle...")
        
        # Execute force control task with distance-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=3,
            end_distance=[0.02, 0.02, 0.10, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    def force_task_touch_right_handle(self):
        """
        Execute force control task to touch the right handle using TCP coordinate system.
        The robot will apply a downward force (relative to TCP) to make contact with the right handle.
        Returns: result from force_control_task
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        # Get current TCP pose to use as task frame (for force control in TCP coordinate system)
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Set force mode parameters
        task_frame = tcp_pose  # Use TCP coordinate system
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in XYZ directions
        wrench = [0, 0, 25, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - touching right handle...")
        
        # Execute force control task with force-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=2,
            end_force=[10, 10, 15, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    def force_task_close_left_handle(self):
        """
        Execute force control task to close the left handle using server coordinate system.
        This function includes multiple motions in sequence.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.server2base_matrix is None:
            print("Server coordinate system transformation matrix not loaded")
            return -1
        
        # ==============Motion 1: push to unlock the handle=================
        print("\n[Motion 1] Pushing to unlock the handle...")
        
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
        
        # Set force mode parameters for unlocking
        selection_vector = [1, 0, 1, 0, 0, 0]  # Enable force control in X and Z directions
        wrench = [25, 0, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - unlocking...")
        
        # Execute force control with distance-based termination
        result1 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=3,
            end_distance=[0.06, 0, 0.05, 0, 0, 0]
        )
        
        if result1 != 0:
            print(f"[ERROR] Motion 1 failed with code: {result1}")
            return result1
        
        print("[INFO] Motion 1 completed successfully")
        time.sleep(0.5)

        # ==============Motion 2: push to close the handle=================
        print("\n[Motion 2] Pushing to close the handle...")
        
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
        
        # Set force mode parameters for closing
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in X, Y, Z directions
        wrench = [25, 25, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - closing handle...")
        
        # Execute force control with distance-based termination
        result2 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=3,
            end_distance=[0.12, 0.15, 0.05, 0, 0, 0]
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
        
        # Set force mode parameters
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in X, Y, Z directions
        wrench = [0, 20, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - pushing handle completely...")
        
        # Execute force control with force-based termination
        result4 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=2,
            end_force=[10, 15, 10, 0, 0, 0]
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

        # ==============Motion 6: push to lock the knob=================
        print("\n[Motion 6] Pushing to lock the knob...")
        
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
        selection_vector = [1, 1, 0, 0, 0, 0]  # Enable force control in X and Y directions
        wrench = [-25, 30, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - locking knob...")
        
        # Execute force control with distance-based termination
        result6 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=3,
            end_distance=[0.09, 0.10, 0, 0, 0, 0]
        )
        
        if result6 != 0:
            print(f"[ERROR] Motion 6 failed with code: {result6}")
            return result6
        
        print("[INFO] Motion 6 completed successfully")
        time.sleep(0.5)
        
        print("[INFO] Close left handle sequence completed successfully")
        return 0

    def force_task_close_right_handle(self):
        """
        Execute force control task to close the right handle using server coordinate system.
        This function includes multiple motions in sequence.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.server2base_matrix is None:
            print("Server coordinate system transformation matrix not loaded")
            return -1
        
        # ==============Motion 1: Move along Z direction up=================
        print("\n[Motion 1] Moving along Z positive direction by 1cm...")
        
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
        result1 = self.robot.movel(target_pose, a=0.1, v=0.05)
        
        if result1 != 0:
            print(f"[ERROR] Motion 1 failed with code: {result1}")
            return result1
        
        print("[INFO] Motion 1 completed successfully")
        time.sleep(0.5)
        
        # Extract server rotation (reusable for all motions)
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # ==============Motion 2: push to unlock the handle=================
        print("\n[Motion 2] Pushing to unlock the handle...")
        
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
        
        # Set force mode parameters for unlocking
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in X and Z directions
        wrench = [0, 20, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - unlocking...")
        
        # Execute force control with distance-based termination
        result2 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=2,
            end_force=[0, 10, 0, 0, 0, 0]
        )
        
        if result2 != 0:
            print(f"[ERROR] Motion 2 failed with code: {result2}")
            return result2
        
        print("[INFO] Motion 2 completed successfully")
        time.sleep(0.5)

        # ==============Motion 3: Move along Z direction down=================
        print("\n[Motion 3] Moving along Z negative direction by 5mm...")
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Create target pose: move 5mm (0.005m) along Z negative direction
        target_pose = [
            tcp_pose[0],         # x (keep current)
            tcp_pose[1],         # y (keep current)
            tcp_pose[2] - 0.01, # z (move 10mm down)
            tcp_pose[3],         # rx (keep current orientation)
            tcp_pose[4],         # ry
            tcp_pose[5]          # rz
        ]
        
        print(f"[INFO] Target pose: {target_pose}")
        
        # Execute movel movement
        result3 = self.robot.movel(target_pose, a=0.1, v=0.05)
        
        if result3 != 0:
            print(f"[ERROR] Motion 3 failed with code: {result3}")
            return result3
        
        print("[INFO] Motion 3 completed successfully")
        time.sleep(0.5)

        # ==============Motion 4: push to lock the knob=================
        print("\n[Motion 4] Pushing to lock the knob...")
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
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
        selection_vector = [1, 1, 0, 0, 0, 0]  # Enable force control in X, Y, Z directions
        wrench = [25, 30, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - locking knob...")
        
        # Execute force control with distance-based termination
        result4 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=3,
            end_distance=[0.09, 0.05, 0.05, 0, 0, 0]
        )
        
        if result4 != 0:
            print(f"[ERROR] Motion 4 failed with code: {result4}")
            return result4

        print("[INFO] Motion 4 completed successfully")
        time.sleep(0.5)
        
        print("[INFO] Close right handle sequence completed successfully")
        return 0


    # ================================ Task Execution Methods ================================
    def execute_complete_close_handles_sequence(self):
        """
        Execute complete close handles sequence following the task flow
        
        Steps:
        1. Correct TCP pose
        2. Move to left handle position
        3. Move to close left handle position
        4. Close left handle
        5. Move away from left handle
        6. Move to right handle position
        7. Touch right handle
        8. Close right handle
        9. Move away from right handle
        """
        print("\n" + "="*70)
        print("STARTING COMPLETE CLOSE HANDLES SEQUENCE")
        print("="*70)
        
        # ===============Close left handle=================
        # Step 1: Correct TCP pose to align with rack coordinate system
        print("\n" + "="*50)
        print("Step 1: Correcting TCP pose...")
        print("="*50)
        result = self.movel_to_correct_tcp_pose(
            tcp_x_to_rack=[1, 0, 0],
            tcp_y_to_rack=[0, 0, -1],
            tcp_z_to_rack=[0, 1, 0],
            angle_deg=-self.tool_angle_z
        )
        if result != 0:
            print(f"[ERROR] Failed to correct TCP pose")
            return result
        time.sleep(0.5)
        
        # Step 2: Move to left handle position
        print("\n" + "="*50)
        print("Step 2: Moving to left handle position...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0, -0.325-self.tool_length, 0]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to left handle position")
            return result
        time.sleep(0.5)

        # Step 3: Move to close left handle position
        print("\n" + "="*50)
        print("Step 3: Moving to close left handle position...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[-0.24, -0.19-self.tool_length, 0]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to left handle position")
            return result
        time.sleep(0.5)

        # Step 4: Close left handle
        print("\n" + "="*50)
        print("Step 4: Closing left handle...")
        print("="*50)
        result = self.force_task_close_left_handle()
        if result != 0:
            print(f"[ERROR] Failed to close left handle")
            return result
        time.sleep(0.5)

        # Step 5: Move away from left handle
        print("\n" + "="*50)
        print("Step 5: Moving away from left handle...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.15, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away from left handle")
            return result
        time.sleep(0.5)

        # ===============Close right handle=================
        # Step 6: Move to right handle position
        print("\n" + "="*50)
        print("Step 6: Moving to right handle position...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0.05, -0.15-self.tool_length, 0]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to right handle position")
            return result
        time.sleep(0.5)

        # Step 7: Touch right handle
        print("\n" + "="*50)
        print("Step 7: Touching right handle...")
        print("="*50)
        result = self.force_task_touch_right_handle()
        if result != 0:
            print(f"[ERROR] Failed to touch right handle")
            return result
        time.sleep(0.5)

        # Step 8: Close right handle
        print("\n" + "="*50)
        print("Step 8: Closing right handle...")
        print("="*50)
        result = self.force_task_close_right_handle()
        if result != 0:
            print(f"[ERROR] Failed to close right handle")
            return result
        time.sleep(0.5)

        # Step 9: Move away from right handle
        print("\n" + "="*50)
        print("Step 9: Moving away from right handle...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.25, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away from right handle")
            return result
        time.sleep(0.5)
        
        print("\n" + "="*70)
        print("COMPLETE CLOSE HANDLES SEQUENCE FINISHED SUCCESSFULLY")
        print("="*70)
        return 0


if __name__ == "__main__":
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='URWobjCloseHandles - Close handles operation using server coordinate system')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot (default: 192.168.1.15)')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot (default: 30002)')
    parser.add_argument('--server-index', type=int, default=14,
                       help='Target server index (default: 14)')
    
    args = parser.parse_args()
    
    # Create URWobjCloseHandles instance
    ur_wobj_close_handles = URWobjCloseHandles(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        server_index=args.server_index
    )
    
    # ==============Display automatically loaded parameters=================
    print("\n" + "="*70)
    print("AUTOMATICALLY LOADED PARAMETERS")
    print("="*70)
    
    if ur_wobj_close_handles.camera_matrix is not None:
        print("\n✓ Camera Matrix loaded")
        print(ur_wobj_close_handles.camera_matrix)
        print("\n✓ Distortion Coefficients loaded")
        print(ur_wobj_close_handles.distortion_coefficients)
    
    if ur_wobj_close_handles.cam2end_matrix is not None:
        print("\n✓ Camera to End-effector Matrix loaded")
        print(ur_wobj_close_handles.cam2end_matrix)
    
    if ur_wobj_close_handles.rack_transformation_matrix_in_base is not None:
        print("\n✓ Rack Coordinate System loaded")
        print(f"  Origin: {ur_wobj_close_handles.rack_origin_in_base}")
    
    if ur_wobj_close_handles.server2base_matrix is not None:
        print("\n✓ Server Coordinate System loaded")
        server_origin = ur_wobj_close_handles.server2base_matrix[:3, 3]
        print(f"  Origin: {server_origin}")
        print(f"  Server Index: {ur_wobj_close_handles.server_index}")
    
    print(f"\n✓ Tool Parameters loaded")
    print(f"  Tool Length: {ur_wobj_close_handles.tool_length}m")
    print(f"  Tool Angle Z: {ur_wobj_close_handles.tool_angle_z}°")

    # ==============Execute the close handles task=================
    print("\n" + "="*70)
    print("STARTING TASK EXECUTION")
    print("="*70)
    
    try:
        result = ur_wobj_close_handles.execute_complete_close_handles_sequence()
        
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
