import os
import time
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R
from ur_operate_wobj import UROperateWobj


class UROperateWobjUnlockKnob(UROperateWobj):
    def __init__(self, robot_ip=None, robot_port=None, server_index=14):
        """
        Initialize UROperateWobjUnlockKnob instance
        Args:
            robot_ip: IP address of the UR15 robot. If None, loads from config file
            robot_port: Port number of the UR15 robot. If None, loads from config file
            server_index: Server index for target position (default: 14)
        """
        # Call parent class constructor
        super().__init__(robot_ip=robot_ip, robot_port=robot_port, server_index=server_index)
        
        # Load tool parameters from config file
        self._load_tool_rotate_parameters_from_config()
        
        # Recalculate server2base with specific offset for unlock knob task
        # This offset aligns with the offset used in movel_to_target_position
        self.server2base_matrix = self._calculate_server2base(index=self.server_index)
        
        print(f"UROperateWobjUnlockKnob initialized for server index: {server_index}")

    def _load_tool_rotate_parameters_from_config(self):
        """
        Load tool_rotate parameters from robot_config.yaml
        
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
                print(f"Using default tool_rotate values: length={defaults['tool_length']}, angle_z={defaults['tool_angle_z']}")
                self.tool_length = defaults['tool_length']
                self.tool_angle_z = defaults['tool_angle_z']
                return
            
            # Load config file
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Get tool_rotate parameters from ur15.tool.tool_rotate
            tool_rotate = config.get('ur15', {}).get('tool', {}).get('tool_rotate', {})
            
            self.tool_length = tool_rotate.get('length', defaults['tool_length'])
            self.tool_angle_z = tool_rotate.get('angle_z', defaults['tool_angle_z'])
            
            print(f"✓ Loaded tool_rotate parameters: length={self.tool_length}m, angle_z={self.tool_angle_z}°")
            
        except Exception as e:
            print(f"Error loading tool_rotate parameters: {e}")
            print(f"Using default values: length={defaults['tool_length']}, angle_z={defaults['tool_angle_z']}")
            self.tool_length = defaults['tool_length']
            self.tool_angle_z = defaults['tool_angle_z']


    # ================================= Movement Functions ================================
    def force_task_touch_knob(self):
        """
        Execute force control task to touch the knob using TCP coordinate system.
        The robot will apply a downward force (relative to TCP) to make contact with the knob.
        Returns: result from force_control_task
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        # Get current TCP pose to use as task frame (for force control in TCP coordinate system)
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Set force mode parameters
        task_frame = tcp_pose  # set task frame to TCP pose
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in XYZ direction relative to task frame
        wrench = [0, 0, 20, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force task: 'Touch Knob'...")
        
        # Execute force control task with force-based termination (end_type=2)
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=2,
            end_force=[15, 15, 25, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    def force_task_unlock_left_knob(self):
        """
        Execute force control task to unlock the left knob using TCP coordinate system.
        Three-step process:
        1. Rotate counter-clockwise around TCP Z-axis to unlock (stops at specified angle)
        2. Release the spring in the knob
        3. Rotate clockwise to return
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.server2base_matrix is None:
            print("Server coordinate system transformation matrix not loaded")
            return -1
        
        # ==============Motion 1: Rotate around Z axis to unlock the knob=================
        print("\n[Motion 1] Rotating counter-clockwise to unlock knob...")
        # Get current TCP pose for position
        tcp_pose = self.robot.get_actual_tcp_pose()
        if tcp_pose is None or len(tcp_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract server rotation from server2base matrix
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame: TCP position + server orientation
        task_frame = [
            tcp_pose[0],              # x from TCP
            tcp_pose[1],              # y from TCP
            tcp_pose[2],              # z from TCP
            server_rotation_vector[0],  # rx from server
            server_rotation_vector[1],  # ry from server
            server_rotation_vector[2]   # rz from server
        ]
        
        print(f"[INFO] Task frame (TCP position + server orientation): {task_frame}")
        
        # Set force mode parameters for unlocking
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable torque control in Z direction only
        wrench = [0, 0, -20, 0, 0, 0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - unlocking...")
        
        # Execute force control with angle-based termination
        result1 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=2,
            end_force=[0, 0, 20, 0, 0, 0]
        )
        
        if result1 != 0:
            print(f"[ERROR] Motion 1 failed with code: {result1}")
            return result1
        
        print("[INFO] Motion 1 completed successfully")
        time.sleep(0.5)

        # ==============Motion 2: Rotate around Z axis to unlock the knob=================
        print("\n[Motion 2] Rotating counter-clockwise to unlock knob...")
        # Get current TCP pose for position
        tcp_pose = self.robot.get_actual_tcp_pose()
        if tcp_pose is None or len(tcp_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract server rotation from server2base matrix
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame: TCP position + server orientation
        task_frame = [
            tcp_pose[0],              # x from TCP
            tcp_pose[1],              # y from TCP
            tcp_pose[2],              # z from TCP
            server_rotation_vector[0],  # rx from server
            server_rotation_vector[1],  # ry from server
            server_rotation_vector[2]   # rz from server
        ]
        
        print(f"[INFO] Task frame (TCP position + server orientation): {task_frame}")
        
        # Set force mode parameters for unlocking
        selection_vector = [0, 1, 0, 0, 1, 0]  # Enable torque control in Z direction only
        wrench = [0, 2, 0, 0, -1.0, 0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - unlocking...")
        
        # Execute force control with angle-based termination
        result2 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.01,
            end_type=4,
            end_angle=155.0  # Stop after rotating 155 degrees
        )
        
        if result2 != 0:
            print(f"[ERROR] Motion 2 failed with code: {result2}")
            return result2
        
        print("[INFO] Motion 2 completed successfully")
        time.sleep(0.5)

        # ==============Motion 3: release the spring in the knob=================
        print("\n[Motion 3] Releasing the spring in the knob...")

        # Get current TCP pose for position
        tcp_pose = self.robot.get_actual_tcp_pose()
        if tcp_pose is None or len(tcp_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Create task frame: TCP position + server orientation
        task_frame = [
            tcp_pose[0],              # x from TCP
            tcp_pose[1],              # y from TCP
            tcp_pose[2],              # z from TCP
            server_rotation_vector[0],  # rx from server
            server_rotation_vector[1],  # ry from server
            server_rotation_vector[2]   # rz from server
        ]
        print(f"[INFO] Task frame (TCP position + server orientation): {task_frame}")
        
        # Set force mode parameters for releasing spring
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Y direction only
        wrench = [0, -15.0, 0, 0, 0, 0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - releasing spring...")
        
        # Execute force control with distance-based termination
        result3 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.01,
            end_type=3,
            end_distance=[0, 0.003, 0, 0, 0, 0]
        )
        
        if result3 != 0:
            print(f"[ERROR] Motion 3 failed with code: {result3}")
            return result3
        
        print("[INFO] Motion 3 completed successfully")
        time.sleep(0.5)
        
        # ==============Motion 4: Rotate back around Z axis to leave the knob=================
        print("\n[Motion 4] Rotating clockwise to return...")
        # Get updated TCP pose for position
        tcp_pose = self.robot.get_actual_tcp_pose()
        if tcp_pose is None or len(tcp_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Create task frame: TCP position + server orientation
        task_frame = [
            tcp_pose[0],              # x from TCP
            tcp_pose[1],              # y from TCP
            tcp_pose[2],              # z from TCP
            server_rotation_vector[0],  # rx from server
            server_rotation_vector[1],  # ry from server
            server_rotation_vector[2]   # rz from server
        ]
        print(f"[INFO] Task frame (TCP position + server orientation): {task_frame}")
        
        # Set force mode parameters for return motion
        selection_vector = [0, 0, 0, 0, 1, 0]  # Enable torque control in Y direction only
        wrench = [0, 0, 0, 0, 1.0, 0]  # Apply 1.0 Nm torque (clockwise)
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - returning...")
        
        # Execute force control with angle-based termination
        result4 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.005,
            end_type=4,
            end_angle=15.0  # Stop after rotating 15 degrees
        )
        
        if result4 != 0:
            print(f"[ERROR] Motion 4 failed with code: {result4}")
            return result4

        print("[INFO] Motion 4 completed successfully")
        time.sleep(0.5)

        print("[INFO] Left knob unlock sequence completed successfully")
        return 0
    
    def force_task_unlock_right_knob(self):
        """
        Execute force control task to unlock the right knob using TCP coordinate system.
        Three-step process:
        1. Rotate clockwise around TCP Z-axis to unlock (stops at specified angle)
        2. Release the spring in the knob
        3. Rotate counter-clockwise to return
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.server2base_matrix is None:
            print("Server coordinate system transformation matrix not loaded")
            return -1
        
        # ==============Motion 1: Rotate around Z axis to unlock the knob=================
        print("\n[Motion 1] Rotating counter-clockwise to unlock knob...")
        # Get current TCP pose for position
        tcp_pose = self.robot.get_actual_tcp_pose()
        if tcp_pose is None or len(tcp_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract server rotation from server2base matrix
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame: TCP position + server orientation
        task_frame = [
            tcp_pose[0],              # x from TCP
            tcp_pose[1],              # y from TCP
            tcp_pose[2],              # z from TCP
            server_rotation_vector[0],  # rx from server
            server_rotation_vector[1],  # ry from server
            server_rotation_vector[2]   # rz from server
        ]
        print(f"[INFO] Task frame (TCP position + server orientation): {task_frame}")
        
        # Set force mode parameters for unlocking
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable torque control in Z direction only
        wrench = [0, 0, -20, 0, 0, 0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - unlocking...")
        
        # Execute force control with angle-based termination
        result1 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=2,
            end_force=[0, 0, 20, 0, 0, 0]
        )
        
        if result1 != 0:
            print(f"[ERROR] Motion 1 failed with code: {result1}")
            return result1
        
        print("[INFO] Motion 1 completed successfully")
        time.sleep(0.5)

        # ==============Motion 2: Rotate around Z axis to unlock the knob=================
        print("\n[Motion 2] Rotating clockwise to unlock knob...")

        # Get current TCP pose for position
        tcp_pose = self.robot.get_actual_tcp_pose()
        if tcp_pose is None or len(tcp_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract server rotation from server2base matrix
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame: TCP position + server orientation
        task_frame = [
            tcp_pose[0],              # x from TCP
            tcp_pose[1],              # y from TCP
            tcp_pose[2],              # z from TCP
            server_rotation_vector[0],  # rx from server
            server_rotation_vector[1],  # ry from server
            server_rotation_vector[2]   # rz from server
        ]
        
        print(f"[INFO] Task frame (TCP position + server orientation): {task_frame}")

        # Set force mode parameters for unlocking
        selection_vector = [0, 1, 0, 0, 1, 0]  # Enable torque control in Z direction only
        wrench = [0, 2, 0, 0, 1.0, 0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - unlocking...")
        
        # Execute force control with angle-based termination
        result2 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.01,
            end_type=4,
            end_angle=160.0  # Stop after rotating 150 degrees
        )
        
        if result2 != 0:
            print(f"[ERROR] Motion 2 failed with code: {result2}")
            return result2
        
        print("[INFO] Motion 2 completed successfully")
        time.sleep(0.5)

        # ==============Motion 3: release the spring in the knob=================
        print("\n[Motion 3] Releasing the spring in the knob...")
        # Get current TCP pose for position
        tcp_pose = self.robot.get_actual_tcp_pose()
        if tcp_pose is None or len(tcp_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Create task frame: TCP position + server orientation
        task_frame = [
            tcp_pose[0],              # x from TCP
            tcp_pose[1],              # y from TCP
            tcp_pose[2],              # z from TCP
            server_rotation_vector[0],  # rx from server
            server_rotation_vector[1],  # ry from server
            server_rotation_vector[2]   # rz from server
        ]
        print(f"[INFO] Task frame (TCP position + server orientation): {task_frame}")
        
        # Set force mode parameters for releasing spring
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Y direction only
        wrench = [0, -15.0, 0, 0, 0, 0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - releasing spring...")
        
        # Execute force control with distance-based termination
        result3 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.002,
            end_type=3,
            end_distance=[0, 0.003, 0, 0, 0, 0]
        )
        
        if result3 != 0:
            print(f"[ERROR] Motion 3 failed with code: {result3}")
            return result3
        
        print("[INFO] Motion 3 completed successfully")
        time.sleep(0.5)
        
        # ==============Motion 4: Rotate back around Z axis to leave the knob=================
        print("\n[Motion 4] Rotating counter-clockwise to return...")
        # Get updated TCP pose for position
        tcp_pose = self.robot.get_actual_tcp_pose()
        if tcp_pose is None or len(tcp_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Create task frame: TCP position + server orientation
        task_frame = [
            tcp_pose[0],              # x from TCP
            tcp_pose[1],              # y from TCP
            tcp_pose[2],              # z from TCP
            server_rotation_vector[0],  # rx from server
            server_rotation_vector[1],  # ry from server
            server_rotation_vector[2]   # rz from server
        ]
        print(f"[INFO] Task frame (TCP position + server orientation): {task_frame}")
        
        # Set force mode parameters for return motion
        selection_vector = [0, 0, 0, 0, 1, 0]  # Enable torque control in Y direction only
        wrench = [0, 0, 0, 0, -1.0, 0]  # Apply -1.0 Nm torque (counter-clockwise)
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - returning...")
        
        # Execute force control with angle-based termination
        result4 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.005,
            end_type=4,
            end_angle=15.0  # Stop after rotating 15 degrees
        )
        
        if result4 != 0:
            print(f"[ERROR] Motion 4 failed with code: {result4}")
            return result4

        print("[INFO] Motion 4 completed successfully")
        time.sleep(0.5)

        print("[INFO] Right knob unlock sequence completed successfully")
        return 0

    def force_task_open_handle(self):
        """
        Execute force control task to touch the knob2 using TCP coordinate system.
        The robot will apply a downward force (relative to TCP) to make contact with the knob2.
        Returns: result from force_control_task
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.server2base_matrix is None:
            print("Server coordinate system transformation matrix not loaded")
            return -1
        
        # ==============Motion 1: open the handle================
        print("\n[Motion 1] Rotating counter-clockwise to unlock knob...")
        # Get current TCP pose for position
        tcp_pose = self.robot.get_actual_tcp_pose()
        if tcp_pose is None or len(tcp_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract server rotation from server2base matrix
        server_rotation_matrix = self.server2base_matrix[:3, :3]
        server_rotation = R.from_matrix(server_rotation_matrix)
        server_rotation_vector = server_rotation.as_rotvec()
        
        # Create task frame: TCP position + server orientation
        task_frame = [
            tcp_pose[0],              # x from TCP
            tcp_pose[1],              # y from TCP
            tcp_pose[2],              # z from TCP
            server_rotation_vector[0],  # rx from server
            server_rotation_vector[1],  # ry from server
            server_rotation_vector[2]   # rz from server
        ]
        print(f"[INFO] Task frame (TCP position + server orientation): {task_frame}")
        
        # Set force mode parameters
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable force control in Z direction (now relative to task frame)
        wrench = [0, 0, 20, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task...")
        
        # Execute force control task with force-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=3,
            end_distance=[0,0,0.02,0,0,0]
        )
        time.sleep(0.5)
        return result


    # ================================ Task Execution Methods ================================
    def execute_complete_unlock_sequence(self):
        """
        Execute complete unlock sequence for both knobs following the task flow
        """
        print("\n" + "="*70)
        print("STARTING COMPLETE KNOB UNLOCK SEQUENCE")
        print("="*70)
        
        # ===============Execute the knob unlocking task=================
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
        
        # Step 2: Move to target position (left knob) using linear movement
        print("\n" + "="*50)
        print("Step 2: Moving to target position...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0, -0.275-self.tool_length, 0]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to left knob position")
            return result
        time.sleep(0.5)

        print("\n" + "="*50)
        print("Step 3: Moving to left knob position")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[-0.105, -0.10-self.tool_length, 0.022]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to left knob position")
            return result
        time.sleep(0.5)

        # ==============Unlock the left knob=================
        # Step 4: Force control to touch the knob
        print("\n" + "="*50)
        print("Step 4: Touching left knob...")
        print("="*50)
        result = self.force_task_touch_knob()
        if result != 0:
            print(f"[ERROR] Failed to touch left knob")
            return result
        time.sleep(0.5)

        # Step 5: Move away from the knob slightly
        print("\n" + "="*50)
        print("Step 5: Moving away from the knob slightly...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.001, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away from left knob")
            return result
        time.sleep(0.5)

        # Step 6: Force control to unlock the left knob
        print("\n" + "="*50)
        print("Step 6: Unlocking left knob...")
        print("="*50)
        result = self.force_task_unlock_left_knob()
        if result != 0:
            print(f"[ERROR] Failed to unlock left knob")
            return result
        time.sleep(0.5)

        # Step 7: Move away from the server
        print("\n" + "="*50)
        print("Step 7: Moving away from server...")
        print("="*50)
        self.movel_in_server_frame([0, -0.25, 0])
        time.sleep(0.5)

        print("\n" + "="*50)
        self.movel_in_server_frame([0.03, 0.23, -0.04])
        time.sleep(0.5)

        # Step 8: Force task to open the left handle
        print("\n" + "="*50)
        print("Step 8: Opening left handle...")
        print("="*50)
        result = self.force_task_open_handle()
        if result != 0:
            print(f"[ERROR] Failed to open left handle")
            return result
        time.sleep(0.5)

        # move away from the server
        print("\n" + "="*50)
        result = self.movel_in_server_frame([0, -0.20, -0.02])
        if result != 0:
            print(f"[ERROR] Failed to move away from left handle")
            return result
        time.sleep(0.5)

        # ==============Unlock the right knob=================
        # Step 9: Correct TCP pose to align with rack coordinate system
        print("\n" + "="*50)
        print("Step 9: Correcting TCP pose...")
        print("="*50)
        result = self.movel_to_correct_tcp_pose(
            tcp_x_to_rack=[1, 0, 0],
            tcp_y_to_rack=[0, 0, -1],
            tcp_z_to_rack=[0, 1, 0],
            angle_deg=-self.tool_angle_z-180
        )
        if result != 0:
            print(f"[ERROR] Failed to correct TCP pose")
            return result
        time.sleep(0.5)

        # Step 10: Move to target position (right knob) using linear movement
        print("\n" + "="*50)
        print("Step 10: Moving to target position...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0, -0.275-self.tool_length, 0]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to left knob position")
            return result
        time.sleep(0.5)

        print("\n" + "="*50)
        print("Step 11: Moving to right knob position")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0.105, -0.10-self.tool_length, 0.022]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to right knob position")
            return result
        time.sleep(0.5)

        # Step 12: Force control to touch the right knob
        print("\n" + "="*50)
        print("Step 12: Touching right knob...")
        print("="*50)
        result = self.force_task_touch_knob()
        if result != 0:
            print(f"[ERROR] Failed to touch right knob")
            return result
        time.sleep(0.5)

        # Step 13: Move away from the right knob slightly
        print("\n" + "="*50)
        print("Step 13: Moving away from right knob slightly...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.001, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away from right knob")
            return result
        time.sleep(0.5)

        # Step 14: Force control to unlock the right knob
        print("\n" + "="*50)
        print("Step 14: Unlocking right knob...")
        print("="*50)
        result = self.force_task_unlock_right_knob()
        if result != 0:
            print(f"[ERROR] Failed to unlock right knob")
            return result
        time.sleep(0.5)

        # Step 15: Move away from the server
        print("\n" + "="*50)
        print("Step 15: Moving away from server...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.25, 0])
        if result != 0:
            print(f"[ERROR] Failed to unlock right knob")
            return result
        time.sleep(0.5)

        self.movel_in_server_frame([-0.03, 0.23, -0.04])
        time.sleep(0.5)

        # Step 16: Force task to open the right handle
        print("\n" + "="*50)
        print("Step 16: Opening right handle...")
        print("="*50)
        result = self.force_task_open_handle()
        if result != 0:
            print(f"[ERROR] Failed to open right handle")
            return result
        time.sleep(0.5)

        # Step 17: Move away from the knob2
        print("\n" + "="*50)
        print("Step 17: Moving away from right handle...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.20, -0.02])
        if result != 0:
            print(f"[ERROR] Failed to move away from right handle")
            return result
        time.sleep(0.5)
        
        print("\n" + "="*70)
        print("COMPLETE KNOB UNLOCK SEQUENCE FINISHED SUCCESSFULLY")
        print("="*70)
        return 0


if __name__ == "__main__":
    try:
        # Parse command line arguments
        parser = argparse.ArgumentParser(description='UR Robot Operate Wobj - Unlock Knob')
        parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                            help='Robot IP address (default: 192.168.1.15)')
        parser.add_argument('--robot-port', type=int, default=30002,
                            help='Robot port (default: 30002)')
        parser.add_argument('--server-index', type=int, default=14,
                            help='Server index (default: 14)')
        
        args = parser.parse_args()
        
        # Create UROperateWobjUnlockKnob instance
        ur_unlock_knob = UROperateWobjUnlockKnob(
            robot_ip=args.robot_ip,
            robot_port=args.robot_port,
            server_index=args.server_index
        )
        print("UROperateWobjUnlockKnob initialized successfully")
        
        # ==============Display automatically loaded parameters=================
        print("\n" + "="*70)
        print("AUTOMATICALLY LOADED PARAMETERS")
        print("="*70)
        
        if ur_unlock_knob.camera_matrix is not None:
            print("\n✓ Camera Matrix loaded")
            print(ur_unlock_knob.camera_matrix)
            print("\n✓ Distortion Coefficients loaded")
            print(ur_unlock_knob.distortion_coefficients)
        
        if ur_unlock_knob.cam2end_matrix is not None:
            print("\n✓ Camera to End-effector Matrix loaded")
            print(ur_unlock_knob.cam2end_matrix)
        
        if ur_unlock_knob.rack_transformation_matrix_in_base is not None:
            print("\n✓ Rack Coordinate System loaded")
            print(f"  Origin: x={ur_unlock_knob.rack_origin_in_base[0]:.6f}, "
                  f"y={ur_unlock_knob.rack_origin_in_base[1]:.6f}, "
                  f"z={ur_unlock_knob.rack_origin_in_base[2]:.6f}")

        # ==============Execute the knob unlocking task=================
        print("\n" + "="*70)
        print("STARTING TASK EXECUTION")
        print("="*70)
        
        result = ur_unlock_knob.execute_complete_unlock_sequence()
        
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
