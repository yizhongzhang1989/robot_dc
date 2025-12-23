import os
import time
import numpy as np
import argparse
from scipy.spatial.transform import Rotation as R
from ur_operate_wobj import UROperateWobj


class URWobjUnlockKnobInsert(UROperateWobj):
    def __init__(self, robot_ip=None, robot_port=None, server_index=14):
        """
        Initialize URWobjUnlockKnobInsert instance
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
        self.server2base_matrix = self._calculate_server2base(index=self.server_index)
        
        print(f"URWobjUnlockKnobInsert initialized for server index: {server_index}")

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

    # ================================= Force Control Functions ================================
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
        wrench = [0, 0, 30, 0, 0, 0]  # Desired force/torque in each direction
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
    
    def force_task_touch_knob_right(self):
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
        wrench = [0, 0, 50, 0, 0, 0]  # Desired force/torque in each direction
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
            end_force=[15, 15, 30, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    def force_task_unlock_left_knob(self):
        """
        Execute force control task to unlock the left knob using TCP coordinate system.
        Three-step process:
        1. Initial contact with knob
        2. Rotate counter-clockwise around TCP Z-axis to unlock (stops at specified angle)
        3. Release the spring in the knob
        4. Rotate clockwise to return
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
            end_force=[0, 0, 10, 0, 0, 0]
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
        wrench = [0, 5, 0, 0, -3.0, 0]
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
        wrench = [0, -35.0, 0, 0, 0, 0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - releasing spring...")
        
        # Execute force control with distance-based termination
        result3 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
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
        wrench = [0, 0, 0, 0, 3.0, 0]  # Apply 3.0 Nm torque (clockwise)
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - returning...")
        
        # Execute force control with angle-based termination
        result4 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.01,
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
            end_force=[0, 0, 10, 0, 0, 0]
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
        wrench = [0, 5, 0, 0, 3.0, 0]
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
        wrench = [0, -35.0, 0, 0, 0, 0]
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
        wrench = [0, 0, 0, 0, -3.0, 0]  # Apply -1.0 Nm torque (counter-clockwise)
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - returning...")
        
        # Execute force control with angle-based termination
        result4 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.01,
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
        Execute force control task to open handle using server coordinate system.
        The robot will apply force to pull the handle open.
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
        wrench = [0, 0, 25, 0, 0, 0]  # Desired force/torque in each direction
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
            end_distance=[0,0,0.03,0,0,0]
        )
        time.sleep(0.5)
        return result

    def force_task_push_server_to_end(self):
        """
        Execute force control task to push the server to end position using TCP coordinate system.
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
        wrench = [0, 80, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - pushing server...")
        
        # Execute force control task with time-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=1,
            end_time=3.5
        )
        time.sleep(0.5)
        return result


    # ================================ Task Execution Methods ================================
    def execute_unlock_and_insert_sequence(self):
        """
        Execute complete unlock and insert sequence following the task flow
        
        Steps:
        1. Correct TCP pose
        2. Move to server push position
        3. Push server inward
        4. Move away from server
        5. Move to left knob position
        6. Touch left knob
        7. Move away from left knob slightly
        8. Unlock left knob (force control)
        9. Move away from server
        10. Move to left handle position
        11. Open left handle (force control)
        12. Move away from left handle
        13. Correct TCP pose for right side
        14. Touch right knob
        15. Move away from right knob slightly
        16. Unlock right knob (force control)
        17. Move away from server
        18. Open right handle (force control)
        19. Move away from right handle
        20. Move to push position
        21. Push server to end (force control)
        22. Move away from server
        """
        print("\n" + "="*70)
        print("STARTING COMPLETE UNLOCK AND INSERT SEQUENCE")
        print("="*70)
        
        # ===============Execute the left knob unlocking task=================
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
        
        # Step 2: Move to left knob position
        print("\n" + "="*50)
        print("Step 2: Moving to target server position to capture and update server2base_matrix...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0.03, -0.30-self.tool_length, 0.03]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to target position (error code: {result})")
            return result
        time.sleep(0.5)

        # Step 3: Push the server to the end position
        print("\n" + "="*50)
        print("Step 3: Pushing server to end position...")
        print("="*50)
        result = self.force_task_push_server_to_end()
        if result != 0:
            print(f"[ERROR] Failed to push server to end")
            return result
        time.sleep(0.5)

        # Step 4: Leave the server
        print("\n" + "="*50)
        print("Step 4: Leaving server area to prepare for positioning...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.15, 0])
        if result != 0:
            print(f"[ERROR] Failed to leave server area")
            return result
        time.sleep(0.5)

        # Step 5: Move to left knob position
        print("\n" + "="*50)
        print("Step 5: Moving to left knob position...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[-0.105, -0.10-self.tool_length, 0.020]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to left knob position")
            return result
        time.sleep(0.5)

        # Step 6: Force control to touch the left knob
        print("\n" + "="*50)
        print("Step 6: Touching left knob...")
        print("="*50)
        result = self.force_task_touch_knob()
        if result != 0:
            print(f"[ERROR] Failed to touch left knob")
            return result
        time.sleep(0.5)

        # Step 7: Move away from the knob slightly
        print("\n" + "="*50)
        print("Step 7: Moving away from knob slightly...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.001, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away from left knob")
            return result
        time.sleep(0.5)

        # Step 8: Force control to unlock the left knob
        print("\n" + "="*50)
        print("Step 8: Unlocking left knob...")
        print("="*50)
        result = self.force_task_unlock_left_knob()
        if result != 0:
            print(f"[ERROR] Failed to unlock left knob")
            return result
        time.sleep(0.5)

        # Step 9: Move away and prepare for handle
        print("\n" + "="*50)
        print("Step 9: Moving away and preparing for handle...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.25, 0])
        if result != 0:
            print(f"[ERROR] Failed to unlock left knob")
            return result
        time.sleep(0.5)

        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[2, 3, 1],
            offset_in_rack=[-0.06, -0.06-self.tool_length, -0.025]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to left knob position")
            return result
        time.sleep(0.5)

        # Step 10: Force control to open left handle
        print("\n" + "="*50)
        print("Step 10: Opening left handle...")
        print("="*50)
        result = self.force_task_open_handle()
        if result != 0:
            print(f"[ERROR] Failed to open left handle")
            return result
        time.sleep(0.5)

        # Step 11: Move away from left handle
        print("\n" + "="*50)
        print("Step 11: Moving away from left handle...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.20, -0.02])
        if result != 0:
            print(f"[ERROR] Failed to move away from left handle")
            return result
        time.sleep(0.5)

        # ===============Execute the right knob unlocking task=================
        # Step 12: Correct TCP pose to align with rack coordinate system
        print("\n" + "="*50)
        print("Step 12: Moving to right knob position...")
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

        # Step 13: Move to right knob position
        print("\n" + "="*50)
        print("Step 13: Moving to right knob position...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0.105, -0.10-self.tool_length, 0.020]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to right knob position")
            return result
        time.sleep(0.5)

        # Step 14: Force control to touch the right knob
        print("\n" + "="*50)
        print("Step 14: Touching right knob...")
        print("="*50)
        result = self.force_task_touch_knob_right()
        if result != 0:
            print(f"[ERROR] Failed to touch right knob")
            return result
        time.sleep(0.5)

        # Step 15: Move away from the knob slightly
        print("\n" + "="*50)
        print("Step 15: Moving away from knob slightly...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.001, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away from right knob")
            return result
        time.sleep(0.5)

        # Step 16: Force control to unlock the right knob
        print("\n" + "="*50)
        print("Step 16: Unlocking right knob...")
        print("="*50)
        result = self.force_task_unlock_right_knob()
        if result != 0:
            print(f"[ERROR] Failed to unlock right knob")
            return result
        time.sleep(0.5)

        # Step 17: Move away from server and prepare for open handle
        print("\n" + "="*50)
        print("Step 17: Moving away from server...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.25, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away from server")
            return result
        time.sleep(0.5)
        
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[2, 3, 1],
            offset_in_rack=[0.10, -0.06-self.tool_length, -0.025]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to left knob position")
            return result
        time.sleep(0.5)

        # Step 18: Force control to open right handle
        print("\n" + "="*50)
        print("Step 18: Opening right handle...")
        print("="*50)
        result = self.force_task_open_handle()
        if result != 0:
            print(f"[ERROR] Failed to open right handle")
            return result
        time.sleep(0.5)

        # Step 19: Move away from right handle
        print("\n" + "="*50)
        print("Step 19: Moving away from right handle...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.20, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away from right handle")
            return result
        time.sleep(0.5)

        # ===============Push server to end position=================
        # Step 20: Move to push position
        print("\n" + "="*50)
        print("Step 20: Moving to push position...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0.03, -0.15-self.tool_length, 0.03]
        )
        if result != 0:
            print(f"[ERROR] Failed to move to push position")
            return result
        time.sleep(0.5)

        # Step 21: Force control to push server to end
        print("\n" + "="*50)
        print("Step 21: Pushing server to end...")
        print("="*50)
        result = self.force_task_push_server_to_end()
        if result != 0:
            print(f"[ERROR] Failed to push server to end")
            return result
        time.sleep(0.5)

        # Step 22: Move away from server
        print("\n" + "="*50)
        print("Step 22: Moving away from server...")
        print("="*50)
        result = self.movel_in_server_frame([0, -0.25, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away from server")
            return result
        time.sleep(0.5)
        
        print("\n" + "="*70)
        print("COMPLETE UNLOCK AND INSERT SEQUENCE FINISHED SUCCESSFULLY")
        print("="*70)
        return 0


if __name__ == "__main__":
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='URWobjUnlockKnobInsert - Unlock knob and insert operation using server coordinate system')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot (default: 192.168.1.15)')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot (default: 30002)')
    parser.add_argument('--server-index', type=int, default=14,
                       help='Target server index (default: 14)')
    
    args = parser.parse_args()
    
    # Create URWobjUnlockKnobInsert instance
    ur_wobj_unlock_knob_insert = URWobjUnlockKnobInsert(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        server_index=args.server_index
    )
    
    # ==============Display automatically loaded parameters=================
    print("\n" + "="*70)
    print("AUTOMATICALLY LOADED PARAMETERS")
    print("="*70)
    
    if ur_wobj_unlock_knob_insert.camera_matrix is not None:
        print("\n✓ Camera Matrix loaded")
        print(ur_wobj_unlock_knob_insert.camera_matrix)
        print("\n✓ Distortion Coefficients loaded")
        print(ur_wobj_unlock_knob_insert.distortion_coefficients)
    
    if ur_wobj_unlock_knob_insert.cam2end_matrix is not None:
        print("\n✓ Camera to End-effector Matrix loaded")
        print(ur_wobj_unlock_knob_insert.cam2end_matrix)
    
    if ur_wobj_unlock_knob_insert.rack_transformation_matrix_in_base is not None:
        print("\n✓ Rack Coordinate System loaded")
        print(f"  Origin: {ur_wobj_unlock_knob_insert.rack_origin_in_base}")
    
    if ur_wobj_unlock_knob_insert.server2base_matrix is not None:
        print("\n✓ Server Coordinate System loaded")
        server_origin = ur_wobj_unlock_knob_insert.server2base_matrix[:3, 3]
        print(f"  Origin: {server_origin}")
        print(f"  Server Index: {ur_wobj_unlock_knob_insert.server_index}")
    
    print(f"\n✓ Tool Parameters loaded")
    print(f"  Tool Length: {ur_wobj_unlock_knob_insert.tool_length}m")
    print(f"  Tool Angle Z: {ur_wobj_unlock_knob_insert.tool_angle_z}°")

    # ==============Execute the unlock and insert task=================
    print("\n" + "="*70)
    print("STARTING TASK EXECUTION")
    print("="*70)
    
    try:
        result = ur_wobj_unlock_knob_insert.execute_unlock_and_insert_sequence()
        
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
