import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_operate_wobj import UROperateWobj


class URWobjPutFrame(UROperateWobj):
    def __init__(self, robot_ip=None, robot_port=None, server_index=14):
        """
        Initialize URWobjPutFrame instance
        Args:
            robot_ip: IP address of the UR15 robot. If None, loads from config file
            robot_port: Port number of the UR15 robot. If None, loads from config file
            server_index: Server index for target server position
        """
        # Call parent class constructor
        super().__init__(robot_ip=robot_ip, robot_port=robot_port, server_index=server_index)
        
        # Load tool parameters from config file
        self._load_tool_frame_parameters_from_config()
        
        # Calculate server coordinate system
        self.server2base_matrix = self._calculate_server2base(self.server_index)
        
        print(f"URWobjPutFrame initialized for server index: {server_index}")

    def _load_tool_frame_parameters_from_config(self):
        """
        Load tool_frame parameters from robot_config.yaml
        
        Loads:
            - tool_length: Length of the tool from flange to tip (meters)
            - tool_angle_z: Rotation angle around Z axis (degrees)
        """
        # Default values
        defaults = {
            'tool_length': 0.345,
            'tool_angle_z': 31
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
                print(f"Using default tool_frame values: length={defaults['tool_length']}, angle_z={defaults['tool_angle_z']}")
                self.tool_length = defaults['tool_length']
                self.tool_angle_z = defaults['tool_angle_z']
                return
            
            # Load config file
            with open(config_path, 'r') as f:
                config = yaml.safe_load(f)
            
            # Get tool_frame parameters from ur15.tool.tool_frame
            tool_frame = config.get('ur15', {}).get('tool', {}).get('tool_frame', {})
            
            self.tool_length = tool_frame.get('length', defaults['tool_length'])
            self.tool_angle_z = tool_frame.get('angle_z', defaults['tool_angle_z'])
            
            print(f"✓ Loaded tool_frame parameters: length={self.tool_length}m, angle_z={self.tool_angle_z}°")
            
        except Exception as e:
            print(f"Error loading tool_frame parameters: {e}")
            print(f"Using default values: length={defaults['tool_length']}, angle_z={defaults['tool_angle_z']}")
            self.tool_length = defaults['tool_length']
            self.tool_angle_z = defaults['tool_angle_z']


    # ================================ Force Control Functions ================================
    def force_task_place_on_crack(self):
        """
        Execute force control task to place frame on crack using server coordinate system.
        The robot will apply a downward force to make contact with the crack surface.
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
        server_rotation = self.server2base_matrix[:3, :3]
        rotation_obj = R.from_matrix(server_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Create task frame using current position with server orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            rotation_vector[0], # rx (server orientation)
            rotation_vector[1], # ry
            rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (server coordinate system): {task_frame}")
        
        # Set force mode parameters for Z direction (downward force)
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Z direction relative to task frame
        wrench = [0, 40, 0, 0, 0, 0]  # Desired force/torque in each direction (40N downward)
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - placing frame on crack...")
        
        # Execute force control task with force-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=2,
            end_force=[0, 30, 0, 0, 0, 0]  # Terminate when Z force reaches 30N
        )
        time.sleep(0.5)
        return result
    
    def force_task_leave_frame(self):
        """
        Execute force control task to leave the frame.
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
        server_rotation = self.server2base_matrix[:3, :3]
        rotation_obj = R.from_matrix(server_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Create task frame using current position with server orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            rotation_vector[0], # rx (server orientation)
            rotation_vector[1], # ry
            rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (server coordinate system): {task_frame}")
        
        # Set force mode parameters for Z direction (downward force)
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Z direction relative to task frame
        wrench = [0, -80, 0, 0, 0, 0]  # Desired force/torque in each direction (40N downward)
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - placing frame on crack...")
        
        # Execute force control task with force-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=3,
            end_distance=[0, 0.10, 0, 0, 0, 0]
        )
        time.sleep(0.5)
        return result
    
    # ================================ Quick Changer Control ================================
    def ur_unlock_quick_changer(self):
        """
        Unlock the UR quick changer using RS485 communication
        """
        if self.rs485_socket is None:
            print("RS485 socket is not initialized")
            return -1
        
        try:
            print("[INFO] Unlocking quick changer...")
            # Send unlock command via RS485
            unlock_command = bytes([0x53, 0x26, 0x01, 0x01, 0x02, 0x7A, 0xD5])
            self.rs485_socket.sendall(unlock_command)
            time.sleep(0.5)
            
            # Read response
            response = self.rs485_socket.recv(1024)
            print(f"[INFO] Quick changer unlock response: {response.hex()}")
            
            print("[INFO] Quick changer unlocked successfully")
            return 0
            
        except Exception as e:
            print(f"[ERROR] Failed to unlock quick changer: {e}")
            return -1

    # ================================ Task Execution Methods ================================
    def execute_put_frame_sequence(self):
        """
        Execute complete put frame sequence following the task flow
        
        Steps:
        1. Correct TCP pose
        2. Move to target server position
        3. Force control to place frame on crack
        4. Unlock quick changer
        5. Force control to leave the frame
        6. Return to safe position
        """
        print("\n" + "="*70)
        print("STARTING COMPLETE PUT FRAME SEQUENCE")
        print("="*70)
        
        # Step 1: Correct tool TCP pose
        print("\n" + "="*50)
        print("Step 1: Correcting tool TCP pose...")
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
        
        # Step 2: Move to target position (above crack position with tool offset)
        print("\n" + "="*50)
        print("Step 2: Moving to target server position...")
        print("="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0, -0.35-self.tool_length, 0]  # Offset to account for tool length
        )
        if result != 0:
            print(f"[ERROR] Failed to move to target position")
            return result
        time.sleep(0.5)

        # Step 3: Force control to place frame on crack
        print("\n" + "="*50)
        print("Step 3: Placing frame on crack with force control...")
        print("="*50)
        result = self.force_task_place_on_crack()
        if result != 0:
            print(f"[ERROR] Failed to place frame on crack")
            return result
        time.sleep(0.5)

        # Step 4: Unlock quick changer to release the frame
        print("\n" + "="*50)
        print("Step 4: Unlocking quick changer to release frame...")
        print("="*50)
        result = self.ur_unlock_quick_changer()
        if result != 0:
            print(f"[ERROR] Failed to unlock quick changer")
            return result
        time.sleep(3)  # Wait for quick changer to fully unlock

        # Step 5: Force task to leave the frame
        print("\n" + "="*50)
        print("Step 5: Force task to leave frame...")
        print("="*50)
        result = self.force_task_leave_frame()
        if result != 0:
            print(f"[ERROR] Failed to leave frame")
            return result
        time.sleep(0.5)

        # Step 6: Return to safe position
        print("\n" + "="*50)
        print("Step 6: Returning to safe position...")
        print("="*50)
        result = self.movel_in_rack_frame([0, -0.15, 0])
        if result != 0:
            print(f"[ERROR] Failed to return to safe position")
            return result
        time.sleep(0.5)
        
        print("\n" + "="*70)
        print("COMPLETE PUT FRAME SEQUENCE FINISHED SUCCESSFULLY")
        print("="*70)
        return 0


if __name__ == "__main__":
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='URWobjPutFrame - Put frame operation using server coordinate system')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot (default: 192.168.1.15)')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot (default: 30002)')
    parser.add_argument('--server-index', type=int, default=14,
                       help='Target server index (default: 14)')
    
    args = parser.parse_args()
    
    # Create URWobjPutFrame instance
    ur_wobj_put_frame = URWobjPutFrame(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        server_index=args.server_index
    )
    
    # ==============Display automatically loaded parameters=================
    print("\n" + "="*70)
    print("AUTOMATICALLY LOADED PARAMETERS")
    print("="*70)
    
    if ur_wobj_put_frame.camera_matrix is not None:
        print("\n✓ Camera Matrix loaded")
        print(ur_wobj_put_frame.camera_matrix)
        print("\n✓ Distortion Coefficients loaded")
        print(ur_wobj_put_frame.distortion_coefficients)
    
    if ur_wobj_put_frame.cam2end_matrix is not None:
        print("\n✓ Camera to End-effector Matrix loaded")
        print(ur_wobj_put_frame.cam2end_matrix)
    
    if ur_wobj_put_frame.rack_transformation_matrix_in_base is not None:
        print("\n✓ Rack Coordinate System loaded")
        print(f"  Origin: {ur_wobj_put_frame.rack_origin_in_base}")
    
    if ur_wobj_put_frame.server2base_matrix is not None:
        print("\n✓ Server Coordinate System loaded")
        server_origin = ur_wobj_put_frame.server2base_matrix[:3, 3]
        print(f"  Origin: {server_origin}")
        print(f"  Server Index: {ur_wobj_put_frame.server_index}")
    
    print(f"\n✓ Tool Parameters loaded")
    print(f"  Tool Length: {ur_wobj_put_frame.tool_length}m")
    print(f"  Tool Angle Z: {ur_wobj_put_frame.tool_angle_z}°")

    # ==============Execute the put frame task=================
    print("\n" + "="*70)
    print("STARTING TASK EXECUTION")
    print("="*70)
    
    try:
        result = ur_wobj_put_frame.execute_put_frame_sequence()
        
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
