import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_operate_wobj import UROperateWobj


class URWobjOpenHandle(UROperateWobj):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, server_index=14):
        """
        Initialize URWobjOpenHandle instance
        Args:
            robot_ip: IP address of the UR15 robot
            robot_port: Port number of the UR15 robot
            server_index: Server index for target server position
        """
        # Call parent class constructor
        super().__init__(robot_ip=robot_ip, robot_port=robot_port, server_index=server_index)
        
        # Calculate server coordinate system
        self._calculate_server2base(self.server_index)
        
        print(f"URWobjOpenHandle initialized for server index: {server_index}")


    # ================================ Force Control Functions ================================
    def force_task_touch_handle_before_open_handle(self):
        """
        Execute force control task to touch the handle before opening handle using server coordinate system.
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
        
        # Set force mode parameters
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable force control in Z direction relative to task frame
        wrench = [0, 0, 15, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - touching handle...")
        
        # Execute force control task with distance-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=3,
            end_distance=[0, 0, 0.035, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    def force_task_open_left_handle(self):
        """
        Execute force control task to open the left handle using server coordinate system.
        This includes two motions: prepull and open.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.server2base_matrix is None:
            print("Server coordinate system transformation matrix not loaded")
            return -1
        
        # ==============Motion 1: Prepull the handle=================
        print("\n[Motion 1] Prepulling handle...")
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
        
        # Set force mode parameters for motion 1
        selection_vector = [1, 1, 0, 0, 0, 0]  # Enable force control in X and Y directions relative to task frame
        wrench = [-25, -25, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - motion 1: prepulling handle...")
        
        # Execute force control task with distance-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=3,
            end_distance=[0.10, 0.10, 0, 0, 0, 0]
        )
        time.sleep(0.5)
        
        if result != 0:
            print(f"[ERROR] Motion 1 failed with code: {result}")
            return result
        
        print("[INFO] Motion 1 completed successfully")
        time.sleep(0.5)

        # ==============Motion 2: Open the handle=================
        print("\n[Motion 2] Opening handle...")
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
        
        # Set force mode parameters for motion 2
        selection_vector = [1, 0, 0, 0, 0, 0]  # Enable force control in X direction relative to task frame
        wrench = [-15, 0, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - motion 2: opening handle...")
        
        # Execute force control task with time-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=1,
            end_time=3
        )
        
        if result != 0:
            print(f"[ERROR] Motion 2 failed with code: {result}")
            return result
        
        print("[INFO] Motion 2 completed successfully")
        time.sleep(0.5)
        return result

    def force_task_push_server_after_open_handle(self):
        """
        Execute force control task to push server to end after opening handle using server coordinate system.
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
        
        # Set force mode parameters
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Y direction relative to task frame
        wrench = [0, 50, 0, 0, 0, 0]  # Desired force/torque in each direction
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
        
        # Set force mode parameters
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable force control in Z direction relative to task frame
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
            end_type=2,
            end_force=[0, 0, 10, 0, 0, 0]
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
        
        # Set force mode parameters
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Y direction relative to task frame
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
            end_distance=[0, 0.05, 0, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    # ================================ Task Execution Methods ================================
    def execute_open_handle_sequence(self):
        """
        Execute complete open handle sequence following the task flow
        """
        print("\n" + "="*70)
        print("STARTING COMPLETE OPEN HANDLE SEQUENCE")
        print("="*70)
        
        # Step 1: Correct tool TCP pose
        print("\n" + "="*50)
        result = self.movel_to_correct_tcp_pose(
            tcp_x_to_rack=[1, 0, 0],
            tcp_y_to_rack=[0, 0, -1],
            tcp_z_to_rack=[0, 1, 0],
            angle_deg=31
        )
        if result != 0:
            return result
        time.sleep(0.5)
        
        # Step 2: Move to target position (handle position)
        print("\n" + "="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0, -0.60, 0]  # Offset to reach handle position
        )
        if result != 0:
            return result
        time.sleep(0.5)

        # move to touch start position
        print("\n" + "="*50)
        print("Step 3: Moving to touch start position...")
        print("="*50)
        result = self.movel_in_server_frame([0, 0, -0.04])
        if result != 0:
            print(f"[ERROR] Failed to move away from right knob")
            return result
        time.sleep(0.5)

        result = self.movel_in_server_frame([-0.08, 0.17, 0])
        if result != 0:
            print(f"[ERROR] Failed to move away from right knob")
            return result
        time.sleep(0.5)


        # Step 3: Force control to touch the handle
        print("\n" + "="*50)
        result = self.force_task_touch_handle_before_open_handle()
        if result != 0:
            return result
        time.sleep(0.5)

        # Step 4: Force control to open the left handle (includes prepull and open)
        print("\n" + "="*50)
        result = self.force_task_open_left_handle()
        if result != 0:
            return result
        time.sleep(0.5)

        # Step 5: Move away from the handle
        print("\n" + "="*50)
        result = self.movel_in_server_frame([0.10, 0, -0.01])
        if result != 0:
            return result
        time.sleep(0.5)

        # Step 6: Move to server push position
        print("\n" + "="*50)
        result = self.movel_to_target_position(
            index=self.server_index,
            execution_order=[1, 3, 2],
            offset_in_rack=[0.03, -0.50, 0.025]  # Offset to reach server push position
        )
        if result != 0:
            return result
        time.sleep(0.5)


        # Step 7: Force control to push server to end
        print("\n" + "="*50)
        result = self.force_task_push_server_after_open_handle()
        if result != 0:
            return result
        time.sleep(0.5)

        # Step 8: Move to leave the server
        print("\n" + "="*50)
        result = self.movel_in_server_frame([0, -0.05, -0.04])
        if result != 0:
            return result
        time.sleep(0.5)

        # Step 9: Move to the server
        print("\n" + "="*50)
        result = self.movel_in_server_frame([0, 0.06, 0])
        if result != 0:
            return result
        time.sleep(0.5)

        # Step 10: Force control to touch the server
        print("\n" + "="*50)
        result = self.force_task_touch_server()
        if result != 0:
            return result
        time.sleep(0.5)

        # Step 11: Force control to pull the server
        print("\n" + "="*50)
        result = self.force_task_pull_server()
        if result != 0:
            return result
        time.sleep(0.5)

        # Step 12: Move to leave the server
        print("\n" + "="*50)
        result = self.movel_in_server_frame([0, -0.02, -0.04])
        if result != 0:
            return result
        time.sleep(0.5)

        # Step 13: Move away from the server
        print("\n" + "="*50)
        result = self.movel_in_server_frame([0, -0.15, 0])
        if result != 0:
            return result
        time.sleep(0.5)
        
        print("\n" + "="*70)
        print("COMPLETE OPEN HANDLE SEQUENCE FINISHED SUCCESSFULLY")
        print("="*70)
        return 0


if __name__ == "__main__":
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='URWobjOpenHandle - Open handle operation using server coordinate system')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot (default: 192.168.1.15)')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot (default: 30002)')
    parser.add_argument('--server_index', type=int, default=14,
                       help='Target server index (default: 14)')
    
    args = parser.parse_args()
    
    # Create URWobjOpenHandle instance
    ur_wobj_open_handle = URWobjOpenHandle(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        server_index=args.server_index
    )
    
    # ==============Display automatically loaded parameters=================
    print("\n" + "="*70)
    print("AUTOMATICALLY LOADED PARAMETERS")
    print("="*70)
    
    if ur_wobj_open_handle.camera_matrix is not None:
        print("\n✓ Camera Matrix loaded")
        print(ur_wobj_open_handle.camera_matrix)
        print("\n✓ Distortion Coefficients loaded")
        print(ur_wobj_open_handle.distortion_coefficients)
    
    if ur_wobj_open_handle.cam2end_matrix is not None:
        print("\n✓ Camera to End-effector Matrix loaded")
        print(ur_wobj_open_handle.cam2end_matrix)
    
    if ur_wobj_open_handle.rack_transformation_matrix_in_base is not None:
        print("\n✓ Rack Coordinate System loaded")
        print(f"  Origin: {ur_wobj_open_handle.rack_origin_in_base}")
    
    if ur_wobj_open_handle.server2base_matrix is not None:
        print("\n✓ Server Coordinate System loaded")
        server_origin = ur_wobj_open_handle.server2base_matrix[:3, 3]
        print(f"  Origin: {server_origin}")
        print(f"  Server Index: {ur_wobj_open_handle.server_index}")

    # ==============Execute the handle opening task=================
    print("\n" + "="*70)
    print("STARTING TASK EXECUTION")
    print("="*70)
    
    try:
        result = ur_wobj_open_handle.execute_open_handle_sequence()
        
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
