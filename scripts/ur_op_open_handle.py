import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_operate import UROperate


class UROpOpenHandle(UROperate):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, operation_name="open_handle"):
        """
        Initialize UROpOpenHandle instance
        Args:
            robot_ip: IP address of the UR15 robot
            robot_port: Port number of the UR15 robot
            operation_name: Operation name for data organization (default: 'open_handle')
        """
        # Call parent class constructor
        super().__init__(robot_ip=robot_ip, robot_port=robot_port, operation_name=operation_name)
        
        print(f"UROpOpenHandle initialized for operation: {operation_name}")


    # ================================ Force Control Functions ================================
    def force_task_touch_handle_before_prepull(self):
        """
        Execute force control task to touch the handle before prepull using wobj coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.wobj_transformation_matrix is None:
            print("Wobj coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Extract rotation matrix from wobj transformation matrix and convert to rotation vector
        wobj_rotation = self.wobj_transformation_matrix[:3, :3]
        rotation_obj = R.from_matrix(wobj_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Create task frame using current position with wobj orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            rotation_vector[0], # rx (wobj orientation)
            rotation_vector[1], # ry
            rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (wobj coordinate system): {task_frame}")
        
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
            end_distance=[0, 0, 0.03, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    def force_task_prepull_handle(self):
        """
        Execute force control task to prepull the handle using wobj coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.wobj_transformation_matrix is None:
            print("Wobj coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Extract rotation matrix from wobj transformation matrix and convert to rotation vector
        wobj_rotation = self.wobj_transformation_matrix[:3, :3]
        rotation_obj = R.from_matrix(wobj_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Create task frame using current position with wobj orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            rotation_vector[0], # rx (wobj orientation)
            rotation_vector[1], # ry
            rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (wobj coordinate system): {task_frame}")
        
        # Set force mode parameters
        selection_vector = [1, 1, 0, 0, 0, 0]  # Enable force control in X and Y directions relative to task frame
        wrench = [-15, -15, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - prepulling handle...")
        
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
        return result

    def force_task_open_handle_after_prepull(self):
        """
        Execute force control task to open the handle after prepull using wobj coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.wobj_transformation_matrix is None:
            print("Wobj coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Extract rotation matrix from wobj transformation matrix and convert to rotation vector
        wobj_rotation = self.wobj_transformation_matrix[:3, :3]
        rotation_obj = R.from_matrix(wobj_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Create task frame using current position with wobj orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            rotation_vector[0], # rx (wobj orientation)
            rotation_vector[1], # ry
            rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (wobj coordinate system): {task_frame}")
        
        # Set force mode parameters
        selection_vector = [1, 0, 0, 0, 0, 0]  # Enable force control in X direction relative to task frame
        wrench = [-15, 0, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - opening handle...")
        
        # Execute force control task with force-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=1,
            end_time=3
        )
        time.sleep(0.5)
        return result

    def force_task_push_server_after_open_handle(self):
        """
        Execute force control task to push server to end after opening handle using wobj coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.wobj_transformation_matrix is None:
            print("Wobj coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Extract rotation matrix from wobj transformation matrix and convert to rotation vector
        wobj_rotation = self.wobj_transformation_matrix[:3, :3]
        rotation_obj = R.from_matrix(wobj_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Create task frame using current position with wobj orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            rotation_vector[0], # rx (wobj orientation)
            rotation_vector[1], # ry
            rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (wobj coordinate system): {task_frame}")
        
        # Set force mode parameters
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Z direction relative to task frame
        wrench = [0, 30, 0, 0, 0, 0]  # Desired force/torque in each direction
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
            end_time=5
        )
        time.sleep(0.5)
        return result

    def force_task_touch_server(self):
        """
        Execute force control task to touch the server using wobj coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.wobj_transformation_matrix is None:
            print("Wobj coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Extract rotation matrix from wobj transformation matrix and convert to rotation vector
        wobj_rotation = self.wobj_transformation_matrix[:3, :3]
        rotation_obj = R.from_matrix(wobj_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Create task frame using current position with wobj orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            rotation_vector[0], # rx (wobj orientation)
            rotation_vector[1], # ry
            rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (wobj coordinate system): {task_frame}")
        
        # Set force mode parameters
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable force control in Y direction relative to task frame
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
            end_force=[0, 0, 2, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    def force_task_pull_server(self):
        """
        Execute force control task to pull the server using wobj coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.wobj_transformation_matrix is None:
            print("Wobj coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Extract rotation matrix from wobj transformation matrix and convert to rotation vector
        wobj_rotation = self.wobj_transformation_matrix[:3, :3]
        rotation_obj = R.from_matrix(wobj_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Create task frame using current position with wobj orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            rotation_vector[0], # rx (wobj orientation)
            rotation_vector[1], # ry
            rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (wobj coordinate system): {task_frame}")
        
        # Set force mode parameters
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Z direction relative to task frame
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
    def execute_complete_open_handle_sequence(self):
        """
        Execute complete open handle sequence following the task flow
        """
        print("\n" + "="*70)
        print("STARTING COMPLETE OPEN HANDLE SEQUENCE")
        print("="*70)
        
        # ===============execute the handle opening task=================
        # move to reference joint positions
        print("\n" + "="*50)
        result = self.movej_to_reference_position()
        if result != 0:
            return result
        time.sleep(0.5)
        
        # align tool TCP with wobj coordinate system
        print("\n" + "="*50)
        result = self.movel_to_correct_tcp_pose(
            tcp_x_to_wobj=[1, 0, 0],
            tcp_y_to_wobj=[0, 0, -1],
            tcp_z_to_wobj=[0, 1, 0],
            angle_deg=31
        )
        if result != 0:
            return result
        time.sleep(0.5)
        
        # move to target position using linear movement
        print("\n" + "="*50)
        result = self.movel_to_start_position(step_key='step1', execution_order=[1, 3, 2])
        if result != 0:
            return result
        time.sleep(0.5)

        # force control to move to touch the handle
        print("\n" + "="*50)
        result = self.force_task_touch_handle_before_prepull()
        if result != 0:
            return result
        time.sleep(0.5)

        # force control to prepull the handle
        print("\n" + "="*50)
        result = self.force_task_prepull_handle()
        if result != 0:
            return result
        time.sleep(0.5)

        # force control to open the handle
        print("\n" + "="*50)
        result = self.force_task_open_handle_after_prepull()
        if result != 0:
            return result
        time.sleep(0.5)

        # move away from the handle
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([0.10, 0, -0.01])
        if result != 0:
            return result
        time.sleep(0.5)

        # ===============Step 2: Pull server out operation=================
        # move to step2 position
        print("\n" + "="*50)
        result = self.movel_to_start_position(step_key='step2', execution_order=[1, 3, 2])
        if result != 0:
            return result
        time.sleep(0.5)

        # force control to push server to end
        print("\n" + "="*50)
        result = self.force_task_push_server_after_open_handle()
        if result != 0:
            return result
        time.sleep(0.5)

        # move to leave the server
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([0, -0.05, -0.04])
        if result != 0:
            return result
        time.sleep(0.5)

        # move to the server
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([0, 0.065, 0])
        if result != 0:
            return result
        time.sleep(0.5)

        # force control to touch the server
        print("\n" + "="*50)
        result = self.force_task_touch_server()
        if result != 0:
            return result
        time.sleep(0.5)

        # force control to pull the server
        print("\n" + "="*50)
        result = self.force_task_pull_server()
        if result != 0:
            return result
        time.sleep(0.5)

        # move to leave the server
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([0, -0.02, -0.04])
        if result != 0:
            return result
        time.sleep(0.5)

        # move away from the server
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([0, -0.15, 0])
        if result != 0:
            return result
        time.sleep(0.5)

        # move back to reference joint positions
        print("\n" + "="*50)
        result = self.movej_to_reference_position()
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
    parser = argparse.ArgumentParser(description='UROpOpenHandle - Open handle operation')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot (default: 192.168.1.15)')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot (default: 30002)')
    parser.add_argument('--operation-name', type=str, default='open_handle',
                       help='Name of the operation (default: open_handle)')
    
    args = parser.parse_args()
    
    # Create UROpOpenHandle instance
    ur_open_handle = UROpOpenHandle(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        operation_name=args.operation_name
    )
    
    # ==============Display automatically loaded parameters=================
    print("\n" + "="*70)
    print("AUTOMATICALLY LOADED PARAMETERS")
    print("="*70)
    
    if ur_open_handle.camera_matrix is not None:
        print("\n✓ Camera Matrix loaded")
        print(ur_open_handle.camera_matrix)
        print("\n✓ Distortion Coefficients loaded")
        print(ur_open_handle.distortion_coefficients)
    
    if ur_open_handle.cam2end_matrix is not None:
        print("\n✓ Camera to End-effector Matrix loaded")
        print(ur_open_handle.cam2end_matrix)
    
    if ur_open_handle.estimated_keypoints is not None:
        print(f"\n✓ Estimated Keypoints loaded ({len(ur_open_handle.estimated_keypoints)} points)")
        for i, kp in enumerate(ur_open_handle.estimated_keypoints[:3]):  # Show first 3
            print(f"  Point {i}: ({kp[0]:.6f}, {kp[1]:.6f}, {kp[2]:.6f})")
        if len(ur_open_handle.estimated_keypoints) > 3:
            print(f"  ... and {len(ur_open_handle.estimated_keypoints) - 3} more")
    
    if ur_open_handle.local_transformation_matrix is not None:
        print("\n✓ Local Coordinate System loaded")
        print(f"  Origin: {ur_open_handle.local_origin}")
    
    if ur_open_handle.wobj_transformation_matrix is not None:
        print("\n✓ Wobj Coordinate System loaded")
        print(f"  Origin: {ur_open_handle.wobj_origin}")
    
    if ur_open_handle.ref_joint_angles is not None:
        print("\n✓ Reference Joint Angles loaded (radians):")
        for i, angle in enumerate(ur_open_handle.ref_joint_angles):
            print(f"  Joint {i}: {angle:.6f}")
    
    if ur_open_handle.offset_in_local is not None:
        print(f"\n✓ Task Position Offset loaded (type: {ur_open_handle.offset_in_local_type})")
        if ur_open_handle.offset_in_local_type == 'multiple':
            for step_key, offset in ur_open_handle.offset_in_local.items():
                print(f"  {step_key}: x={offset['x']:.6f}, y={offset['y']:.6f}, z={offset['z']:.6f}")
        else:
            print(f"  x={ur_open_handle.offset_in_local['x']:.6f}, "
                  f"y={ur_open_handle.offset_in_local['y']:.6f}, "
                  f"z={ur_open_handle.offset_in_local['z']:.6f}")

    # ==============Execute the handle opening task=================
    print("\n" + "="*70)
    print("STARTING TASK EXECUTION")
    print("="*70)
    
    try:
        result = ur_open_handle.execute_complete_open_handle_sequence()
        
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
