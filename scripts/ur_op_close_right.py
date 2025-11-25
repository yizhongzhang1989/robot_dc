import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_operate import UROperate


class UROpCloseRight(UROperate):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, operation_name="close_right"):
        """
        Initialize UROpCloseRight instance
        Args:
            robot_ip: IP address of the UR15 robot
            robot_port: Port number of the UR15 robot
            operation_name: Operation name for data organization (default: 'close_right')
        """
        # Call parent class constructor
        super().__init__(robot_ip=robot_ip, robot_port=robot_port, operation_name=operation_name)
        
        print(f"UROpCloseRight initialized for operation: {operation_name}")


    # ================================ Force Control Functions ================================
    def force_task_touch_right_handle(self):
        """
        Execute force control task to touch the right handle using wobj coordinate system.
        The robot will apply a downward force (relative to wobj) to make contact with the right handle.
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
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in X, Y, Z directions
        wrench = [0, 25, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - touching right handle...")
        
        # Execute force control task with distance-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=3,
            end_distance=[0.02, 0.09, 0.02, 0, 0, 0]
        )
        time.sleep(0.5)
        return result

    def force_task_close_right_handle(self):
        """
        Execute force control task to close the right handle using wobj coordinate system.
        This function includes all 6 motions (0-5) in sequence.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.wobj_transformation_matrix is None:
            print("Wobj coordinate system transformation matrix not loaded")
            return -1

        # ==============Motion 1: push to close the handle (first push)=================
        print("\n[Motion 1] Pushing to close the handle (first push)...")

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
            end_distance=[0.06, 0, 0, 0, 0, 0]
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
        
        # Set force mode parameters for main push
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in X, Y, Z directions
        wrench = [-25, 20, 0, 0, 0, -1.0]
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
            end_distance=[0.13, 0.10, 0.05, 0, 0, 0]
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
        
        # Set force mode parameters for locking
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in X, Y, Z directions
        wrench = [15, 40, 0, 0, 0, -1.0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - locking knob...")
        
        # Execute force control with distance-based termination
        result5 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.05,
            end_type=3,
            end_distance=[0.07, 0.20, 0.05, 0, 0, 0]
        )
        
        if result5 != 0:
            print(f"[ERROR] Motion 6 failed with code: {result5}")
            return result5
        
        print("[INFO] Motion 6 completed successfully")
        time.sleep(0.5)
        
        print("[INFO] CloseRight handle sequence completed successfully")
        return 0
    
    def force_task_touch_server(self):
        """
        Execute force control task to touch the server before pulling using wobj coordinate system.
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
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Z direction
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

    # ================================ Task Execution Methods ================================
    def execute_complete_close_right_sequence(self):
        """
        Execute complete close right handle sequence following the task flow
        """
        print("\n" + "="*70)
        print("STARTING COMPLETE CLOSE RIGHT HANDLE SEQUENCE")
        print("="*70)
        
        # ===============Step 1: Close right handle operation=================
        # move to reference joint positions
        print("\n" + "="*50)
        result = self.movej_to_reference_position()
        if result != 0:
            return result
        time.sleep(0.5)
        
        # align tool TCP with local coordinate system
        # Tool X+ -> Local X+, Tool Y+ -> Local Z-, Tool Z+ -> Local Y+
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
        result = self.movel_to_start_position(execution_order=[1, 3, 2])
        if result != 0:
            return result
        time.sleep(0.5)

        # force control to touch right handle
        print("\n" + "="*50)
        result = self.force_task_touch_right_handle()
        if result != 0:
            return result
        time.sleep(0.5)

        # force control to close right handle (includes Motion 0-5)
        print("\n" + "="*50)
        result = self.force_task_close_right_handle()
        if result != 0:
            return result
        time.sleep(0.5)

        # move away from the handle
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([0, -0.10, 0])
        if result != 0:
            return result
        time.sleep(0.5)

        # move up slightly
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([0, 0, -0.025])
        if result != 0:
            return result
        time.sleep(0.5)

        # ===============Step 2: Pull server operation=================
        # move to the server handle
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([-0.11, 0.095, 0])
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
        result = self.movel_in_wobj_frame([0, -0.015, -0.03])
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
        print("COMPLETE CLOSE RIGHT HANDLE SEQUENCE FINISHED SUCCESSFULLY")
        print("="*70)
        return 0


if __name__ == "__main__":
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='UROpCloseRight - Close right handle operation')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot (default: 192.168.1.15)')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot (default: 30002)')
    parser.add_argument('--operation-name', type=str, default='close_right',
                       help='Name of the operation (default: close_right)')
    
    args = parser.parse_args()
    
    # Create UROpCloseRight instance
    ur_close_right = UROpCloseRight(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        operation_name=args.operation_name
    )
    
    # ==============Display automatically loaded parameters=================
    print("\n" + "="*70)
    print("AUTOMATICALLY LOADED PARAMETERS")
    print("="*70)
    
    if ur_close_right.camera_matrix is not None:
        print("\n✓ Camera Matrix loaded")
        print(ur_close_right.camera_matrix)
        print("\n✓ Distortion Coefficients loaded")
        print(ur_close_right.distortion_coefficients)
    
    if ur_close_right.cam2end_matrix is not None:
        print("\n✓ Camera to End-effector Matrix loaded")
        print(ur_close_right.cam2end_matrix)
    
    if ur_close_right.estimated_keypoints is not None:
        print(f"\n✓ Estimated Keypoints loaded ({len(ur_close_right.estimated_keypoints)} points)")
        for i, kp in enumerate(ur_close_right.estimated_keypoints[:3]):  # Show first 3
            print(f"  Point {i}: ({kp[0]:.6f}, {kp[1]:.6f}, {kp[2]:.6f})")
        if len(ur_close_right.estimated_keypoints) > 3:
            print(f"  ... and {len(ur_close_right.estimated_keypoints) - 3} more")
    
    if ur_close_right.local_transformation_matrix is not None:
        print("\n✓ Local Coordinate System loaded")
        print(f"  Origin: {ur_close_right.local_origin}")
    
    if ur_close_right.wobj_transformation_matrix is not None:
        print("\n✓ Wobj Coordinate System loaded")
        print(f"  Origin: {ur_close_right.wobj_origin}")
    
    if ur_close_right.ref_joint_angles is not None:
        print("\n✓ Reference Joint Angles loaded (radians):")
        for i, angle in enumerate(ur_close_right.ref_joint_angles):
            print(f"  Joint {i}: {angle:.6f}")
    
    if ur_close_right.offset_in_local is not None:
        print(f"\n✓ Task Position Offset loaded (type: {ur_close_right.offset_in_local_type})")
        if ur_close_right.offset_in_local_type == 'multiple':
            for step_key, offset in ur_close_right.offset_in_local.items():
                print(f"  {step_key}: x={offset['x']:.6f}, y={offset['y']:.6f}, z={offset['z']:.6f}")
        else:
            print(f"  x={ur_close_right.offset_in_local['x']:.6f}, "
                  f"y={ur_close_right.offset_in_local['y']:.6f}, "
                  f"z={ur_close_right.offset_in_local['z']:.6f}")

    # ==============Execute the close right handle task=================
    print("\n" + "="*70)
    print("STARTING TASK EXECUTION")
    print("="*70)
    
    try:
        result = ur_close_right.execute_complete_close_right_sequence()
        
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
