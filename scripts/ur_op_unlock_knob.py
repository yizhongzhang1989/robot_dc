import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_operate import UROperate


class UROpUnlockKnob(UROperate):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, operation_name="unlock_knob"):
        """
        Initialize UROpUnlockKnob instance
        Args:
            robot_ip: IP address of the UR15 robot
            robot_port: Port number of the UR15 robot
            operation_name: Operation name for data organization (default: 'unlock_knob')
        """
        # Call parent class constructor
        super().__init__(robot_ip=robot_ip, robot_port=robot_port, operation_name=operation_name)
        
        print(f"UROpUnlockKnob initialized for operation: {operation_name}")


    # ================================= Movement Functions ================================
    def movel_to_right_knob(self):
        """
        Move robot to right knob position using linear movement based on wobj coordinate system.
        This method moves from target position (left knob) along wobj X axis by 0.21m to reach right knob.
        
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.wobj_transformation_matrix is None:
            print("Wobj coordinate system transformation matrix not loaded")
            return -1
        
        # Get target position in base coordinate system (left knob position)
        target_position = self.calculate_start_position_in_base()
        if target_position is None:
            print("Failed to calculate target position")
            return -1
        
        # Get current TCP pose to preserve orientation
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract wobj coordinate system axes from transformation matrix
        wobj_rotation = self.wobj_transformation_matrix[:3, :3]
        wobj_x = wobj_rotation[:, 0]  # Wobj X+ direction in base coordinates
        wobj_y = wobj_rotation[:, 1]  # Wobj Y+ direction in base coordinates
        wobj_z = wobj_rotation[:, 2]  # Wobj Z+ direction in base coordinates
        
        # Calculate movement vector from current position to target
        current_position = np.array(current_pose[:3])
        target_position_array = np.array(target_position)
        movement_vector = target_position_array - current_position
        
        print(f"\n[Right Knob] Movement vector in base coordinates: {movement_vector}")
        
        # Project movement vector onto wobj coordinate system axes
        movement_wobj_x = np.dot(movement_vector, wobj_x)
        movement_wobj_y = np.dot(movement_vector, wobj_y)
        movement_wobj_z = np.dot(movement_vector, wobj_z)
        
        # Step 1: Move along wobj X and Z directions (keep wobj Y unchanged)
        # Add 0.21m offset along wobj X axis to reach right knob
        intermediate_movement_x = (movement_wobj_x + 0.21) * wobj_x
        intermediate_movement_z = movement_wobj_z * wobj_z
        intermediate_movement = intermediate_movement_x + intermediate_movement_z
        intermediate_position = current_position + intermediate_movement
        
        intermediate_pose = [
            intermediate_position[0],  # x
            intermediate_position[1],  # y
            intermediate_position[2],  # z
            current_pose[3],          # rx (keep current orientation)
            current_pose[4],          # ry
            current_pose[5]           # rz
        ]
        
        print(f"Step 1: Moving along wobj X (with 0.21m offset) and Z directions...")
        print(f"  Movement in wobj frame: X={movement_wobj_x + 0.21:.6f}m, Y=0m, Z={movement_wobj_z:.6f}m")
        print(f"  Target pose: {intermediate_pose}")
        
        res = self.robot.movel(intermediate_pose, a=0.1, v=0.1)
        if res != 0:
            print(f"Step 1 failed with error code: {res}")
            return res
        
        time.sleep(0.5)
        
        # Step 2: Move along wobj Y direction to reach final position
        current_pose = self.robot.get_actual_tcp_pose()
        current_position = np.array(current_pose[:3])
        
        final_movement_y = movement_wobj_y * wobj_y
        final_position = current_position + final_movement_y
        
        final_pose = [
            final_position[0],  # x
            final_position[1],  # y
            final_position[2],  # z
            current_pose[3],   # rx (keep current orientation)
            current_pose[4],   # ry
            current_pose[5]    # rz
        ]
        
        print(f"Step 2: Moving along wobj Y direction...")
        print(f"  Movement in wobj frame: Y={movement_wobj_y:.6f}m")
        print(f"  Target pose: {final_pose}")
        
        res = self.robot.movel(final_pose, a=0.1, v=0.1)
        if res != 0:
            print(f"Step 2 failed with error code: {res}")
            return res
        
        print("[INFO] Successfully moved to right knob position")
        return 0

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
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in * direction relative to task frame
        wrench = [0, 0, 15, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits, use default values in user manual
        
        print("[INFO] Starting force task: 'Touch Knob'...")
        
        # Execute force control task with manual termination (end_type=0)
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=2,
            end_force=[15, 15, 20, 0, 0, 0]
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
        
        # ==============Motion 1: Rotate around Z axis to unlock the knob=================
        print("\n[Motion 1] Rotating counter-clockwise to unlock knob...")
        
        # Get current TCP pose to use as task frame
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Set force mode parameters for unlocking
        task_frame = tcp_pose
        selection_vector = [0, 0, 0, 0, 0, 1]  # Enable torque control in Z direction only
        wrench = [0, 0, 0, 0, 0, -1.0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - unlocking...")
        
        # Execute force control with angle-based termination
        result1 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.01,
            end_type=4,
            end_angle=135.0  # Stop after rotating 135 degrees
        )
        
        if result1 != 0:
            print(f"[ERROR] Motion 1 failed with code: {result1}")
            return result1
        
        print("[INFO] Motion 1 completed successfully")
        time.sleep(0.5)

        # ==============Motion 2: release the spring in the knob=================
        print("\n[Motion 2] Releasing the spring in the knob...")

        # Get current TCP pose to use as task frame
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Set force mode parameters for releasing spring
        task_frame = tcp_pose
        selection_vector = [0, 1, 1, 0, 0, 0]  # Enable force control in Y and Z directions
        wrench = [0, 0, -20.0, 0, 0, 0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - releasing spring...")
        
        # Execute force control with distance-based termination
        result2 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.002,
            end_type=3,
            end_distance=[0, 0.01, 0.003, 0, 0, 0]
        )
        
        if result2 != 0:
            print(f"[ERROR] Motion 2 failed with code: {result2}")
            return result2
        
        print("[INFO] Motion 2 completed successfully")
        time.sleep(0.5)
        
        # ==============Motion 3: Rotate back around Z axis to leave the knob=================
        print("\n[Motion 3] Rotating clockwise to return...")

        # Get updated TCP pose after first motion
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Updated TCP pose: {tcp_pose}")
        
        # Set force mode parameters for return motion
        task_frame = tcp_pose
        selection_vector = [0, 0, 0, 0, 0, 1]  # Enable torque control in Z direction only
        wrench = [0, 0, 0, 0, 0, 1.0]  # Apply 1.0 Nm torque (clockwise)
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - returning...")
        
        # Execute force control with angle-based termination
        result3 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.005,
            end_type=4,
            end_angle=15.0  # Stop after rotating 15 degrees
        )
        
        if result3 != 0:
            print(f"[ERROR] Motion 3 failed with code: {result3}")
            return result3

        print("[INFO] Motion 3 completed successfully")
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
        
        # ==============Motion 1: Rotate around Z axis to unlock the knob=================
        print("\n[Motion 1] Rotating clockwise to unlock knob...")
        
        # Get current TCP pose to use as task frame
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Set force mode parameters for unlocking
        task_frame = tcp_pose
        selection_vector = [0, 0, 0, 0, 0, 1]  # Enable torque control in Z direction only
        wrench = [0, 0, 0, 0, 0, 1.0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - unlocking...")
        
        # Execute force control with angle-based termination
        result1 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.01,
            end_type=4,
            end_angle=145.0  # Stop after rotating 145 degrees
        )
        
        if result1 != 0:
            print(f"[ERROR] Motion 1 failed with code: {result1}")
            return result1
        
        print("[INFO] Motion 1 completed successfully")
        time.sleep(0.5)

        # ==============Motion 2: release the spring in the knob=================
        print("\n[Motion 2] Releasing the spring in the knob...")

        # Get current TCP pose to use as task frame
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Set force mode parameters for releasing spring
        task_frame = tcp_pose
        selection_vector = [0, 1, 1, 0, 0, 0]  # Enable force control in Y and Z directions
        wrench = [0, 0, -25.0, 0, 0, 0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - releasing spring...")
        
        # Execute force control with distance-based termination
        result2 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.002,
            end_type=3,
            end_distance=[0, 0.01, 0.003, 0, 0, 0]
        )
        
        if result2 != 0:
            print(f"[ERROR] Motion 2 failed with code: {result2}")
            return result2
        
        print("[INFO] Motion 2 completed successfully")
        time.sleep(0.5)
        
        # ==============Motion 3: Rotate back around Z axis to leave the knob=================
        print("\n[Motion 3] Rotating counter-clockwise to return...")

        # Get updated TCP pose after first motion
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Updated TCP pose: {tcp_pose}")
        
        # Set force mode parameters for return motion
        task_frame = tcp_pose
        selection_vector = [0, 0, 0, 0, 0, 1]  # Enable torque control in Z direction only
        wrench = [0, 0, 0, 0, 0, -1.0]  # Apply -1.0 Nm torque (counter-clockwise)
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - returning...")
        
        # Execute force control with angle-based termination
        result3 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.005,
            end_type=4,
            end_angle=15.0  # Stop after rotating 15 degrees
        )
        
        if result3 != 0:
            print(f"[ERROR] Motion 3 failed with code: {result3}")
            return result3

        print("[INFO] Motion 3 completed successfully")
        time.sleep(0.5)

        print("[INFO] Right knob unlock sequence completed successfully")
        return 0

    # ================================ Task Execution Methods ================================
    def execute_complete_unlock_sequence(self):
        """
        Execute complete unlock sequence for both knobs following the task flow
        """
        print("\n" + "="*70)
        print("STARTING COMPLETE KNOB UNLOCK SEQUENCE")
        print("="*70)
        
        # ===============execute the knob unlocking task=================
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
        
        # move to target position (step1) using linear movement
        print("\n" + "="*50)
        result = self.movel_to_start_position(execution_order=[1, 3, 2])
        if result != 0:
            return result
        time.sleep(0.5)

        # ==============unlock the left knob=================
        # force control to move to touch the knob
        print("\n" + "="*50)
        result = self.force_task_touch_knob()
        if result != 0:
            return result
        time.sleep(0.5)

        # move away from the knob
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([0, -0.001, 0])
        if result != 0:
            return result
        time.sleep(0.5)

        # force control to unlock the left knob
        print("\n" + "="*50)
        result = self.force_task_unlock_left_knob()
        if result != 0:
            return result
        time.sleep(0.5)

        # move away from the knob
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([0, -0.2, 0])
        if result != 0:
            return result
        time.sleep(0.5)

        # move back to reference joint positions
        print("\n" + "="*50)
        result = self.movej_to_reference_position()
        if result != 0:
            return result
        time.sleep(0.5)

        # ==============unlock the right knob=================
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

        # move to right knob position along wobj X axis (offset 0.21m from left knob)
        print("\n" + "="*50)
        result = self.movel_to_right_knob()
        if result != 0:
            return result
        time.sleep(0.5)

        # force control to move to touch the right knob
        print("\n" + "="*50)
        result = self.force_task_touch_knob()
        if result != 0:
            return result
        time.sleep(0.5)

        # move away from the right knob
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([0, -0.001, 0])
        if result != 0:
            return result
        time.sleep(0.5)

        # force control to unlock the right knob
        print("\n" + "="*50)
        result = self.force_task_unlock_right_knob()
        if result != 0:
            return result
        time.sleep(0.5)

        # move away from the right knob
        print("\n" + "="*50)
        result = self.movel_in_wobj_frame([0, -0.2, 0])
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
        print("COMPLETE KNOB UNLOCK SEQUENCE FINISHED SUCCESSFULLY")
        print("="*70)
        return 0


if __name__ == "__main__":
    import argparse
    
    # Parse command line arguments
    parser = argparse.ArgumentParser(description='UROpUnlockKnob - Unlock knob operation')
    parser.add_argument('--robot-ip', type=str, default='192.168.1.15',
                       help='IP address of the UR15 robot (default: 192.168.1.15)')
    parser.add_argument('--robot-port', type=int, default=30002,
                       help='Port number of the UR15 robot (default: 30002)')
    parser.add_argument('--operation-name', type=str, default='unlock_knob',
                       help='Name of the operation (default: unlock_knob)')
    
    args = parser.parse_args()
    
    # Create UROpUnlockKnob instance
    ur_unlock_knob = UROpUnlockKnob(
        robot_ip=args.robot_ip,
        robot_port=args.robot_port,
        operation_name=args.operation_name
    )
    
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
    
    if ur_unlock_knob.estimated_keypoints is not None:
        print(f"\n✓ Estimated Keypoints loaded ({len(ur_unlock_knob.estimated_keypoints)} points)")
        for i, kp in enumerate(ur_unlock_knob.estimated_keypoints[:3]):  # Show first 3
            print(f"  Point {i}: ({kp[0]:.6f}, {kp[1]:.6f}, {kp[2]:.6f})")
        if len(ur_unlock_knob.estimated_keypoints) > 3:
            print(f"  ... and {len(ur_unlock_knob.estimated_keypoints) - 3} more")
    
    if ur_unlock_knob.local_transformation_matrix is not None:
        print("\n✓ Local Coordinate System loaded")
        print(f"  Origin: {ur_unlock_knob.local_origin}")
    
    if ur_unlock_knob.wobj_transformation_matrix is not None:
        print("\n✓ Wobj Coordinate System loaded")
        print(f"  Origin: {ur_unlock_knob.wobj_origin}")
    
    if ur_unlock_knob.ref_joint_angles is not None:
        print("\n✓ Reference Joint Angles loaded (radians):")
        for i, angle in enumerate(ur_unlock_knob.ref_joint_angles):
            print(f"  Joint {i}: {angle:.6f}")
    
    if ur_unlock_knob.offset_in_local is not None:
        print(f"\n✓ Task Position Offset loaded (type: {ur_unlock_knob.offset_in_local_type})")
        if ur_unlock_knob.offset_in_local_type == 'multiple':
            for step_key, offset in ur_unlock_knob.offset_in_local.items():
                print(f"  {step_key}: x={offset['x']:.6f}, y={offset['y']:.6f}, z={offset['z']:.6f}")
        else:
            print(f"  x={ur_unlock_knob.offset_in_local['x']:.6f}, "
                  f"y={ur_unlock_knob.offset_in_local['y']:.6f}, "
                  f"z={ur_unlock_knob.offset_in_local['z']:.6f}")

    # ==============Execute the knob unlocking task=================
    print("\n" + "="*70)
    print("STARTING TASK EXECUTION")
    print("="*70)
    
    try:
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
