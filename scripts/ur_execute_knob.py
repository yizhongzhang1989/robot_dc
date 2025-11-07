import os
from ur_execute_base import URExecuteBase
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur15_robot_arm.ur15 import UR15Robot

class URExecuteKnob(URExecuteBase):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002):
        # Call parent class constructor first
        super().__init__(robot_ip, robot_port)
        
        # Override data_dir and result_dir with knob-specific paths
        self.data_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_knob_data"
        )
        
        self.result_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_knob_result"
        )
        
        # Reload parameters from the new directories
        self._load_estimated_kp_coordinates()
        self._load_local_coordinate_system()
        self._load_ref_joint_angles()
        self._load_task_position_information()
    
    def movel_to_target_position(self):
        """
        Move robot to target position using linear movement based on local coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.local_transformation_matrix is None:
            print("Local coordinate system transformation matrix not loaded")
            return -1
        
        # Get target position in base coordinate system
        target_position = self.get_target_position()
        if target_position is None:
            print("Failed to calculate target position")
            return -1
        
        # Get current TCP pose to preserve orientation
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract local coordinate system axes from transformation matrix
        local_rotation = self.local_transformation_matrix[:3, :3]
        local_x = local_rotation[:, 0]  # Local X+ direction in base coordinates
        local_y = local_rotation[:, 1]  # Local Y+ direction in base coordinates
        local_z = local_rotation[:, 2]  # Local Z+ direction in base coordinates

        # Calculate movement vector from current position to target
        current_position = np.array(current_pose[:3])
        target_position_array = np.array(target_position)
        movement_vector = target_position_array - current_position
        print(f"\nMovement vector in base coordinates: {movement_vector}")
        
        # Project movement vector onto local coordinate system axes
        movement_local_x = np.dot(movement_vector, local_x)
        movement_local_y = np.dot(movement_vector, local_y) 
        movement_local_z = np.dot(movement_vector, local_z)

        # Step 1: Move along local X and Z directions (keep local Y unchanged)
        # Calculate intermediate position by adding local X and Z movements
        intermediate_movement_x = movement_local_x * local_x
        intermediate_movement_z = movement_local_z * local_z
        intermediate_movement = intermediate_movement_x + intermediate_movement_z
        intermediate_position = current_position + intermediate_movement
        
        intermediate_pose = [
            intermediate_position[0], # x
            intermediate_position[1], # y
            intermediate_position[2], # z
            current_pose[3],          # rx (keep current orientation)
            current_pose[4],          # ry
            current_pose[5]           # rz
        ]
        
        print(f"\nStep 1: Moving along local X and Z directions...")
        print(f"Intermediate pose: {intermediate_pose}")
        
        res = self.robot.movel(intermediate_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res != 0:
            print(f"Failed to move along local X and Z (error code: {res})")
            return res
        
        print("Step 1 completed successfully")
        
        # Step 2: Move along local Y direction to final target
        final_movement_y = (movement_local_y - 0.4) * local_y
        final_position = intermediate_position + final_movement_y
        
        final_pose = [
            final_position[0],        # x
            final_position[1],        # y
            final_position[2],        # z
            current_pose[3],          # rx (keep current orientation)
            current_pose[4],          # ry
            current_pose[5]           # rz
        ]
        
        print(f"\nStep 2: Moving along local Y direction...")
        print(f"Final pose: {final_pose}")
        
        res = self.robot.movel(final_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res == 0:
            print("Robot moved to target position successfully")
        else:
            print(f"Failed to move along local Y (error code: {res})")
        
        return res
    
    def movel_to_right_knob(self):
        """
        Move robot to right knob position using linear movement based on local coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.local_transformation_matrix is None:
            print("Local coordinate system transformation matrix not loaded")
            return -1
        
        # Get target position in base coordinate system
        target_position = self.get_target_position()
        if target_position is None:
            print("Failed to calculate target position")
            return -1
        
        # Get current TCP pose to preserve orientation
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract local coordinate system axes from transformation matrix
        local_rotation = self.local_transformation_matrix[:3, :3]
        local_x = local_rotation[:, 0]  # Local X+ direction in base coordinates
        local_y = local_rotation[:, 1]  # Local Y+ direction in base coordinates
        local_z = local_rotation[:, 2]  # Local Z+ direction in base coordinates
        
        # Calculate movement vector from current position to target
        current_position = np.array(current_pose[:3])
        target_position_array = np.array(target_position)
        movement_vector = target_position_array - current_position
        
        print(f"\n[Right Knob] Movement vector in base coordinates: {movement_vector}")
        
        # Project movement vector onto local coordinate system axes
        movement_local_x = np.dot(movement_vector, local_x)
        movement_local_y = np.dot(movement_vector, local_y)
        movement_local_z = np.dot(movement_vector, local_z)
        
        # Step 1: Move along local X and Z directions (keep local Y unchanged)
        # Calculate intermediate position by adding local X and Z movements
        intermediate_movement_x = (movement_local_x + 0.21) * local_x
        intermediate_movement_z = movement_local_z * local_z
        intermediate_movement = intermediate_movement_x + intermediate_movement_z
        intermediate_position = current_position + intermediate_movement
        
        intermediate_pose = [
            intermediate_position[0], # x
            intermediate_position[1], # y
            intermediate_position[2], # z
            current_pose[3],          # rx (keep current orientation)
            current_pose[4],          # ry
            current_pose[5]           # rz
        ]
        
        print(f"\n[Right Knob] Step 1: Moving along local X and Z directions...")
        print(f"Intermediate pose: {intermediate_pose}")
        
        res = self.robot.movel(intermediate_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res != 0:
            print(f"Failed to move along local X and Z (error code: {res})")
            return res
        
        print("Step 1 completed successfully")
        
        # Step 2: Move along local Y direction to final target (with right knob offset)
        # Apply -0.21m offset in local Y direction for right knob, plus tool offset (-0.4m)
        final_movement_y = (movement_local_y - 0.4) * local_y
        final_position = intermediate_position + final_movement_y
        
        final_pose = [
            final_position[0],        # x
            final_position[1],        # y
            final_position[2],        # z
            current_pose[3],          # rx (keep current orientation)
            current_pose[4],          # ry
            current_pose[5]           # rz
        ]
        
        print(f"\n[Right Knob] Step 2: Moving along local Y direction...")
        print(f"Final pose: {final_pose}")
        
        res = self.robot.movel(final_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res == 0:
            print("Robot moved to right knob position successfully")
        else:
            print(f"Failed to move along local Y (error code: {res})")
        
        return res
    
    def movel_to_correct_tool_tcp(self):
        """
        Align tool TCP orientation with local coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.local_transformation_matrix is None:
            print("Local coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Extract rotation matrix from local transformation matrix (3x3)
        local_rotation = self.local_transformation_matrix[:3, :3]
        
        # Local coordinate system axes in base knob
        local_x = local_rotation[:, 0]  # Local X+ direction
        local_y = local_rotation[:, 1]  # Local Y+ direction
        local_z = local_rotation[:, 2]  # Local Z+ direction
        
        # Construct target rotation matrix for tool knob:
        # Tool X+ -> Local X+
        # Tool Y+ -> Local Z- (negative Z)
        # Tool Z+ -> projection of Local Y+ onto base XOY plane (parallel to base XOY)
        
        # Project local_y onto base XOY plane (set z component to 0 and normalize)
        tool_z_direction = np.array([local_y[0], local_y[1], 0.0])
        tool_z_direction = tool_z_direction / np.linalg.norm(tool_z_direction)
        
        # Tool Y+ should be perpendicular to both Tool X+ and Tool Z+
        # Use cross product: Tool Y+ = Tool Z+ × Tool X+
        tool_y_direction = np.cross(tool_z_direction, local_x)
        tool_y_direction = tool_y_direction / np.linalg.norm(tool_y_direction)
        
        target_tool_rotation = np.column_stack([
            local_x,           # Tool X+ = Local X+
            tool_y_direction,  # Tool Y+ = perpendicular to Tool X+ and Tool Z+
            tool_z_direction   # Tool Z+ = Local Y+ projected onto base XOY plane
        ])
        
        # Convert rotation matrix to rotation vector (axis-angle representation)
        # UR uses rotation vector [rx, ry, rz] where the direction is the axis and the magnitude is the angle in radians
        rotation_obj = R.from_matrix(target_tool_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Step 1: Align with local coordinate system
        target_pose = [
            current_pose[0],      # x (keep current position)
            current_pose[1],      # y
            current_pose[2],      # z
            rotation_vector[0],   # rx
            rotation_vector[1],   # ry
            rotation_vector[2]    # rz
        ]
        
        print("\nStep 1: Aligning tool TCP with local coordinate system...")
        print(f"Target pose: {target_pose}")
        
        res = self.robot.movel(target_pose, a=0.1, v=0.05)
        time.sleep(0.5)
        
        if res != 0:
            print(f"Failed to align tool TCP (error code: {res})")
            return res
        
        print("Step 1 completed successfully")
        
        # Step 2: Rotate 31 degrees around TCP Z axis
        angle_deg = 31
        angle_rad = np.deg2rad(angle_deg)
        
        # The Z axis in tool knob corresponds to local_y in base knob
        rotation_axis = local_y / np.linalg.norm(local_y)  # Normalize (should already be normalized)
        additional_rotation = R.from_rotvec(angle_rad * rotation_axis)
        
        # Combine the rotations: first align, then rotate around Z
        combined_rotation = additional_rotation * rotation_obj
        combined_rotation_vector = combined_rotation.as_rotvec()
        
        final_pose = [
            current_pose[0],              # x (keep current position)
            current_pose[1],              # y
            current_pose[2],              # z
            combined_rotation_vector[0],  # rx
            combined_rotation_vector[1],  # ry
            combined_rotation_vector[2]   # rz
        ]
        
        print(f"\nStep 2: Rotating {angle_deg} degrees around TCP Z axis...")
        print(f"Final pose: {final_pose}")
        
        res = self.robot.movel(final_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res == 0:
            print("Tool TCP aligned and rotated successfully")
        else:
            print(f"Failed to rotate around TCP Z axis (error code: {res})")
        
        return res

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
        task_frame = tcp_pose  # Use TCP coordinate system instead of base
        selection_vector = [1, 1, 1, 0, 0, 0]  # Enable force control in Z direction (now relative to TCP)
        wrench = [0, 0, 15, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task...")
        
        # Execute force control task with manual termination (end_type=0)
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=2,
            end_force=[15,15,20,0,0,0]
        )
        time.sleep(0.5)
        return result

    def force_task_unlock_left_knob(self):
        """
        Execute force control task to unlock the left knob using TCP coordinate system.
        Two-step process:
        1. Rotate counter-clockwise around TCP Z-axis to unlock (stops at 4 Nm)
        2. Rotate clockwise 30 degrees to return
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
        
        # Execute force control with force-based termination
        result1 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.01,
            end_type=4,
            end_angle=135.0  # Stop after rotating 150 degrees
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
        
        # Set force mode parameters for unlocking
        task_frame = tcp_pose
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable torque control in Z direction only
        wrench = [0, 0, -25.0, 0, 0, 0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - unlocking...")
        
        # Execute force control with force-based termination
        result1 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.002,
            end_type=3,
            end_distance=[0,0,0.003,0,0,0]
        )
        
        if result1 != 0:
            print(f"[ERROR] Motion 2 failed with code: {result1}")
            return result1
        
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
        result2 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.005,
            end_type=4,
            end_angle=15.0  # Stop after rotating 15 degrees
        )
        
        if result2 != 0:
            print(f"[ERROR] Motion 3 failed with code: {result2}")
            return result2

        print("[INFO] Motion 3 completed successfully")
        time.sleep(0.5)

        print("[INFO] Knob unlock sequence completed successfully")
        return 0
    
    def force_task_unlock_right_knob(self):
        """
        Execute force control task to unlock the right knob using TCP coordinate system.
        Two-step process:
        1. Rotate counter-clockwise around TCP Z-axis to unlock (stops at 4 Nm)
        2. Rotate clockwise 30 degrees to return
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
        wrench = [0, 0, 0, 0, 0, 1.0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - unlocking...")
        
        # Execute force control with force-based termination
        result1 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.01,
            end_type=4,
            end_angle=135.0  # Stop after rotating 150 degrees
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
        
        # Set force mode parameters for unlocking
        task_frame = tcp_pose
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable torque control in Z direction only
        wrench = [0, 0, -25.0, 0, 0, 0]
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - unlocking...")
        
        # Execute force control with force-based termination
        result1 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.002,
            end_type=3,
            end_distance=[0,0,0.003,0,0,0]
        )
        
        if result1 != 0:
            print(f"[ERROR] Motion 2 failed with code: {result1}")
            return result1
        
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
        wrench = [0, 0, 0, 0, 0, -1.0]  # Apply 1.0 Nm torque (clockwise)
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task - returning...")
        
        # Execute force control with angle-based termination
        result2 = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.005,
            end_type=4,
            end_angle=15.0  # Stop after rotating 15 degrees
        )
        
        if result2 != 0:
            print(f"[ERROR] Motion 3 failed with code: {result2}")
            return result2

        print("[INFO] Motion 3 completed successfully")
        time.sleep(0.5)

        print("[INFO] Knob unlock sequence completed successfully")
        return 0

    def movel_to_leave_the_knob(self, distance):
        """
        Move robot based on "distance" (dx, dy, dz) in local coordinate system to leave the knob.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.local_transformation_matrix is None:
            print("Local coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Parse distance parameter - must be a 3D array
        if isinstance(distance, (list, tuple)) and len(distance) == 3:
            local_displacement = np.array(distance)
        else:
            print("Invalid distance parameter. Must be [dx, dy, dz] in local coordinate system")
            return -1
        
        # Extract rotation matrix from local transformation matrix (3x3)
        local_rotation = self.local_transformation_matrix[:3, :3]
        
        # Transform displacement from local coordinate system to base coordinate system
        # displacement_base = R_local_to_base * displacement_local
        base_displacement = local_rotation @ local_displacement
        
        # Calculate target pose in base coordinate system
        target_pose = [
            current_pose[0] + base_displacement[0],  # x
            current_pose[1] + base_displacement[1],  # y
            current_pose[2] + base_displacement[2],  # z
            current_pose[3],                         # rx (keep current orientation)
            current_pose[4],                         # ry
            current_pose[5]                          # rz
        ]
        
        print(f"\n[INFO] Moving away from knob: {local_displacement} in local frame...")
        print(f"Local displacement (local frame): {local_displacement}")
        print(f"Base displacement (base frame): {base_displacement}")
        
        res = self.robot.movel(target_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res == 0:
            print("[INFO] Successfully moved away from knob")
        else:
            print(f"[ERROR] Failed to move away from knob (error code: {res})")
        
        return res

    

if __name__ == "__main__":
    # Create URExecuteKnob instance
    ur_knob = URExecuteKnob()
    
    # All parameters are automatically loaded during initialization
    # Access them directly from instance variables
    
    if ur_knob.camera_matrix is not None:
        print("\nCamera Matrix:")
        print(ur_knob.camera_matrix)
        print("\nDistortion Coefficients:")
        print(ur_knob.distortion_coefficients)
    
    if ur_knob.cam2end_matrix is not None:
        print("\nCamera to End-effector Matrix:")
        print(ur_knob.cam2end_matrix)
    
    if ur_knob.estimated_keypoints is not None:
        print("\nEstimated Keypoints:")
        for kp in ur_knob.estimated_keypoints:
            print(f"Keypoint {kp['keypoint_index']}: ({kp['x']:.6f}, {kp['y']:.6f}, {kp['z']:.6f})")
    
    if ur_knob.local_transformation_matrix is not None:
        print("\nLocal Coordinate System Transformation Matrix:")
        print(ur_knob.local_transformation_matrix)
    
    if ur_knob.ref_joint_angles is not None:
        print("\nReference Joint Angles (radians):")
        for i, angle in enumerate(ur_knob.ref_joint_angles):
            print(f"Joint {i}: {angle:.6f}")
    
    if ur_knob.task_position_offset is not None:
        print("\nTask Position Offset (in local coordinate system):")
        print(f"x: {ur_knob.task_position_offset.get('x', 0):.6f}")
        print(f"y: {ur_knob.task_position_offset.get('y', 0):.6f}")
        print(f"z: {ur_knob.task_position_offset.get('z', 0):.6f}")

    # Calculate target position in base coordinate system
    target_position = ur_knob.get_target_position()
    if target_position is not None:
        print("\nTarget Position in Base Coordinate System:")
        print(target_position)

    # move to reference joint positions (commented out for safety)
    print("\n" + "="*50)
    ur_knob.movej_to_reference_joint_positions()
    time.sleep(0.5)
    
    # align tool TCP with local coordinate system
    print("\n" + "="*50)
    ur_knob.movel_to_correct_tool_tcp()
    time.sleep(0.5)
    
    # move to target position using linear movement
    print("\n" + "="*50)
    ur_knob.movel_to_target_position()
    time.sleep(0.5)

    # #==============unlock the left knob=================
    # force control to move to touch the knob
    print("\n" + "="*50)
    ur_knob.force_task_touch_knob()
    time.sleep(0.5)

    # move away from the knob
    print("\n" + "="*50)
    ur_knob.movel_to_leave_the_knob([0, -0.001, 0])
    time.sleep(0.5)

    # force control to unlock the left knob
    print("\n" + "="*50)
    ur_knob.force_task_unlock_left_knob()
    time.sleep(0.5)

    # move away from the knob
    print("\n" + "="*50)
    ur_knob.movel_to_leave_the_knob([0, -0.2, 0])
    time.sleep(0.5)

    # move back to reference joint positions
    print("\n" + "="*50)
    ur_knob.movej_to_reference_joint_positions()
    time.sleep(0.5)

    # #==============unlock the right knob=================
    # align tool TCP with local coordinate system
    print("\n" + "="*50)
    ur_knob.movel_to_correct_tool_tcp()
    time.sleep(0.5)

    # move to right knob position using linear movement
    print("\n" + "="*50)
    ur_knob.movel_to_right_knob()
    time.sleep(0.5)

    # force control to move to touch the right knob
    print("\n" + "="*50)
    ur_knob.force_task_touch_knob()
    time.sleep(0.5)

    # move away from the right knob
    print("\n" + "="*50)
    ur_knob.movel_to_leave_the_knob([0, -0.001, 0])
    time.sleep(0.5)

    # force control to unlock the right knob
    print("\n" + "="*50)
    ur_knob.force_task_unlock_right_knob()
    time.sleep(0.5)

    # move away from the right knob
    print("\n" + "="*50)
    ur_knob.movel_to_leave_the_knob([0, -0.2, 0])
    time.sleep(0.5)

    # move back to reference joint positions
    print("\n" + "="*50)
    ur_knob.movej_to_reference_joint_positions()
    time.sleep(0.5)
