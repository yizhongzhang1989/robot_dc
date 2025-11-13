import os
import json
from ur_execute_base import URExecuteBase
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur15_robot_arm.ur15 import UR15Robot

class URExecutePrePull2(URExecuteBase):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002):
        # Call parent class constructor first
        super().__init__(robot_ip, robot_port)
        
        # Override data_dir and result_dir with prepull2-specific paths
        self.data_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_prepull2_data"
        )
        
        self.result_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_prepull2_result"
        )
        
        # Reload parameters from the new directories
        self._load_estimated_kp_coordinates()
        self._load_local_coordinate_system()
        self._load_ref_joint_angles()
        self._load_task_position_information()
        self._load_crack_local_coordinate_system()

# ================================= Movement Functions ================================
    def movel_to_correct_tool_tcp(self):
        """
        Correct tool TCP orientation based on crack local coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1

        if not hasattr(self, 'crack_coord_transformation') or self.crack_coord_transformation is None:
            print("Crack local coordinate system transformation matrix not loaded")
            return -1

        # Get current TCP pose
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1

        # Extract rotation matrix from crack local coordinate system transformation matrix (3x3)
        crack_local_rotation = self.crack_coord_transformation[:3, :3]

        # Crack local coordinate system axes in base coordinates
        crack_local_x = crack_local_rotation[:, 0]  # Crack Local X+ direction
        crack_local_y = crack_local_rotation[:, 1]  # Crack Local Y+ direction
        crack_local_z = crack_local_rotation[:, 2]  # Crack Local Z+ direction

        # Construct target rotation matrix for tool based on crack local coordinate system:
        # Tool X+ -> Crack Local X+
        # Tool Y+ -> Crack Local Z- (negative Z)
        # Tool Z+ -> Crack Local Y+

        target_tool_rotation = np.column_stack([
            crack_local_x,     # Tool X+ = Crack Local X+
            -crack_local_z,    # Tool Y+ = Crack Local Z- (negative Z)
            crack_local_y      # Tool Z+ = Crack Local Y+
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

        print("\nStep 1: Aligning tool TCP...")
        print(f"Target pose: {target_pose}")

        res = self.robot.movel(target_pose, a=0.1, v=0.1)
        time.sleep(0.5)

        if res != 0:
            print(f"Failed to align tool TCP (error code: {res})")
            return res
        else:
            print("Aligned tool tcp pose with crack local coordinate system successfully")

        # Step 2: Rotate 31 degrees around TCP Z axis
        angle_deg = 31
        angle_rad = np.deg2rad(angle_deg)

        # The Z axis in tool corresponds to crack_local_y in base coordinates
        rotation_axis = crack_local_y / np.linalg.norm(crack_local_y)  # Normalize (should already be normalized)
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

        print(f"\nStep 2: Rotating {angle_deg} degrees around TCP Z axis to correct tool orientation...")
                
        res = self.robot.movel(final_pose, a=0.1, v=0.1)
        time.sleep(0.5)

        if res == 0:
            print("Tool TCP orientation corrected successfully")
        else:
            print(f"Failed to rotate around TCP Z axis (error code: {res})")

        return res

    def movel_to_target_position(self):
        """
        Move robot to target position using linear movement based on crack local coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if not hasattr(self, 'crack_coord_transformation') or self.crack_coord_transformation is None:
            print("Crack local coordinate system transformation matrix not loaded")
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
        
        # Extract crack local coordinate system axes from transformation matrix
        crack_local_rotation = self.crack_coord_transformation[:3, :3]
        crack_local_x = crack_local_rotation[:, 0]  # Crack Local X+ direction in base coordinates
        crack_local_y = crack_local_rotation[:, 1]  # Crack Local Y+ direction in base coordinates
        crack_local_z = crack_local_rotation[:, 2]  # Crack Local Z+ direction in base coordinates

        # Calculate movement vector from current position to target
        current_position = np.array(current_pose[:3])
        target_position_array = np.array(target_position)
        movement_vector = target_position_array - current_position
        print(f"\nMovement vector in base coordinates: {movement_vector}")
        
        # Project movement vector onto crack local coordinate system axes
        movement_crack_local_x = np.dot(movement_vector, crack_local_x)
        movement_crack_local_y = np.dot(movement_vector, crack_local_y) 
        movement_crack_local_z = np.dot(movement_vector, crack_local_z)

        # Step 1: Move along crack local X and Z directions (keep crack local Y unchanged)
        # Calculate intermediate position by adding crack local X and Z movements
        intermediate_movement_x = movement_crack_local_x * crack_local_x
        intermediate_movement_z = movement_crack_local_z * crack_local_z
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
        
        print(f"\nStep 1: Moving along crack local X and Z directions...")
        print(f"Intermediate pose: {intermediate_pose}")
        
        res = self.robot.movel(intermediate_pose, a=0.1, v=0.1)
        time.sleep(0.5)

        if res != 0:
            print(f"Failed to move along crack local X and Z (error code: {res})")
            return res
        
        print("Step 1 completed successfully")
        
        # Step 2: Move along crack local Y direction to final target (with tool offset)
        tool_offset_y = -0.2265  # Tool offset along crack local Y direction (in meters)
        correct_offset = -0.10  # Desired offset from target along crack local Y direction (in meters)
        final_movement_y = (movement_crack_local_y + tool_offset_y + correct_offset) * crack_local_y
        final_position = intermediate_position + final_movement_y
        
        final_pose = [
            final_position[0],        # x
            final_position[1],        # y
            final_position[2],        # z
            current_pose[3],          # rx (keep current orientation)
            current_pose[4],          # ry
            current_pose[5]           # rz
        ]
        
        print(f"\nStep 2: Moving along crack local Y direction...")
        print(f"Final pose: {final_pose}")
        
        res = self.robot.movel(final_pose, a=0.1, v=0.1)
        time.sleep(0.5)

        if res == 0:
            print("Robot moved to target position successfully")
        else:
            print(f"Failed to move along crack local Y (error code: {res})")
        
        return res

# ================================ Force Control Functions ================================
    def force_task_touch_server(self):
        """
        Execute force control task to touch the left handle before prepull2 using TCP coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.local_transformation_matrix is None:
            print("Local coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Reconstruct the task frame BEFORE the 31-degree Z-axis rotation
        # This is the frame after Step 1 of movel_to_correct_tool_tcp (aligned with local coordinate system)
        
        # Extract rotation matrix from local transformation matrix
        local_rotation = self.local_transformation_matrix[:3, :3]
        local_x = local_rotation[:, 0]  # Local X+ direction
        local_y = local_rotation[:, 1]  # Local Y+ direction
        
        # Reconstruct the tool rotation before the 31-degree rotation
        # Tool Z+ is projection of Local Y+ onto base XOY plane
        tool_z_direction = np.array([local_y[0], local_y[1], 0.0])
        tool_z_direction = tool_z_direction / np.linalg.norm(tool_z_direction)
        
        # Tool Y+ = Tool Z+ × Tool X+
        tool_y_direction = np.cross(tool_z_direction, local_x)
        tool_y_direction = tool_y_direction / np.linalg.norm(tool_y_direction)
        
        # Construct rotation matrix before Z-axis rotation
        pre_rotation_matrix = np.column_stack([
            local_x,           # Tool X+ = Local X+
            tool_y_direction,  # Tool Y+ 
            tool_z_direction   # Tool Z+
        ])
        
        # Convert to rotation vector
        rotation_obj = R.from_matrix(pre_rotation_matrix)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Create task frame with current position but pre-rotation orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            rotation_vector[0], # rx (orientation before Z-rotation)
            rotation_vector[1], # ry
            rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (before Z-rotation): {task_frame}")
        
        # Set force mode parameters
        selection_vector = [0, 1, 0, 0, 0, 0]  # Enable force control in Z direction (now relative to task frame)
        wrench = [0, -15, 0, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task...")
        
        # Execute force control task with force-based termination
        result = self.robot.force_control_task(
            task_frame=task_frame,
            selection_vector=selection_vector,
            wrench=wrench,
            limits=limits,
            damping=0.1,
            end_type=2,
            end_force=[0,2,0,0,0,0]
        )
        time.sleep(0.5)
        return result

    def force_task_prepull_server(self):
        """
        Execute force control task to prepull2 the left handle using TCP coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        if self.local_transformation_matrix is None:
            print("Local coordinate system transformation matrix not loaded")
            return -1
        
        # Get current TCP pose
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Reconstruct the task frame BEFORE the 31-degree Z-axis rotation
        # This is the frame after Step 1 of movel_to_correct_tool_tcp (aligned with local coordinate system)
        
        # Extract rotation matrix from local transformation matrix
        local_rotation = self.local_transformation_matrix[:3, :3]
        local_x = local_rotation[:, 0]  # Local X+ direction
        local_y = local_rotation[:, 1]  # Local Y+ direction
        
        # Reconstruct the tool rotation before the 31-degree rotation
        # Tool Z+ is projection of Local Y+ onto base XOY plane
        tool_z_direction = np.array([local_y[0], local_y[1], 0.0])
        tool_z_direction = tool_z_direction / np.linalg.norm(tool_z_direction)
        
        # Tool Y+ = Tool Z+ × Tool X+
        tool_y_direction = np.cross(tool_z_direction, local_x)
        tool_y_direction = tool_y_direction / np.linalg.norm(tool_y_direction)
        
        # Construct rotation matrix before Z-axis rotation
        pre_rotation_matrix = np.column_stack([
            local_x,           # Tool X+ = Local X+
            tool_y_direction,  # Tool Y+ 
            tool_z_direction   # Tool Z+
        ])
        
        # Convert to rotation vector
        rotation_obj = R.from_matrix(pre_rotation_matrix)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Create task frame with current position but pre-rotation orientation
        task_frame = [
            tcp_pose[0],        # x (current position)
            tcp_pose[1],        # y
            tcp_pose[2],        # z
            rotation_vector[0], # rx (orientation before Z-rotation)
            rotation_vector[1], # ry
            rotation_vector[2]  # rz
        ]
        
        print(f"[INFO] Task frame (before Z-rotation): {task_frame}")
        
        # Set force mode parameters
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable force control in Z direction (now relative to task frame)
        wrench = [0, 0, -50, 0, 0, 0]  # Desired force/torque in each direction
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
            end_distance=[0,0,0.10,0,0,0]
        )
        time.sleep(0.5)
        return result


if __name__ == "__main__":
    # Create URExecutePrePull2 instance
    ur_prepull2 = URExecutePrePull2()
    
    # All parameters are automatically loaded during initialization
    # Access them directly from instance variables
    
    if ur_prepull2.camera_matrix is not None:
        print("\nCamera Matrix:")
        print(ur_prepull2.camera_matrix)
        print("\nDistortion Coefficients:")
        print(ur_prepull2.distortion_coefficients)
    
    if ur_prepull2.cam2end_matrix is not None:
        print("\nCamera to End-effector Matrix:")
        print(ur_prepull2.cam2end_matrix)
    
    if ur_prepull2.estimated_keypoints is not None:
        print("\nEstimated Keypoints:")
        for kp in ur_prepull2.estimated_keypoints:
            print(f"Keypoint {kp['keypoint_index']}: ({kp['x']:.6f}, {kp['y']:.6f}, {kp['z']:.6f})")
    
    if ur_prepull2.local_transformation_matrix is not None:
        print("\nLocal Coordinate System Transformation Matrix:")
        print(ur_prepull2.local_transformation_matrix)
    
    if ur_prepull2.ref_joint_angles is not None:
        print("\nReference Joint Angles (radians):")
        for i, angle in enumerate(ur_prepull2.ref_joint_angles):
            print(f"Joint {i}: {angle:.6f}")
    
    if ur_prepull2.task_position_offset is not None:
        print("\nTask Position Offset (in local coordinate system):")
        print(f"x: {ur_prepull2.task_position_offset.get('x', 0):.6f}")
        print(f"y: {ur_prepull2.task_position_offset.get('y', 0):.6f}")
        print(f"z: {ur_prepull2.task_position_offset.get('z', 0):.6f}")

    # Calculate target position in base coordinate system
    target_position = ur_prepull2.get_target_position()
    if target_position is not None:
        print("\nTarget Position in Base Coordinate System:")
        print(target_position)

    # ======================= Task execution ======================
    # move to reference joint positions (commented out for safety)
    print("\n" + "="*50)
    ur_prepull2.movej_to_reference_joint_positions()
    time.sleep(0.5)
    
    # align tool TCP with local coordinate system
    print("\n" + "="*50)
    ur_prepull2.movel_to_correct_tool_tcp()
    time.sleep(0.5)
    
    # move to target position using linear movement
    print("\n" + "="*50)
    ur_prepull2.movel_to_target_position()
    time.sleep(0.5)

    # execute force task to push server to the end
    print("\n" + "="*50)
    ur_prepull2.force_task_touch_server()
    time.sleep(0.5)

    # execute force task to pull server out again
    print("\n" + "="*50)
    ur_prepull2.force_task_prepull_server()
    time.sleep(0.5)
    
    # # move to leave the left handle
    print("\n" + "="*50)
    ur_prepull2.movel_in_crack_frame([0, 0.003, -0.02])
    time.sleep(0.5)

    # # move to leave the left handle
    print("\n" + "="*50)
    ur_prepull2.movel_in_crack_frame([0, -0.20, 0])
    time.sleep(0.5)

    # move to reference joint positions (commented out for safety)
    print("\n" + "="*50)
    ur_prepull2.movej_to_reference_joint_positions()
    time.sleep(0.5)





