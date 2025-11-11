import os
from ur_execute_base import URExecuteBase
import time
import numpy as np
from scipy.spatial.transform import Rotation as R


class URExecuteFrame(URExecuteBase):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002):
        # Call parent class constructor first
        super().__init__(robot_ip, robot_port)
        
        # Override data_dir and result_dir with frame-specific paths
        self.data_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_frame_data"
        )
        
        self.result_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_frame_result"
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
        
        # Step 2: Move along local Y direction to final target (with tool offset)
        tool_offset_y = -0.08  # Tool offset along local Y direction (in meters)
        final_movement_y = (movement_local_y + tool_offset_y) * local_y
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
    
    def movel_to_correct_tool_tcp(self):
        """
        Align tool TCP orientation with local coordinate system
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
        
        # Local coordinate system axes in base frame
        local_x = local_rotation[:, 0]  # Local X+ direction
        local_y = local_rotation[:, 1]  # Local Y+ direction
        local_z = local_rotation[:, 2]  # Local Z+ direction
        
        # Construct target rotation matrix for tool frame:
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
        
        
        # Step 2: Rotate 30 degrees around TCP Z axis
        # Create rotation around Z axis (30 degrees = pi/6 radians)
        angle_deg = 32
        angle_rad = np.deg2rad(angle_deg)
        
        # The Z axis in tool frame corresponds to local_y in base frame
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
    
    def movel_to_exit_target_position(self):
        """
        Move robot along negative X direction by 0.2m from current position
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        # Get current TCP pose
        current_pose = self.robot.get_actual_tcp_pose()
        if current_pose is None or len(current_pose) < 6:
            print("Failed to get current robot pose")
            return -1
        
        # Create exit pose: move 0.2m in negative X direction
        exit_pose = [
            current_pose[0] - 0.2,  # x - 0.2m
            current_pose[1],        # y (unchanged)
            current_pose[2],        # z (unchanged)
            current_pose[3],        # rx (keep orientation)
            current_pose[4],        # ry
            current_pose[5]         # rz
        ]
        
        print("\nMoving to exit position (X - 0.2m)...")
        print(f"Current pose: {current_pose}")
        print(f"Exit pose: {exit_pose}")
        
        res = self.robot.movel(exit_pose, a=0.1, v=0.05)
        
        if res == 0:
            print("Robot moved to exit position successfully")
        else:
            print(f"Failed to move to exit position (error code: {res})")
        
        return res



if __name__ == "__main__":
    # Create URExecuteFrame instance
    ur_frame = URExecuteFrame()
    
    # All parameters are automatically loaded during initialization
    # Access them directly from instance variables
    
    if ur_frame.camera_matrix is not None:
        print("\nCamera Matrix:")
        print(ur_frame.camera_matrix)
        print("\nDistortion Coefficients:")
        print(ur_frame.distortion_coefficients)
    
    if ur_frame.cam2end_matrix is not None:
        print("\nCamera to End-effector Matrix:")
        print(ur_frame.cam2end_matrix)
    
    if ur_frame.estimated_keypoints is not None:
        print("\nEstimated Keypoints:")
        for kp in ur_frame.estimated_keypoints:
            print(f"Keypoint {kp['keypoint_index']}: ({kp['x']:.6f}, {kp['y']:.6f}, {kp['z']:.6f})")
    
    if ur_frame.local_transformation_matrix is not None:
        print("\nLocal Coordinate System Transformation Matrix:")
        print(ur_frame.local_transformation_matrix)
    
    if ur_frame.ref_joint_angles is not None:
        print("\nReference Joint Angles (radians):")
        for i, angle in enumerate(ur_frame.ref_joint_angles):
            print(f"Joint {i}: {angle:.6f}")
    
    if ur_frame.task_position_offset is not None:
        print("\nTask Position Offset (in local coordinate system):")
        print(f"x: {ur_frame.task_position_offset.get('x', 0):.6f}")
        print(f"y: {ur_frame.task_position_offset.get('y', 0):.6f}")
        print(f"z: {ur_frame.task_position_offset.get('z', 0):.6f}")

    # Calculate target position in base coordinate system
    target_position = ur_frame.get_target_position()
    if target_position is not None:
        print("\nTarget Position in Base Coordinate System:")
        print(target_position)

    # move to reference joint positions (commented out for safety)
    print("\n" + "="*50)
    ur_frame.movej_to_reference_joint_positions()
    time.sleep(0.5)
    
    # align tool TCP with local coordinate system
    print("\n" + "="*50)
    ur_frame.movel_to_correct_tool_tcp()
    time.sleep(0.5)
    
    # move to target position using linear movement
    print("\n" + "="*50)
    ur_frame.movel_to_target_position()
    time.sleep(0.5)

    # unlock quick changer
    print("\n" + "="*50)
    ur_frame.ur_unlock_quick_changer()
    time.sleep(5)
    
    # move to exit position
    print("\n" + "="*50)
    ur_frame.movel_to_exit_target_position()
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_frame.movej_to_reference_joint_positions()
    time.sleep(0.5)


