import os
from ur_execute_base import URExecuteBase
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur15_robot_arm.ur15 import UR15Robot

class URExecutePush2End(URExecuteBase):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002):
        # Call parent class constructor first
        super().__init__(robot_ip, robot_port)
        
        # Override data_dir and result_dir with push2end-specific paths
        self.data_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_push2end_data"
        )
        
        self.result_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_push2end_result"
        )
        
        # Reload parameters from the new directories
        self._load_estimated_kp_coordinates()
        self._load_local_coordinate_system()
        self._load_ref_joint_angles()
        self._load_task_position_information()
        self._load_crack_local_coordinate_system()

# =============================== Movement control functions =============================
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
        
        res = self.robot.movel(intermediate_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res != 0:
            print(f"Failed to move along crack local X and Z (error code: {res})")
            return res
        
        print("Step 1 completed successfully")
        
        # Step 2: Move along crack local Y direction to final target (with tool offset)
        tool_offset_y = -0.2265  # Tool offset along crack local Y direction (in meters)
        correct_offset = -0.20  # Desired offset from target along crack local Y direction (in meters)
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
        
        res = self.robot.movel(final_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res == 0:
            print("Robot moved to target position successfully")
        else:
            print(f"Failed to move along crack local Y (error code: {res})")
        
        return res

# ================================ Force control functions ================================
    def force_task_push_server_to_end(self):
        """
        Execute force control task to push the server using TCP coordinate system.
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        # Get current TCP pose to use as task frame (for force control in TCP coordinate system)
        tcp_pose = self.robot.get_actual_tcp_pose()
        print(f"[INFO] Current TCP pose: {tcp_pose}")
        
        # Set force mode parameters
        task_frame = tcp_pose  # Use TCP coordinate system instead of base
        selection_vector = [0, 0, 1, 0, 0, 0]  # Enable force control in Z direction (now relative to TCP)
        wrench = [0, 0, 50, 0, 0, 0]  # Desired force/torque in each direction
        limits = [0.2, 0.1, 0.1, 0.785, 0.785, 1.57]  # Force/torque limits
        
        print("[INFO] Starting force control task...")
        
        # Execute force control task with manual termination (end_type=0)
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


if __name__ == "__main__":
    # Create URExecutePush2End instance
    ur_push2end = URExecutePush2End()
    
    # All parameters are automatically loaded during initialization
    # Access them directly from instance variables
    
    if ur_push2end.camera_matrix is not None:
        print("\nCamera Matrix:")
        print(ur_push2end.camera_matrix)
        print("\nDistortion Coefficients:")
        print(ur_push2end.distortion_coefficients)
    
    if ur_push2end.cam2end_matrix is not None:
        print("\nCamera to End-effector Matrix:")
        print(ur_push2end.cam2end_matrix)
    
    if ur_push2end.estimated_keypoints is not None:
        print("\nEstimated Keypoints:")
        for kp in ur_push2end.estimated_keypoints:
            print(f"Keypoint {kp['keypoint_index']}: ({kp['x']:.6f}, {kp['y']:.6f}, {kp['z']:.6f})")
    
    if ur_push2end.local_transformation_matrix is not None:
        print("\nLocal Coordinate System Transformation Matrix:")
        print(ur_push2end.local_transformation_matrix)
    
    if ur_push2end.ref_joint_angles is not None:
        print("\nReference Joint Angles (radians):")
        for i, angle in enumerate(ur_push2end.ref_joint_angles):
            print(f"Joint {i}: {angle:.6f}")
    
    if ur_push2end.task_position_offset is not None:
        print("\nTask Position Offset (in local coordinate system):")
        print(f"x: {ur_push2end.task_position_offset.get('x', 0):.6f}")
        print(f"y: {ur_push2end.task_position_offset.get('y', 0):.6f}")
        print(f"z: {ur_push2end.task_position_offset.get('z', 0):.6f}")

    # Calculate target position in base coordinate system
    target_position = ur_push2end.get_target_position()
    if target_position is not None:
        print("\nTarget Position in Base Coordinate System:")
        print(target_position)

    # ============================= Task Execution ===========================
    # move to reference joint positions (commented out for safety)
    print("\n" + "="*50)
    ur_push2end.movej_to_reference_joint_positions()
    time.sleep(0.5)
    
    # align tool TCP with local coordinate system
    print("\n" + "="*50)
    ur_push2end.movel_to_correct_tool_tcp()
    time.sleep(0.5)
    
    # move to target position using linear movement
    print("\n" + "="*50)
    ur_push2end.movel_to_target_position()
    time.sleep(0.5)

    # execute force task to push server to the end
    print("\n" + "="*50)
    ur_push2end.force_task_push_server_to_end()
    time.sleep(0.5)

    # # move to leave the server
    print("\n" + "="*50)
    ur_push2end.movel_in_crack_frame([0, -0.3, 0])
    time.sleep(0.5)

    # move to reference joint positions (commented out for safety)
    print("\n" + "="*50)
    ur_push2end.movej_to_reference_joint_positions()
    time.sleep(0.5)





