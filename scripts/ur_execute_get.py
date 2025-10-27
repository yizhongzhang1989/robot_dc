import os
from ur_execute_base import URExecuteBase
import time
import numpy as np
from scipy.spatial.transform import Rotation as R


class URExecuteGet(URExecuteBase):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, rs485_port=54321):
        # Initialize parent class first
        # We need to temporarily store the custom paths
        self._custom_data_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_get_data"
        )
        
        self._custom_result_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_get_result"
        )
        
        # Call parent constructor
        super().__init__(robot_ip, robot_port, rs485_port)
        
        # Override the data and result directories after parent initialization
        self.data_dir = self._custom_data_dir
        self.result_dir = self._custom_result_dir
        
        # Reload parameters with the new directories
        self._reload_parameters()
    
    def _reload_parameters(self):
        """Reload parameters that depend on data_dir and result_dir"""
        print("\n" + "="*50)
        print("Reloading parameters with custom directories...")
        print("="*50)
        
        # Reload estimated keypoints from the new result directory
        self._load_estimated_kp_coordinates()
        
        # Reload local coordinate system from the new result directory
        self._load_local_coordinate_system()
        
        # Reload reference joint angles from the new data directory
        self._load_ref_joint_angles()
        
        # Reload task position information from the new data directory
        self._load_task_position_information()

    def movel_to_correct_tool_tcp(self):
        """
        Align tool TCP orientation with local coordinate system in two steps:
        Step 1: Align tool axes with local coordinate system
        - Tool X+ aligns with Local X+
        - Tool Y+ aligns with Local Z-
        - Tool Z+ aligns with Local Y+
        Step 2: Rotate 30 degrees around TCP Z axis
        Returns: 0 if successful, error code otherwise
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
        # Tool Z+ -> Local Y+
        target_tool_rotation = np.column_stack([
            local_x,      # Tool X+ = Local X+
            -local_z,     # Tool Y+ = Local Z-
            local_y       # Tool Z+ = Local Y+
        ])
        
        # Convert rotation matrix to rotation vector (axis-angle representation)
        # UR uses rotation vector [rx, ry, rz] where the direction is the axis
        # and the magnitude is the angle in radians
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
        print(f"Local X+ direction: {local_x}")
        print(f"Local Y+ direction: {local_y}")
        print(f"Local Z+ direction: {local_z}")
        print(f"Target rotation vector: {rotation_vector}")
        print(f"Target pose: {target_pose}")
        
        res = self.robot.movel(target_pose, a=0.1, v=0.05)
        
        if res != 0:
            print(f"Failed to align tool TCP (error code: {res})")
            return res
        
        print("Step 1 completed successfully")
        time.sleep(5)
        
        # Step 2: Rotate 30 degrees around TCP Z axis
        # Create rotation around Z axis (30 degrees = pi/6 radians)
        angle_deg = 30

        angle_rad = np.deg2rad(angle_deg)
        
        # The Z axis in tool frame corresponds to local_y in base frame
        # We need to rotate around this axis
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
        print(f"Combined rotation vector: {combined_rotation_vector}")
        print(f"Final pose: {final_pose}")
        
        res = self.robot.movel(final_pose, a=0.1, v=0.05)
        
        if res == 0:
            print("Tool TCP aligned and rotated successfully")
        else:
            print(f"Failed to rotate around TCP Z axis (error code: {res})")
        
        return res

    def movel_to_target_position(self):
        """
        Move robot to target position using linear movement (movel) in two steps:
        1. Move along Y and Z directions
        2. Move along X direction
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
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
        
        # Step 1: Move along Y and Z directions (keep X unchanged)
        intermediate_pose = [
            current_pose[0],      # x (keep current X)
            target_position[1],   # y (move to target Y)
            target_position[2],   # z (move to target Z)
            current_pose[3],      # rx (keep current orientation)
            current_pose[4],      # ry
            current_pose[5]       # rz
        ]
        
        print("\nStep 1: Moving along Y and Z directions...")
        print(f"Intermediate pose: {intermediate_pose}")
        
        res = self.robot.movel(intermediate_pose, a=0.1, v=0.05)
        
        if res != 0:
            print(f"Failed to move along Y and Z (error code: {res})")
            return res
        
        print("Step 1 completed successfully")
        time.sleep(5)
        
        # Step 2: Move along X direction to final target
        final_pose = [
            target_position[0]-0.08-0.01,  # x (move to target X with tool offset)
            target_position[1],       # y (already at target Y)
            target_position[2],       # z (already at target Z)
            current_pose[3],          # rx (keep current orientation)
            current_pose[4],          # ry
            current_pose[5]           # rz
        ]
        
        print("\nStep 2: Moving along X direction...")
        print(f"Final pose: {final_pose}")
        
        res = self.robot.movel(final_pose, a=0.1, v=0.05)
        
        if res == 0:
            print("Robot moved to target position successfully")
        else:
            print(f"Failed to move along X (error code: {res})")
        
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
    # Example usage
    print("="*50)
    print("Initializing URExecuteGet...")
    print("="*50)
    
    ur_get = URExecuteGet()
    
    # Print camera parameters
    if ur_get.camera_matrix is not None:
        print("\n" + "="*50)
        print("Camera Intrinsic Parameters")
        print("="*50)
        print("Camera Matrix:")
        print(ur_get.camera_matrix)
        print("\nDistortion Coefficients:")
        print(ur_get.distortion_coefficients)
    
    if ur_get.cam2end_matrix is not None:
        print("\n" + "="*50)
        print("Camera Extrinsic Parameters")
        print("="*50)
        print("Camera to End-effector Matrix:")
        print(ur_get.cam2end_matrix)
    
    # Print estimated keypoints
    if ur_get.estimated_keypoints is not None:
        print("\n" + "="*50)
        print("Estimated Keypoints")
        print("="*50)
        for kp in ur_get.estimated_keypoints:
            print(f"Keypoint {kp['keypoint_index']}: "
                  f"({kp['x']:.6f}, {kp['y']:.6f}, {kp['z']:.6f}), "
                  f"views: {kp.get('num_views', 'N/A')}, "
                  f"residual: {kp.get('residual_norm', 'N/A')}")
    
    # Print local coordinate system
    if ur_get.local_transformation_matrix is not None:
        print("\n" + "="*50)
        print("Local Coordinate System")
        print("="*50)
        print("Transformation Matrix:")
        print(ur_get.local_transformation_matrix)
        if ur_get.local_origin is not None:
            print(f"\nOrigin: ({ur_get.local_origin['x']:.6f}, "
                  f"{ur_get.local_origin['y']:.6f}, "
                  f"{ur_get.local_origin['z']:.6f})")
    
    # Print reference joint angles
    if ur_get.ref_joint_angles is not None:
        print("\n" + "="*50)
        print("Reference Joint Angles")
        print("="*50)
        for i, angle in enumerate(ur_get.ref_joint_angles):
            print(f"Joint {i}: {angle:.6f} rad ({angle * 180 / 3.14159:.2f}°)")
    
    # Print task position offset
    if ur_get.task_position_offset is not None:
        print("\n" + "="*50)
        print("Task Position Offset (in local coordinate system)")
        print("="*50)
        print(f"x: {ur_get.task_position_offset.get('x', 0):.6f}")
        print(f"y: {ur_get.task_position_offset.get('y', 0):.6f}")
        print(f"z: {ur_get.task_position_offset.get('z', 0):.6f}")
    
    # Calculate and print target position in base coordinate system
    target_position = ur_get.get_target_position()
    if target_position is not None:
        print("\n" + "="*50)
        print("Target Position in Base Coordinate System")
        print("="*50)
        print(f"x: {target_position[0]:.6f}")
        print(f"y: {target_position[1]:.6f}")
        print(f"z: {target_position[2]:.6f}")
    
    print("\n" + "="*50)
    print("URExecuteGet initialization complete")
    print("="*50)
    
    # move to reference joint positions
    print("\n" + "="*50)
    print("Moving to reference position...")
    print("="*50)
    ur_get.movej_to_reference_joint_positions()
    time.sleep(0.5)
        
    # align tool TCP with local coordinate system
    print("\n" + "="*50)
    ur_get.movel_to_correct_tool_tcp()
    time.sleep(0.5)
    
    # move to target position using linear movement
    print("\n" + "="*50)
    ur_get.movel_to_target_position()
    time.sleep(0.5)

    # lock quick changer
    print("\n" + "="*50)
    ur_get.ur_lock_quick_changer()
    time.sleep(5)

    # move to exit position
    print("\n" + "="*50)
    ur_get.movel_to_exit_target_position()
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_get.movej_to_reference_joint_positions()
    time.sleep(0.5)

