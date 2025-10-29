import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_execute_base import URExecuteBase


class URExecuteHandle2(URExecuteBase):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, rs485_port=54321):
        # Call parent class initialization
        super().__init__(robot_ip, robot_port, rs485_port)
        
        # Override data directory path for handle2
        self.data_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_handle2_data"
        )
        
        # Override result directory path for handle2
        self.result_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_handle2_result"
        )
        
        # Reload parameters from handle2-specific directories
        self._load_estimated_kp_coordinates()
        self._load_local_coordinate_system()
        self._load_ref_joint_angles()
        self._load_task_position_information()
    
    def movel_to_correct_tool_tcp(self):
        """
        Align tool TCP orientation with base and local coordinate systems:
        - Tool Z+ aligns with Base Z- (pointing downward)
        - Tool Y+ aligns with Local Y-
        - Tool X+ aligns with Local X+
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
        
        # Base coordinate system Z axis (downward direction for tool)
        base_z_negative = np.array([0, 0, -1])  # Base Z- direction
        
        # Construct target rotation matrix for tool frame:
        # Priority 1: Tool Z+ -> Base Z- (pointing downward)
        # Priority 2: Tool X+ -> Local X+
        # Priority 3: Tool Y+ -> Calculated by right-hand rule: Y = Z × X
        tool_z = base_z_negative  # Base Z-
        tool_x = local_x  # Local X+
        
        # Normalize to ensure unit vectors
        tool_z = tool_z / np.linalg.norm(tool_z)
        tool_x = tool_x / np.linalg.norm(tool_x)
        
        # Calculate tool_y using right-hand rule: Y = Z × X
        tool_y = np.cross(tool_z, tool_x)
        tool_y = tool_y / np.linalg.norm(tool_y)
        
        target_tool_rotation = np.column_stack([
            tool_x,  # Tool X+ = Local X+
            tool_y,  # Tool Y+ = Local Y- (adjusted if needed)
            tool_z   # Tool Z+ = Base Z-
        ])
        
        # Verify determinant is +1 (right-handed coordinate system)
        det = np.linalg.det(target_tool_rotation)
        if abs(det - 1.0) > 0.01:
            print(f"Warning: Rotation matrix determinant is {det:.6f}, not 1.0")
            print("This may indicate a left-handed coordinate system")
        
        # Convert rotation matrix to rotation vector (axis-angle representation)
        rotation_obj = R.from_matrix(target_tool_rotation)
        rotation_vector = rotation_obj.as_rotvec()
        
        # Create target pose with aligned orientation
        target_pose = [
            current_pose[0],      # x (keep current position)
            current_pose[1],      # y
            current_pose[2],      # z
            rotation_vector[0],   # rx
            rotation_vector[1],   # ry
            rotation_vector[2]    # rz
        ]
        
        print("\nStep 1: Aligning tool TCP with base and local coordinate systems...")
        print(f"Tool Z+ (Base Z-): {tool_z}")
        print(f"Tool X+ (Local X+): {tool_x}")
        print(f"Tool Y+ (Z × X, right-hand rule): {tool_y}")
        print(f"Target rotation matrix determinant: {det:.6f}")
        print(f"Target rotation vector: {rotation_vector}")
        print(f"Target pose: {target_pose}")
        
        res = self.robot.movel(target_pose, a=0.1, v=0.05)
        
        if res != 0:
            print(f"Failed to align tool TCP (error code: {res})")
            return res
        
        print("Step 1 completed successfully")
        time.sleep(0.5)

        # Step 2: Rotate 30 degrees around TCP Z axis
        angle_deg = 30
        angle_rad = np.deg2rad(angle_deg)
        
        # The Z axis in tool frame corresponds to base_z_negative in base frame
        # We need to rotate around this axis
        rotation_axis = base_z_negative / np.linalg.norm(base_z_negative)  # Normalize
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
        time.sleep(0.5)
        
        if res == 0:
            print("Tool TCP aligned and rotated successfully")
        else:
            print(f"Failed to rotate around TCP Z axis (error code: {res})")
        
        return res


    def movel_to_target_position(self):
        """
        Move robot to target position using linear movement (movel) in two steps:
        1. Move along X and Y directions
        2. Move along Z direction
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
        
        # Step 1: Move along X and Y directions (keep Z unchanged)
        intermediate_pose = [
            target_position[0],  # x (move to target X with tool offset)
            target_position[1],       # y (move to target Y)
            current_pose[2],          # z (keep current Z)
            current_pose[3],          # rx (keep current orientation)
            current_pose[4],          # ry
            current_pose[5]           # rz
        ]
        
        print("\nStep 1: Moving along X and Y directions...")
        print(f"Intermediate pose: {intermediate_pose}")
        
        res = self.robot.movel(intermediate_pose, a=0.1, v=0.05)
        
        if res != 0:
            print(f"Failed to move along X and Y (error code: {res})")
            return res
        
        print("Step 1 completed successfully")
        time.sleep(0.5)
        
        # Step 2: Move along Z direction to final target
        final_pose = [
            target_position[0],  # x (already at target X)
            target_position[1],       # y (already at target Y)
            target_position[2] + 0.2265 + 0.02,       # z (move to target Z)
            current_pose[3],          # rx (keep current orientation)
            current_pose[4],          # ry
            current_pose[5]           # rz
        ]
        
        print("\nStep 2: Moving along Z direction...")
        print(f"Final pose: {final_pose}")
        
        res = self.robot.movel(final_pose, a=0.1, v=0.05)
        
        if res == 0:
            print("Robot moved to target position successfully")
        else:
            print(f"Failed to move along Z (error code: {res})")
        
        return res

    def movel_to_insert_server(self, distance):
        """
        Move robot along X negative direction by distance to insert server
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
        
        # Calculate target pose: move a distance meters in X direction
        target_pose = [
            current_pose[0] + distance,  # x (move distance m in x direction)
            current_pose[1],         # y (keep unchanged)
            current_pose[2],         # z (keep unchanged)
            current_pose[3],         # rx (keep current orientation)
            current_pose[4],         # ry
            current_pose[5]          # rz
        ]
        
        print("\nExtracting server: Moving 0.85m along X negative direction...")
        print(f"Current pose: {current_pose}")
        print(f"Target pose: {target_pose}")
        
        res = self.robot.movel(target_pose, a=0.1, v=0.05)
        
        if res == 0:
            print("Server extraction completed successfully")
        else:
            print(f"Failed to extract server (error code: {res})")
        
        return res

    def movel_to_exit(self):
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
        
        exit_pose = [
            current_pose[0],  
            current_pose[1],        # y (unchanged)
            current_pose[2] + 0.3,  # z (unchanged)
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

    def movej_to_execute_start(self):
        """
        Move robot to the get tool start position using joint movement
        Returns: 0 if successful, error code otherwise
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        # Target joint angles
        target_joint_angles = [-1.5214632193194788, -1.5912000141539515, -0.061849094927310944,
                               0.06347672521557612, 1.4398412704467773, -1.2330482641803187]
        
        print("\nMoving to get tool start position...")
        print(f"Target joint angles: {target_joint_angles}")
        
        res = self.robot.movej(target_joint_angles, a=0.5, v=0.5)
        time.sleep(0.5)

        target_joint_angles = [-1.5103414694415491, -1.712231775323385, -1.6820600032806396,
                               0.06347672521557612, 1.4398412704467773, -1.2330482641803187]
        
        print(f"Target joint angles: {target_joint_angles}")
        
        res = self.robot.movej(target_joint_angles, a=0.5, v=0.5)
        time.sleep(0.5)

        target_joint_angles = [-1.5103414694415491, -1.712231775323385, -1.6820600032806396,
                               -1.3431838166764756, 1.4398412704467773, -1.2330482641803187]
        
        print(f"Target joint angles: {target_joint_angles}")
        
        res = self.robot.movej(target_joint_angles, a=0.5, v=0.5)
        time.sleep(0.5)

        target_joint_angles = [-1.5103414694415491, -1.712231775323385, -1.6820600032806396,
                               -1.3431838166764756,  1.5084102153778076, 1.6445358991622925]
        
        print(f"Target joint angles: {target_joint_angles}")
        
        res = self.robot.movej(target_joint_angles, a=0.5, v=0.5)
        time.sleep(0.5)

        if res == 0:
            print("Robot moved to get tool start position successfully")
        else:
            print(f"Failed to move to get tool start position (error code: {res})")
        
        return res

    def movel_to_move_in_z(self, distance):
        """
        Move robot along X negative direction by distance to extract server
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
        
        # Calculate target pose: move 0.2m in X negative direction
        target_pose = [
            current_pose[0] - 0.01,  # x (move distance m in x direction)
            current_pose[1],         # y (keep unchanged)
            current_pose[2] + distance,         # z (keep unchanged)
            current_pose[3],         # rx (keep current orientation)
            current_pose[4],         # ry
            current_pose[5]          # rz
        ]
        
        print("\nExtracting server: Moving 0.85m along X negative direction...")
        print(f"Current pose: {current_pose}")
        print(f"Target pose: {target_pose}")
        
        res = self.robot.movel(target_pose, a=0.1, v=0.05)
        
        if res == 0:
            print("Server extraction completed successfully")
        else:
            print(f"Failed to extract server (error code: {res})")
        
        return res
        

if __name__ == "__main__":
    # Create instance of URExecuteHandle2
    ur_handle2 = URExecuteHandle2()
    
    # Print all loaded parameters
    print("\n" + "="*70)
    print("UR Execute Handle2 - Loaded Parameters")
    print("="*70)
    
    # Print directories
    print("\n📁 Directory Configuration:")
    print(f"  Data directory: {ur_handle2.data_dir}")
    print(f"  Result directory: {ur_handle2.result_dir}")
    print(f"  Camera parameters directory: {ur_handle2.camera_params_dir}")
    
    # Print camera intrinsic parameters
    if ur_handle2.camera_matrix is not None:
        print("\n📷 Camera Intrinsic Parameters:")
        print("  Camera Matrix:")
        print(ur_handle2.camera_matrix)
        print("\n  Distortion Coefficients:")
        print(ur_handle2.distortion_coefficients)
    else:
        print("\n📷 Camera Intrinsic Parameters: Not loaded")
    
    # Print camera extrinsic parameters
    if ur_handle2.cam2end_matrix is not None:
        print("\n🔗 Camera to End-effector Transformation Matrix:")
        print(ur_handle2.cam2end_matrix)
    else:
        print("\n🔗 Camera to End-effector Matrix: Not loaded")
    
    # Print estimated keypoints
    if ur_handle2.estimated_keypoints is not None:
        print(f"\n📍 Estimated Keypoints ({len(ur_handle2.estimated_keypoints)} total):")
        for kp in ur_handle2.estimated_keypoints:
            print(f"  Keypoint {kp['keypoint_index']}: "
                  f"({kp['x']:.6f}, {kp['y']:.6f}, {kp['z']:.6f}) "
                  f"[views: {kp.get('num_views', 'N/A')}, residual: {kp.get('residual_norm', 'N/A')}]")
    else:
        print("\n📍 Estimated Keypoints: Not loaded")
    
    # Print local coordinate system
    if ur_handle2.local_transformation_matrix is not None:
        print("\n🌐 Local Coordinate System:")
        print("  Transformation Matrix:")
        print(ur_handle2.local_transformation_matrix)
        if ur_handle2.local_origin is not None:
            print(f"\n  Origin: ({ur_handle2.local_origin['x']:.6f}, "
                  f"{ur_handle2.local_origin['y']:.6f}, "
                  f"{ur_handle2.local_origin['z']:.6f})")
    else:
        print("\n🌐 Local Coordinate System: Not loaded")
    
    # Print reference joint angles
    if ur_handle2.ref_joint_angles is not None:
        print("\n🤖 Reference Joint Angles (radians):")
        for i, angle in enumerate(ur_handle2.ref_joint_angles):
            print(f"  Joint {i}: {angle:.6f} rad ({angle * 180 / 3.14159:.2f}°)")
    else:
        print("\n🤖 Reference Joint Angles: Not loaded")
    
    # Print task position offset
    if ur_handle2.task_position_offset is not None:
        print("\n🎯 Task Position Offset (in local coordinate system):")
        print(f"  x: {ur_handle2.task_position_offset['x']:.6f}")
        print(f"  y: {ur_handle2.task_position_offset['y']:.6f}")
        print(f"  z: {ur_handle2.task_position_offset['z']:.6f}")
    else:
        print("\n🎯 Task Position Offset: Not loaded")
    
    # Calculate and print target position in base coordinate system
    print("\n" + "="*70)
    target_position = ur_handle2.get_target_position()
    if target_position is not None:
        print("\n✅ Target Position in Base Coordinate System:")
        print(f"  x: {target_position[0]:.6f}")
        print(f"  y: {target_position[1]:.6f}")
        print(f"  z: {target_position[2]:.6f}")
    else:
        print("\n❌ Target Position: Cannot be calculated")
    
    print("\n" + "="*70)
    print("Parameter loading complete!")
    print("="*70)

    # initialize platform height
    print("\n" + "="*50)
    ur_handle2.pushrod_to_base()
    time.sleep(6)

    print("\n" + "="*50)
    ur_handle2.lift_platform_to_base()
    time.sleep(6)

    print("\n" + "="*50)
    ur_handle2.lift_platform_coarse_adjust(910)
    time.sleep(5)   

    print("\n" + "="*50)
    ur_handle2.pushrod_fine_adjust(935)
    time.sleep(5)

    print("\n" + "="*50)
    ur_handle2.movej_to_execute_start()
    time.sleep(0.5)

    print("\n" + "="*70)
    print("Moving robot to reference joint positions...")
    ur_handle2.movej_to_reference_joint_positions()
    time.sleep(0.5)

    # align tool TCP with local coordinate system
    print("\n" + "="*50)
    ur_handle2.movel_to_correct_tool_tcp()
    time.sleep(0.5)
    
    # move to target position
    print("\n" + "="*50)
    ur_handle2.movel_to_target_position()
    time.sleep(0.5)

    # step1: move to positon 1
    print("\n" + "="*50)
    ur_handle2.movel_to_insert_server(distance=0.60)
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle2.pushrod_fine_adjust(925)
    time.sleep(5)   

    print("\n" + "="*50)
    ur_handle2.movel_to_move_in_z(-0.01)
    time.sleep(0.5)

    # step2: move to positon 2
    print("\n" + "="*50)
    ur_handle2.movel_to_insert_server(distance=0.60)
    time.sleep(0.5)

    # move to exit position
    print("\n" + "="*50)
    ur_handle2.movel_to_exit()
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle2.movej_to_reference_joint_positions()
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle2.pushrod_to_base()
    time.sleep(3)

    print("\n" + "="*50)
    ur_handle2.lift_platform_to_base()
    time.sleep(10)

    print("\n" + "="*50)
    ur_handle2.movej_to_get_tool_start()
    time.sleep(0.5)
