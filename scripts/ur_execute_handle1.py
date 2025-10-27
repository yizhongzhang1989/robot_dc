import os
import time
import numpy as np
from scipy.spatial.transform import Rotation as R
from ur_execute_base import URExecuteBase

# when the tool 

class URExecuteHandle1(URExecuteBase):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, rs485_port=54321):
        # Call parent class initialization first
        super().__init__(robot_ip, robot_port, rs485_port)
        
        # Override camera parameters path (if needed)
        self.camera_params_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur15_cam_calibration_result",
            "ur15_camera_parameters"
        )
        
        # Data directory path - customized for handle1
        self.data_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_handle1_data"
        )
        
        # Result directory path - customized for handle1
        self.result_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur_locate_handle1_result"
        )
        
        # Initialize parameter storage
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.cam2end_matrix = None
        
        # Local coordinate system storage
        self.local_transformation_matrix = None
        self.local_origin = None
        
        # Reference pose storage
        self.ref_joint_angles = None
        
        # Estimated keypoints storage
        self.estimated_keypoints = None
        
        # Task position information storage
        self.task_position_offset = None

        # Reload parameters from handle1-specific directories
        self._load_estimated_kp_coordinates()
        self._load_local_coordinate_system()
        self._load_ref_joint_angles()
        self._load_task_position_information()

    def movel_to_correct_tool_tcp(self):
        """
        Align tool TCP orientation with base and local coordinate system in two steps:
        Step 1: Align tool axes with coordinate systems
        - Tool Z+ aligns with Base Z- (pointing downward)
        - Tool X+ aligns with Local X+
        - Tool Y+ is determined by right-hand rule (Y = Z × X)
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
        
        # Base coordinate system Z axis (downward direction for tool)
        base_z_negative = np.array([0, 0, -1])  # Base Z- direction
        
        # Construct target rotation matrix for tool frame:
        # Tool Z+ -> Base Z- (pointing downward)
        # Tool X+ -> Local X+ projected to XY plane (no Z component)
        # Tool Y+ -> determined by right-hand rule: Y = Z × X
        tool_z = base_z_negative
        
        # Project Local X+ to XY plane by removing Z component
        local_x_xy = local_x.copy()
        local_x_xy[2] = 0  # Remove Z component
        tool_x = local_x_xy / np.linalg.norm(local_x_xy)  # Normalize
        
        tool_y = np.cross(tool_z, tool_x)  # Right-hand rule: Y = Z × X
        
        # Normalize to ensure orthonormal basis
        tool_y = tool_y / np.linalg.norm(tool_y)
        
        target_tool_rotation = np.column_stack([
            tool_x,  # Tool X+ = Local X+
            tool_y,  # Tool Y+ = Z × X (right-hand rule)
            tool_z   # Tool Z+ = Base Z-
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
        
        print("\nStep 1: Aligning tool TCP with base and local coordinate systems...")
        print(f"Base Z- direction (Tool Z+): {base_z_negative}")
        print(f"Local X+ direction (original): {local_x}")
        print(f"Local X+ projected to XY plane (Tool X+): {tool_x}")
        print(f"Tool Y+ (Z × X, right-hand rule): {tool_y}")
        print(f"Target rotation vector: {rotation_vector}")
        print(f"Target pose: {target_pose}")
        
        res = self.robot.movel(target_pose, a=0.1, v=0.05)
        
        if res != 0:
            print(f"Failed to align tool TCP (error code: {res})")
            return res
        
        print("Step 1 completed successfully")
        time.sleep(0.5)
        
        # Step 2: Rotate 30 degrees around TCP Z axis
        # Create rotation around Z axis (30 degrees = pi/6 radians)
        angle_deg = 30

        angle_rad = np.deg2rad(angle_deg)
        
        # The Z axis in tool frame corresponds to base_z_negative in base frame
        # We need to rotate around this axis
        rotation_axis = base_z_negative / np.linalg.norm(base_z_negative)  # Normalize (should already be normalized)
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
            target_position[2] + 0.2265 - 0.02,       # z (move to target Z)
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

    def movel_to_extract_server(self, distance):
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
        
        # Create exit pose: move 0.2m in negative X direction
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
    

if __name__ == "__main__":
    # Example usage
    ur_handle1 = URExecuteHandle1()
    
    # All parameters are automatically loaded during initialization
    # Access them directly from instance variables
    
    print("\n" + "="*60)
    print("UR Execute Handle1 - Loaded Parameters")
    print("="*60)
    
    print("\n[Data Directory]")
    print(f"Data directory: {ur_handle1.data_dir}")
    
    print("\n[Result Directory]")
    print(f"Result directory: {ur_handle1.result_dir}")
    
    if ur_handle1.camera_matrix is not None:
        print("\n[Camera Intrinsic Parameters]")
        print("Camera Matrix:")
        print(ur_handle1.camera_matrix)
        print("\nDistortion Coefficients:")
        print(ur_handle1.distortion_coefficients)
    
    if ur_handle1.cam2end_matrix is not None:
        print("\n[Camera Extrinsic Parameters]")
        print("Camera to End-effector Matrix:")
        print(ur_handle1.cam2end_matrix)
    
    if ur_handle1.estimated_keypoints is not None:
        print("\n[Estimated Keypoints]")
        print(f"Total number of keypoints: {len(ur_handle1.estimated_keypoints)}")
        for kp in ur_handle1.estimated_keypoints:
            print(f"  Keypoint {kp['keypoint_index']}: "
                  f"({kp['x']:.6f}, {kp['y']:.6f}, {kp['z']:.6f}), "
                  f"views: {kp.get('num_views', 'N/A')}, "
                  f"residual: {kp.get('residual_norm', 'N/A')}")
    
    if ur_handle1.local_transformation_matrix is not None:
        print("\n[Local Coordinate System]")
        print("Transformation Matrix:")
        print(ur_handle1.local_transformation_matrix)
        if ur_handle1.local_origin is not None:
            print(f"\nOrigin: ({ur_handle1.local_origin['x']:.6f}, "
                  f"{ur_handle1.local_origin['y']:.6f}, "
                  f"{ur_handle1.local_origin['z']:.6f})")
    
    if ur_handle1.ref_joint_angles is not None:
        print("\n[Reference Joint Angles]")
        print("Joint angles (radians):")
        for i, angle in enumerate(ur_handle1.ref_joint_angles):
            print(f"  Joint {i}: {angle:.6f}")
    
    if ur_handle1.task_position_offset is not None:
        print("\n[Task Position Offset]")
        print("Offset in local coordinate system:")
        print(f"  x: {ur_handle1.task_position_offset.get('x', 0):.6f}")
        print(f"  y: {ur_handle1.task_position_offset.get('y', 0):.6f}")
        print(f"  z: {ur_handle1.task_position_offset.get('z', 0):.6f}")
    
    # Calculate target position in base coordinate system
    target_position = ur_handle1.get_target_position()
    if target_position is not None:
        print("\n[Target Position in Base Coordinate System]")
        print(f"Position: ({target_position[0]:.6f}, "
              f"{target_position[1]:.6f}, "
              f"{target_position[2]:.6f})")
    
    print("\n" + "="*60)
    
    # initialize platform height
    print("\n" + "="*50)
    ur_handle1.pushrod_to_base()
    time.sleep(5)
    ur_handle1.pushrod_to_execution_position()
    time.sleep(5)

    print("\n" + "="*50)
    ur_handle1.lift_platform_to_height(target_height=789.0)
    time.sleep(5)   

    # move to reference joint positions
    print("\n" + "="*60)
    print("Moving to Reference Joint Positions...")
    print("="*60)
    ur_handle1.movej_to_reference_joint_positions()
    time.sleep(0.5)

    # align tool TCP with local coordinate system
    print("\n" + "="*50)
    ur_handle1.movel_to_correct_tool_tcp()
    time.sleep(0.5)

    # move to target position using linear movement
    print("\n" + "="*50)
    ur_handle1.movel_to_target_position()
    time.sleep(0.5)

    # step1: extract to 0.2m
    print("\n" + "="*50)
    ur_handle1.movel_to_extract_server(-0.2)
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle1.lift_platform_to_height(target_height=799.0)
    time.sleep(3)

    # step2: extract to 0.5m
    print("\n" + "="*50)
    ur_handle1.movel_to_extract_server(-0.25)
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle1.lift_platform_to_height(target_height=789.0)
    time.sleep(3)

    print("\n" + "="*50)
    ur_handle1.lift_platform_to_height(target_height=794.0)
    time.sleep(3)

    # step3: extract to 0.7m
    print("\n" + "="*50)
    ur_handle1.movel_to_extract_server(-0.25)
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle1.lift_platform_to_height(target_height=799.0)
    time.sleep(3)

    # step4: extract to 1.20m
    print("\n" + "="*50)
    ur_handle1.movel_to_extract_server(-0.5)
    time.sleep(0.5)

    # move to exit position
    print("\n" + "="*50)
    ur_handle1.movel_to_exit()
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle1.movej_to_reference_joint_positions()
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle1.pushrod_to_base()
    time.sleep(3)

    print("\n" + "="*50)
    ur_handle1.lift_platform_to_init()
    time.sleep(10)


