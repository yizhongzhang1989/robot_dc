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
        Align tool TCP orientation with base and local coordinate system
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
        
        # Right-hand rule: Y = Z × X
        tool_y = np.cross(tool_z, tool_x)  
        # Normalize to ensure orthonormal basis
        tool_y = tool_y / np.linalg.norm(tool_y)
        
        target_tool_rotation = np.column_stack([
            tool_x,  # Tool X+ = Local X+
            tool_y,  # Tool Y+ = Z × X (right-hand rule)
            tool_z   # Tool Z+ = Base Z-
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
        
        print("\nStep 1: Aligning tool TCP with base and local coordinate systems...")
        print(f"Target pose: {target_pose}")

        res = self.robot.movel(target_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res != 0:
            print(f"Failed to align tool TCP (error code: {res})")
            return res
        
        print("Step 1 completed successfully")
        
        # Step 2: Rotate 31 degrees around TCP Z axis to eliminate tool installation errors
        angle_deg = 31
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

        # Step 1: Move along local X and Y directions (keep local Z unchanged)
        # Calculate intermediate position by adding local X and Y movements
        intermediate_movement_x = movement_local_x * local_x
        intermediate_movement_y = movement_local_y * local_y
        intermediate_movement = intermediate_movement_x + intermediate_movement_y
        intermediate_position = current_position + intermediate_movement
        
        intermediate_pose = [
            intermediate_position[0], # x
            intermediate_position[1], # y
            intermediate_position[2], # z
            current_pose[3],          # rx (keep current orientation)
            current_pose[4],          # ry
            current_pose[5]           # rz
        ]
        
        print(f"\nStep 1: Moving along local X and Y directions...")
        print(f"Intermediate pose: {intermediate_pose}")
        
        res = self.robot.movel(intermediate_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res != 0:
            print(f"Failed to move along local X and Y (error code: {res})")
            return res
        
        print("Step 1 completed successfully")

        
        # Step 2: Move along local Z direction to final target pulling position (between the two handles)
        tool_offset_z = 0.2265  # Tool offset along local Z direction (in meters)
        correct_offset = -0.01  # Correct offset to accurate (meters)
        final_movement_z = (movement_local_z + tool_offset_z + correct_offset) * local_z
        final_position = intermediate_position + final_movement_z
        
        final_pose = [
            final_position[0],        # x
            final_position[1],        # y
            final_position[2],        # z
            current_pose[3],          # rx (keep current orientation)
            current_pose[4],          # ry
            current_pose[5]           # rz
        ]
        
        print(f"\nStep 2: Moving along local Z direction...")
        print(f"Final pose: {final_pose}")
        
        res = self.robot.movel(final_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res == 0:
            print("Robot moved to target position successfully")
        else:
            print(f"Failed to move along local Z (error code: {res})")
        
        return res

    def force_task_extract_server(self,distance):
        """
        Execute force control task to extract the server out of crack for the first step.
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

        # Reconstruct the task frame BEFORE the 30-degree Z-axis rotation
        # This is the frame after Step 1 of movel_to_correct_tool_tcp (aligned with local coordinate system)
        
        # Extract rotation matrix from local transformation matrix
        local_rotation = self.local_transformation_matrix[:3, :3]
        local_x = local_rotation[:, 0]  # Local X+ direction
        local_y = local_rotation[:, 1]  # Local Y+ direction
        local_z = local_rotation[:, 2]  # Local Z+ direction
        
        # Base coordinate system Z axis (downward direction for tool)
        base_z_negative = np.array([0, 0, -1])  # Base Z- direction
        
        # Reconstruct target rotation matrix for tool frame before Z-rotation:
        # Tool Z+ -> Base Z- (pointing downward)
        # Tool X+ -> Local X+ projected to XY plane (no Z component)
        # Tool Y+ -> determined by right-hand rule: Y = Z × X
        tool_z = base_z_negative
        
        # Project Local X+ to XY plane by removing Z component
        local_x_xy = local_x.copy()
        local_x_xy[2] = 0  # Remove Z component
        tool_x = local_x_xy / np.linalg.norm(local_x_xy)  # Normalize
        
        # Right-hand rule: Y = Z × X
        tool_y = np.cross(tool_z, tool_x)  
        # Normalize to ensure orthonormal basis
        tool_y = tool_y / np.linalg.norm(tool_y)
        
        target_tool_rotation = np.column_stack([
            tool_x,  # Tool X+ = Local X+
            tool_y,  # Tool Y+ = Z × X (right-hand rule)
            tool_z   # Tool Z+ = Base Z-
        ])
        
        # Convert rotation matrix to rotation vector
        rotation_obj = R.from_matrix(target_tool_rotation)
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
        selection_vector = [0, 1, 1, 0, 0, 0]  # Enable force control in Z direction (now relative to task frame)
        wrench = [0, 60, 0, 0, 0, 0]  # Desired force/torque in each direction
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
            end_distance=[0,distance,0.05,0,0,0]
        )
        time.sleep(0.5)
        return result

    def movel_to_leave_server(self, distance):
        """
        Move robot based on "distance" (dx, dy, dz) in local coordinate system to leave the server.
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
        
        print(f"\n[INFO] Moving away from handle: {local_displacement} in local frame...")
        print(f"Local displacement (local frame): {local_displacement}")
        print(f"Base displacement (base frame): {base_displacement}")
        
        res = self.robot.movel(target_pose, a=0.1, v=0.05)
        time.sleep(0.5)

        if res == 0:
            print("[INFO] Successfully moved away from handle")
        else:
            print(f"[ERROR] Failed to move away from handle (error code: {res})")
        
        return res

    def movej_to_get_start_position_after_process(self):
        """
        Move robot to get tool start position after process
        """
        if self.robot is None:
            print("Robot is not initialized")
            return -1
        
        pose = [-0.7685130278216761, -1.5912000141539515, -0.061849094927310944, -0.962276355629303, 1.8501255512237549, -0.6623795668231409]
        print("Moving robot to mid-position1...")
        res = self.robot.movej(pose, a=0.5, v=0.5)
        time.sleep(0.5)

        pose = [-1.5214632193194788, -1.5912000141539515, -0.061849094927310944,  0.06347672521557612, 1.4398412704467773, -1.2330482641803187]
        print("Moving robot to mid-position2...")
        res = self.robot.movej(pose, a=0.5, v=0.5)
        time.sleep(0.5)
        
        if res == 0:
            print("Robot moved to start position after process successfully")
        else:
            print(f"Failed to move robot to start position after process (error code: {res})")
        
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
    
    # setup other cooperate robots

    # initialize platform height
    print("\n" + "="*50)
    ur_handle1.pushrod_to_base()
    time.sleep(3)

    ur_handle1.lift_platform_to_base()
    time.sleep(3)

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

    print("\n" + "="*50)
    ur_handle1.lift_platform_coarse_adjust(908)
    time.sleep(10)   

    print("\n" + "="*50)
    ur_handle1.pushrod_fine_adjust(926)
    time.sleep(5)

    # step1: extract 0.5m
    print("\n" + "="*50)
    ur_handle1.force_task_extract_server(0.60)
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle1.movel_to_leave_server([0,0,0.015])
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle1.lift_platform_coarse_adjust(937)
    time.sleep(5)   

    # step3: extract 0.6m
    print("\n" + "="*50)
    ur_handle1.force_task_extract_server(0.60)
    time.sleep(0.5)

    # move to exit position
    print("\n" + "="*50)
    ur_handle1.movel_to_leave_server([0,0,0.20])
    time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle1.movej_to_reference_joint_positions()
    time.sleep(0.5)

    # # print("\n" + "="*50)
    # ur_handle1.movej_to_get_start_position_after_process()
    # time.sleep(0.5)

    print("\n" + "="*50)
    ur_handle1.pushrod_to_base()
    time.sleep(8)

    print("\n" + "="*50)
    ur_handle1.lift_platform_to_base()
    time.sleep(8)
    
