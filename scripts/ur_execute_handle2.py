import os
import time
from ur_execute_base import URExecuteBase


class URExecuteHandle2(URExecuteBase):
    def __init__(self, robot_ip="192.168.1.15", robot_port=30002, rs485_port=54321):
        # Store connection parameters
        self.robot_ip = robot_ip
        self.robot_port = robot_port
        self.rs485_port = rs485_port
        
        # Initialize robot and RS485 connection
        self.robot = None
        self.rs485_socket = None
        
        # Define the path to camera parameters (same as parent)
        self.camera_params_dir = os.path.join(
            os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
            "temp",
            "ur15_cam_calibration_result",
            "ur15_camera_parameters"
        )
        
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
        
        # Initialize parameter storage
        self.camera_matrix = None
        self.distortion_coefficients = None
        self.cam2end_matrix = None
        self.local_transformation_matrix = None
        self.local_origin = None
        self.ref_joint_angles = None
        self.estimated_keypoints = None
        self.task_position_offset = None
        
        # Initialize the robot connection
        self._initialize_robot()
        
        # Initialize RS485 socket connection
        self._init_rs485_socket()
        
        # Automatically load all parameters
        self._load_camera_intrinsic()
        self._load_camera_extrinsic()
        self._load_estimated_kp_coordinates()
        self._load_local_coordinate_system()
        self._load_ref_joint_angles()
        self._load_task_position_information()


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
    
    # print("\n" + "="*70)
    # print("Moving robot to reference joint positions...")
    # ur_handle2.movej_to_reference_joint_positions()
    # time.sleep(10)
