#!/usr/bin/env python3
"""
Robot Move to Target Pose Script

This script implements functionality to move a robot arm to a specified target pose
within the robot's workspace. Given a target TCP pose, it calculates the required
joint angles using inverse kinematics and optionally moves the robot to that position.

Features:
1. Accept target pose input (position + orientation)
2. Use inverse kinematics (cal_ikine) to calculate joint angles
3. Validate the solution using forward kinematics
4. Provide detailed accuracy analysis
5. Optional robot movement to target position

Usage:
    python3 duco_move2target.py

Inputs:
- Target pose: [x, y, z, rx, ry, rz] where:
  - x, y, z: position in meters (robot base coordinate system)  
  - rx, ry, rz: orientation in degrees (ZYX intrinsic rpy angles)

Outputs:
- Calculated joint angles in degrees
- Accuracy analysis between target and achievable pose
- Optional robot movement
"""

import numpy as np
import os
import sys
import matplotlib.pyplot as plt
import time

# Add the duco_robot_arm directory and its lib subdirectory to the Python path
try:
    sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'colcon_ws', 'src'))
    from common.workspace_utils import get_workspace_root
    project_root = get_workspace_root()
except ImportError:
    script_dir = os.path.dirname(os.path.abspath(__file__))
    project_root = os.path.dirname(script_dir)
duco_robot_arm_dir = os.path.join(project_root, 'colcon_ws', 'src', 'duco_robot_arm', 'duco_robot_arm')
sys.path.insert(0, duco_robot_arm_dir)
sys.path.insert(0, os.path.join(duco_robot_arm_dir, 'lib'))

from DucoCobot import DucoCobot  
from gen_py.robot.ttypes import Op  
from thrift import Thrift  
   
# Robot connection parameters  
ip = '192.168.1.10'     # real robot
port = 7003   

def draw_coordinate_frame(ax, origin, rotation_matrix, label, size=0.1, alpha=0.7):
    """
    Draw a coordinate frame (X-Y-Z axes) at specified origin with given rotation.
    
    Coordinate axis color convention:
    - RED arrow: X-axis (forward/backward direction)
    - GREEN arrow: Y-axis (left/right direction)  
    - BLUE arrow: Z-axis (up/down direction)
    
    Parameters:
    - ax: matplotlib 3D axes
    - origin: [x, y, z] position of frame origin
    - rotation_matrix: 3x3 rotation matrix
    - label: string label for the frame
    - size: length of axes arrows
    - alpha: transparency of arrows
    """
    # Define unit vectors for X, Y, Z axes
    x_axis = np.array([1, 0, 0]) * size
    y_axis = np.array([0, 1, 0]) * size
    z_axis = np.array([0, 0, 1]) * size
    
    # Rotate axes according to rotation matrix
    x_rotated = rotation_matrix @ x_axis
    y_rotated = rotation_matrix @ y_axis
    z_rotated = rotation_matrix @ z_axis
    
    # Draw axes as arrows with thicker lines and better arrow heads
    ax.quiver(origin[0], origin[1], origin[2], 
              x_rotated[0], x_rotated[1], x_rotated[2], 
              color='red', alpha=alpha, arrow_length_ratio=0.15, linewidth=3,
              normalize=False)
    ax.quiver(origin[0], origin[1], origin[2], 
              y_rotated[0], y_rotated[1], y_rotated[2], 
              color='green', alpha=alpha, arrow_length_ratio=0.15, linewidth=3,
              normalize=False)
    ax.quiver(origin[0], origin[1], origin[2], 
              z_rotated[0], z_rotated[1], z_rotated[2], 
              color='blue', alpha=alpha, arrow_length_ratio=0.15, linewidth=3,
              normalize=False)
    
    # Add axis labels at the end of each arrow
    ax.text(origin[0] + x_rotated[0] * 1.2, origin[1] + x_rotated[1] * 1.2, origin[2] + x_rotated[2] * 1.2, 
            'X', color='red', fontsize=12, fontweight='bold')
    ax.text(origin[0] + y_rotated[0] * 1.2, origin[1] + y_rotated[1] * 1.2, origin[2] + y_rotated[2] * 1.2, 
            'Y', color='green', fontsize=12, fontweight='bold')
    ax.text(origin[0] + z_rotated[0] * 1.2, origin[1] + z_rotated[1] * 1.2, origin[2] + z_rotated[2] * 1.2, 
            'Z', color='blue', fontsize=12, fontweight='bold')
    
    # Add frame label with offset
    label_offset = np.array([0, 0, size * 1.5])
    ax.text(origin[0] + label_offset[0], origin[1] + label_offset[1], origin[2] + label_offset[2], 
            label, fontsize=10, fontweight='bold', ha='center')  


def get_target_pose():
    """
    Get target pose from user input or configuration.
    
    Returns:
        list: target pose [x, y, z, rx, ry, rz] where:
            - x, y, z: position in meters
            - rx, ry, rz: orientation in degrees (ZYX intrinsic rpy angles)
    """
    print("\n=== Target Pose Input ===")
    print("Enter target TCP pose in robot base coordinate system:")
    print("Position should be in meters, orientation in degrees")
    
    try:
        # Example target poses for quick testing (uncomment one)
        # target_pose = [0.3, 0.1, 0.4, 180, 0, 0]  # Simple forward position
        # target_pose = [0.2, 0.2, 0.3, 180, 0, 45]  # Angled position
        
        # Interactive input
        x = float(input("Enter X position (m): "))
        y = float(input("Enter Y position (m): "))
        z = float(input("Enter Z position (m): "))
        rx = float(input("Enter RX rotation (deg): "))
        ry = float(input("Enter RY rotation (deg): "))
        rz = float(input("Enter RZ rotation (deg): "))
        
        target_pose = [x, y, z, rx, ry, rz]
        
        print(f"\nTarget pose set to (m, deg): {target_pose}")
        
        return target_pose
        
    except ValueError as e:
        print(f"Invalid input: {e}")
        print("Using default target pose...")
        # Default pose - adjust as needed for your robot setup
        default_pose = [0.3, 0.0, 0.4, 0, 0, 0]
        print(f"Default target pose: {default_pose}")
        return default_pose
    except KeyboardInterrupt:
        print("\nInput cancelled by user")
        return None


def calculate_target_joint_angles(robot, target_pose):
    """
    Calculate target joint angles for the robot using inverse kinematics.

    Args:
        robot: DucoCobot instance
        target_pose: [x, y, z, rx, ry, rz] where position is in meters, 
                     orientation is in degrees (ZYX intrinsic rpy angles)

    Returns:
        list: joint angles in radians on success, or None if inverse kinematics fails.
    """
    if target_pose is None:
        return None
        
    print("\n=== Inverse Kinematics Calculation ===")
    
    # Convert target pose format for cal_ikine
    # Position: already in meters
    # Orientation: convert degrees to radians
    target_pos_m = target_pose[:3]  # [x, y, z] in meters
    target_rpy_deg = target_pose[3:6]  # [rx, ry, rz] in degrees
    target_rpy_rad = [float(np.radians(a)) for a in target_rpy_deg]
    
    # Format for cal_ikine: [x, y, z, rx, ry, rz] where xyz in meters, rxryrz in radians
    target_pose_for_ik = target_pos_m + target_rpy_rad
    
    print(f"Target TCP position (m): [{target_pos_m[0]:.6f}, {target_pos_m[1]:.6f}, {target_pos_m[2]:.6f}]")
    print(f"Target TCP orientation (deg): [{target_rpy_deg[0]:.2f}, {target_rpy_deg[1]:.2f}, {target_rpy_deg[2]:.2f}]")

    # Get current robot state for reference
    try:
        current_tcp_pose = robot.get_tcp_pose() 
        current_joint_angles = robot.get_actual_joints_position()
        
        print(f"Current TCP position (m): [{current_tcp_pose[0]:.6f}, {current_tcp_pose[1]:.6f}, {current_tcp_pose[2]:.6f}]")
        current_tcp_rpy_deg = [float(np.degrees(a)) for a in current_tcp_pose[3:6]]
        print(f"Current TCP orientation (deg): [{current_tcp_rpy_deg[0]:.2f}, {current_tcp_rpy_deg[1]:.2f}, {current_tcp_rpy_deg[2]:.2f}]")

    except Exception as e:
        print(f"Warning: Could not get current robot state: {e}")
    
    # Calculate inverse kinematics
    try:
        print("\nCalculating inverse kinematics...")
        target_joint_angles = robot.cal_ikine(target_pose_for_ik, '', '', '')  # Returns joint angles in radians
        
        if target_joint_angles is not None:
            target_joint_angles_deg = [float(np.degrees(a)) for a in target_joint_angles]
            print("✓ Target joint angles calculated successfully!")
            print(f"Current joint angles (deg): {[float(np.degrees(a)) for a in current_joint_angles]}")
            print(f"Target joint angles (deg): {[float(a) for a in target_joint_angles_deg]}")
            
            # Check if joint angles are within reasonable bounds (optional)
            joint_limits_deg = [(-170, 170), (-135, 135), (-135, 135), (-270, 270), (-135, 135), (-360, 360)]
            within_limits = True
            for i, (angle_deg, (min_limit, max_limit)) in enumerate(zip(target_joint_angles_deg, joint_limits_deg)):
                if not (min_limit <= angle_deg <= max_limit):
                    print(f"⚠ Warning: Joint {i+1} angle {angle_deg:.1f}° is outside typical limits [{min_limit}, {max_limit}]°")
                    within_limits = False
            
            if within_limits:
                print("✓ All joint angles are within typical joint limits")
            else:
                print("⚠ Some joint angles are outside typical limits - please verify target pose")
            
            return target_joint_angles
        else:
            print("✗ Inverse kinematics returned None - no solution found")
            return None
    
    except Exception as e:
        print(f"✗ Inverse kinematics failed: {e}")
        return None


def validate_solution_with_forward_kinematics(robot, target_pose, target_joint_angles):
    """
    Validate the inverse kinematics solution using forward kinematics.
    Compare the target pose with the actual achievable pose.
    
    Args:
        robot: DucoCobot instance
        target_pose: Original target pose [x, y, z, rx, ry, rz] (meters, degrees)
        target_joint_angles: Calculated joint angles in radians
        
    Returns:
        dict: validation results with position and orientation errors
    """
    if target_joint_angles is None:
        return None
        
    print("\n=== Forward Kinematics Validation ===")
    
    try:
        # Use forward kinematics to calculate actual achievable TCP pose
        actual_tcp_pose = robot.cal_fkine(target_joint_angles, '', '')  # Returns [x, y, z, rx, ry, rz]
        
        # Extract position and orientation
        actual_pos_m = actual_tcp_pose[:3]  # [x, y, z] in meters
        actual_rpy_rad = actual_tcp_pose[3:6]  # [rx, ry, rz] in radians
        actual_rpy_deg = [float(np.degrees(a)) for a in actual_rpy_rad]
        
        # Target values for comparison
        target_pos_m = target_pose[:3]
        target_rpy_deg = target_pose[3:6]
        
        print(f"Target position (m): [{target_pos_m[0]:.6f}, {target_pos_m[1]:.6f}, {target_pos_m[2]:.6f}]")
        print(f"Actual position (m): [{actual_pos_m[0]:.6f}, {actual_pos_m[1]:.6f}, {actual_pos_m[2]:.6f}]")
        print(f"Target orientation (deg): [{target_rpy_deg[0]:.2f}, {target_rpy_deg[1]:.2f}, {target_rpy_deg[2]:.2f}]")
        print(f"Actual orientation (deg): [{actual_rpy_deg[0]:.2f}, {actual_rpy_deg[1]:.2f}, {actual_rpy_deg[2]:.2f}]")

        # Calculate position deviation
        pos_deviation = np.array(actual_pos_m) - np.array(target_pos_m)
        pos_deviation_norm = np.linalg.norm(pos_deviation)
        
        print(f"\nPosition deviation (m): [{pos_deviation[0]:.6f}, {pos_deviation[1]:.6f}, {pos_deviation[2]:.6f}]")
        
        # Calculate orientation deviation  
        rpy_deviation = np.array(actual_rpy_deg) - np.array(target_rpy_deg)
        
        # Handle angle wrapping for orientation comparison
        for i in range(len(rpy_deviation)):
            while rpy_deviation[i] > 180:
                rpy_deviation[i] -= 360
            while rpy_deviation[i] < -180:
                rpy_deviation[i] += 360
        
        # max_angle_deviation = np.max(np.abs(rpy_deviation))
        
        print(f"Orientation deviation (deg): [{rpy_deviation[0]:.4f}, {rpy_deviation[1]:.4f}, {rpy_deviation[2]:.4f}]")
            
        validation_result = {
            'actual_pose': actual_tcp_pose,
            'position_error': pos_deviation_norm,
            'orientation_error': rpy_deviation,
        }
        
        return validation_result
        
    except Exception as e:
        print(f"✗ Forward kinematics validation failed: {e}")
        return None


def visualize_target_poses(target_pose, actual_tcp_info, current_tcp_pose):
    """
    Visualize the target poses and current TCP position in 3D coordinate system.
    
    Parameters:
    - target_pose: [x, y, z, rx, ry, rz] target pose (meters, degrees)
    - actual_tcp_info: tuple (position, orientation_deg, rotation_matrix) from forward kinematics
    - current_tcp_pose: [x, y, z, rx, ry, rz] current TCP pose from robot
    """
    print("\n=== 3D Visualization ===")
    
    # Create 3D plot
    fig = plt.figure(figsize=(12, 10))
    ax = fig.add_subplot(111, projection='3d')
    
    # Extract positions and orientations
    target_pos = np.array(target_pose[:3])  # [x, y, z]
    target_rpy_deg = target_pose[3:6]  # [rx, ry, rz] in degrees
    
    # Convert target orientation to rotation matrix
    rx, ry, rz = [np.radians(a) for a in target_rpy_deg]
    cos_rx, sin_rx = np.cos(rx), np.sin(rx)
    cos_ry, sin_ry = np.cos(ry), np.sin(ry)
    cos_rz, sin_rz = np.cos(rz), np.sin(rz)
    
    target_rot_matrix = np.array([
        [cos_rz*cos_ry, cos_rz*sin_ry*sin_rx - sin_rz*cos_rx, cos_rz*sin_ry*cos_rx + sin_rz*sin_rx],
        [sin_rz*cos_ry, sin_rz*sin_ry*sin_rx + cos_rz*cos_rx, sin_rz*sin_ry*cos_rx - cos_rz*sin_rx],
        [-sin_ry, cos_ry*sin_rx, cos_ry*cos_rx]
    ])
    
    all_points = [np.array([0, 0, 0]), target_pos]  # Base origin and target position
    
    # Draw base coordinate system (origin at robot base)
    base_origin = np.array([0.0, 0.0, 0.0])
    base_rotation = np.eye(3)  # Identity matrix for base frame
    draw_coordinate_frame(ax, base_origin, base_rotation, 'Base Frame', size=0.08, alpha=0.8)
    
    # Draw ideal target frame
    draw_coordinate_frame(ax, target_pos, target_rot_matrix, 'Ideal Target', size=0.08, alpha=0.8)
    
    # Draw actual TCP target frame if available
    if actual_tcp_info is not None:
        actual_tcp_pos, actual_tcp_rpy_deg, actual_tcp_rot = actual_tcp_info
        actual_tcp_pos_array = np.array(actual_tcp_pos)
        all_points.append(actual_tcp_pos_array)
        
        # Add small offset to make it visible when overlapping with ideal frame
        offset = np.array([0.01, 0.01, 0.01])  # 1cm offset for visibility
        draw_coordinate_frame(ax, actual_tcp_pos_array + offset, actual_tcp_rot, 
                             'Actual Target', size=0.06, alpha=0.9)
        
        # Draw line connecting ideal and actual positions to show difference
        ax.plot([target_pos[0], actual_tcp_pos[0]], 
               [target_pos[1], actual_tcp_pos[1]], 
               [target_pos[2], actual_tcp_pos[2]], 
               'r--', alpha=0.7, linewidth=2, label='Ideal↔Actual Deviation')
    
    # Draw current TCP frame if available
    if current_tcp_pose is not None:
        current_tcp_pos = np.array(current_tcp_pose[:3])
        current_tcp_rxryrz = current_tcp_pose[3:6]  # rx, ry, rz in radians
        all_points.append(current_tcp_pos)
        
        # Convert current TCP orientation to rotation matrix
        rx, ry, rz = current_tcp_rxryrz
        cos_rx, sin_rx = np.cos(rx), np.sin(rx)
        cos_ry, sin_ry = np.cos(ry), np.sin(ry)
        cos_rz, sin_rz = np.cos(rz), np.sin(rz)
        
        current_tcp_rot = np.array([
            [cos_rz*cos_ry, cos_rz*sin_ry*sin_rx - sin_rz*cos_rx, cos_rz*sin_ry*cos_rx + sin_rz*sin_rx],
            [sin_rz*cos_ry, sin_rz*sin_ry*sin_rx + cos_rz*cos_rx, sin_rz*sin_ry*cos_rx - cos_rz*sin_rx],
            [-sin_ry, cos_ry*sin_rx, cos_ry*cos_rx]
        ])
        
        # Draw current TCP frame
        draw_coordinate_frame(ax, current_tcp_pos, current_tcp_rot, 
                             'Current TCP', size=0.08, alpha=0.8)
    
    # Calculate workspace bounds
    all_coords = np.array(all_points)
    x_min, x_max = all_coords[:, 0].min(), all_coords[:, 0].max()
    y_min, y_max = all_coords[:, 1].min(), all_coords[:, 1].max()
    z_min, z_max = all_coords[:, 2].min(), all_coords[:, 2].max()
    
    padding = 0.1
    x_range = max(x_max - x_min + 2*padding, 0.2)
    y_range = max(y_max - y_min + 2*padding, 0.2)
    z_range = max(z_max - z_min + 2*padding, 0.2)
    
    # Create custom legend for coordinate axes colors
    import matplotlib.patches as mpatches
    red_patch = mpatches.Patch(color='red', label='X-axis')
    green_patch = mpatches.Patch(color='green', label='Y-axis')
    blue_patch = mpatches.Patch(color='blue', label='Z-axis')
    
    # Create legend with coordinate axis colors
    ax.legend(handles=[red_patch, green_patch, blue_patch], 
             title='Coordinate Axes', loc='upper right')
    
    # Set labels and title
    ax.set_xlabel('X (m)', fontsize=12, fontweight='bold')
    ax.set_ylabel('Y (m)', fontsize=12, fontweight='bold')
    ax.set_zlabel('Z (m)', fontsize=12, fontweight='bold')
    ax.set_title('Robot Target Poses Visualization', fontsize=14, fontweight='bold')
    
    # Set bounds based on actual data
    center_x = (x_min + x_max) / 2
    center_y = (y_min + y_max) / 2
    center_z = (z_min + z_max) / 2
    
    max_range = max(x_range, y_range, z_range) / 2
    ax.set_xlim([center_x - max_range, center_x + max_range])
    ax.set_ylim([center_y - max_range, center_y + max_range])
    ax.set_zlim([max(0, center_z - max_range), center_z + max_range])
    
    # Add grid with better styling
    ax.grid(True, alpha=0.3, linestyle='--')
    ax.xaxis.pane.fill = False
    ax.yaxis.pane.fill = False
    ax.zaxis.pane.fill = False
    
    # Make pane edges more visible
    ax.xaxis.pane.set_edgecolor('gray')
    ax.yaxis.pane.set_edgecolor('gray')
    ax.zaxis.pane.set_edgecolor('gray')
    ax.xaxis.pane.set_alpha(0.1)
    ax.yaxis.pane.set_alpha(0.1)
    ax.zaxis.pane.set_alpha(0.1)
    
    # Set better viewing angle
    ax.view_init(elev=20, azim=45)
    
    plt.tight_layout()
    
    # Save figure to file
    try:
        from common.workspace_utils import get_workspace_root
        project_root = get_workspace_root()
    except:
        script_dir = os.path.dirname(os.path.abspath(__file__))
        project_root = os.path.dirname(script_dir)
    output_dir = os.path.join(project_root, 'temp', 'test')
    os.makedirs(output_dir, exist_ok=True)
    
    output_path = os.path.join(output_dir, 'auto_move2target_result.jpg')
    plt.savefig(output_path, dpi=300, bbox_inches='tight', format='jpg')
    plt.close()  # Close the figure to free memory

    print(f"Target poses visualization saved to: {output_path}")
    
    return output_path


def main():
    """
    Main function that orchestrates the complete process:
    1. Get target pose input
    2. Connect to robot
    3. Calculate joint angles using inverse kinematics  
    4. Validate solution using forward kinematics
    5. Create 3D visualization of all poses
    """
    
    # Step 1: Get target pose
    target_pose = get_target_pose()
    if target_pose is None:
        print("No target pose provided. Exiting.")
        return
    
    # Step 2: Connect to robot
    print("\n=== Robot Connection ===")
    robot = DucoCobot(ip, port)  
    res = robot.open()  
    print("Open connection:", res)  

    op = Op()
    op.time_or_dist_1 = 0
    op.trig_io_1 = 1
    op.trig_value_1 = False
    op.trig_time_1 = 0.0
    op.trig_dist_1 = 0.0
    op.trig_event_1 = ""
    op.time_or_dist_2 = 0
    op.trig_io_2 = 1
    op.trig_value_2 = False
    op.trig_time_2 = 0.0
    op.trig_dist_2 = 0.0
    op.trig_event_2 = ""
        
    try:
        # Step 3: Calculate joint angles
        target_joint_angles = calculate_target_joint_angles(robot, target_pose)
        
        if target_joint_angles is None:
            print("\n✗ Failed to calculate joint angles. Target pose may be unreachable.")
            return          
        
        # Step 4: Validate solution using forward kinematics
        validation_result = validate_solution_with_forward_kinematics(robot, target_pose, target_joint_angles)
        
        # Step 5: Get current TCP pose for visualization
        current_tcp_pose = None
        try:
            current_tcp_pose = robot.get_tcp_pose()
        except Exception as e:
            print(f"Warning: Could not get current TCP pose: {e}")
        
        # Step 6: Create 3D visualization
        actual_tcp_info = None
        if validation_result is not None:
            # Get actual TCP info from validation result
            actual_pose = validation_result['actual_pose']
            actual_pos = actual_pose[:3]
            actual_rpy_rad = actual_pose[3:6]
            actual_rpy_deg = [float(np.degrees(a)) for a in actual_rpy_rad]
            
            # Convert to rotation matrix
            rx, ry, rz = actual_rpy_rad
            cos_rx, sin_rx = np.cos(rx), np.sin(rx)
            cos_ry, sin_ry = np.cos(ry), np.sin(ry)
            cos_rz, sin_rz = np.cos(rz), np.sin(rz)
            
            actual_rot_matrix = np.array([
                [cos_rz*cos_ry, cos_rz*sin_ry*sin_rx - sin_rz*cos_rx, cos_rz*sin_ry*cos_rx + sin_rz*sin_rx],
                [sin_rz*cos_ry, sin_rz*sin_ry*sin_rx + cos_rz*cos_rx, sin_rz*sin_ry*cos_rx - cos_rz*sin_rx],
                [-sin_ry, cos_ry*sin_rx, cos_ry*cos_rx]
            ])
            
            actual_tcp_info = (actual_pos, actual_rpy_deg, actual_rot_matrix)
        
        # Generate visualization
        visualize_target_poses(target_pose, actual_tcp_info, current_tcp_pose)
        
        # move to target position
        res = robot.movej2(target_joint_angles, 1.0, 1.0, 0.0, True, op)
        print("Move command result:", res)
        time.sleep(0.5)
        
        print("\nTask completed successfully!")
        
    except Exception as e:
        print(f"\nUnexpected error in main process: {e}")
        
    finally:
        # Close robot connection
        try:
            res = robot.close()  
            print("Close connection:", res)
        except Exception as e:
            print(f"Warning: Error closing robot connection: {e}")


if __name__ == "__main__":
    main()
