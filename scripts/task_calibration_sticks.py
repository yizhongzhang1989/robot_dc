#!/usr/bin/env python3
"""
Stick Tip Calibration using Least Squares Method
-------------------------------------------------
Calibrates the 3D position of a stick tip relative to the robot end-effector
using the constraint that the tip touches a fixed pyramid apex point.

Core Method:
For each pose i, the constraint is:
    R_i @ t_x + c_i = p_apex_base
where:
    R_i: rotation matrix of end-effector in base frame (pose i)
    c_i: translation of end-effector in base frame (pose i)
    t_x: stick tip position in end-effector frame (unknown)
    p_apex_base: pyramid apex position in base frame (constant)

This forms a linear system: A @ t_x = b
where A stacks all R_i and b stacks all (p_apex_base - c_i)

Usage:
    python3 stick_calibration_lstsq.py \
        --data_dir temp/stick_calibration_data \
        --pyr_config temp/stick_calibration_data/pyramid_parameters.json \
        --camera_intrinsic temp/camera_parameters/calibration_result.json \
        --camera_extrinsic temp/camera_parameters/eye_in_hand_result.json \
        --output temp/stick_calibration_result/stick_lstsq_result.json
"""

import argparse
import json
import os
import sys
from typing import List, Tuple, Dict, Any
import glob

import cv2
import numpy as np

# Add ThirdParty modules to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'ThirdParty/camera_calibration_toolkit'))
from core.calibration_patterns.standard_chessboard import StandardChessboard


def load_json(path: str) -> Dict[str, Any]:
    """Load JSON file and return dict."""
    with open(path, 'r') as f:
        return json.load(f)


def load_camera_intrinsics(path: str) -> Tuple[np.ndarray, np.ndarray]:
    """Load camera matrix K and distortion coefficients from calibration result.
    
    Returns:
        K: 3x3 camera matrix
        dist: distortion coefficients (5,)
    """
    data = load_json(path)
    K = np.array(data['camera_matrix'], dtype=np.float64)
    dist = np.array(data['distortion_coefficients'], dtype=np.float64)
    return K, dist


def load_cam2base_transform(path: str) -> np.ndarray:
    """Load camera to base transformation matrix.
    
    For eye-in-hand: cam2base = end2base @ cam2end
    
    Returns:
        T_base_cam: 4x4 transformation matrix
    """
    data = load_json(path)
    
    # Check if we have target2base (chessboard to base from eye-in-hand calibration)
    # This is actually the base2target, we need to invert it
    if 'target2base_matrix' in data:
        T_target2base = np.array(data['target2base_matrix'], dtype=np.float64)
        # For our purpose, we can use this as a reference frame
        # But we need cam2base, which we compute from cam2end and end2base
        
    # Get cam2end transformation
    if 'cam2end_matrix' in data:
        T_cam2end = np.array(data['cam2end_matrix'], dtype=np.float64)
    else:
        raise ValueError("cam2end_matrix not found in extrinsic calibration file")
    
    return T_cam2end


def load_pyramid_config(path: str) -> Dict[str, Any]:
    """Load pyramid configuration from JSON."""
    data = load_json(path)
    pyr = data['pyramid']
    size = pyr['size']
    mapping = pyr['chessboard_mapping']
    
    return {
        'base_length': float(size['base_length']),
        'height': float(size['height']),
        'col_min': int(mapping['pyr_col_min']),
        'col_max': int(mapping['pyr_col_max']),
        'row_min': int(mapping['pyr_row_min']),
        'row_max': int(mapping['pyr_row_max'])
    }


def build_chessboard_object_points(cols: int, rows: int, square_size: float) -> np.ndarray:
    """Generate 3D object points for chessboard corners.
    
    Args:
        cols: number of inner corners in column direction
        rows: number of inner corners in row direction
        square_size: size of each square in meters
        
    Returns:
        objp: (cols*rows, 3) array of 3D points in chessboard frame
    """
    objp = np.zeros((cols * rows, 3), np.float32)
    objp[:, :2] = np.mgrid[0:cols, 0:rows].T.reshape(-1, 2)
    objp *= square_size
    return objp


def compute_pyramid_apex_in_board_frame(pyr_config: Dict[str, Any], square_size: float) -> np.ndarray:
    """Compute pyramid apex position in chessboard coordinate frame.
    
    The apex is at the geometric center of the rectangle defined by the
    chessboard corner indices, elevated by pyramid height.
    
    Args:
        pyr_config: pyramid configuration dict
        square_size: chessboard square size in meters
        
    Returns:
        apex_board: (3,) array of apex position in board frame
    """
    col_min = pyr_config['col_min']
    col_max = pyr_config['col_max']
    row_min = pyr_config['row_min']
    row_max = pyr_config['row_max']
    height = pyr_config['height']
    
    # Center position in board indices
    cx = (col_min + col_max) / 2.0
    cy = (row_min + row_max) / 2.0
    
    # Convert to metric coordinates
    x = cx * square_size
    y = cy * square_size
    z = height
    
    return np.array([x, y, z], dtype=np.float64)


def detect_chessboard_pose(image: np.ndarray, 
                           objp: np.ndarray,
                           pattern_size: Tuple[int, int],
                           K: np.ndarray,
                           dist: np.ndarray) -> Tuple[bool, np.ndarray, np.ndarray]:
    """Detect chessboard and compute its pose.
    
    Args:
        image: input image
        objp: 3D object points
        pattern_size: (cols, rows) chessboard inner corners
        K: camera matrix
        dist: distortion coefficients
        
    Returns:
        success: whether detection succeeded
        R_cam_board: 3x3 rotation matrix from board to camera
        t_cam_board: (3,) translation from board to camera
    """
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    
    found, corners = cv2.findChessboardCorners(
        gray, pattern_size,
        flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE
    )
    
    if not found:
        return False, None, None
    
    # Refine corner positions
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
    
    # Solve PnP
    success, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
    
    if not success:
        return False, None, None
    
    R_cam_board, _ = cv2.Rodrigues(rvec)
    t_cam_board = tvec.reshape(3)
    
    return True, R_cam_board, t_cam_board


def load_robot_pose(json_path: str) -> np.ndarray:
    """Load robot end-effector pose from JSON file.
    
    Args:
        json_path: path to pose JSON file
        
    Returns:
        T_base_end: 4x4 transformation matrix from end-effector to base
    """
    data = load_json(json_path)
    T_base_end = np.array(data['end2base'], dtype=np.float64)
    return T_base_end


def solve_stick_tip_lstsq(apex_positions_base: List[np.ndarray],
                          end_poses_base: List[np.ndarray]) -> Tuple[np.ndarray, Dict[str, Any]]:
    """Solve for stick tip position using least squares.
    
    For each measurement i:
        R_base_end_i @ t_stick + t_base_end_i = p_apex_base_i
    
    Rearranging:
        R_base_end_i @ t_stick = p_apex_base_i - t_base_end_i
    
    Stack all equations:
        A @ t_stick = b
    where:
        A = [R_1; R_2; ...; R_n]  (3n x 3)
        b = [p_1 - c_1; p_2 - c_2; ...; p_n - c_n]  (3n,)
    
    Args:
        apex_positions_base: list of apex positions in base frame
        end_poses_base: list of end-effector poses (4x4) in base frame
        
    Returns:
        t_stick: (3,) stick tip position in end-effector frame
        stats: dict with error statistics
    """
    n = len(apex_positions_base)
    
    # Build linear system
    A_blocks = []
    b_blocks = []
    
    for p_apex, T_end in zip(apex_positions_base, end_poses_base):
        R_i = T_end[:3, :3]
        c_i = T_end[:3, 3]
        
        A_blocks.append(R_i)
        b_blocks.append(p_apex - c_i)
    
    A = np.vstack(A_blocks)  # (3n, 3)
    b = np.hstack(b_blocks)  # (3n,)
    
    # Solve least squares
    t_stick, residuals, rank, s = np.linalg.lstsq(A, b, rcond=None)
    
    # Compute individual errors
    errors = []
    for p_apex, T_end in zip(apex_positions_base, end_poses_base):
        pred = T_end[:3, :3] @ t_stick + T_end[:3, 3]
        error = np.linalg.norm(pred - p_apex)
        errors.append(error)
    
    errors = np.array(errors)
    
    stats = {
        'mean_error': float(np.mean(errors)),
        'std_error': float(np.std(errors)),
        'max_error': float(np.max(errors)),
        'min_error': float(np.min(errors)),
        'rmse': float(np.sqrt(np.mean(errors**2))),
        'individual_errors': errors.tolist(),
        'lstsq_residuals': float(residuals[0]) if len(residuals) > 0 else None,
        'matrix_rank': int(rank),
        'singular_values': s.tolist()
    }
    
    return t_stick, stats


def visualize_calibration_results(K: np.ndarray,
                                  dist: np.ndarray,
                                  T_cam2end: np.ndarray,
                                  t_stick: np.ndarray,
                                  stats: Dict[str, Any],
                                  output_dir: str,
                                  image_files: List[str],
                                  objp: np.ndarray,
                                  pattern_size: Tuple[int, int],
                                  apex_board: np.ndarray,
                                  valid_indices: List[int],
                                  end_poses_base: List[np.ndarray],
                                  apex_positions_base: List[np.ndarray],
                                  pyr_config: Dict[str, Any]):
    """Visualize calibration results for each image with 3 separate validation images:
    
    For each image, output 3 validation results:
    1. Intrinsic validation: chessboard corner reprojection
    2. Extrinsic validation: coordinate frame visualization
    3. Stick calibration validation: apex vs predicted stick tip
    """
    
    os.makedirs(output_dir, exist_ok=True)
    
    errors_mm = np.array(stats['individual_errors']) * 1000
    
    # Process each image separately and output 3 validation images
    for idx, (i, img_path) in enumerate(zip(valid_indices, image_files)):
        # Load image
        image_orig = cv2.imread(img_path)
        if image_orig is None:
            print(f"      Warning: Failed to load image {img_path}")
            continue
            
        # Detect chessboard
        gray = cv2.cvtColor(image_orig, cv2.COLOR_BGR2GRAY)
        found, corners = cv2.findChessboardCorners(gray, pattern_size, 
                                                     flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
        
        if not found:
            print(f"      Warning: Chessboard not detected in image {i}")
            continue
        
        criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
        cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        
        # Solve PnP to get board pose
        success, rvec, tvec = cv2.solvePnP(objp, corners, K, dist, flags=cv2.SOLVEPNP_ITERATIVE)
        
        if not success:
            print(f"      Warning: PnP failed for image {i}")
            continue
        
        # Get robot pose
        T_base_end = end_poses_base[idx]
        
        # ===== IMAGE 1: Intrinsic Validation (Chessboard Corner Reprojection) =====
        image1 = image_orig.copy()
        
        # Reproject all corners
        corners_reproj, _ = cv2.projectPoints(objp, rvec, tvec, K, dist)
        
        # Draw detected corners (green)
        for corner in corners:
            pt = tuple(corner[0].astype(int))
            cv2.circle(image1, pt, 6, (0, 255, 0), -1)
        
        # Draw reprojected corners (red)
        for corner in corners_reproj:
            pt = tuple(corner[0].astype(int))
            cv2.circle(image1, pt, 4, (0, 0, 255), 2)
        
        # Calculate reprojection error
        reproj_error = np.sqrt(np.mean(np.sum((corners - corners_reproj)**2, axis=2)))
        
        # Add title and info
        cv2.putText(image1, 'Intrinsic Validation', (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        cv2.putText(image1, 'Intrinsic Validation', (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 128, 0), 2)
        cv2.putText(image1, f'Reproj Error: {reproj_error:.3f} px', (20, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
        cv2.putText(image1, 'Green: Detected | Red: Reprojected', (20, image1.shape[0]-20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        output1 = os.path.join(output_dir, f'intrinsic_{i:03d}.png')
        cv2.imwrite(output1, image1)
        
        # ===== IMAGE 2: Extrinsic Validation (Corner Reprojection using cam2end transform) =====
        image2 = image_orig.copy()
        
        # Validation chain for cam2end extrinsic calibration:
        # board -> cam (PnP) -> end (cam2end) -> base (robot pose) -> end -> cam -> image
        # This validates the cam2end transformation
        
        R_cam_board, _ = cv2.Rodrigues(rvec)
        t_cam_board = tvec.reshape(3)
        
        # Build transformation matrices
        T_cam_board = np.eye(4)
        T_cam_board[:3, :3] = R_cam_board
        T_cam_board[:3, 3] = t_cam_board
        
        T_base_end = end_poses_base[idx]
        T_end_base = np.linalg.inv(T_base_end)
        T_end_cam = np.linalg.inv(T_cam2end)
        
        # Reproject corners using cam2end extrinsic
        # Validation path: board -> cam (PnP) -> end (cam2end) -> base (robot) -> end -> cam -> image
        corners_via_extrinsic = []
        for obj_pt in objp:
            # Point in board frame (homogeneous)
            pt_board = np.array([obj_pt[0], obj_pt[1], obj_pt[2], 1.0])
            
            # Transform chain:
            # 1. board -> camera (using PnP result)
            pt_cam_direct = T_cam_board @ pt_board
            
            # 2. camera -> end (using calibrated cam2end)
            pt_end = T_cam2end @ pt_cam_direct
            
            # 3. end -> base (using robot pose)
            pt_base = T_base_end @ pt_end
            
            # 4. base -> end (inverse of robot pose)
            pt_end_back = T_end_base @ pt_base
            
            # 5. end -> camera (using calibrated cam2end inverse)
            pt_cam_back = T_end_cam @ pt_end_back
            
            # Project to image
            pt_3d = pt_cam_back[:3].reshape(1, 3)
            pt_2d, _ = cv2.projectPoints(pt_3d, np.zeros(3), np.zeros(3), K, dist)
            corners_via_extrinsic.append(pt_2d[0, 0])
        
        corners_via_extrinsic = np.array(corners_via_extrinsic)
        
        # Draw detected corners (green)
        for corner in corners:
            pt = tuple(corner[0].astype(int))
            cv2.circle(image2, pt, 6, (0, 255, 0), -1)
        
        # Draw reprojected corners via extrinsic (red)
        for corner_2d in corners_via_extrinsic:
            pt = tuple(corner_2d.astype(int))
            cv2.circle(image2, pt, 4, (0, 0, 255), 2)
        
        # Calculate reprojection error
        extrinsic_reproj_error = np.sqrt(np.mean(np.sum((corners.reshape(-1, 2) - corners_via_extrinsic)**2, axis=1)))
        
        # Add title and info
        cv2.putText(image2, 'Extrinsic Validation', (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        cv2.putText(image2, 'Extrinsic Validation', (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 128, 0), 2)
        cv2.putText(image2, f'Reproj Error: {extrinsic_reproj_error:.3f} px', (20, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
        cv2.putText(image2, 'Green: Detected | Red: Via Extrinsic', (20, image2.shape[0]-20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        output2 = os.path.join(output_dir, f'extrinsic_{i:03d}.png')
        cv2.imwrite(output2, image2)
        
        # ===== IMAGE 3: Stick Calibration Validation =====
        image3 = image_orig.copy()
        
        # Draw chessboard corners
        cv2.drawChessboardCorners(image3, pattern_size, corners, found)
        
        # Compute transformation matrices needed for both apex and stick tip projection
        T_end_cam = np.linalg.inv(T_cam2end)
        T_base_cam = T_base_end @ T_end_cam
        T_cam_base = np.linalg.inv(T_base_cam)
        
        # Project pyramid apex directly from chessboard frame (observed position)
        # This shows where the apex actually appears in the image based on chessboard detection
        apex_3d = apex_board.reshape(1, 3)
        apex_observed_2d, _ = cv2.projectPoints(apex_3d, rvec, tvec, K, dist)
        apex_observed_2d = apex_observed_2d.reshape(2)
        
        # Draw observed apex as red circle (ground truth from chessboard)
        cv2.circle(image3, tuple(apex_observed_2d.astype(int)), 12, (0, 0, 255), -1)
        cv2.circle(image3, tuple(apex_observed_2d.astype(int)), 18, (255, 255, 255), 3)
        cv2.putText(image3, 'Apex (observed)', tuple((apex_observed_2d + [0, -25]).astype(int)), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Also draw apex from base frame for comparison (accumulates all calibration errors)
        apex_base_homo = np.append(apex_positions_base[idx], 1.0)
        apex_cam_homo = T_cam_base @ apex_base_homo
        apex_cam = apex_cam_homo[:3]
        apex_2d_norm = apex_cam[:2] / apex_cam[2]
        r2 = np.sum(apex_2d_norm**2)
        apex_2d_dist = apex_2d_norm * (1 + dist[0]*r2 + dist[1]*r2**2)
        apex_via_base = K[:2, :2] @ apex_2d_dist + K[:2, 2]
        
        # Draw apex via base frame as cyan circle (for comparison)
        cv2.circle(image3, tuple(apex_via_base.astype(int)), 8, (255, 255, 0), 2)
        cv2.putText(image3, 'Apex (via base)', tuple((apex_via_base + [0, 25]).astype(int)), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 0), 2)
        
        # Project calibrated stick tip
        
        stick_in_end = np.array([t_stick[0], t_stick[1], t_stick[2], 1.0])
        stick_in_base = T_base_end @ stick_in_end
        stick_in_cam = T_cam_base @ stick_in_base
        
        stick_3d = stick_in_cam[:3].reshape(1, 3)
        rvec_identity = np.zeros(3)
        tvec_identity = np.zeros(3)
        stick_2d, _ = cv2.projectPoints(stick_3d, rvec_identity, tvec_identity, K, dist)
        stick_2d = stick_2d.reshape(2)
        
        # Draw stick tip as green cross
        cross_size = 20
        pt = tuple(stick_2d.astype(int))
        cv2.line(image3, (pt[0]-cross_size, pt[1]), (pt[0]+cross_size, pt[1]), (0, 255, 0), 4)
        cv2.line(image3, (pt[0], pt[1]-cross_size), (pt[0], pt[1]+cross_size), (0, 255, 0), 4)
        cv2.circle(image3, pt, 5, (0, 255, 0), -1)
        cv2.putText(image3, 'Predicted', tuple((stick_2d + [0, 30]).astype(int)), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
        
        # Draw line connecting observed apex and predicted stick tip
        cv2.line(image3, tuple(apex_observed_2d.astype(int)), tuple(stick_2d.astype(int)), 
                (255, 255, 0), 2, cv2.LINE_AA)
        
        # Calculate errors
        reproj_error_px = np.linalg.norm(apex_observed_2d - stick_2d)
        apex_deviation_px = np.linalg.norm(apex_observed_2d - apex_via_base)
        
        # Add title and info
        cv2.putText(image3, 'Stick Calibration Validation', (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        cv2.putText(image3, 'Stick Calibration Validation', (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 128, 0), 2)
        cv2.putText(image3, f'3D Error: {errors_mm[idx]:.2f} mm', (20, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 255), 2)
        cv2.putText(image3, f'2D Reproj: {reproj_error_px:.1f} px', (20, 135), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 0, 255), 2)
        cv2.putText(image3, f'Apex deviation: {apex_deviation_px:.1f} px', (20, 180), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.9, (255, 255, 0), 2)
        
        output3 = os.path.join(output_dir, f'stick_{i:03d}.png')
        cv2.imwrite(output3, image3)
        
        # ===== IMAGE 4: Corner Detection Order =====
        image4 = image_orig.copy()
        
        # Draw all corners with index numbers
        for corner_idx, corner in enumerate(corners):
            pt = tuple(corner[0].astype(int))
            
            # Draw circle for corner
            cv2.circle(image4, pt, 5, (0, 255, 0), -1)
            
            # Calculate row and col from linear index
            row = corner_idx // pattern_size[0]
            col = corner_idx % pattern_size[0]
            
            # Draw index number
            label = f"{corner_idx}"
            cv2.putText(image4, label, (pt[0]+8, pt[1]-8), 
                       cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1)
        
        # Highlight first corner (red), last corner (blue)
        first_pt = tuple(corners[0][0].astype(int))
        last_pt = tuple(corners[-1][0].astype(int))
        cv2.circle(image4, first_pt, 10, (0, 0, 255), 2)
        cv2.circle(image4, last_pt, 10, (255, 0, 0), 2)
        
        # Mark the pyramid corners
        pyr_col_min = pyr_config['col_min']
        pyr_col_max = pyr_config['col_max']
        pyr_row_min = pyr_config['row_min']
        pyr_row_max = pyr_config['row_max']
        pyr_height = pyr_config['height']
        
        # Calculate pyramid corner indices
        pyramid_corners = [
            pyr_row_min * pattern_size[0] + pyr_col_min,  # top-left
            pyr_row_min * pattern_size[0] + pyr_col_max,  # top-right
            pyr_row_max * pattern_size[0] + pyr_col_min,  # bottom-left
            pyr_row_max * pattern_size[0] + pyr_col_max   # bottom-right
        ]
        
        # Draw pyramid boundary
        for pyr_idx in pyramid_corners:
            if 0 <= pyr_idx < len(corners):
                pt = tuple(corners[pyr_idx][0].astype(int))
                cv2.circle(image4, pt, 8, (255, 0, 255), 2)
        
        # Draw pyramid rectangle
        if all(0 <= idx < len(corners) for idx in pyramid_corners):
            pts = np.array([corners[idx][0] for idx in pyramid_corners], dtype=np.int32)
            # Reorder to draw rectangle: top-left, top-right, bottom-right, bottom-left
            rect_pts = pts[[0, 1, 3, 2]]
            cv2.polylines(image4, [rect_pts], isClosed=True, color=(255, 0, 255), thickness=2)
            
            # Also draw the center of pyramid base (where apex should be directly above)
            center_2d = pts.mean(axis=0).astype(int)
            cv2.circle(image4, tuple(center_2d), 6, (0, 255, 255), -1)
            cv2.line(image4, tuple(center_2d), tuple(apex_observed_2d.astype(int)), (0, 255, 255), 2)
        
        # Draw apex position (from PnP calculation)
        cv2.circle(image4, tuple(apex_observed_2d.astype(int)), 12, (0, 0, 255), -1)
        cv2.circle(image4, tuple(apex_observed_2d.astype(int)), 18, (255, 255, 255), 3)
        cv2.putText(image4, 'Apex (calc)', tuple((apex_observed_2d + [10, -25]).astype(int)),
                   cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
        
        # Calculate and draw expected apex position (geometric center of pyramid base + height)
        if all(0 <= idx < len(corners) for idx in pyramid_corners):
            pts_2d = np.array([corners[idx][0] for idx in pyramid_corners])
            center_2d = pts_2d.mean(axis=0)
            
            # Project base center elevated by pyramid height
            # Note: square_size is hardcoded as 0.02m
            pyr_base_center_3d = np.array([
                (pyr_col_min + pyr_col_max) / 2.0 * 0.02,
                (pyr_row_min + pyr_row_max) / 2.0 * 0.02,
                pyr_height
            ])
            
            # Also project the BASE center (z=0) to compare with 2D mean of corners
            pyr_base_center_z0 = pyr_base_center_3d.copy()
            pyr_base_center_z0[2] = 0.0  # Set z=0 (on the chessboard plane)
            base_center_projected, _ = cv2.projectPoints(pyr_base_center_z0.reshape(1, 3), rvec, tvec, K, dist)
            base_center_projected = base_center_projected.reshape(2)
            
            # Compare with 2D mean of 4 corners
            corners_2d_mean = pts_2d.mean(axis=0)
            projection_error = np.linalg.norm(base_center_projected - corners_2d_mean)
            
            print(f"        Base center (z=0) projected: {base_center_projected}, 2D mean: {corners_2d_mean}, diff: {projection_error:.2f}px")
            
            expected_apex_2d, _ = cv2.projectPoints(pyr_base_center_3d.reshape(1, 3), rvec, tvec, K, dist)
            expected_apex_2d = expected_apex_2d.reshape(2)
            
            # Draw expected position
            cv2.circle(image4, tuple(expected_apex_2d.astype(int)), 10, (0, 255, 0), 2)
            cv2.putText(image4, 'Expected', tuple((expected_apex_2d + [10, 25]).astype(int)),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2)
            
            # Draw comparison
            apex_offset = apex_observed_2d - center_2d
            apex_error = apex_observed_2d - expected_apex_2d
            apex_error_mag = np.linalg.norm(apex_error)
            
            cv2.line(image4, tuple(apex_observed_2d.astype(int)), tuple(expected_apex_2d.astype(int)),
                    (255, 128, 0), 2, cv2.LINE_AA)
            
            cv2.putText(image4, f'Offset from base center: ({apex_offset[0]:.1f}, {apex_offset[1]:.1f}) px', 
                       (20, 200), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2)
            cv2.putText(image4, f'Error from expected: {apex_error_mag:.1f} px', 
                       (20, 235), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 128, 0), 2)
        
        # Add title and info
        cv2.putText(image4, 'Corner Detection Order', (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 255, 255), 3)
        cv2.putText(image4, 'Corner Detection Order', (20, 40), 
                   cv2.FONT_HERSHEY_SIMPLEX, 1.2, (255, 128, 0), 2)
        cv2.putText(image4, f'Pattern: {pattern_size[0]}x{pattern_size[1]} corners', (20, 90), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.8, (255, 255, 0), 2)
        cv2.putText(image4, f'Pyramid cols: [{pyr_col_min}, {pyr_col_max}]', (20, 130), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        cv2.putText(image4, f'Pyramid rows: [{pyr_row_min}, {pyr_row_max}]', (20, 165), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 0, 255), 2)
        cv2.putText(image4, 'Red circle: first (0) | Blue circle: last', (20, image4.shape[0]-50), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        cv2.putText(image4, 'Magenta: pyramid corners | Red dot: apex', (20, image4.shape[0]-20), 
                   cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
        
        output4 = os.path.join(output_dir, f'corners_{i:03d}.png')
        cv2.imwrite(output4, image4)
        
        print(f"      Image {i}: intrinsic | extrinsic | stick | corners")
    
    print(f"    Saved {len(valid_indices)*4} validation images ({len(valid_indices)} sets x 4 types)")


def main():
    parser = argparse.ArgumentParser(
        description='Calibrate stick tip position using least squares method'
    )
    parser.add_argument('--data_dir', type=str, required=True,
                        help='Directory containing image and pose JSON files')
    parser.add_argument('--pyr_config', type=str, required=True,
                        help='Pyramid configuration JSON file')
    parser.add_argument('--camera_intrinsic', type=str, required=True,
                        help='Camera intrinsic calibration JSON file')
    parser.add_argument('--camera_extrinsic', type=str, required=True,
                        help='Eye-in-hand calibration JSON file')
    parser.add_argument('--output', type=str, 
                        default='temp/stick_calibration_result/stick_lstsq_result.json',
                        help='Output calibration result file')
    
    args = parser.parse_args()
    
    # Hard-coded chessboard parameters (12x9 board has 11x8 inner corners)
    pattern_cols = 11
    pattern_rows = 8
    square_size = 0.02  # meters
    
    print("=" * 60)
    print("Stick Tip Calibration - Least Squares Method")
    print("=" * 60)
    
    # Load configurations
    print("\n[1/6] Loading configurations...")
    K, dist = load_camera_intrinsics(args.camera_intrinsic)
    T_cam2end = load_cam2base_transform(args.camera_extrinsic)
    pyr_config = load_pyramid_config(args.pyr_config)
    
    print(f"  Camera matrix K:\n{K}")
    print(f"  Distortion coefficients: {dist}")
    print(f"  Pyramid height: {pyr_config['height']} m")
    print(f"  Pyramid base: {pyr_config['base_length']} m")
    
    # Build chessboard object points
    print("\n[2/6] Building chessboard model...")
    pattern_size = (pattern_cols, pattern_rows)
    objp = build_chessboard_object_points(pattern_cols, pattern_rows, square_size)
    apex_board = compute_pyramid_apex_in_board_frame(pyr_config, square_size)
    print(f"  Pattern size: {pattern_size}")
    print(f"  Square size: {square_size} m")
    print(f"  Apex in board frame: {apex_board}")
    
    # Find all data files
    print("\n[3/6] Loading calibration data...")
    image_files = sorted(glob.glob(os.path.join(args.data_dir, '*.jpg')))
    if len(image_files) == 0:
        image_files = sorted(glob.glob(os.path.join(args.data_dir, '*.png')))
    
    print(f"  Found {len(image_files)} images")
    
    # Process each image-pose pair
    print("\n[4/6] Processing images and computing apex positions...")
    apex_positions_base = []
    end_poses_base = []
    valid_indices = []
    
    for i, img_path in enumerate(image_files):
        # Get corresponding pose JSON
        base_name = os.path.splitext(os.path.basename(img_path))[0]
        json_path = os.path.join(args.data_dir, f"{base_name}.json")
        
        if not os.path.exists(json_path):
            print(f"  [{i}] SKIP: No pose file for {base_name}")
            continue
        
        # Load image and robot pose
        image = cv2.imread(img_path)
        if image is None:
            print(f"  [{i}] SKIP: Failed to load image {img_path}")
            continue
        
        T_base_end = load_robot_pose(json_path)
        
        # Detect chessboard
        success, R_cam_board, t_cam_board = detect_chessboard_pose(
            image, objp, pattern_size, K, dist
        )
        
        if not success:
            print(f"  [{i}] SKIP: Chessboard not detected in {base_name}")
            continue
        
        # Compute apex position in camera frame
        p_cam_apex = R_cam_board @ apex_board + t_cam_board
        
        # Transform to end-effector frame
        R_cam2end = T_cam2end[:3, :3]
        t_cam2end = T_cam2end[:3, 3]
        p_end_apex = R_cam2end @ p_cam_apex + t_cam2end
        
        # Transform to base frame
        R_base_end = T_base_end[:3, :3]
        t_base_end = T_base_end[:3, 3]
        p_base_apex = R_base_end @ p_end_apex + t_base_end
        
        apex_positions_base.append(p_base_apex)
        end_poses_base.append(T_base_end)
        valid_indices.append(i)
        
        print(f"  [{i}] OK: {base_name} -> apex_base = [{p_base_apex[0]:.4f}, {p_base_apex[1]:.4f}, {p_base_apex[2]:.4f}]")
    
    print(f"\n  Valid measurements: {len(apex_positions_base)}")
    
    if len(apex_positions_base) < 3:
        print("\n[ERROR] Insufficient valid measurements (need at least 3)")
        return
    
    # Analyze sample distribution before solving
    print("\n[5/7] Analyzing sample distribution...")
    apex_array = np.array(apex_positions_base)
    apex_range = apex_array.max(axis=0) - apex_array.min(axis=0)
    apex_std = apex_array.std(axis=0)
    
    print(f"  Apex position range in base frame:")
    print(f"    X: {apex_range[0]*1000:.2f} mm (std: {apex_std[0]*1000:.2f} mm)")
    print(f"    Y: {apex_range[1]*1000:.2f} mm (std: {apex_std[1]*1000:.2f} mm)")
    print(f"    Z: {apex_range[2]*1000:.2f} mm (std: {apex_std[2]*1000:.2f} mm)")
    
    # Check if Z range is too small
    if apex_range[2] < 0.005:  # less than 5mm
        print(f"\n  [WARNING] Z-direction range ({apex_range[2]*1000:.2f} mm) is very small!")
        print(f"  This may cause poor calibration accuracy in Z direction.")
        print(f"  Recommendation: Collect samples with larger Z-direction variation.")
    
    # Analyze end-effector orientation diversity
    end_z_axes = []
    for T in end_poses_base:
        # Z-axis of end-effector in base frame
        z_axis = T[:3, 2]
        end_z_axes.append(z_axis)
    
    end_z_axes = np.array(end_z_axes)
    z_axis_std = end_z_axes.std(axis=0)
    
    print(f"\n  End-effector Z-axis orientation diversity:")
    print(f"    Std: [{z_axis_std[0]:.4f}, {z_axis_std[1]:.4f}, {z_axis_std[2]:.4f}]")
    
    if z_axis_std.max() < 0.05:
        print(f"  [WARNING] End-effector orientation diversity is low!")
        print(f"  Recommendation: Collect samples with more varied orientations.")
    
    # Solve for stick tip position
    print("\n[6/7] Solving least squares problem...")
    t_stick, stats = solve_stick_tip_lstsq(apex_positions_base, end_poses_base)
    
    print(f"\n  Stick tip position in end-effector frame:")
    print(f"    t_x = [{t_stick[0]:.6f}, {t_stick[1]:.6f}, {t_stick[2]:.6f}] m")
    print(f"\n  Error statistics:")
    print(f"    Mean error:  {stats['mean_error']*1000:.3f} mm")
    print(f"    Std error:   {stats['std_error']*1000:.3f} mm")
    print(f"    RMSE:        {stats['rmse']*1000:.3f} mm")
    print(f"    Max error:   {stats['max_error']*1000:.3f} mm")
    print(f"    Min error:   {stats['min_error']*1000:.3f} mm")
    
    # Analyze condition number of A matrix
    print(f"\n  Linear system analysis:")
    print(f"    Matrix rank: {stats['matrix_rank']}/3")
    print(f"    Condition number: {stats['singular_values'][0]/stats['singular_values'][-1]:.2f}")
    print(f"    Singular values: [{stats['singular_values'][0]:.4f}, {stats['singular_values'][1]:.4f}, {stats['singular_values'][2]:.4f}]")
    
    if stats['singular_values'][-1] < 0.1:
        print(f"  [WARNING] Smallest singular value is very small ({stats['singular_values'][-1]:.4f})!")
        print(f"  This indicates poor observability in some direction.")
        print(f"  The solution may be sensitive to measurement noise.")
    
    # Build transformation matrix
    T_end_stick = np.eye(4)
    T_end_stick[:3, 3] = t_stick
    
    # Prepare output
    print("\n[7/8] Saving results...")
    result = {
        'success': True,
        'method': 'least_squares',
        'stick_tip_translation': t_stick.tolist(),
        'stick_tip_rotation': np.eye(3).tolist(),
        'transform_end2stick': T_end_stick.tolist(),
        'statistics': stats,
        'measurements': {
            'total_images': len(image_files),
            'valid_measurements': len(apex_positions_base),
            'valid_indices': valid_indices
        },
        'configuration': {
            'pattern_cols': pattern_cols,
            'pattern_rows': pattern_rows,
            'square_size': square_size,
            'pyramid': pyr_config,
            'apex_board_frame': apex_board.tolist()
        }
    }
    
    # Save to file
    os.makedirs(os.path.dirname(args.output), exist_ok=True)
    with open(args.output, 'w') as f:
        json.dump(result, f, indent=2)
    
    print(f"  Results saved to: {args.output}")
    
    # Visualize results
    print("\n[8/8] Generating visualizations...")
    viz_output_dir = os.path.join(os.path.dirname(args.output), 'visualizations')
    try:
        # Get valid image files
        valid_image_files = [image_files[i] for i in valid_indices]
        
        visualize_calibration_results(
            K=K,
            dist=dist,
            T_cam2end=T_cam2end,
            t_stick=t_stick,
            stats=stats,
            output_dir=viz_output_dir,
            image_files=valid_image_files,
            objp=objp,
            pattern_size=pattern_size,
            apex_board=apex_board,
            valid_indices=valid_indices,
            end_poses_base=end_poses_base,
            apex_positions_base=apex_positions_base,
            pyr_config=pyr_config
        )
        print(f"  Visualizations saved to: {viz_output_dir}")
    except Exception as e:
        print(f"  Warning: Visualization failed: {e}")
        import traceback
        traceback.print_exc()
    
    print("\n" + "=" * 60)
    print("Calibration completed successfully!")
    print("=" * 60)


if __name__ == '__main__':
    main()
