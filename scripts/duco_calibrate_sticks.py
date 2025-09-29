#!/usr/bin/env python3
"""
Stick Tip Calibration Script
----------------------------
Calibrates the 3D position of a stick tip in the robot end-effector coordinate frame.

Method:
- Uses eye-in-hand camera to detect chessboard with pyramid reference
- Stick tip touches pyramid apex at different robot poses
- Computes robust mean of tip positions across multiple observations

Input Parameters:
- --pyr-config: Pyramid geometry & chessboard mapping JSON
- --camera_intrinsic: Camera intrinsic parameters JSON  
- --camera_extrinsic: Eye-in-hand calibration result JSON
- --output: Output calibration result file (optional)

Usage:
    python3 duco_calibrate_sticks.py \
        --pyr-config temp/stick_calibration_data/pyramid_parameters.json \
        --camera_intrinsic temp/camera_parameters/calibration_result.json \
        --camera_extrinsic temp/camera_parameters/eye_in_hand_result.json

Controls: SPACE to capture sample, Q to quit
Output: 4x4 transformation matrix from end-effector to stick tip
"""

from __future__ import annotations

import argparse
import json
import os
import sys
from dataclasses import dataclass
from typing import List, Tuple, Dict, Any

import cv2
import numpy as np

# Add ThirdParty modules to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), 'ThirdParty/camera_calibration_toolkit'))
from core.calibration_patterns.standard_chessboard import StandardChessboard


@dataclass
class SampleResult:
	idx: int
	p_ee_tip: np.ndarray  # shape (3,)
	accepted: bool
	reason: str = ""


def load_intrinsics(path: str):
	"""Load camera intrinsics from calibration result JSON file.

	Expected JSON structure (from calibration_result.json):
	{
		"camera_matrix": [[fx, 0, cx], [0, fy, cy], [0, 0, 1]],
		"distortion_coefficients": [k1, k2, p1, p2, k3]
	}
	"""
	with open(path, "r") as f:
		data = json.load(f)
	K = np.array(data["camera_matrix"], dtype=np.float64)
	D = np.array(data.get("distortion_coefficients", [0, 0, 0, 0, 0]), dtype=np.float64).reshape(-1, 1)
	return K, D


def load_transform_4x4(path: str) -> np.ndarray:
	"""Load camera-to-end-effector transform from eye-in-hand calibration result.

	Expected JSON structure (from eye_in_hand_result.json):
	{
		"cam2end_matrix": [[r11,r12,r13,tx], [r21,r22,r23,ty], [r31,r32,r33,tz], [0,0,0,1]]
	}
	"""
	with open(path, "r") as f:
		data = json.load(f)
	M = np.array(data["cam2end_matrix"], dtype=np.float64)
	if M.shape != (4, 4):
		raise ValueError("cam2end_matrix must be 4x4")
	return M


def median_absolute_deviation(data: np.ndarray) -> float:
	med = np.median(data)
	return np.median(np.abs(data - med))


def robust_filter(points: np.ndarray, z_thresh: float = 3.5) -> np.ndarray:
	"""Return boolean mask of inliers using MAD on norms from median."""
	if len(points) < 3:
		return np.ones(len(points), dtype=bool)
	norms = np.linalg.norm(points - np.median(points, axis=0), axis=1)
	mad = median_absolute_deviation(norms)
	if mad < 1e-9:
		return np.ones(len(points), dtype=bool)
	modified_z = 0.6745 * (norms - np.median(norms)) / mad
	return np.abs(modified_z) < z_thresh


def build_board_object_points(cols: int, rows: int, square_size: float) -> np.ndarray:
	"""Generate 3D object points for chessboard using camera calibration toolkit."""
	# Use the standard chessboard pattern from toolkit
	chessboard = StandardChessboard(cols, rows, square_size)
	return chessboard.generate_object_points()


def compute_apex_point_rectangle(col_min: int, col_max: int, row_min: int, row_max: int, square_size: float, apex_height: float) -> np.ndarray:
	"""Apex projection at geometric center of rectangle defined by four board inner-corner indices.

	Rectangle corners (axis-aligned):
	  (col_min,row_min), (col_max,row_min), (col_max,row_max), (col_min,row_max)
	Center indices: ((col_min+col_max)/2, (row_min+row_max)/2)
	"""
	cx = (col_min + col_max) / 2.0
	cy = (row_min + row_max) / 2.0
	x = cx * square_size
	y = cy * square_size
	z = apex_height
	return np.array([x, y, z], dtype=np.float64)


def estimate_tip_translation(samples: List[np.ndarray]) -> Tuple[np.ndarray, np.ndarray]:
	pts = np.vstack(samples)
	mask = robust_filter(pts)
	inliers = pts[mask]
	mean = inliers.mean(axis=0)
	return mean, mask


def load_pyramid_config(path: str) -> Dict[str, Any]:
	with open(path, 'r') as f:
		data = json.load(f)
	return data


def resolve_pyramid_parameters(config: Dict[str, Any]) -> Dict[str, Any]:
	try:
		pyr = config["pyramid"]
		size = pyr["size"]
		mapping = pyr["chessboard_mapping"]
	except KeyError as e:
		raise KeyError(f"Missing key in pyramid config JSON: {e}")

	col_min = int(mapping["pyr_col_min"])
	col_max = int(mapping["pyr_col_max"])
	row_min = int(mapping["pyr_row_min"])
	row_max = int(mapping["pyr_row_max"])
	apex_height = float(size["height"])  # in meters

	return {
		"pyr_col_min": col_min,
		"pyr_col_max": col_max,
		"pyr_row_min": row_min,
		"pyr_row_max": row_max,
		"pyr_apex_height": apex_height,
		"base_length": float(size.get("base_length", 0.0))
	}


def main():
	parser = argparse.ArgumentParser(description="Calibrate stick tip position in end-effector frame")
	# Fixed board parameters: 12x9 inner corners, 0.02 m square size
	pattern_cols = 12
	pattern_rows = 9
	square_size = 0.02  # meters

	parser.add_argument("--pyr-config", type=str, required=True, help="JSON file with pyramid size & chessboard mapping")
	parser.add_argument("--camera_intrinsic", type=str, required=True, help="JSON with camera intrinsic calibration result")
	parser.add_argument("--camera_extrinsic", type=str, required=True, help="JSON with eye-in-hand calibration result")
	parser.add_argument("--output", type=str, default="temp/stick_calibration_result/stick_calibration_result.json", help="Output calibration result file")
	args = parser.parse_args()

	pyr_config = load_pyramid_config(args.pyr_config)
	pyr_params = resolve_pyramid_parameters(pyr_config)

	pyr_col_min = pyr_params["pyr_col_min"]
	pyr_col_max = pyr_params["pyr_col_max"]
	pyr_row_min = pyr_params["pyr_row_min"]
	pyr_row_max = pyr_params["pyr_row_max"]
	pyr_apex_height = pyr_params["pyr_apex_height"]

	# Load camera parameters & transform
	K, D = load_intrinsics(args.camera_intrinsic)
	T_ee_cam = load_transform_4x4(args.camera_extrinsic)
	R_ee_cam = T_ee_cam[:3, :3]
	t_ee_cam = T_ee_cam[:3, 3]

	# Prepare chessboard object points (fixed pattern)
	objp = build_board_object_points(pattern_cols, pattern_rows, square_size)
	pattern_size = (pattern_cols, pattern_rows)

	# Validate indices
	def _check_range(name, val, max_val):
		if not (0 <= val < max_val):
			raise ValueError(f"{name}={val} out of range [0, {max_val-1}]")
	for name, val, max_v in [
		("pyr_col_min", pyr_col_min, pattern_cols),
		("pyr_col_max", pyr_col_max, pattern_cols),
		("pyr_row_min", pyr_row_min, pattern_rows),
		("pyr_row_max", pyr_row_max, pattern_rows),
	]:
		_check_range(name, val, max_v)
	if not (pyr_col_min < pyr_col_max and pyr_row_min < pyr_row_max):
		raise ValueError("Require pyr_col_min < pyr_col_max and pyr_row_min < pyr_row_max")

	p_board_apex = compute_apex_point_rectangle(
		pyr_col_min, pyr_col_max, pyr_row_min, pyr_row_max, square_size, pyr_apex_height
	)

	# Open RTSP camera (similar to estimation_task_auto_positioning.py)
	rtsp_url = "rtsp://admin:123456@192.168.1.102/stream0"
	cap = cv2.VideoCapture(rtsp_url)
	if cap.isOpened():
		# Set buffer size to minimize latency for IP cameras
		cap.set(cv2.CAP_PROP_BUFFERSIZE, 1)
		print(f"Successfully connected to IP camera: {rtsp_url}")
	else:
		print(f"Failed to connect to IP camera: {rtsp_url}")
		print("Trying fallback camera source 0...")
		cap = cv2.VideoCapture(0)
		if not cap.isOpened():
			raise RuntimeError("Cannot open any camera source")

	print("Instructions: Move robot so stick tip touches apex. Press SPACE to capture, 'q' to quit early.")
	print("Using IP camera for calibration...")
	collected: List[SampleResult] = []
	sample_count = 0
	idx = 0
	samples = 20  # fixed sample count
	min_success = 8  # fixed minimum success

	win_name = "stick_calibration"
	cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)

	while True:
		ret, frame = cap.read()
		if not ret:
			print("[WARN] Failed to read frame")
			break

		display = frame.copy()
		gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
		found, corners = cv2.findChessboardCorners(gray, pattern_size,
												   flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
		pose_ok = False
		rvec = tvec = None
		if found:
			cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1),
							  criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
			retval, rvec, tvec = cv2.solvePnP(objp, corners, K, D, flags=cv2.SOLVEPNP_ITERATIVE)
			pose_ok = retval
			cv2.drawChessboardCorners(display, pattern_size, corners, found)

		key = cv2.waitKey(1) & 0xFF

		cv2.putText(display, f"Samples: {sample_count}/{samples}", (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)
		cv2.putText(display, f"PoseOK: {pose_ok}", (10, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0,255,255), 2)
		cv2.imshow(win_name, display)

		capture_triggered = False
		if key == ord(' '):
			capture_triggered = True

		if capture_triggered and sample_count < samples:
			if not pose_ok:
				collected.append(SampleResult(idx=idx, p_ee_tip=np.zeros(3), accepted=False, reason="pose_not_found"))
				print(f"[Sample {idx}] Rejected: pose not found")
			else:
				R_cam_board, _ = cv2.Rodrigues(rvec)
				t_cam_board = tvec.reshape(3)
				# apex in camera
				p_cam_apex = R_cam_board @ p_board_apex + t_cam_board
				# transform to ee: p_ee = R_ee_cam * p_cam + t_ee_cam
				p_ee_tip = R_ee_cam @ p_cam_apex + t_ee_cam
				collected.append(SampleResult(idx=idx, p_ee_tip=p_ee_tip, accepted=True))
				sample_count += 1
				print(f"[Sample {idx}] Accepted: p_ee_tip = {p_ee_tip}")

		if key == ord('q'):
			print("User requested quit.")
			break
		if sample_count >= samples:
			break
		idx += 1

	cap.release()
	cv2.destroyAllWindows()

	accepted_points = [s.p_ee_tip for s in collected if s.accepted]
	if len(accepted_points) < min_success:
		print(f"[ERROR] Only {len(accepted_points)} valid samples (< {min_success}). Aborting.")
		return

	mean_tip, mask = estimate_tip_translation(accepted_points)
	inlier_pts = np.vstack(accepted_points)[mask]
	residuals = np.linalg.norm(inlier_pts - mean_tip, axis=1)

	R_ee_stick = np.eye(3)  # identity rotation

	T_ee_stick = np.eye(4)
	T_ee_stick[:3, :3] = R_ee_stick
	T_ee_stick[:3, 3] = mean_tip

	result = {
		"translation": mean_tip.tolist(),
		"rotation_matrix": R_ee_stick.tolist(),
		"transform": T_ee_stick.tolist(),
		"samples_total": len(collected),
		"samples_valid": len(accepted_points),
		"samples_inliers": int(mask.sum()),
		"residual_mean": float(residuals.mean()) if len(residuals) else None,
		"residual_std": float(residuals.std()) if len(residuals) else None,
		"residual_max": float(residuals.max()) if len(residuals) else None,
		"apex_board": p_board_apex.tolist(),
		"config": {
			"pattern_cols": pattern_cols,
			"pattern_rows": pattern_rows,
			"square_size": square_size,
			"pyr_col_min": pyr_col_min,
			"pyr_col_max": pyr_col_max,
			"pyr_row_min": pyr_row_min,
			"pyr_row_max": pyr_row_max,
			"pyr_apex_height": pyr_apex_height,
			"base_length": pyr_params["base_length"],
		}
	}

	with open(args.output, "w") as f:
		# Ensure output directory exists
		os.makedirs(os.path.dirname(args.output), exist_ok=True)
		json.dump(result, f, indent=2)
	print(f"Calibration saved to {args.output}")
	print("T_ee_stick (4x4):\n", T_ee_stick)


if __name__ == "__main__":
	main()