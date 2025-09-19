import numpy as np
import pandas as pd
import math
from dataclasses import dataclass

# --------------------------
# Math utilities
# --------------------------
def skew(v: np.ndarray) -> np.ndarray:
    """v = [vx, vy, vz] -> skew-symmetric matrix S(v)."""
    vx, vy, vz = v
    return np.array([[ 0, -vz,  vy],
                     [ vz,  0, -vx],
                     [-vy, vx,   0]], dtype=float)

def quat_to_rot_matrix(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    """(x, y, z, w) -> R (assuming s->b quaternion represents base frame pose)."""
    # Normalize
    if (n := math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)) < 1e-12:
        return np.eye(3)
    x, y, z, w = qx/n, qy/n, qz/n, qw/n
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    return np.array([
        [1 - 2*(yy + zz), 2*(xy - wz),   2*(xz + wy)],
        [2*(xy + wz),     1 - 2*(xx+zz), 2*(yz - wx)],
        [2*(xz - wy),     2*(yz + wx),   1 - 2*(xx+yy)]
    ], dtype=float)

# --------------------------
# Result structure
# --------------------------
@dataclass
class FTCalibration:
    bf: np.ndarray    # (3,) force bias
    bt: np.ndarray    # (3,) torque bias
    m: float          # mass
    c: np.ndarray     # (3,) CoM in sensor frame
    p: np.ndarray     # (3,) p = m*c
    g_b: np.ndarray   # (3,) gravity vector in base (default [0,0,-9.80665])
    R_pose_to_sensor: np.ndarray | None = None  # optional fixed rotation if pose != sensor

    def predict_gravity_wrench(self, qx: float, qy: float, qz: float, qw: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Predict gravity-induced (f_g, tau_g) in sensor frame for current quaternion.
        """
        R_b_pose = quat_to_rot_matrix(qx, qy, qz, qw)  # assuming s==pose: R_b_s
        R_b_s = R_b_pose @ self.R_pose_to_sensor if self.R_pose_to_sensor is not None else R_b_pose

        # Gravity vector in sensor frame
        h = R_b_s.T @ self.g_b  # h = R^T g_b

        f_g = self.m * h
        tau_g = np.cross(self.p, h)  # p x h  (p = m c)
        return f_g, tau_g

    def compensate(self, fx: float, fy: float, fz: float, tx: float, ty: float, tz: float, 
                   qx: float, qy: float, qz: float, qw: float) -> tuple[np.ndarray, np.ndarray]:
        """
        Compensate new measurements: (f_meas, tau_meas, quaternion) -> (f_comp, tau_comp)
        """
        f_g, tau_g = self.predict_gravity_wrench(qx, qy, qz, qw)
        f_meas = np.array([fx, fy, fz], dtype=float)
        t_meas = np.array([tx, ty, tz], dtype=float)
        return f_meas - self.bf - f_g, t_meas - self.bt - tau_g

# --------------------------
# Calibration (least squares)
# --------------------------
def calibrate_from_dataframe(
    df: pd.DataFrame,
    g: float = 9.80665,
    g_b: np.ndarray | None = None,
    use_quat_cols: bool = False,
    R_pose_to_sensor: np.ndarray | None = None,
    outlier_z: float | None = 3.0
) -> FTCalibration:
    """
    df: DataFrame from previous step. Requires minimum columns:
        - Rotation matrix: r11..r33  (or qx,qy,qz,qw if use_quat_cols=True)
        - Force/torque: fx,fy,fz, tx,ty,tz

    g_b: gravity vector in base frame. None defaults to [0,0,-g].
    use_quat_cols: True to use qx..qw instead of r11..r33 for R calculation.
    R_pose_to_sensor: fixed rotation between pose and sensor frames (3x3 if exists).
    outlier_z: outlier removal threshold based on residual z-score (None to disable).
    """
    g_b = np.array([0.0, 0.0, -g], dtype=float) if g_b is None else g_b

    # Calculate h_i = R_b_s^T g_b for all samples
    H_list, F_list, T_list = [], [], []

    for _, row in df.iterrows():
        if use_quat_cols:
            R_b_pose = quat_to_rot_matrix(row["qx"], row["qy"], row["qz"], row["qw"])
        else:
            R_b_pose = np.array([
                [row["r11"], row["r12"], row["r13"]],
                [row["r21"], row["r22"], row["r23"]],
                [row["r31"], row["r32"], row["r33"]],
            ], dtype=float)

        R_b_s = R_b_pose @ R_pose_to_sensor if R_pose_to_sensor is not None else R_b_pose
        h_i = R_b_s.T @ g_b  # gravity expressed in sensor frame

        H_list.append(h_i)
        F_list.append([row["fx"], row["fy"], row["fz"]])
        T_list.append([row["tx"], row["ty"], row["tz"]])

    H = np.vstack(H_list)      # (N, 3)
    Fm = np.vstack(F_list)     # (N, 3)
    Tm = np.vstack(T_list)     # (N, 3)

    N = H.shape[0]
    if N < 4:
        raise ValueError("Too few samples. At least 4 different poses required.")

    # Construct linear system
    # [F]   [ I  0  H  0     ] [bf]
    # [T] = [ 0  I  0  -S(H) ] [bt]
    #                   [m]
    #                   [p]
    #
    # Matrix A: (6N, 10), vector y: (6N,)
    A = np.zeros((6*N, 10), dtype=float)
    y = np.zeros(6*N, dtype=float)

    for i in range(N):
        hi = H[i]
        Si = skew(hi)

        # Force rows
        A[6*i:6*i+3, 0:3] = np.eye(3)     # bf
        A[6*i:6*i+3, 6] = hi              # m * h_i
        y[6*i:6*i+3] = Fm[i]

        # Torque rows
        A[6*i+3:6*i+6, 3:6] = np.eye(3)   # bt
        A[6*i+3:6*i+6, 7:10] = -Si        # -S(h_i) * p
        y[6*i+3:6*i+6] = Tm[i]

    # Initial LS estimation
    theta, *_ = np.linalg.lstsq(A, y, rcond=None)
    bf, bt, m, p = theta[0:3], theta[3:6], float(theta[6]), theta[7:10]
    c = p / m if abs(m) > 1e-9 else np.zeros(3)

    # Outlier removal (optional)
    if outlier_z is not None and N >= 8:
        y_hat = A @ theta
        resid = (y - y_hat).reshape(N, 6)
        resid_norm = np.linalg.norm(resid, axis=1)
        
        mu, sigma = np.mean(resid_norm), np.std(resid_norm) + 1e-12
        keep = np.abs((resid_norm - mu)/sigma) < outlier_z
        
        if 4 <= (n_keep := keep.sum()) < N:
            # Re-estimate with outliers removed
            A2, y2 = A[keep.repeat(6)], y[keep.repeat(6)]
            theta, *_ = np.linalg.lstsq(A2, y2, rcond=None)
            bf, bt, m, p = theta[0:3], theta[3:6], float(theta[6]), theta[7:10]
            c = p / m if abs(m) > 1e-9 else np.zeros(3)

    return FTCalibration(
        bf=bf, bt=bt, m=m, c=c, p=p, g_b=g_b, R_pose_to_sensor=R_pose_to_sensor
    )

#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
Calibration JSONs -> Rotation matrix + Force/Torque pairing

- Searches for calib_data_*.json files when given a folder.
- Reads pose.orientation (x,y,z,w) from each file, normalizes, and converts to 3x3 rotation matrix.
- Extracts ft_sensor.force, ft_sensor.torque and pairs them into a single record.
- Optionally rotates force/torque to base frame for additional analysis.

Usage example:
    python calib_extract.py --dir /path/to/calib --out calib_pairs.csv --rotate-to-base

Output:
    CSV: file, timestamp, frame ids, quaternion, position, r11..r33, fx,fy,fz, tx,ty,tz
    (Optional) rotated force/torque: fx_base,fy_base,fz_base, tx_base,ty_base,tz_base
"""

import os
import glob
import json
import math
from typing import Dict, Any, List, Optional

import numpy as np
import pandas as pd


def _safe_json_load(fp):
    """
    Safe JSON loading for files that may contain NaN, Infinity, etc.
    Python's json allows NaN/Infinity but we handle it explicitly with parse_constant.
    """
    return json.load(fp, parse_constant=lambda x: float('nan'))


def normalize_quaternion(q: tuple[float, float, float, float]) -> tuple[float, float, float, float]:
    x, y, z, w = q
    if (norm := math.sqrt(x*x + y*y + z*z + w*w)) < 1e-12:
        # Replace invalid quaternion with identity rotation
        return (0.0, 0.0, 0.0, 1.0)
    return (x / norm, y / norm, z / norm, w / norm)


def extract_from_json(data: Dict[str, Any], filename: str, rotate_to_base: bool) -> Optional[Dict[str, Any]]:
    """
    Extract required fields from single JSON and pair rotation matrix with force/torque.
    """
    try:
        ts_iso = data.get("timestamp", None)

        # Pose
        pose = data["pose"]
        pose_hdr = pose.get("header", {})
        pose_frame = pose_hdr.get("frame_id", None)
        pose_stamp = pose_hdr.get("stamp", {})
        pose_sec = pose_stamp.get("sec", None)
        pose_nsec = pose_stamp.get("nanosec", None)

        pos = pose.get("position", {})
        px, py, pz = (float(pos.get(k, float("nan"))) for k in ["x", "y", "z"])

        ori = pose.get("orientation", {})
        qx, qy, qz, qw = (float(ori.get(k, v)) for k, v in [("x", 0.0), ("y", 0.0), ("z", 0.0), ("w", 1.0)])

        R = quat_to_rot_matrix(qx, qy, qz, qw)

        # FT Sensor
        ft = data["ft_sensor"]
        ft_hdr = ft.get("header", {})
        ft_frame = ft_hdr.get("frame_id", None)
        ft_stamp = ft_hdr.get("stamp", {})
        ft_sec = ft_stamp.get("sec", None)
        ft_nsec = ft_stamp.get("nanosec", None)

        force = ft.get("force", {})
        fx, fy, fz = (float(force.get(k, float("nan"))) for k in ["x", "y", "z"])

        torque = ft.get("torque", {})
        tx, ty, tz = (float(torque.get(k, float("nan"))) for k in ["x", "y", "z"])

        row = {
            "file": os.path.basename(filename),
            "timestamp": ts_iso,
            "pose_frame_id": pose_frame,
            "pose_sec": pose_sec,
            "pose_nanosec": pose_nsec,
            "ft_frame_id": ft_frame,
            "ft_sec": ft_sec,
            "ft_nanosec": ft_nsec,
            # Position
            "px": px, "py": py, "pz": pz,
            # Quaternion
            "qx": qx, "qy": qy, "qz": qz, "qw": qw,
            # Rotation matrix flattened
            **{f"r{i+1}{j+1}": R[i, j] for i in range(3) for j in range(3)},
            # Force/Torque in sensor frame (original)
            "fx": fx, "fy": fy, "fz": fz,
            "tx": tx, "ty": ty, "tz": tz,
        }

        if rotate_to_base:
            # Assume sensor frame == pose frame.
            # F_base = R_base_from_sensor @ F_sensor
            F_sensor = np.array([fx, fy, fz], dtype=float)
            T_sensor = np.array([tx, ty, tz], dtype=float)
            F_base, T_base = R @ F_sensor, R @ T_sensor
            row.update({
                **{f"f{axis}_base": val for axis, val in zip("xyz", F_base)},
                **{f"t{axis}_base": val for axis, val in zip("xyz", T_base)},
            })

        return row

    except KeyError as e:
        # Skip on missing required keys
        print(f"[WARN] {filename}: missing key {e}; skip.")
        return None
    except Exception as e:
        print(f"[ERROR] {filename}: {e}")
        return None


def collect_calib_pairs(
    dir_path: str,
    pattern: str = "calib_data_*.json",
    rotate_to_base: bool = False
) -> pd.DataFrame:
    """
    Read JSON files from specified folder and pair rotation matrices with force/torque into DataFrame.
    """
    if not (files := sorted(glob.glob(os.path.join(dir_path, pattern)))):
        raise FileNotFoundError(f"No JSON files matched: {os.path.join(dir_path, pattern)}")

    rows = []
    for f in files:
        try:
            with open(f, "r", encoding="utf-8") as fp:
                data = _safe_json_load(fp)
            if (row := extract_from_json(data, f, rotate_to_base=rotate_to_base)) is not None:
                rows.append(row)
        except Exception as e:
            print(f"[ERROR] Failed to parse {f}: {e}")

    if not rows:
        raise RuntimeError("No valid calibration records parsed.")

    df = pd.DataFrame(rows)

    # Sort by available timestamp columns
    sort_cols = [c for c in ["timestamp", "pose_sec", "pose_nanosec", "ft_sec", "ft_nanosec"] if c in df.columns]
    if sort_cols:
        df = df.sort_values(sort_cols, kind="mergesort").reset_index(drop=True)

    return df


def n_fold_validation(df: pd.DataFrame, **calib_kwargs) -> Dict[str, Any]:
    """
    N-fold cross-validation for FT calibration.
    
    Args:
        df: DataFrame with calibration data
        **calib_kwargs: keyword arguments for calibrate_from_dataframe
    
    Returns:
        Dictionary with validation results
    """
    N = len(df)
    print(f"\n=== Starting {N}-fold Cross-Validation ===\nTotal samples: {N}")
    
    validation_results = []
    errors_before = {"force": [], "torque": []}
    errors_after = {"force": [], "torque": []}
    
    for i in range(N):
        print(f"\n--- Fold {i+1}/{N} ---")
        
        # Split data: one for test, rest for training
        test_df = df.iloc[[i]]
        train_df = df.drop(index=i).reset_index(drop=True)
        
        print(f"Training samples: {len(train_df)}, Test samples: {len(test_df)}")
        
        # Train calibration on training data
        try:
            calib = calibrate_from_dataframe(train_df, **calib_kwargs)
        except Exception as e:
            print(f"ERROR in fold {i+1}: Failed to calibrate - {e}")
            continue
        
        # Test on the held-out sample
        test_row = test_df.iloc[0]
        
        # Extract test data
        if calib_kwargs.get('use_quat_cols', False):
            qx, qy, qz, qw = test_row["qx"], test_row["qy"], test_row["qz"], test_row["qw"]
        else:
            # Convert rotation matrix back to quaternion for prediction
            R = np.array([[test_row[f"r{i+1}{j+1}"] for j in range(3)] for i in range(3)])
            # Simple rotation matrix to quaternion conversion
            if (qw := np.sqrt(1.0 + R[0,0] + R[1,1] + R[2,2]) / 2.0) > 1e-6:
                qx, qy, qz = [(R[2,1] - R[1,2]), (R[0,2] - R[2,0]), (R[1,0] - R[0,1])] / (4*qw)
            else:
                qx, qy, qz, qw = 0, 0, 0, 1
        
        # Raw measurements
        fx_raw, fy_raw, fz_raw = test_row["fx"], test_row["fy"], test_row["fz"]
        tx_raw, ty_raw, tz_raw = test_row["tx"], test_row["ty"], test_row["tz"]
        
        # Before compensation (raw values should ideally be zero for contact-free state)
        force_before = np.array([fx_raw, fy_raw, fz_raw])
        torque_before = np.array([tx_raw, ty_raw, tz_raw])
        
        # After compensation using calibration
        force_after, torque_after = calib.compensate(
            fx_raw, fy_raw, fz_raw, tx_raw, ty_raw, tz_raw, qx, qy, qz, qw
        )
        
        # Calculate errors (ideally should be close to zero after compensation)
        force_error_before, force_error_after = np.linalg.norm(force_before), np.linalg.norm(force_after)
        torque_error_before, torque_error_after = np.linalg.norm(torque_before), np.linalg.norm(torque_after)
        
        # Store results
        fold_result = {
            'fold': i+1,
            'test_file': test_row.get('file', f'row_{i}'),
            'quaternion': [qx, qy, qz, qw],
            'force_before': force_before,
            'force_after': force_after,
            'torque_before': torque_before, 
            'torque_after': torque_after,
            'force_error_before': force_error_before,
            'force_error_after': force_error_after,
            'torque_error_before': torque_error_before,
            'torque_error_after': torque_error_after,
            'calibration': calib
        }
        validation_results.append(fold_result)
        
        # Accumulate errors for statistics
        errors_before["force"].append(force_error_before)
        errors_after["force"].append(force_error_after)
        errors_before["torque"].append(torque_error_before)
        errors_after["torque"].append(torque_error_after)
        
        # Print detailed results for this fold
        print(f"Test file: {fold_result['test_file']}")
        print(f"Quaternion: [{qx:.4f}, {qy:.4f}, {qz:.4f}, {qw:.4f}]")
        print(f"Force BEFORE: [{fx_raw:8.4f}, {fy_raw:8.4f}, {fz_raw:8.4f}] (norm: {force_error_before:8.4f})")
        print(f"Force AFTER:  [{force_after[0]:8.4f}, {force_after[1]:8.4f}, {force_after[2]:8.4f}] (norm: {force_error_after:8.4f})")
        print(f"Torque BEFORE:[{tx_raw:8.4f}, {ty_raw:8.4f}, {tz_raw:8.4f}] (norm: {torque_error_before:8.4f})")
        print(f"Torque AFTER: [{torque_after[0]:8.4f}, {torque_after[1]:8.4f}, {torque_after[2]:8.4f}] (norm: {torque_error_after:8.4f})")
        print(f"Improvement - Force: {force_error_before:8.4f} -> {force_error_after:8.4f} ({100*(force_error_after-force_error_before)/force_error_before:+6.2f}%)")
        print(f"Improvement - Torque: {torque_error_before:8.4f} -> {torque_error_after:8.4f} ({100*(torque_error_after-torque_error_before)/torque_error_before:+6.2f}%)")
    
    # Summary statistics
    if validation_results:
        print(f"\n=== Validation Summary ===\nSuccessful folds: {len(validation_results)}/{N}")
        
        for error_type in ["force", "torque"]:
            before = np.array(errors_before[error_type])
            after = np.array(errors_after[error_type])
            
            print(f"\n{error_type.title()} Error Statistics:")
            print(f"  Before compensation - Mean: {np.mean(before):.4f}, Std: {np.std(before):.4f}")
            print(f"  After compensation  - Mean: {np.mean(after):.4f}, Std: {np.std(after):.4f}")
            print(f"  Average improvement: {np.mean(before - after):.4f}")
            print(f"  Improvement ratio: {np.mean(after / before):.4f}")
        
        # Find best and worst cases
        force_improvements = np.array(errors_before["force"]) - np.array(errors_after["force"])
        best_idx, worst_idx = np.argmax(force_improvements), np.argmin(force_improvements)
        
        for desc, idx in [("Best", best_idx), ("Worst", worst_idx)]:
            print(f"\n{desc} improvement (fold {validation_results[idx]['fold']}):")
            print(f"  File: {validation_results[idx]['test_file']}")
            print(f"  Force: {errors_before['force'][idx]:.4f} -> {errors_after['force'][idx]:.4f}")
    
    return {
        'results': validation_results,
        **{f'{error_type}_errors_{when}': np.array(errors[error_type]) 
           for error_type in ["force", "torque"] for when, errors in [("before", errors_before), ("after", errors_after)]},
        'n_successful': len(validation_results),
        'n_total': N
    }

def main():
    import argparse
    from datetime import datetime

    parser = argparse.ArgumentParser(description="Extract rotation matrices and force/torque pairs from calibration JSONs.")
    parser.add_argument("--dir", required=True, help="Directory containing calib_data_*.json")
    parser.add_argument("--pattern", default="calib_data_*.json", help="Glob pattern for files (default: calib_data_*.json)")
    parser.add_argument("--out", default=None, help="CSV output path (optional)")
    parser.add_argument("--rotate-to-base", action="store_true",
                        help="Rotate force/torque from sensor frame to base frame using pose quaternion (assumes sensor frame == pose frame).")
    parser.add_argument("--validate", action="store_true", help="Run N-fold cross-validation")
    parser.add_argument("--use-quat", action="store_true", help="Use quaternion columns instead of rotation matrix")
    parser.add_argument("--calib-out", default=None, help="Calibration output file path (JSON format)")

    args = parser.parse_args()

    df = collect_calib_pairs(args.dir, args.pattern, rotate_to_base=args.rotate_to_base)
    print(f"{df.head(10).to_string(index=False)}\n\nTotal records: {len(df)}")

    if args.out:
        df.to_csv(args.out, index=False)
        print(f"Saved CSV to: {args.out}")

    if args.validate:
        # Run N-fold validation
        validation_results = n_fold_validation(
            df, 
            g=9.80665, 
            g_b=None, 
            use_quat_cols=args.use_quat, 
            R_pose_to_sensor=None,
            outlier_z=3.0
        )
        
        # Optionally save validation results
        if args.out:
            import pickle
            validation_file = args.out.replace('.csv', '_validation.pkl')
            with open(validation_file, 'wb') as f:
                pickle.dump(validation_results, f)
            print(f"Saved validation results to: {validation_file}")
    else:
        # Original single calibration
        calib = calibrate_from_dataframe(df, g=9.80665, g_b=None, use_quat_cols=args.use_quat, R_pose_to_sensor=None)

        print("=== Calibration Result ===")
        print(f"Force bias (bf): {calib.bf}")
        print(f"Torque bias (bt): {calib.bt}")
        print(f"Mass (m): {calib.m} kg")
        print(f"CoM in sensor frame (c): {calib.c} m")
        print(f"p = m*c: {calib.p}")
        
        # Determine output file
        if args.calib_out:
            calib_file = args.calib_out
        elif args.out:
            calib_file = args.out.replace('.csv', '_calibration.json')
        else:
            timestamp = datetime.now().strftime('%Y%m%d_%H%M%S')
            calib_file = f'ft_calibration_{timestamp}.json'
        
        # Create calibration data dictionary for saving
        calib_data = {
            'calibration': {
                'force_bias': calib.bf.tolist(),
                'torque_bias': calib.bt.tolist(),
                'mass': float(calib.m),
                'center_of_mass': calib.c.tolist(),
                'p_vector': calib.p.tolist(),
                'gravity_vector': calib.g_b.tolist(),
                'pose_to_sensor_rotation': calib.R_pose_to_sensor.tolist() if calib.R_pose_to_sensor is not None else None
            },
            'metadata': {
                'timestamp': datetime.now().isoformat(),
                'n_samples': len(df),
                'data_dir': args.dir,
                'pattern': args.pattern,
                'use_quat_cols': args.use_quat,
                'rotate_to_base': args.rotate_to_base,
                'gravity_magnitude': 9.80665,
                'outlier_z_threshold': 3.0,
                'units': {
                    'force_bias': 'N',
                    'torque_bias': 'N*m',
                    'mass': 'kg',
                    'center_of_mass': 'm',
                    'gravity_vector': 'm/s^2'
                }
            },
            'usage_example': {
                'description': 'To use this calibration, load the data and apply compensation',
                'python_code': [
                    'import json',
                    'import numpy as np',
                    f'with open("{calib_file}", "r") as f:',
                    '    calib_data = json.load(f)',
                    'cal = calib_data["calibration"]',
                    '# Apply compensation:',
                    '# force_compensated = force_measured - force_bias - mass * gravity_in_sensor_frame',
                    '# torque_compensated = torque_measured - torque_bias - cross(p_vector, gravity_in_sensor_frame)'
                ]
            }
        }
        
        try:
            with open(calib_file, 'w') as f:
                json.dump(calib_data, f, indent=2)
            print(f"\nCalibration saved to: {calib_file}")
            print(f"Calibration parameters:\n  Force bias: {calib.bf}\n  Torque bias: {calib.bt}")
            print(f"  Mass: {calib.m:.6f} kg\n  Center of mass: {calib.c}")
            print(f"\nUse this file to load calibration in other scripts:")
            print(f"  import json\n  with open('{calib_file}', 'r') as f:")
            print(f"      calib_data = json.load(f)\n      cal = calib_data['calibration']")
        except Exception as e:
            print(f"ERROR: Failed to save calibration to {calib_file}: {e}")


if __name__ == "__main__":
    main()