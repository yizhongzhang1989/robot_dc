#!/usr/bin/env python3
"""
debug_rack_fitter.py
====================

Dedicated debug script for the rack-corner localization problem reported in
``temp/workflow_results/workflow_result_20260512_215315.json`` (bad run).

Goal
----
The bad run produced rack corners whose Y/Z axes are swapped relative to the
known-good run ``workflow_result_20260512_202513.json``.  TCP joint angles,
intrinsics, distortion and ``cam2end_matrix`` are byte-for-byte identical
between the two runs, so the failure is not in calibration / namespace /
parameter loading.  The user has independently verified that the keypoint
detection is correct in both runs, which leaves only one place the bug can
live: the rigid-body solver in
``scripts/ThirdParty/robot_vision/core/triangulation.py::fitting_multiview``.

This script reproduces both runs from disk and stress-tests the solver:

1. Reads the per-view sidecar JSON files
   (``end2base``, ``camera_matrix``, ``distortion_coefficients``,
   ``cam2end_matrix``) for the bad-run session
   ``session_1778593964_c0315bae`` and the good-run session
   ``session_1778588681_2a623bd5``.
2. Re-tracks the 4 captured target images against the corresponding rack
   reference images by calling the FlowFormer++ tracking service directly
   (default endpoint ``10.172.100.39:8101``, override via ``--ffpp-url``).
3. Reproduces ``fitting_multiview`` end-to-end and prints the resulting
   pose, mean reprojection error and per-view residuals.
4. Probes for degeneracy by re-running the optimisation from a grid of 24
   distinct initial rotations and reports every local minimum it finds.
5. Re-runs the same probe on the good run for direct comparison.

Run:
    python3 scripts/debug_rack_fitter.py
    python3 scripts/debug_rack_fitter.py --ffpp-url http://10.172.100.39:8101
    python3 scripts/debug_rack_fitter.py --keypoints my_keypoints.json
    python3 scripts/debug_rack_fitter.py --only bad

The script does not modify any file and does not require ROS to be running.
"""

from __future__ import annotations

import argparse
import base64
import io
import json
import os
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

import numpy as np

# Make the upstream triangulation module importable
REPO_ROOT = Path(__file__).resolve().parents[1]
ROBOT_VISION = REPO_ROOT / 'scripts' / 'ThirdParty' / 'robot_vision'
if str(ROBOT_VISION) not in sys.path:
    sys.path.insert(0, str(ROBOT_VISION))

import cv2  # noqa: E402
from scipy.optimize import least_squares, brentq, minimize_scalar  # noqa: E402
from scipy.spatial.transform import Rotation as R  # noqa: E402

from core.triangulation import fitting_multiview, _umeyama  # noqa: E402

# ---------------------------------------------------------------------------
# Dataset layout for the two runs being debugged
# ---------------------------------------------------------------------------

DATASET_ROOT = REPO_ROOT / 'dataset'

# Each entry: (reference_name, keypoint_name_used_in_reference)
RACK_CORNERS = [
    ('rack_bottom_left',  'GB200_Rack_Bottom_Left_Corner'),
    ('rack_bottom_right', 'GB200_Rack_Bottom_Right_Corner'),
    ('rack_top_left',     'GB200_Rack_Top_Left_Corner'),
    ('rack_top_right',    'GB200_Rack_Top_Right_Corner'),
]

# Template points in the rack-local frame (X = rack width, Y = depth, Z = up).
TEMPLATE_POINTS_LOCAL = np.array([
    [0.00, 0.0, 0.000],   # bottom_left
    [0.55, 0.0, 0.000],   # bottom_right
    [0.00, 0.0, 2.145],   # top_left
    [0.55, 0.0, 2.145],   # top_right
], dtype=np.float64)

RUNS = {
    'bad':  ('session_1778593964_c0315bae', 'temp/workflow_results/workflow_result_20260512_215315.json'),
    'good': ('session_1778588681_2a623bd5', 'temp/workflow_results/workflow_result_20260512_202513.json'),
}

# ---------------------------------------------------------------------------
# Data loading
# ---------------------------------------------------------------------------


@dataclass
class ViewCalib:
    reference_name: str
    image_path: Path
    sidecar: Dict
    intrinsic: np.ndarray
    distortion: np.ndarray
    cam2end: np.ndarray
    end2base: np.ndarray
    extrinsic: np.ndarray              # world->camera, i.e. inv(end2base @ cam2end)
    image_size: Tuple[int, int]        # (width, height)


def _load_sidecar(corner_dir: Path, session: str) -> Tuple[Path, Dict]:
    test_dir = corner_dir / 'test' / session
    if not test_dir.is_dir():
        raise FileNotFoundError(f'Missing session folder: {test_dir}')
    jpgs = sorted(test_dir.glob('*.jpg'))
    if not jpgs:
        raise FileNotFoundError(f'No jpg found in {test_dir}')
    img_path = jpgs[0]
    json_path = img_path.with_suffix('.json')
    if not json_path.is_file():
        raise FileNotFoundError(f'Missing sidecar JSON: {json_path}')
    with open(json_path, 'r') as f:
        return img_path, json.load(f)


def load_views(session: str) -> List[ViewCalib]:
    views: List[ViewCalib] = []
    for ref_name, _ in RACK_CORNERS:
        corner_dir = DATASET_ROOT / ref_name
        img_path, sc = _load_sidecar(corner_dir, session)
        intrinsic = np.array(sc['camera_matrix'], dtype=np.float64)
        distortion = np.array(sc['distortion_coefficients'], dtype=np.float64)
        cam2end = np.array(sc['cam2end_matrix'], dtype=np.float64)
        end2base = np.array(sc['end2base'], dtype=np.float64)
        cam2base = end2base @ cam2end
        extrinsic = np.linalg.inv(cam2base)
        # image_size from the actual file (avoid trusting metadata)
        img = cv2.imread(str(img_path))
        if img is None:
            raise FileNotFoundError(f'Cannot read image: {img_path}')
        h, w = img.shape[:2]
        views.append(ViewCalib(
            reference_name=ref_name,
            image_path=img_path,
            sidecar=sc,
            intrinsic=intrinsic,
            distortion=distortion,
            cam2end=cam2end,
            end2base=end2base,
            extrinsic=extrinsic,
            image_size=(w, h),
        ))
    return views


def _load_reference_keypoint(ref_name: str) -> Tuple[Dict, np.ndarray]:
    """Load reference image + its single labelled keypoint."""
    ref_dir = DATASET_ROOT / ref_name
    img = cv2.imread(str(ref_dir / 'ref_img_1.jpg'))
    if img is None:
        raise FileNotFoundError(f'Missing reference image for {ref_name}')
    with open(ref_dir / 'ref_img_1.json', 'r') as f:
        meta = json.load(f)
    kp = meta['keypoints'][0]
    return img, np.array([kp['x'], kp['y']], dtype=np.float64)


# ---------------------------------------------------------------------------
# Keypoint tracking via FFPP (FlowFormer++) service
# ---------------------------------------------------------------------------


def _numpy_to_data_url(image_bgr: np.ndarray) -> str:
    """BGR numpy → ``data:image/png;base64,...`` (PNG keeps everything lossless)."""
    from PIL import Image
    rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    pil = Image.fromarray(rgb)
    buf = io.BytesIO()
    pil.save(buf, format='PNG')
    return 'data:image/png;base64,' + base64.b64encode(buf.getvalue()).decode('utf-8')


def track_via_ffpp(target_bgr: np.ndarray, reference_name: str, ffpp_url: str) -> Optional[np.ndarray]:
    import requests
    payload = {
        'image_base64': _numpy_to_data_url(target_bgr),
        'reference_name': reference_name,
        'bidirectional': True,
        'return_flow': False,
    }
    try:
        r = requests.post(f'{ffpp_url.rstrip("/")}/track_keypoints',
                          json=payload, timeout=60)
        r.raise_for_status()
        body = r.json()
    except Exception as e:
        print(f'  ! FFPP request failed for {reference_name}: {e}', file=sys.stderr)
        return None
    if not body.get('success'):
        print(f'  ! FFPP returned failure for {reference_name}: {body.get("message")}', file=sys.stderr)
        return None
    tracked = body.get('result', {}).get('tracked_keypoints', [])
    if not tracked:
        print(f'  ! No tracked keypoints returned for {reference_name}', file=sys.stderr)
        return None
    kp = tracked[0]
    # Different versions return either {'x', 'y'} or [x, y]
    if isinstance(kp, dict):
        return np.array([kp['x'], kp['y']], dtype=np.float64)
    return np.array([kp[0], kp[1]], dtype=np.float64)


def gather_keypoints(views: List[ViewCalib], ffpp_url: str,
                     override: Optional[Dict[str, List[float]]] = None
                     ) -> List[Optional[np.ndarray]]:
    """Return per-view detected 2D keypoint position (or None if tracking failed).

    ``override`` lets the caller pass a JSON dict keyed by reference_name to
    bypass the live FFPP service entirely.
    """
    out: List[Optional[np.ndarray]] = []
    for v in views:
        if override is not None and v.reference_name in override:
            uv = override[v.reference_name]
            out.append(np.array(uv, dtype=np.float64))
            print(f'  [override] {v.reference_name:<18} kp=({uv[0]:.2f}, {uv[1]:.2f})')
            continue
        target = cv2.imread(str(v.image_path))
        kp = track_via_ffpp(target, v.reference_name, ffpp_url)
        if kp is not None:
            print(f'  [tracked]  {v.reference_name:<18} '
                  f'image={v.image_path.name}  kp=({kp[0]:.2f}, {kp[1]:.2f})')
        out.append(kp)
    return out


# ---------------------------------------------------------------------------
# Forward kinematics used by the solver
# ---------------------------------------------------------------------------


def project_points(R_local: np.ndarray, t_local: np.ndarray,
                   X_local: np.ndarray, view: ViewCalib) -> np.ndarray:
    """Project a single rack-local point through one view.

    ``X_world = R_local @ X_local + t_local``, ``X_cam = R_wc @ X_world + t_wc``.
    """
    R_wc = view.extrinsic[:3, :3]
    t_wc = view.extrinsic[:3, 3]
    R_comb = R_wc @ R_local
    t_comb = (R_wc @ t_local) + t_wc
    rvec_comb, _ = cv2.Rodrigues(R_comb)
    projected, _ = cv2.projectPoints(
        X_local.reshape(1, 3).astype(np.float64),
        rvec_comb,
        t_comb.reshape(3, 1),
        view.intrinsic,
        view.distortion,
    )
    return projected.reshape(2)


def build_residual_fn(views: List[ViewCalib], keypoints: List[Optional[np.ndarray]],
                      template: np.ndarray):
    """Return scipy least_squares residual fn that minimises pixel reprojection error."""
    obs = []  # (view_idx, point_idx, uv)
    for vidx, (v, uv) in enumerate(zip(views, keypoints)):
        if uv is None:
            continue
        obs.append((vidx, vidx, uv))  # one corner per view, matched diagonally

    def residual(x: np.ndarray) -> np.ndarray:
        rvec = x[:3]
        tvec = x[3:6]
        R_local, _ = cv2.Rodrigues(rvec.reshape(3, 1))
        res = []
        for vidx, pidx, uv in obs:
            proj = project_points(R_local, tvec, template[pidx], views[vidx])
            res.extend((uv - proj).tolist())
        return np.array(res, dtype=np.float64)

    return residual, obs


def build_point_to_ray_residual(views: List[ViewCalib],
                                keypoints: List[Optional[np.ndarray]],
                                template: np.ndarray):
    """Coarse (point-to-ray) residual used by the production fitter as its first stage."""
    rays = []  # list of (point_idx, O_world, v_world)
    for vidx, (v, uv) in enumerate(zip(views, keypoints)):
        if uv is None:
            continue
        K = v.intrinsic
        dist = v.distortion
        R_wc = v.extrinsic[:3, :3]
        t_wc = v.extrinsic[:3, 3]
        O = -R_wc.T @ t_wc
        p_arr = np.array([[uv]], dtype=np.float32)
        normalised = cv2.undistortPoints(p_arr, K, dist, P=None).reshape(2)
        d_cam = np.array([normalised[0], normalised[1], 1.0]); d_cam /= np.linalg.norm(d_cam)
        d_world = R_wc.T @ d_cam; d_world /= np.linalg.norm(d_world)
        rays.append((vidx, O, d_world))

    def residual(x: np.ndarray) -> np.ndarray:
        rvec = x[:3]
        tvec = x[3:6]
        R_local, _ = cv2.Rodrigues(rvec.reshape(3, 1))
        res = []
        for pidx, O, dw in rays:
            X_world = R_local @ template[pidx] + tvec
            diff = X_world - O
            perp = diff - dw * float(dw @ diff)
            res.extend(perp.tolist())
        return np.array(res, dtype=np.float64)

    return residual, rays


# ---------------------------------------------------------------------------
# Reproduce the production fitter for the four-view-diagonal case
# ---------------------------------------------------------------------------


def run_production_fitter(views: List[ViewCalib], keypoints: List[Optional[np.ndarray]],
                          template: np.ndarray) -> Dict:
    """Call ``fitting_multiview`` exactly as the live service does."""
    view_data = []
    for v, uv in zip(views, keypoints):
        # One-corner-per-view → use the diagonal pattern (None for non-matched)
        pts_2d = [None] * len(template)
        if uv is not None:
            idx = next(i for i, (rn, _) in enumerate(RACK_CORNERS) if rn == v.reference_name)
            pts_2d[idx] = np.array(uv, dtype=np.float64)
        view_data.append({
            'points_2d': pts_2d,
            'image_size': v.image_size,
            'intrinsic': v.intrinsic,
            'distortion': v.distortion,
            'extrinsic': v.extrinsic,
        })
    target = [np.asarray(p, dtype=np.float64) for p in template]
    return fitting_multiview(view_data, target)


# ---------------------------------------------------------------------------
# Degeneracy probe: optimisation from many initial rotations
# ---------------------------------------------------------------------------


def _seed_initial_rotations() -> List[Tuple[str, np.ndarray]]:
    """24-rotation grid spanning axis swaps + small perturbations."""
    seeds: List[Tuple[str, np.ndarray]] = []
    seeds.append(('I', np.eye(3)))
    for axis, label in [('X', 'X'), ('Y', 'Y'), ('Z', 'Z')]:
        for ang_deg in (-180, -90, 90, 180):
            rot = R.from_euler(axis, ang_deg, degrees=True).as_matrix()
            seeds.append((f'R{label}({ang_deg:+4d}°)', rot))
    # face rotations of a cube (intercardinal): rotate by 90° about composite axes
    for a, b, deg_a, deg_b in [
        ('X', 'Y',  90,  90),
        ('X', 'Y', -90,  90),
        ('Y', 'Z',  90,  90),
        ('Z', 'X',  90,  90),
        ('X', 'Z',  90, -90),
        ('Y', 'X',  90, -90),
    ]:
        rot = R.from_euler(a, deg_a, degrees=True).as_matrix() @ \
              R.from_euler(b, deg_b, degrees=True).as_matrix()
        seeds.append((f'R{a}({deg_a:+d}°)·R{b}({deg_b:+d}°)', rot))
    return seeds


def _initial_translation(views: List[ViewCalib], keypoints: List[Optional[np.ndarray]],
                         template: np.ndarray, R_init: np.ndarray) -> np.ndarray:
    """Average over closest-point-on-ray to the rotated model centroid."""
    centroid = template.mean(axis=0)
    closest = []
    for v, uv in zip(views, keypoints):
        if uv is None:
            continue
        K = v.intrinsic
        dist = v.distortion
        R_wc = v.extrinsic[:3, :3]
        t_wc = v.extrinsic[:3, 3]
        O = -R_wc.T @ t_wc
        p_arr = np.array([[uv]], dtype=np.float32)
        normalised = cv2.undistortPoints(p_arr, K, dist, P=None).reshape(2)
        d_cam = np.array([normalised[0], normalised[1], 1.0]); d_cam /= np.linalg.norm(d_cam)
        d_world = R_wc.T @ d_cam; d_world /= np.linalg.norm(d_world)
        w = (R_init @ centroid) - O
        s = float(np.dot(w, d_world))
        closest.append(O + s * d_world)
    if not closest:
        return np.zeros(3)
    mean_p = np.mean(np.vstack(closest), axis=0)
    return mean_p - R_init @ centroid


def probe_degeneracy(views: List[ViewCalib], keypoints: List[Optional[np.ndarray]],
                     template: np.ndarray, label: str) -> List[Dict]:
    """Run the pixel-reprojection optimiser from a grid of initial rotations.

    Returns a list of local minima keyed by their final cost; clusters are
    inferred a posteriori from rotation-vector proximity.
    """
    residual, obs = build_residual_fn(views, keypoints, template)
    seeds = _seed_initial_rotations()
    results: List[Dict] = []
    for name, R0 in seeds:
        t0 = _initial_translation(views, keypoints, template, R0)
        rvec0, _ = cv2.Rodrigues(R0)
        x0 = np.hstack([rvec0.flatten(), t0.flatten()])
        try:
            sol = least_squares(residual, x0, method='lm', max_nfev=800)
        except Exception as e:
            results.append({'seed': name, 'cost': float('inf'),
                            'message': f'failed: {e}'})
            continue
        rvec_f = sol.x[:3]
        tvec_f = sol.x[3:6]
        R_f, _ = cv2.Rodrigues(rvec_f.reshape(3, 1))
        residuals_f = residual(sol.x)
        mean_px = float(np.sqrt((residuals_f.reshape(-1, 2) ** 2).sum(axis=1).mean()))
        results.append({
            'seed': name,
            'R_init': R0.tolist(),
            't_init': t0.tolist(),
            'cost': float(sol.cost),
            'mean_pixel_error': mean_px,
            'R_final': R_f.tolist(),
            't_final': tvec_f.tolist(),
            'rvec_final': rvec_f.tolist(),
            'success': bool(sol.success),
        })

    # Cluster by rounded rotation vector
    clusters: Dict[Tuple[float, float, float, float, float, float], List[Dict]] = {}
    for r in results:
        if 'rvec_final' not in r:
            continue
        key = tuple(np.round(r['rvec_final'], 1).tolist() + np.round(r['t_final'], 1).tolist())
        clusters.setdefault(key, []).append(r)

    print(f'\n--- Degeneracy probe ({label}): {len(seeds)} seeds → {len(clusters)} distinct minima ---')
    sorted_clusters = sorted(clusters.values(), key=lambda lst: lst[0]['mean_pixel_error'])
    for ci, cluster in enumerate(sorted_clusters):
        rep = cluster[0]
        seeds_str = ', '.join(c['seed'] for c in cluster)
        print(f'  Cluster {ci}: '
              f'mean_err={rep["mean_pixel_error"]:7.3f} px  '
              f'cost={rep["cost"]:.3e}  '
              f'rvec=({rep["rvec_final"][0]:+.3f}, {rep["rvec_final"][1]:+.3f}, {rep["rvec_final"][2]:+.3f})  '
              f't=({rep["t_final"][0]:+.3f}, {rep["t_final"][1]:+.3f}, {rep["t_final"][2]:+.3f})')
        print(f'    seeded by: {seeds_str}')

    return results


# ---------------------------------------------------------------------------
# Reporting helpers
# ---------------------------------------------------------------------------


def report_fit(label: str, views: List[ViewCalib],
               keypoints: List[Optional[np.ndarray]],
               template: np.ndarray, result: Dict) -> None:
    print(f'\n=== Production fitter — {label} ===')
    if not result.get('success'):
        print('  FAILED:', result.get('error_message'))
        return
    l2w = np.array(result['local2world'])
    print('  local2world (rack → base):')
    for row in l2w:
        print('   ', '  '.join(f'{c:+9.5f}' for c in row))
    print('  world points (= rack corners in base frame):')
    for i, p in enumerate(result['points_3d']):
        name = RACK_CORNERS[i][0]
        print(f'    [{i}] {name:<18} ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})')
    rep = result['reprojection_errors']
    flat = [e for row in rep for e in row if e is not None]
    if flat:
        print(f'  mean_pixel_error: {float(np.mean(flat)):.3f} px')
        print('  per-view per-point error (only matched cells shown):')
        for vidx, row in enumerate(rep):
            for pidx, e in enumerate(row):
                if e is not None:
                    print(f'    view {vidx} ({views[vidx].reference_name:<18})  → '
                          f'point {pidx} ({RACK_CORNERS[pidx][0]:<18})  '
                          f'err={e:.3f} px')


def report_template_geometry() -> None:
    print('\n=== Template geometry (rack-local) ===')
    for (ref, _), p in zip(RACK_CORNERS, TEMPLATE_POINTS_LOCAL):
        print(f'  {ref:<18} local = ({p[0]:+.3f}, {p[1]:+.3f}, {p[2]:+.3f})')
    print('  → 4 corners in a single Y_local=0 plane: width X=0.55 m, height Z=2.145 m')


# ---------------------------------------------------------------------------
# Pure-NumPy generalized P3P (GP3P) solver
# ---------------------------------------------------------------------------
#
# Reference: Nistér & Stewenius, "A Minimal Solution to the Generalised
# 3-Point Pose Problem" (CVPR 2007); Kneip et al., "Using Multi-Camera Systems
# in Robotics: Efficient Solutions to the NPnP Problem" (ICRA 2013).
#
# Problem statement.  Given 3 model points X_i (i = 0, 1, 2) in a *local*
# frame and 3 rays in the *world* frame, where ray i is parameterised as
# ``O_i + λ_i d_i`` with O_i the camera centre, d_i a unit direction and
# λ_i the unknown positive depth, find a rigid transform (R, t) such that
#
#     R · X_i + t = O_i + λ_i · d_i        for i = 0, 1, 2.
#
# The transform is rigid ⇒ inter-point distances are invariant ⇒
#
#     ‖p_i − p_j‖² = ‖X_i − X_j‖²           for all (i, j)
#
# with p_i := O_i + λ_i d_i.  This gives three quadratic equations in
# (λ_0, λ_1, λ_2); by Bézout's theorem at most 2·2·2 = 8 complex solutions
# exist.  We isolate λ_0 by:
#
#   • Treating ``f_{01}`` as a quadratic in λ_1 (with coefficients
#     polynomial in λ_0) ⇒ two real branches λ_1±(λ_0).
#   • Treating ``f_{02}`` as a quadratic in λ_2 ⇒ two real branches
#     λ_2±(λ_0).
#   • Plugging into ``f_{12}``, each of the four branch combinations
#     yields a univariate scalar ``h_{k_1,k_2}(λ_0)``.  Sign changes of
#     ``h`` along a dense grid over [0, λ_max] are then refined with
#     Brent's method.
#
# This finds *all* real roots in [0, λ_max] without any initial guess.


def _gp3p_depths(O: np.ndarray, d: np.ndarray, X: np.ndarray,
                 lam_max: float = 20.0, n_grid: int = 400) -> List[Tuple[float, float, float]]:
    """Return every (λ_0, λ_1, λ_2) with all-positive depths satisfying the
    three rigid-distance constraints for the 3 (point, ray) correspondences.

    For NOISELESS data this returns exact algebraic roots (sign changes of
    each branch's univariate residual ``h_{k1,k2}(λ_0)``).  For NOISY data
    the algebraic system may be infeasible (no real roots); we then also
    harvest the *local minima* of ``|h_{k1,k2}(λ_0)|`` on each branch as
    approximate candidates.  This gives the downstream LM optimiser a
    well-placed initial guess even when the three pairwise-distance
    constraints are slightly inconsistent due to keypoint noise.

    Args:
        O: ``(3, 3)`` — camera centres in world frame.
        d: ``(3, 3)`` — unit ray directions in world frame.
        X: ``(3, 3)`` — model points in local (template) frame.
        lam_max: search upper bound for depths (metres).  20 m is
            comfortably larger than any expected camera→target distance.
        n_grid: number of grid steps over [0, lam_max] used to bracket
            zeros / local minima of the branch residual.

    Returns:
        List of ``(λ_0, λ_1, λ_2)`` triples (positive depths).  Empty only
        if no valid depth window exists on any branch.
    """
    D01_sq = float(np.dot(X[0] - X[1], X[0] - X[1]))
    D02_sq = float(np.dot(X[0] - X[2], X[0] - X[2]))
    D12_sq = float(np.dot(X[1] - X[2], X[1] - X[2]))

    dot01 = float(np.dot(d[0], d[1]))
    dot02 = float(np.dot(d[0], d[2]))
    dot12 = float(np.dot(d[1], d[2]))

    delta01 = O[0] - O[1]
    delta02 = O[0] - O[2]
    delta12 = O[1] - O[2]

    sq01 = float(np.dot(delta01, delta01))
    sq02 = float(np.dot(delta02, delta02))
    sq12 = float(np.dot(delta12, delta12))

    delta01_d0 = float(np.dot(delta01, d[0]))
    delta01_d1 = float(np.dot(delta01, d[1]))
    delta02_d0 = float(np.dot(delta02, d[0]))
    delta02_d2 = float(np.dot(delta02, d[2]))
    delta12_d1 = float(np.dot(delta12, d[1]))
    delta12_d2 = float(np.dot(delta12, d[2]))

    def lam1_branches(lam0: float) -> Optional[Tuple[float, float]]:
        # f_01(lam0, lam1) = 0 as a quadratic in lam1:
        #   lam1² + B·lam1 + C = 0
        #   B = -2 lam0 (d0·d1) - 2 (O0-O1)·d1
        #   C = lam0² + 2 lam0 (O0-O1)·d0 + |O0-O1|² - |X0-X1|²
        B = -2.0 * lam0 * dot01 - 2.0 * delta01_d1
        C = lam0 * lam0 + 2.0 * lam0 * delta01_d0 + sq01 - D01_sq
        disc = B * B - 4.0 * C
        if disc < 0.0:
            return None
        s = float(np.sqrt(disc))
        return ((-B + s) * 0.5, (-B - s) * 0.5)

    def lam2_branches(lam0: float) -> Optional[Tuple[float, float]]:
        B = -2.0 * lam0 * dot02 - 2.0 * delta02_d2
        C = lam0 * lam0 + 2.0 * lam0 * delta02_d0 + sq02 - D02_sq
        disc = B * B - 4.0 * C
        if disc < 0.0:
            return None
        s = float(np.sqrt(disc))
        return ((-B + s) * 0.5, (-B - s) * 0.5)

    def f12(lam1: float, lam2: float) -> float:
        return (lam1 * lam1 + lam2 * lam2
                - 2.0 * lam1 * lam2 * dot12
                + 2.0 * (lam1 * delta12_d1 - lam2 * delta12_d2)
                + sq12 - D12_sq)

    def h_branch(lam0: float, k1: int, k2: int) -> float:
        l1s = lam1_branches(lam0)
        l2s = lam2_branches(lam0)
        if l1s is None or l2s is None:
            return float('nan')
        return f12(l1s[k1], l2s[k2])

    def make_triple(lam0: float, k1: int, k2: int) -> Optional[Tuple[float, float, float]]:
        l1s = lam1_branches(lam0)
        l2s = lam2_branches(lam0)
        if l1s is None or l2s is None:
            return None
        lam1, lam2 = l1s[k1], l2s[k2]
        if lam0 < 0.0 or lam1 < 0.0 or lam2 < 0.0:
            return None
        return (float(lam0), float(lam1), float(lam2))

    grid = np.linspace(0.0, lam_max, n_grid + 1)
    candidates: List[Tuple[float, float, float]] = []

    for k1 in (0, 1):
        for k2 in (0, 1):
            vals = np.array([h_branch(g, k1, k2) for g in grid])
            valid = np.isfinite(vals)

            # (a) Real roots — sign changes of h on the branch.
            for i in range(len(grid) - 1):
                if not (valid[i] and valid[i + 1]):
                    continue
                a, b = vals[i], vals[i + 1]
                if a * b > 0:
                    continue
                if a == 0.0:
                    lam0 = float(grid[i])
                else:
                    try:
                        lam0 = brentq(lambda l: h_branch(l, k1, k2),
                                      grid[i], grid[i + 1], xtol=1e-9)
                    except (ValueError, RuntimeError):
                        continue
                triple = make_triple(lam0, k1, k2)
                if triple is not None:
                    candidates.append(triple)

            # (b) Local minima of |h|² — robust to noise (no real root).
            # Identify contiguous valid windows and scan for interior minima.
            i = 0
            n = len(grid)
            while i < n:
                if not valid[i]:
                    i += 1
                    continue
                j = i
                while j + 1 < n and valid[j + 1]:
                    j += 1
                # Valid window [i, j].
                if j - i >= 2:
                    abs_vals = np.abs(vals[i:j + 1])
                    for k in range(1, len(abs_vals) - 1):
                        if abs_vals[k] < abs_vals[k - 1] and abs_vals[k] < abs_vals[k + 1]:
                            # Local minimum bracketed by (grid[i+k-1], grid[i+k+1]).
                            try:
                                opt = minimize_scalar(
                                    lambda l: h_branch(l, k1, k2) ** 2,
                                    bracket=(grid[i + k - 1], grid[i + k], grid[i + k + 1]),
                                    method='brent',
                                    options={'xtol': 1e-9},
                                )
                                lam0 = float(opt.x)
                            except (ValueError, RuntimeError):
                                lam0 = float(grid[i + k])
                            triple = make_triple(lam0, k1, k2)
                            if triple is not None:
                                candidates.append(triple)
                # (c) Window-edge candidates — value at the disc=0 boundary.
                # When both branches merge at the discriminant boundary, the
                # closest approach of the constraint surfaces often lies right
                # at the edge of the valid window.  Add the edge points.
                for edge in (i, j):
                    triple = make_triple(float(grid[edge]), k1, k2)
                    if triple is not None:
                        candidates.append(triple)
                i = j + 1

    # Deduplicate near-identical roots (branch crossings can produce duplicates)
    unique: List[Tuple[float, float, float]] = []
    for c in candidates:
        if all(abs(c[0] - u[0]) + abs(c[1] - u[1]) + abs(c[2] - u[2]) > 1e-3
               for u in unique):
            unique.append(c)
    return unique


def _rt_from_depths(O: np.ndarray, d: np.ndarray, X: np.ndarray,
                    lambdas: Tuple[float, float, float]) -> Tuple[np.ndarray, np.ndarray]:
    """Closed-form rigid transform (X local → world) via Umeyama after fixing depths."""
    p_world = np.array([O[i] + lambdas[i] * d[i] for i in range(3)], dtype=np.float64)
    _, R_, t_ = _umeyama(X, p_world, with_scale=False)
    return R_, t_


def _score_pose_point_to_ray(R_mat: np.ndarray, t_vec: np.ndarray,
                             model_pts: np.ndarray,
                             rays: List[Tuple[int, np.ndarray, np.ndarray]]) -> float:
    """Sum of squared point-to-ray perpendicular distances over the given rays."""
    score = 0.0
    for pi, O, dw in rays:
        Xw = R_mat @ model_pts[pi] + t_vec
        diff = Xw - O
        perp = diff - dw * float(dw @ diff)
        score += float(perp @ perp)
    return score


def gp3p_fit_diagonal(views: List['ViewCalib'],
                      keypoints: List[Optional[np.ndarray]],
                      template: np.ndarray) -> Dict:
    """GP3P + LM refine for the rack-localisation diagonal case.

    Each view contributes one ray (corner index = view index).  We run
    GP3P over every C(N, 3) triplet, score each candidate against the
    held-out point's ray, then refine the top-K candidates by minimising
    pixel reprojection error.  Returns the lowest-cost refined result.
    """
    from itertools import combinations

    # 1. Build per-point ray (O_world, d_world_unit).
    points_obs: Dict[int, Tuple[np.ndarray, np.ndarray]] = {}
    for vidx, (v, uv) in enumerate(zip(views, keypoints)):
        if uv is None:
            continue
        K = v.intrinsic
        dist = v.distortion
        R_wc = v.extrinsic[:3, :3]
        t_wc = v.extrinsic[:3, 3]
        O = -R_wc.T @ t_wc
        normalised = cv2.undistortPoints(
            np.array([[uv]], dtype=np.float32), K, dist, P=None).reshape(2)
        d_cam = np.array([normalised[0], normalised[1], 1.0], dtype=np.float64)
        d_cam /= np.linalg.norm(d_cam)
        d_world = R_wc.T @ d_cam
        d_world /= np.linalg.norm(d_world)
        # Diagonal: corner index = view index
        points_obs[vidx] = (O.astype(np.float64), d_world.astype(np.float64))

    obs_indices = sorted(points_obs.keys())
    if len(obs_indices) < 3:
        return {'success': False, 'error_message': 'Need ≥3 observations for GP3P'}

    rays_for_score = [(pi, points_obs[pi][0], points_obs[pi][1])
                      for pi in obs_indices]

    # 2. Enumerate triplets; GP3P each; score every candidate.
    all_candidates: List[Dict] = []
    for triplet in combinations(obs_indices, 3):
        O_arr = np.array([points_obs[pi][0] for pi in triplet])
        d_arr = np.array([points_obs[pi][1] for pi in triplet])
        X_arr = np.array([template[pi] for pi in triplet])
        depths_list = _gp3p_depths(O_arr, d_arr, X_arr)
        for lambdas in depths_list:
            R_cand, t_cand = _rt_from_depths(O_arr, d_arr, X_arr, lambdas)
            score = _score_pose_point_to_ray(R_cand, t_cand, template, rays_for_score)
            all_candidates.append({
                'triplet': triplet,
                'lambdas': lambdas,
                'R': R_cand,
                't': t_cand,
                'score_point_to_ray': score,
            })

    if not all_candidates:
        return {'success': False, 'error_message': 'GP3P found no real solutions'}

    # 3. Refine the top-K candidates by minimising pixel reprojection error.
    all_candidates.sort(key=lambda c: c['score_point_to_ray'])
    refine_residual, _ = build_residual_fn(views, keypoints, template)

    K_refine = min(20, len(all_candidates))
    best: Optional[Dict] = None
    refined_attempts: List[Dict] = []
    for cand in all_candidates[:K_refine]:
        rvec0, _ = cv2.Rodrigues(cand['R'])
        x0 = np.hstack([rvec0.flatten(), cand['t']])
        try:
            sol = least_squares(refine_residual, x0, method='lm', max_nfev=400)
        except Exception:
            continue
        res = refine_residual(sol.x)
        mean_px = float(np.sqrt((res.reshape(-1, 2) ** 2).sum(axis=1).mean()))
        R_f, _ = cv2.Rodrigues(sol.x[:3].reshape(3, 1))
        attempt = {
            'mean_pixel_error': mean_px,
            'cost': float(sol.cost),
            'R': R_f,
            't': sol.x[3:6].copy(),
            'triplet': cand['triplet'],
            'lambdas': cand['lambdas'],
            'pre_refine_score': cand['score_point_to_ray'],
        }
        refined_attempts.append(attempt)
        if best is None or mean_px < best['mean_pixel_error']:
            best = attempt

    if best is None:
        return {'success': False, 'error_message': 'All LM refinements failed'}

    return {
        'success': True,
        'mean_pixel_error': best['mean_pixel_error'],
        'cost': best['cost'],
        'R': best['R'],
        't': best['t'],
        'triplet': best['triplet'],
        'lambdas': best['lambdas'],
        'candidate_score_pre_refine': best['pre_refine_score'],
        'num_candidates': len(all_candidates),
        'num_triplets_tried': sum(1 for _ in combinations(obs_indices, 3)),
        'num_refined': len(refined_attempts),
    }


def report_gp3p(label: str, views: List['ViewCalib'], template: np.ndarray,
                result: Dict) -> None:
    print(f'\n--- GP3P + LM refine ({label}) ---')
    if not result.get('success'):
        print(f'  FAILED: {result.get("error_message")}')
        return
    print(f'  Tried {result["num_triplets_tried"]} triplets → '
          f'{result["num_candidates"]} closed-form candidates → '
          f'{result["num_refined"]} refined')
    print(f'  Best triplet = {result["triplet"]},  '
          f'depths = ({result["lambdas"][0]:.3f}, '
          f'{result["lambdas"][1]:.3f}, {result["lambdas"][2]:.3f}) m')
    print(f'  Pre-refine point-to-ray score = {result["candidate_score_pre_refine"]:.4e}')
    print(f'  After LM refine: mean_pixel_error = {result["mean_pixel_error"]:.3f} px')
    print('  Rack corners in base frame:')
    for i, X_loc in enumerate(template):
        X_w = result['R'] @ X_loc + result['t']
        print(f'    [{i}] {RACK_CORNERS[i][0]:<18} '
              f'({X_w[0]:+.3f}, {X_w[1]:+.3f}, {X_w[2]:+.3f})')


# ---------------------------------------------------------------------------
# Demonstrate the proposed fix: multi-start coarse → refine pipeline
# ---------------------------------------------------------------------------


def multi_start_fit(views: List[ViewCalib], keypoints: List[Optional[np.ndarray]],
                    template: np.ndarray) -> Dict:
    """Run the production two-stage pipeline (coarse point-to-ray → reprojection refine)
    from every seed in :func:`_seed_initial_rotations`, then keep the lowest-cost
    refined result.  This is exactly what the production fitter does but
    multi-started over many initial rotations.
    """
    coarse_residual, _ = build_point_to_ray_residual(views, keypoints, template)
    refine_residual, _ = build_residual_fn(views, keypoints, template)

    best: Optional[Dict] = None
    attempts = []
    for name, R0 in _seed_initial_rotations():
        t0 = _initial_translation(views, keypoints, template, R0)
        rvec0, _ = cv2.Rodrigues(R0)
        x0 = np.hstack([rvec0.flatten(), t0.flatten()])
        try:
            sol_c = least_squares(coarse_residual, x0, method='lm', max_nfev=200)
            sol_r = least_squares(refine_residual, sol_c.x, loss='soft_l1',
                                  f_scale=1.0, max_nfev=500)
        except Exception:
            continue
        res = refine_residual(sol_r.x)
        mean_px = float(np.sqrt((res.reshape(-1, 2) ** 2).sum(axis=1).mean()))
        record = {
            'seed': name,
            'mean_pixel_error': mean_px,
            'cost': float(sol_r.cost),
            'rvec_final': sol_r.x[:3].tolist(),
            't_final': sol_r.x[3:6].tolist(),
        }
        attempts.append(record)
        if best is None or mean_px < best['mean_pixel_error']:
            best = record
    return {
        'attempts': attempts,
        'best_seed': best['seed'] if best else None,
        'best_mean_error_px': best['mean_pixel_error'] if best else float('inf'),
        'best_rvec': best['rvec_final'] if best else None,
        'best_t': best['t_final'] if best else None,
    }


def report_multistart(label: str, views: List[ViewCalib], template: np.ndarray,
                      mstart: Dict) -> None:
    print(f'\n--- Multi-start two-stage pipeline ({label}) ---')
    print(f'  Tried {len(mstart["attempts"])} seeds → best mean error = '
          f'{mstart["best_mean_error_px"]:.3f} px (seed = {mstart["best_seed"]!r})')
    if mstart['best_rvec'] is None:
        return
    R_best, _ = cv2.Rodrigues(np.array(mstart['best_rvec']).reshape(3, 1))
    t_best = np.array(mstart['best_t'])
    print('  Best world points (rack corners in base frame):')
    for i, X_loc in enumerate(template):
        X_w = R_best @ X_loc + t_best
        print(f'    [{i}] {RACK_CORNERS[i][0]:<18} ({X_w[0]:+.3f}, {X_w[1]:+.3f}, {X_w[2]:+.3f})')


# ---------------------------------------------------------------------------
# Main
# ---------------------------------------------------------------------------


def main(argv: Optional[List[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__,
                                     formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--ffpp-url', default='http://10.172.100.39:8101',
                        help='URL of the FFPP keypoint-tracking service.')
    parser.add_argument('--only', choices=('bad', 'good', 'both'), default='both',
                        help='Which run(s) to analyse.')
    parser.add_argument('--keypoints', type=str, default=None,
                        help='Optional JSON file with manual keypoint overrides; '
                             'format: {"bad": {"rack_top_right": [u, v], ...}, "good": {...}}')
    parser.add_argument('--out', type=str, default=None,
                        help='Optional path to dump the full diagnostic JSON.')
    args = parser.parse_args(argv)

    override_data = {}
    if args.keypoints and os.path.isfile(args.keypoints):
        with open(args.keypoints, 'r') as f:
            override_data = json.load(f)

    report_template_geometry()

    diag: Dict[str, Dict] = {}

    runs_to_run = ['bad', 'good'] if args.only == 'both' else [args.only]
    for run_label in runs_to_run:
        session, wf_path = RUNS[run_label]
        print('\n' + '=' * 80)
        print(f'RUN: {run_label}  (session={session})')
        print('=' * 80)
        views = load_views(session)
        for v in views:
            cam_center = -v.extrinsic[:3, :3].T @ v.extrinsic[:3, 3]
            print(f'  view {v.reference_name:<18}  image={v.image_path.name}  '
                  f'size={v.image_size}  '
                  f'cam_center_base=({cam_center[0]:+.3f}, {cam_center[1]:+.3f}, {cam_center[2]:+.3f})')

        print(f'\n  Re-tracking keypoints via FFPP ({args.ffpp_url}):')
        keypoints = gather_keypoints(views, args.ffpp_url,
                                     override=override_data.get(run_label))
        if any(kp is None for kp in keypoints):
            print(f'  WARNING: some keypoints could not be tracked. '
                  f'You can supply them manually via --keypoints '
                  f'(see docstring for format).')

        # 1) production fit
        result = run_production_fitter(views, keypoints, TEMPLATE_POINTS_LOCAL)
        report_fit(run_label, views, keypoints, TEMPLATE_POINTS_LOCAL, result)

        # 2) compare to the workflow-result JSON
        wf = json.loads(Path(REPO_ROOT / wf_path).read_text())
        wf_pr = wf['results']['get_result_fitting']['outputs']['positioning_result']
        print(f'\n  Workflow result (recorded at runtime): '
              f'mean_error={wf_pr["mean_error"]:.3f} px')

        # 3) degeneracy probe
        probe = probe_degeneracy(views, keypoints, TEMPLATE_POINTS_LOCAL, run_label)

        # 4) demonstrate the proposed fix: multi-start initialisation
        mstart = multi_start_fit(views, keypoints, TEMPLATE_POINTS_LOCAL)
        report_multistart(run_label, views, TEMPLATE_POINTS_LOCAL, mstart)

        # 5) GP3P + LM refine (closed-form initial guess)
        gp3p_result = gp3p_fit_diagonal(views, keypoints, TEMPLATE_POINTS_LOCAL)
        report_gp3p(run_label, views, TEMPLATE_POINTS_LOCAL, gp3p_result)

        diag[run_label] = {
            'session': session,
            'keypoints': [kp.tolist() if kp is not None else None for kp in keypoints],
            'production_result': result,
            'workflow_recorded_mean_error': wf_pr['mean_error'],
            'workflow_recorded_points_3d': wf_pr['points_3d'],
            'probe': probe,
            'multistart_best_mean_error_px': mstart['best_mean_error_px'],
            'multistart_best_seed': mstart['best_seed'],
            'gp3p_success': gp3p_result.get('success', False),
            'gp3p_mean_error_px': gp3p_result.get('mean_pixel_error'),
            'gp3p_R': gp3p_result.get('R').tolist() if gp3p_result.get('success') else None,
            'gp3p_t': gp3p_result.get('t').tolist() if gp3p_result.get('success') else None,
            'gp3p_num_candidates': gp3p_result.get('num_candidates'),
            'gp3p_triplet': list(gp3p_result.get('triplet', [])) if gp3p_result.get('success') else None,
        }

    # Side-by-side summary
    if len(runs_to_run) == 2:
        print('\n' + '=' * 80)
        print('SIDE-BY-SIDE SUMMARY')
        print('=' * 80)
        bad_min = min((p['mean_pixel_error'] for p in diag['bad']['probe']),  default=float('inf'))
        good_min = min((p['mean_pixel_error'] for p in diag['good']['probe']), default=float('inf'))
        bad_prod_err = diag['bad']['production_result'].get('mean_error') \
            if isinstance(diag['bad']['production_result'].get('mean_error'), float) \
            else _flat_mean(diag['bad']['production_result'].get('reprojection_errors', []))
        good_prod_err = diag['good']['production_result'].get('mean_error') \
            if isinstance(diag['good']['production_result'].get('mean_error'), float) \
            else _flat_mean(diag['good']['production_result'].get('reprojection_errors', []))
        print(f'  BAD  run: production mean_err={bad_prod_err:.3f} px, '
              f'best-of-probe={bad_min:.3f} px, recorded={diag["bad"]["workflow_recorded_mean_error"]:.3f} px')
        print(f'  GOOD run: production mean_err={good_prod_err:.3f} px, '
              f'best-of-probe={good_min:.3f} px, recorded={diag["good"]["workflow_recorded_mean_error"]:.3f} px')
        print('\n  Interpretation:')
        if bad_min < 5 and bad_prod_err > 50:
            print('    >>> The bad run\'s OPTIMIZER got stuck in a local minimum.')
            print('    >>> A better initialisation exists that drives the cost down to '
                  f'{bad_min:.2f} px.')
            print('    >>> This is a genuine solver degeneracy with 4 single-view rays.')
        elif bad_min > 50:
            print('    >>> No initial guess found a low-cost minimum: at least one of the')
            print('    >>> tracked 2D keypoints is geometrically inconsistent with the rest.')
        else:
            print('    >>> See per-cluster output above for details.')

    if args.out:
        out_path = Path(args.out)
        out_path.write_text(json.dumps(diag, indent=2, default=_to_jsonable))
        print(f'\nDiagnostic JSON written to: {out_path}')

    return 0


def _flat_mean(rep) -> float:
    flat = []
    for row in rep:
        for e in row:
            if e is not None:
                flat.append(e)
    return float(np.mean(flat)) if flat else float('nan')


def _to_jsonable(o):
    if isinstance(o, np.ndarray):
        return o.tolist()
    if isinstance(o, (np.float32, np.float64)):
        return float(o)
    if isinstance(o, (np.int32, np.int64)):
        return int(o)
    raise TypeError(f'Not JSON serialisable: {type(o)}')


if __name__ == '__main__':
    sys.exit(main())
