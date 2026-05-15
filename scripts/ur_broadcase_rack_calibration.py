#!/usr/bin/env python3
"""Broadcast a freshly calibrated rack pose to all other robots.

When two (or more) UR arms share the same physical chessboard during
hand-eye calibration, each robot independently learns where the *target*
sits in **its own** base frame (``target2base_matrix``). The rack,
however, is normally calibrated only on one arm at a time. Because the
chessboard is a shared world anchor, the rack pose for any other robot
can be derived without re-running the rack calibration on that arm.

Given a "base" namespace whose ``rack2base_matrix`` is current, and any
other namespace ``ns`` that already has a ``target2base_matrix``, the
rack pose in that other base frame is::

    rack2base_ns = target2base_ns @ inv(target2base_base) @ rack2base_base

i.e. ``target2base_ns2 * base2target_ns1 * rack2base_ns1``.

The same change-of-basis is applied to ``rack_points_3d`` (an ``Nx3``
array of points expressed in the base namespace's base frame): with
``T = target2base_ns @ inv(target2base_base)``, every point
``p`` becomes ``(T @ [p; 1])[:3]`` in the target namespace's base frame.
If the base namespace exposes ``rack_points_3d``, the broadcaster
propagates it alongside ``rack2base_matrix``.

The :class:`RackCalibrationBroadcaster` class encapsulates this so that
the same logic can be reused from other scripts/services (e.g. the
dashboard could call it right after a successful rack calibration).
This module also exposes a small CLI for ad-hoc use::

    # Propagate ur15's rack2base to every other namespace that exposes
    # a target2base_matrix.
    python3 scripts/ur_broadcase_rack_calibration.py --base-namespace ur15

    # Dry-run: compute and print, but do not write back.
    python3 scripts/ur_broadcase_rack_calibration.py -b ur15 --dry-run

    # Limit broadcast to specific namespaces.
    python3 scripts/ur_broadcase_rack_calibration.py -b ur15 -n ur10e
"""

from __future__ import annotations

import argparse
import sys
from typing import Dict, Iterable, List, Optional

import numpy as np

from robot_status_redis.client_utils import RobotStatusClient


DEFAULT_TARGET_KEY = 'target2base_matrix'
DEFAULT_RACK_KEY = 'rack2base_matrix'
DEFAULT_RACK_POINTS_KEY = 'rack_points_3d'


def _coerce_4x4(value) -> Optional[np.ndarray]:
    """Convert a robot_status value (list/tuple/ndarray) into a 4x4 float64 array.

    Returns ``None`` if the value is missing or cannot be coerced into a
    valid 4x4 matrix. Callers should treat ``None`` as "not available"
    and skip the corresponding namespace.
    """
    if value is None:
        return None
    try:
        arr = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if arr.shape != (4, 4):
        return None
    return arr


def _coerce_points_3d(value) -> Optional[np.ndarray]:
    """Convert a robot_status value into an ``(N, 3)`` float64 array.

    Returns ``None`` if the value is missing or cannot be coerced into a
    valid ``(N, 3)`` array with ``N >= 1``.
    """
    if value is None:
        return None
    try:
        arr = np.asarray(value, dtype=np.float64)
    except (TypeError, ValueError):
        return None
    if arr.ndim != 2 or arr.shape[1] != 3 or arr.shape[0] < 1:
        return None
    return arr


def _apply_transform_to_points(transform: np.ndarray, points: np.ndarray) -> np.ndarray:
    """Apply a 4x4 homogeneous transform to an ``(N, 3)`` array of points.

    Returns the transformed points as an ``(N, 3)`` float64 array.
    """
    homogeneous = np.hstack([points, np.ones((points.shape[0], 1), dtype=points.dtype)])
    transformed = homogeneous @ transform.T  # equivalent to (transform @ homogeneous.T).T
    return transformed[:, :3]


class RackCalibrationBroadcaster:
    """Propagate one robot's calibrated rack pose to all other robots.

    Parameters
    ----------
    base_namespace : str
        Namespace on the robot_status server whose ``rack2base_matrix``
        has just been calibrated and should be considered the source of
        truth.
    status_client : RobotStatusClient, optional
        Reuse an existing client (useful for integration tests or when
        the caller already holds one). A new :class:`RobotStatusClient`
        is constructed if omitted.
    target_key : str, optional
        Status key under which each robot's chessboard pose is stored.
        Defaults to ``'target2base_matrix'`` (what ``ur_cam_calibrate.py``
        writes after hand-eye calibration).
    rack_key : str, optional
        Status key under which the rack pose is stored. Defaults to
        ``'rack2base_matrix'``.
    rack_points_key : str, optional
        Status key under which the rack's corner points are stored
        (``(N, 3)`` array, in the robot's base frame). Defaults to
        ``'rack_points_3d'``. If the base namespace does not expose
        this key, the broadcaster propagates only ``rack_key`` and the
        per-namespace report's ``rack_points_3d`` field stays ``None``.

    Notes
    -----
    The base namespace must expose **both** ``rack2base_matrix`` and
    ``target2base_matrix`` — without the latter we have no shared
    world anchor to use as a bridge to the other robots.
    ``rack_points_3d`` is *optional*: when present in the base
    namespace, it is propagated alongside the matrix; when absent,
    the broadcast still succeeds with matrix-only.
    """

    def __init__(
        self,
        base_namespace: str,
        status_client: Optional[RobotStatusClient] = None,
        target_key: str = DEFAULT_TARGET_KEY,
        rack_key: str = DEFAULT_RACK_KEY,
        rack_points_key: str = DEFAULT_RACK_POINTS_KEY,
    ):
        if not base_namespace:
            raise ValueError("base_namespace must be a non-empty string")
        self.base_namespace = base_namespace
        self.target_key = target_key
        self.rack_key = rack_key
        self.rack_points_key = rack_points_key
        self.client = status_client if status_client is not None else RobotStatusClient()

        # Cached anchors for the base namespace, populated lazily by
        # ``_ensure_base_anchors`` so we read from Redis at most once
        # per broadcast call (or per compute_for_namespace call chain).
        self._base_rack2base: Optional[np.ndarray] = None
        self._base_base2target: Optional[np.ndarray] = None
        self._base_rack_points: Optional[np.ndarray] = None
        # rack_points is optional on the base, so distinguish "not
        # loaded yet" from "loaded and absent" with a separate flag.
        self._base_points_loaded: bool = False

    # ------------------------------------------------------------------
    # Internal helpers
    # ------------------------------------------------------------------
    def _ensure_base_anchors(self) -> None:
        """Load and cache the base namespace's rack2base, inv(target2base), and points."""
        if self._base_rack2base is None or self._base_base2target is None:
            rack = _coerce_4x4(self.client.get_status(self.base_namespace, self.rack_key))
            if rack is None:
                raise RuntimeError(
                    f"Base namespace '{self.base_namespace}' has no valid "
                    f"'{self.rack_key}' (expected 4x4)."
                )

            target = _coerce_4x4(self.client.get_status(self.base_namespace, self.target_key))
            if target is None:
                raise RuntimeError(
                    f"Base namespace '{self.base_namespace}' has no valid "
                    f"'{self.target_key}' (expected 4x4). Without it we cannot "
                    f"anchor other robots to the same chessboard."
                )

            try:
                base2target = np.linalg.inv(target)
            except np.linalg.LinAlgError as exc:
                raise RuntimeError(
                    f"'{self.target_key}' for '{self.base_namespace}' is singular "
                    f"and cannot be inverted: {exc}"
                ) from exc

            self._base_rack2base = rack
            self._base_base2target = base2target

        # rack_points_3d is optional — attempt once, remember the result
        # (even if absent or malformed) so we don't refetch every loop.
        if not self._base_points_loaded:
            self._base_rack_points = _coerce_points_3d(
                self.client.get_status(self.base_namespace, self.rack_points_key)
            )
            self._base_points_loaded = True

    def reset_cache(self) -> None:
        """Drop cached base anchors so the next call re-reads from Redis.

        Useful if the caller re-runs the base calibration in the same
        Python process and wants the broadcaster to pick up the new
        ``rack2base_matrix`` (and ``rack_points_3d``) without
        constructing a new instance.
        """
        self._base_rack2base = None
        self._base_base2target = None
        self._base_rack_points = None
        self._base_points_loaded = False

    # ------------------------------------------------------------------
    # Public API
    # ------------------------------------------------------------------
    def _ns_to_base_transform(self, namespace: str) -> Optional[np.ndarray]:
        """Return the 4x4 transform mapping base-namespace poses/points
        into ``namespace``'s base frame.

        This is ``T = target2base_ns @ inv(target2base_base)``. Returns
        ``None`` if ``namespace`` has no valid ``target2base_matrix``.
        """
        target_other = _coerce_4x4(self.client.get_status(namespace, self.target_key))
        if target_other is None:
            return None
        return target_other @ self._base_base2target

    def compute_for_namespace(self, namespace: str) -> Optional[np.ndarray]:
        """Compute ``rack2base_matrix`` for ``namespace`` without writing.

        Returns ``None`` if ``namespace`` has no valid ``target2base_matrix``
        (the only reason this method can fail per-namespace).

        Raises
        ------
        RuntimeError
            If the **base** namespace's anchors are missing/singular — the
            entire broadcast is unrecoverable in that case.
        """
        self._ensure_base_anchors()

        transform = self._ns_to_base_transform(namespace)
        if transform is None:
            return None

        # rack2base_ns = target2base_ns @ inv(target2base_base) @ rack2base_base
        return transform @ self._base_rack2base

    def compute_points_for_namespace(self, namespace: str) -> Optional[np.ndarray]:
        """Compute ``rack_points_3d`` for ``namespace`` without writing.

        Returns ``None`` if either:
        * the base namespace has no ``rack_points_3d``, or
        * ``namespace`` has no valid ``target2base_matrix``.
        """
        self._ensure_base_anchors()
        if self._base_rack_points is None:
            return None

        transform = self._ns_to_base_transform(namespace)
        if transform is None:
            return None

        return _apply_transform_to_points(transform, self._base_rack_points)

    def discover_target_namespaces(self) -> List[str]:
        """Return every namespace (other than the base) that has a target2base.

        The list is sorted for deterministic ordering in reports and tests.
        """
        all_status = self.client.list_status()
        return sorted(
            ns for ns, keys in all_status.items()
            if ns != self.base_namespace and self.target_key in keys
        )

    def broadcast(
        self,
        target_namespaces: Optional[Iterable[str]] = None,
        dry_run: bool = False,
    ) -> Dict[str, Dict[str, object]]:
        """Compute and (optionally) publish rack2base for each target namespace.

        Parameters
        ----------
        target_namespaces : iterable of str, optional
            Explicit list of namespaces to broadcast to. If omitted, the
            broadcaster auto-discovers every namespace on the server that
            has a ``target2base_matrix`` (excluding the base namespace).
        dry_run : bool, optional
            When True, compute the matrices but skip the ``set_status``
            write. Useful for inspecting what *would* be published.

        Returns
        -------
        dict
            ``{namespace: {'status': 'ok'|'skipped'|'failed',
                           'message': str,
                           'rack2base_matrix': np.ndarray | None,
                           'rack_points_3d': np.ndarray | None}}``

            ``rack_points_3d`` is ``None`` when the base namespace does
            not expose those points; otherwise it carries the propagated
            ``(N, 3)`` array.
        """
        self._ensure_base_anchors()

        if target_namespaces is None:
            namespaces_iter: Iterable[str] = self.discover_target_namespaces()
        else:
            namespaces_iter = list(target_namespaces)

        report: Dict[str, Dict[str, object]] = {}
        for ns in namespaces_iter:
            if ns == self.base_namespace:
                report[ns] = {
                    'status': 'skipped',
                    'message': 'same as base namespace',
                    'rack2base_matrix': None,
                    'rack_points_3d': None,
                }
                continue

            transform = self._ns_to_base_transform(ns)
            if transform is None:
                report[ns] = {
                    'status': 'skipped',
                    'message': f"no valid '{self.target_key}' in this namespace",
                    'rack2base_matrix': None,
                    'rack_points_3d': None,
                }
                continue

            matrix = transform @ self._base_rack2base
            points = (
                _apply_transform_to_points(transform, self._base_rack_points)
                if self._base_rack_points is not None
                else None
            )

            if dry_run:
                report[ns] = {
                    'status': 'ok',
                    'message': 'dry-run (not written)',
                    'rack2base_matrix': matrix,
                    'rack_points_3d': points,
                }
                continue

            matrix_ok = self.client.set_status(ns, self.rack_key, matrix)
            points_ok = True
            if points is not None:
                points_ok = self.client.set_status(ns, self.rack_points_key, points)

            if matrix_ok and points_ok:
                status = 'ok'
                if points is None:
                    msg = (
                        f"written (matrix only; base namespace has no "
                        f"'{self.rack_points_key}')"
                    )
                else:
                    msg = 'written'
            else:
                status = 'failed'
                problems = []
                if not matrix_ok:
                    problems.append(f"set_status('{self.rack_key}') returned False")
                if not points_ok:
                    problems.append(f"set_status('{self.rack_points_key}') returned False")
                msg = '; '.join(problems)

            report[ns] = {
                'status': status,
                'message': msg,
                'rack2base_matrix': matrix,
                'rack_points_3d': points,
            }

        return report


# ----------------------------------------------------------------------
# CLI
# ----------------------------------------------------------------------
def _format_matrix(matrix: np.ndarray) -> str:
    with np.printoptions(precision=6, suppress=True, linewidth=120):
        return str(matrix)


def _format_points(points: np.ndarray) -> str:
    with np.printoptions(precision=6, suppress=True, linewidth=120):
        return str(points)


def _parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description=(
            "Broadcast a freshly calibrated rack2base_matrix from one robot's "
            "namespace to every other robot on the robot_status server, using "
            "each robot's target2base_matrix as a shared world anchor."
        ),
        formatter_class=argparse.ArgumentDefaultsHelpFormatter,
    )
    parser.add_argument(
        '--base-namespace', '-b',
        required=True,
        help='Namespace on the robot_status server whose rack2base_matrix '
             'has just been calibrated and should be propagated.',
    )
    parser.add_argument(
        '--namespace', '-n',
        dest='target_namespaces',
        action='append',
        default=None,
        help='Limit broadcast to this target namespace (repeatable). '
             'Default: auto-discover every namespace that has a target2base_matrix.',
    )
    parser.add_argument(
        '--target-key',
        default=DEFAULT_TARGET_KEY,
        help='Status key holding the chessboard-in-base transform.',
    )
    parser.add_argument(
        '--rack-key',
        default=DEFAULT_RACK_KEY,
        help='Status key holding the rack-in-base transform.',
    )
    parser.add_argument(
        '--rack-points-key',
        default=DEFAULT_RACK_POINTS_KEY,
        help='Status key holding the rack corner points '
             '((N, 3) array in the robot\'s base frame). '
             'Propagated only when the base namespace exposes this key.',
    )
    parser.add_argument(
        '--dry-run',
        action='store_true',
        help='Compute and print the new matrices but do not write them back.',
    )
    return parser.parse_args()


def main() -> int:
    args = _parse_args()

    try:
        broadcaster = RackCalibrationBroadcaster(
            base_namespace=args.base_namespace,
            target_key=args.target_key,
            rack_key=args.rack_key,
            rack_points_key=args.rack_points_key,
        )
    except Exception as exc:  # ConnectionError, ImportError from RobotStatusClient
        print(f"ERROR: could not connect to robot_status: {exc}", file=sys.stderr)
        return 2

    try:
        report = broadcaster.broadcast(
            target_namespaces=args.target_namespaces,
            dry_run=args.dry_run,
        )
    except RuntimeError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    if not report:
        print(
            f"No other namespaces with '{args.target_key}' found on robot_status; "
            f"nothing to broadcast."
        )
        return 0

    print(
        f"Broadcasting rack2base from namespace '{args.base_namespace}' "
        f"({'dry-run' if args.dry_run else 'writing to robot_status'}):"
    )
    written = skipped = failed = 0
    for ns, info in report.items():
        status = info['status']
        marker = {'ok': '✓', 'skipped': '·', 'failed': '✗'}.get(status, '?')
        print(f"\n{marker} {ns}: {status} — {info['message']}")
        matrix = info['rack2base_matrix']
        if matrix is not None:
            print(f"  {args.rack_key}:")
            print(_format_matrix(matrix))
        points = info.get('rack_points_3d')
        if points is not None:
            print(f"  {args.rack_points_key}:")
            print(_format_points(points))
        if status == 'ok':
            written += 1
        elif status == 'skipped':
            skipped += 1
        else:
            failed += 1

    print(f"\nSummary: {written} written, {skipped} skipped, {failed} failed")
    return 0 if failed == 0 else 1


if __name__ == '__main__':
    sys.exit(main())
