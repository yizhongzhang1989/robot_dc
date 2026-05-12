#!/usr/bin/env python3
"""
Record and replay TCP poses defined in the rack coordinate system.

After rack calibration (Stage 3/4 of doc/rack_calibration.md) the matrix
``rack2base_matrix`` is stored in the Redis-backed robot status service
under namespace ``ur15``. This module lets you:

  1. Record the live TCP pose, expressed in the *rack* frame, under a
     user-chosen name. The saved pose is independent of where the robot
     base is parked — only the geometry "tool relative to rack" is stored.

  2. Move the TCP back to a saved rack-frame pose at any time. The current
     ``rack2base_matrix`` is used to translate the stored rack-frame pose
     into a base-frame target, which is then sent as a single ``movel``.

This is the script counterpart of the dashboard's **Localize TCP** panel:
both paths call this same class so behaviour is identical.

Usage (CLI):
    # Record current TCP pose, name it "approach_left_knob"
    python3 scripts/ur_tcp_localizer.py record approach_left_knob

    # Move robot to a previously saved pose
    python3 scripts/ur_tcp_localizer.py move approach_left_knob

    # List / delete saved poses
    python3 scripts/ur_tcp_localizer.py list
    python3 scripts/ur_tcp_localizer.py delete approach_left_knob

Usage (library):
    from ur_tcp_localizer import TCPLocalizer

    loc = TCPLocalizer()
    loc.record('approach_left_knob')
    ...
    loc.move_to('approach_left_knob')             # saved pose by name
    loc.move_to_pose_in_rack(tcp_in_rack_matrix)  # ad-hoc 4x4 matrix

Prerequisites:
    - UR15 bringup running and reachable on the network.
    - ``rack2base_matrix`` populated in robot status (run Stage 3 first).
    - The colcon workspace is sourced so ``ur_robot_arm`` and
      ``robot_status_redis`` can be imported.
"""

import argparse
import json
import os
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Optional

import numpy as np
import yaml
from scipy.spatial.transform import Rotation as R

# Make the colcon install tree importable so we can use RobotStatusClient
# and UR15Robot without requiring the user to source setup.bash beforehand.
_WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
for _pkg in ('robot_status_redis', 'ur_robot_arm'):
    _install_root = _WORKSPACE_ROOT / 'colcon_ws' / 'install' / _pkg
    _candidates = [
        _install_root / 'local' / 'lib' / 'python3.10' / 'dist-packages',
        _install_root / 'lib' / 'python3.10' / 'site-packages',
        _install_root / 'lib' / 'python3.10' / 'dist-packages',
    ]
    for _site in _candidates:
        if (_site / _pkg).is_dir():
            if str(_site) not in sys.path:
                sys.path.insert(0, str(_site))
            break
    else:
        for _candidate in _install_root.rglob(f'{_pkg}/__init__.py'):
            _site = _candidate.parent.parent
            if str(_site) not in sys.path:
                sys.path.insert(0, str(_site))
            break

from robot_status_redis.client_utils import RobotStatusClient  # noqa: E402

try:
    from ur_robot_arm.ur15 import UR15Robot  # noqa: E402
except ImportError as _exc:  # pragma: no cover
    UR15Robot = None
    _UR15_IMPORT_ERROR = _exc
else:
    _UR15_IMPORT_ERROR = None


POSES_DIR = _WORKSPACE_ROOT / 'temp' / 'tcp_poses'
STATUS_NAMESPACE = 'ur15'
RACK2BASE_KEY = 'rack2base_matrix'
DEFAULT_ROBOT_IP = '192.168.1.15'
DEFAULT_ROBOT_PORT = 30002


def _load_robot_config() -> dict:
    """Read robot IP / control port from ``config/robot_config.yaml``.

    Falls back to defaults if the file is missing or malformed.
    """
    defaults = {'robot_ip': DEFAULT_ROBOT_IP, 'robot_port': DEFAULT_ROBOT_PORT}
    config_path = _WORKSPACE_ROOT / 'config' / 'robot_config.yaml'
    if not config_path.exists():
        return defaults
    try:
        with open(config_path, 'r') as f:
            cfg = yaml.safe_load(f) or {}
        ur15 = cfg.get('ur15', {})
        return {
            'robot_ip': ur15.get('robot', {}).get('ip', defaults['robot_ip']),
            'robot_port': ur15.get('robot', {}).get('ports', {}).get(
                'control', defaults['robot_port']
            ),
        }
    except (OSError, yaml.YAMLError):
        return defaults


def _pose_to_matrix(tcp_pose):
    """Convert a 6-element TCP pose [x, y, z, rx, ry, rz] to a 4x4 matrix."""
    arr = np.asarray(tcp_pose, dtype=float).reshape(-1)
    if arr.size != 6:
        raise ValueError(f"Expected 6-element TCP pose, got shape {arr.shape}")
    matrix = np.eye(4)
    matrix[:3, :3] = R.from_rotvec(arr[3:6]).as_matrix()
    matrix[:3, 3] = arr[0:3]
    return matrix


def _matrix_to_pose(matrix) -> list:
    """Convert a 4x4 transformation matrix to a 6-element TCP pose."""
    arr = np.asarray(matrix, dtype=float)
    if arr.shape != (4, 4):
        raise ValueError(f"Expected 4x4 matrix, got shape {arr.shape}")
    rotvec = R.from_matrix(arr[:3, :3]).as_rotvec()
    return [float(arr[0, 3]), float(arr[1, 3]), float(arr[2, 3]),
            float(rotvec[0]), float(rotvec[1]), float(rotvec[2])]


def _validate_pose_name(name: str) -> str:
    """Reject names that would escape the poses directory or contain bad chars."""
    if not name:
        raise ValueError("Pose name cannot be empty.")
    stripped = name.strip()
    if stripped != name:
        raise ValueError("Pose name cannot have leading/trailing whitespace.")
    if any(c in name for c in ('/', '\\', '..', '\x00')):
        raise ValueError(
            f"Pose name '{name}' contains invalid characters (/, \\, .., NUL)."
        )
    if name in ('.', '..'):
        raise ValueError(f"Pose name '{name}' is reserved.")
    return name


class TCPLocalizer:
    """Record + replay TCP poses defined in the rack coordinate system.

    Storage layout: one JSON file per pose under ``temp/tcp_poses/<name>.json``::

        {
            "name": "approach_left_knob",
            "recorded_at": "2026-05-11T12:34:56+00:00",
            "tcp_in_rack": [[...], [...], [...], [...]],   # 4x4
            "tcp_in_base_at_record": [...],                # 6-elem [x,y,z,rx,ry,rz]
            "rack2base_at_record":    [[...], ...]         # 4x4
        }

    Only ``tcp_in_rack`` is required for playback; the other fields are kept
    for audit / debugging.

    The constructor accepts injected ``status_client`` and ``robot`` instances
    primarily for testing. In normal use both are created lazily on first need.
    """

    def __init__(
        self,
        poses_dir: Optional[os.PathLike] = None,
        robot_ip: Optional[str] = None,
        robot_port: Optional[int] = None,
        namespace: str = STATUS_NAMESPACE,
        rack2base_key: str = RACK2BASE_KEY,
        status_client: Optional[RobotStatusClient] = None,
        robot=None,
    ):
        self.poses_dir = Path(poses_dir) if poses_dir is not None else POSES_DIR
        self.poses_dir.mkdir(parents=True, exist_ok=True)

        cfg = _load_robot_config()
        self.robot_ip = robot_ip if robot_ip is not None else cfg['robot_ip']
        self.robot_port = robot_port if robot_port is not None else cfg['robot_port']

        self._namespace = namespace
        self._rack2base_key = rack2base_key
        self._status_client = status_client
        self._robot = robot
        self._owns_robot = robot is None  # close at end if we opened it

    # ---------------------------------------------------------- file helpers
    def pose_path(self, name: str) -> Path:
        """Return the absolute path of the JSON file backing a named pose."""
        _validate_pose_name(name)
        return self.poses_dir / f'{name}.json'

    def list_poses(self) -> list:
        """Return a sorted list of saved pose names (no .json suffix)."""
        return sorted(p.stem for p in self.poses_dir.glob('*.json'))

    def delete_pose(self, name: str) -> bool:
        """Delete a saved pose. Returns True if deleted, False if absent."""
        path = self.pose_path(name)
        if not path.exists():
            return False
        path.unlink()
        return True

    def get_pose_in_rack(self, name: str) -> np.ndarray:
        """Load the 4x4 ``tcp_in_rack`` matrix for the named pose."""
        path = self.pose_path(name)
        if not path.exists():
            raise FileNotFoundError(f"Pose '{name}' not found: {path}")
        with open(path, 'r') as f:
            data = json.load(f)
        matrix = np.asarray(data['tcp_in_rack'], dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError(
                f"Stored tcp_in_rack has invalid shape {matrix.shape} (expected 4x4)"
            )
        return matrix

    # ---------------------------------------------------- status / robot wiring
    def _get_status_client(self) -> RobotStatusClient:
        if self._status_client is None:
            self._status_client = RobotStatusClient()
        return self._status_client

    def _get_rack2base(self) -> np.ndarray:
        """Fetch the current ``rack2base_matrix`` from robot status."""
        client = self._get_status_client()
        value = client.get_status(self._namespace, self._rack2base_key)
        if value is None:
            raise RuntimeError(
                f"'{self._rack2base_key}' is not stored under namespace "
                f"'{self._namespace}'. Run rack calibration first "
                "(see doc/rack_calibration.md)."
            )
        if isinstance(value, str):
            value = json.loads(value)
        matrix = np.asarray(value, dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError(
                f"Stored rack2base_matrix has invalid shape {matrix.shape}"
            )
        return matrix

    def _get_robot(self):
        """Open a UR15Robot connection on first use."""
        if self._robot is not None:
            return self._robot
        if UR15Robot is None:
            raise RuntimeError(
                f"UR15Robot is not importable: {_UR15_IMPORT_ERROR}. "
                "Make sure the colcon workspace is sourced."
            )
        print(f"Connecting to UR15 at {self.robot_ip}:{self.robot_port}...")
        robot = UR15Robot(ip=self.robot_ip, port=self.robot_port)
        rc = robot.open()
        if rc != 0:
            raise ConnectionError(
                f"Failed to connect to UR15 at {self.robot_ip}:{self.robot_port} "
                f"(open() returned {rc})."
            )
        print("✓ UR15 connected")
        self._robot = robot
        return robot

    def close(self) -> None:
        """Close the UR15 connection if we opened it."""
        if self._robot is not None and self._owns_robot:
            try:
                close_fn = getattr(self._robot, 'close', None)
                if callable(close_fn):
                    close_fn()
            except Exception:
                pass
            self._robot = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    # -------------------------------------------------------------- record
    def record(self, name: str) -> Path:
        """Capture the live TCP pose, express it in rack frame, save as JSON.

        Returns the path of the saved file.
        Raises:
            ValueError: if ``name`` is invalid.
            RuntimeError: if ``rack2base_matrix`` is not in robot status.
            ConnectionError: if the UR15 robot cannot be reached.
        """
        _validate_pose_name(name)
        path = self.pose_path(name)

        rack2base = self._get_rack2base()
        robot = self._get_robot()

        tcp_in_base_pose = robot.get_actual_tcp_pose()
        if tcp_in_base_pose is None or len(tcp_in_base_pose) < 6:
            raise RuntimeError(
                "Failed to read TCP pose from UR15 "
                "(get_actual_tcp_pose returned no usable value)."
            )
        tcp_in_base_pose = [float(v) for v in tcp_in_base_pose[:6]]

        tcp_in_base = _pose_to_matrix(tcp_in_base_pose)
        # tcp_in_rack = inv(rack2base) @ tcp_in_base
        tcp_in_rack = np.linalg.inv(rack2base) @ tcp_in_base

        record = {
            'name': name,
            'recorded_at': datetime.now(timezone.utc).isoformat(timespec='seconds'),
            'tcp_in_rack': tcp_in_rack.tolist(),
            'tcp_in_base_at_record': tcp_in_base_pose,
            'rack2base_at_record': rack2base.tolist(),
        }
        with open(path, 'w') as f:
            json.dump(record, f, indent=2)

        print(f"✓ Recorded TCP pose '{name}' → {path}")
        with np.printoptions(precision=6, suppress=True):
            print("  tcp_in_rack (4x4):")
            print(np.asarray(tcp_in_rack))
        return path

    # -------------------------------------------------------------- move
    def move_to_pose_in_rack(self, tcp_in_rack, a: float = 0.1, v: float = 0.1,
                             label: str = '<inline>') -> int:
        """Move the TCP to a directly-given rack-frame pose.

        This is the core motion primitive. ``move_to(name)`` is a thin wrapper
        that loads the matrix from a saved JSON file and forwards here.

        Args:
            tcp_in_rack: 4x4 transformation matrix expressing the desired TCP
                pose in the rack coordinate frame. Accepts numpy.ndarray or
                any nested-list shape compatible with ``np.asarray``.
            a:     Linear acceleration for ``movel`` (m/s²).
            v:     Linear speed for ``movel`` (m/s).
            label: Optional label included in log output (used by ``move_to``
                to print the saved pose name).

        Returns:
            The return code from ``movel`` (0 on success).

        Raises:
            ValueError: if ``tcp_in_rack`` is not 4x4.
            RuntimeError: if ``rack2base_matrix`` is not in robot status.
            ConnectionError: if the UR15 robot cannot be reached.
        """
        matrix = np.asarray(tcp_in_rack, dtype=float)
        if matrix.shape != (4, 4):
            raise ValueError(
                f"tcp_in_rack must be a 4x4 matrix, got shape {matrix.shape}"
            )

        rack2base = self._get_rack2base()
        # tcp_in_base = rack2base @ tcp_in_rack  (current calibration)
        tcp_in_base = rack2base @ matrix
        target_pose = _matrix_to_pose(tcp_in_base)

        robot = self._get_robot()

        with np.printoptions(precision=6, suppress=True):
            print(f"Moving TCP to pose {label}")
            print(f"  target pose (base frame): {target_pose}")
            print(f"  speed: v={v} m/s, a={a} m/s²")

        rc = robot.movel(target_pose, a=a, v=v)
        if rc == 0:
            print(f"✓ movel completed (pose {label})")
        else:
            print(f"✗ movel failed with code {rc}")
        return rc

    def move_to(self, name: str, a: float = 0.1, v: float = 0.1) -> int:
        """Send a single ``movel`` driving the TCP to the named rack-frame pose.

        Args:
            name: Pose name (must already be recorded).
            a:    Linear acceleration for ``movel`` (m/s^2).
            v:    Linear speed for ``movel`` (m/s).

        Returns:
            The return code from ``movel`` (0 on success).

        Raises:
            FileNotFoundError: if the pose is not saved.
            RuntimeError: if ``rack2base_matrix`` is not in robot status.
            ConnectionError: if the UR15 robot cannot be reached.
        """
        tcp_in_rack = self.get_pose_in_rack(name)
        return self.move_to_pose_in_rack(
            tcp_in_rack, a=a, v=v, label=f"'{name}'"
        )


# =============================================================================
# CLI
# =============================================================================

def _cmd_record(args) -> int:
    with TCPLocalizer() as loc:
        try:
            loc.record(args.name)
        except (ValueError, FileNotFoundError) as exc:
            print(f"✗ {exc}", file=sys.stderr)
            return 2
        except RuntimeError as exc:
            print(f"✗ {exc}", file=sys.stderr)
            return 3
        except ConnectionError as exc:
            print(f"✗ {exc}", file=sys.stderr)
            return 4
    return 0


def _cmd_move(args) -> int:
    with TCPLocalizer() as loc:
        try:
            rc = loc.move_to(args.name, a=args.a, v=args.v)
        except (ValueError, FileNotFoundError) as exc:
            print(f"✗ {exc}", file=sys.stderr)
            return 2
        except RuntimeError as exc:
            print(f"✗ {exc}", file=sys.stderr)
            return 3
        except ConnectionError as exc:
            print(f"✗ {exc}", file=sys.stderr)
            return 4
    return rc if rc == 0 else 5


def _cmd_list(args) -> int:
    loc = TCPLocalizer()
    poses = loc.list_poses()
    if not poses:
        print(f"No saved poses under {loc.poses_dir}")
        return 0
    print(f"Saved TCP poses ({loc.poses_dir}):")
    for name in poses:
        print(f"  • {name}")
    return 0


def _cmd_delete(args) -> int:
    loc = TCPLocalizer()
    try:
        removed = loc.delete_pose(args.name)
    except ValueError as exc:
        print(f"✗ {exc}", file=sys.stderr)
        return 2
    if not removed:
        print(f"✗ Pose '{args.name}' does not exist.", file=sys.stderr)
        return 2
    print(f"✓ Deleted pose '{args.name}'")
    return 0


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Record / replay TCP poses defined in the rack coordinate system. "
            "Reads rack2base_matrix from robot status (Redis, namespace 'ur15')."
        )
    )
    sub = parser.add_subparsers(dest='command', required=True)

    p_record = sub.add_parser('record', help='Record current TCP pose under a name')
    p_record.add_argument('name', help='Name to save the pose under')
    p_record.set_defaults(func=_cmd_record)

    p_move = sub.add_parser('move', help='Move TCP to a saved pose')
    p_move.add_argument('name', help='Name of the saved pose')
    p_move.add_argument('--a', type=float, default=0.1,
                        help='Linear acceleration (m/s², default 0.1)')
    p_move.add_argument('--v', type=float, default=0.1,
                        help='Linear speed (m/s, default 0.1)')
    p_move.set_defaults(func=_cmd_move)

    p_list = sub.add_parser('list', help='List saved pose names')
    p_list.set_defaults(func=_cmd_list)

    p_delete = sub.add_parser('delete', help='Delete a saved pose')
    p_delete.add_argument('name', help='Name of the saved pose to delete')
    p_delete.set_defaults(func=_cmd_delete)

    args = parser.parse_args()
    return args.func(args)


if __name__ == '__main__':
    sys.exit(main())
