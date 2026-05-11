#!/usr/bin/env python3
"""
Run rack calibration workflow and print the resulting transform + corner points.

Provides a reusable `RackCalibrator` class plus a CLI entry point. The class
is intended to be imported and used from other scripts/services.

The class:
  1. Spawns `ros2 run ur15_workflow run_workflow.py --config <workflow.json>`
     (the exact same command the dashboard's "Run Current Selected Workflow"
     button uses).
  2. After the workflow exits successfully, reads `rack2base_matrix` and
     `rack_points_3d` from the Redis-backed robot status service under the
     `ur15` namespace.

Usage (CLI):
    # Use a workflow file from temp/workflow_files/ (basename only)
    python3 scripts/ur_rack_calibration.py localize_rack_at_working_pos.json

    # Or pass an absolute path
    python3 scripts/ur_rack_calibration.py /abs/path/to/workflow.json

Usage (library):
    from ur_rack_calibration import RackCalibrator

    cal = RackCalibrator()
    cal.set_config('localize_rack_at_working_pos.json')
    cal.run()                       # spawns the ros2 workflow
    result = cal.get_result()       # {'rack2base_matrix': ndarray,
                                    #  'rack_points_3d':   ndarray}

Prerequisites:
    - UR15 bringup must already be running:
        ros2 launch robot_bringup ur15_bringup.py
    - Redis-backed robot_status service must be reachable (default: localhost:6379)
    - The colcon workspace must be sourced so `ros2 run ur15_workflow` resolves
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Optional

import numpy as np

# Make the colcon install tree importable so we can use RobotStatusClient
# without requiring the user to source setup.bash beforehand. The exact
# layout under colcon_ws/install/<pkg>/ varies by ROS distro / build flavour,
# so probe the common locations and fall back to a glob search.
_WORKSPACE_ROOT = Path(__file__).resolve().parent.parent
_INSTALL_ROOT = _WORKSPACE_ROOT / 'colcon_ws' / 'install' / 'robot_status_redis'
_CANDIDATE_SITES = [
    _INSTALL_ROOT / 'local' / 'lib' / 'python3.10' / 'dist-packages',
    _INSTALL_ROOT / 'lib' / 'python3.10' / 'site-packages',
    _INSTALL_ROOT / 'lib' / 'python3.10' / 'dist-packages',
]
for _site in _CANDIDATE_SITES:
    if (_site / 'robot_status_redis' / 'client_utils.py').exists():
        if str(_site) not in sys.path:
            sys.path.insert(0, str(_site))
        break
else:
    # Last-resort glob in case the layout differs
    for _candidate in _INSTALL_ROOT.rglob('robot_status_redis/client_utils.py'):
        _site = _candidate.parent.parent
        if str(_site) not in sys.path:
            sys.path.insert(0, str(_site))
        break

from robot_status_redis.client_utils import RobotStatusClient  # noqa: E402


WORKFLOW_DIR = _WORKSPACE_ROOT / 'temp' / 'workflow_files'
STATUS_NAMESPACE = 'ur15'
RESULT_KEYS = ('rack2base_matrix', 'rack_points_3d')


def resolve_workflow_path(workflow_arg: str) -> Path:
    """Resolve a workflow file argument to an absolute path.

    Accepts either a full path or a basename (looked up under
    temp/workflow_files/). Adds the .json suffix if missing.
    """
    path = Path(workflow_arg)
    if not path.suffix:
        path = path.with_suffix('.json')

    if path.is_absolute() or path.exists():
        return path.resolve()

    return (WORKFLOW_DIR / path).resolve()


def format_matrix(value) -> str:
    """Render the stored matrix in a readable form.

    Handles numpy arrays, nested Python lists, and JSON-string fallbacks.
    """
    if isinstance(value, str):
        try:
            value = json.loads(value)
        except json.JSONDecodeError:
            return value

    try:
        arr = np.asarray(value, dtype=float)
    except (TypeError, ValueError):
        return repr(value)

    # Pretty-print with fixed precision and aligned columns
    with np.printoptions(precision=6, suppress=True, linewidth=120):
        return str(arr)


class RackCalibrator:
    """Run a rack-calibration workflow and retrieve the resulting state.

    The workflow is executed via `ros2 run ur15_workflow run_workflow.py
    --config <path>` — the same code path used by the dashboard's "Run
    Workflow" button. After successful execution the 4x4 `rack2base_matrix`
    and the four `rack_points_3d` are read from the Redis-backed robot
    status service.

    Typical use:
        cal = RackCalibrator()
        cal.set_config('localize_rack_at_working_pos.json')
        cal.run()
        result = cal.get_result()
        matrix = result['rack2base_matrix']
        points = result['rack_points_3d']
    """

    def __init__(
        self,
        config: Optional[str] = None,
        namespace: str = STATUS_NAMESPACE,
        result_keys: tuple = RESULT_KEYS,
        status_client: Optional[RobotStatusClient] = None,
    ):
        self._namespace = namespace
        self._result_keys = tuple(result_keys)
        self._status_client = status_client
        self._workflow_path: Optional[Path] = None
        self._last_returncode: Optional[int] = None
        if config is not None:
            self.set_config(config)

    # ------------------------------------------------------------------ config
    def set_config(self, config: str) -> Path:
        """Set the workflow configuration file.

        Accepts a basename under `temp/workflow_files/` or an absolute path.
        Returns the resolved absolute path.

        Raises:
            FileNotFoundError: if the resolved file does not exist.
        """
        path = resolve_workflow_path(config)
        if not path.exists():
            raise FileNotFoundError(f"Workflow file not found: {path}")
        self._workflow_path = path
        return path

    @property
    def config(self) -> Optional[Path]:
        """Currently configured workflow file (absolute path) or None."""
        return self._workflow_path

    # ------------------------------------------------------------------- run
    def run(self) -> int:
        """Execute the configured workflow synchronously.

        Streams the workflow's stdout/stderr to the current terminal.

        Returns:
            The subprocess return code (0 on success).

        Raises:
            RuntimeError: if no config has been set.
            FileNotFoundError: if the `ros2` CLI is not on PATH.
        """
        if self._workflow_path is None:
            raise RuntimeError(
                "No workflow config set. Call set_config(...) first."
            )

        cmd = [
            'ros2', 'run', 'ur15_workflow', 'run_workflow.py',
            '--config', str(self._workflow_path),
        ]
        print(f">>> Running: {' '.join(cmd)}\n")
        try:
            result = subprocess.run(cmd, check=False)
        except FileNotFoundError:
            raise FileNotFoundError(
                "`ros2` not found on PATH. Source your ROS 2 environment first:\n"
                "    source /opt/ros/humble/setup.bash\n"
                "    source ~/Documents/robot_dc/colcon_ws/install/setup.bash"
            )
        self._last_returncode = result.returncode
        return result.returncode

    @property
    def last_returncode(self) -> Optional[int]:
        """Return code from the most recent `run()` call, or None."""
        return self._last_returncode

    # --------------------------------------------------------------- results
    @staticmethod
    def _normalize(value):
        """Best-effort conversion of a stored status value to numpy.ndarray.

        Returns the raw value if it cannot be coerced.
        """
        if value is None:
            return None
        if isinstance(value, str):
            try:
                value = json.loads(value)
            except json.JSONDecodeError:
                return value
        try:
            return np.asarray(value, dtype=float)
        except (TypeError, ValueError):
            return value

    def get_result(self) -> dict:
        """Fetch the latest calibration result from robot status.

        Returns:
            Dict with one entry per configured `result_keys`, e.g.::

                {
                    'rack2base_matrix': 4x4 numpy.ndarray or None,
                    'rack_points_3d':   numpy.ndarray or None,
                }

            A value is None if the corresponding key is not stored under
            the configured namespace.

        Raises:
            ConnectionError: if the Redis-backed status service is unreachable.
        """
        if self._status_client is None:
            self._status_client = RobotStatusClient()

        result = {}
        for key in self._result_keys:
            value = self._status_client.get_status(self._namespace, key)
            result[key] = self._normalize(value)
        return result

    def calibrate(self) -> Optional[dict]:
        """Convenience: run the workflow and return the result dict in one call.

        Raises the same errors as `run()` and `get_result()`. Returns None
        if the workflow exited non-zero.
        """
        rc = self.run()
        if rc != 0:
            return None
        return self.get_result()


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run a rack-calibration workflow via `ros2 run ur15_workflow` "
            "and print the resulting rack2base_matrix and rack_points_3d "
            "from robot status."
        )
    )
    parser.add_argument(
        'workflow',
        help=(
            "Workflow file: basename under temp/workflow_files/ "
            "(e.g. localize_rack_at_working_pos.json) or an absolute path."
        ),
    )
    args = parser.parse_args()

    cal = RackCalibrator()
    try:
        cal.set_config(args.workflow)
    except FileNotFoundError as exc:
        print(f"✗ {exc}", file=sys.stderr)
        return 2

    try:
        rc = cal.run()
    except FileNotFoundError as exc:
        print(f"✗ {exc}", file=sys.stderr)
        return 127

    if rc != 0:
        print(f"\n✗ Workflow exited with non-zero status: {rc}", file=sys.stderr)
        return rc

    print("\n>>> Workflow finished. Reading calibration result from robot status...")
    try:
        result = cal.get_result()
    except ConnectionError as exc:
        print(f"✗ Cannot reach robot status (Redis): {exc}", file=sys.stderr)
        return 3

    matrix = result.get('rack2base_matrix')
    if matrix is None:
        print(
            f"✗ 'rack2base_matrix' not found under namespace '{STATUS_NAMESPACE}'.\n"
            "  The workflow may have failed to compute a result.",
            file=sys.stderr,
        )
        return 4

    for key, value in result.items():
        print(f"\n{key} (namespace='{STATUS_NAMESPACE}'):")
        if value is None:
            print("  <not stored>")
        else:
            print(format_matrix(value))
    return 0


if __name__ == '__main__':
    sys.exit(main())
