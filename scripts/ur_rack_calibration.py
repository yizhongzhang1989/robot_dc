#!/usr/bin/env python3
"""
Run rack calibration workflow and print the resulting transform + corner points.

Provides a reusable `RackCalibrator` class plus a CLI entry point. The class
is intended to be imported and used from other scripts/services.

The class:
  1. Spawns `ros2 run ur_workflow run_workflow.py --config <workflow.json>`
     (the exact same command the dashboard's "Run Current Selected Workflow"
     button uses).
  2. After the workflow exits successfully, reads `rack2base_matrix` and
     `rack_points_3d` from the Redis-backed robot status service. The
     namespace is auto-detected from the workflow JSON itself (the
     `positioning` step that writes `rack2base_matrix` carries
     `status_namespace`), so the same script works for both `ur15` and
     `ur10e` without any CLI flags. Pass ``namespace=...`` to the
     constructor to override the detection.

Usage (CLI):
    # Use a workflow file from temp/workflow_files/ (basename only)
    python3 scripts/ur_rack_calibration.py localize_rack_at_working_pos.json

    # Or pass an absolute path
    python3 scripts/ur_rack_calibration.py /abs/path/to/workflow.json

    # --config / -c is accepted as an alternative to the positional argument
    # (parity with `ros2 run ur_workflow run_workflow.py --config ...`)
    python3 scripts/ur_rack_calibration.py --config localize_rack_at_working_pos.json

    # Add --broadcast (or --broadcase) to also propagate the resulting
    # rack2base_matrix and rack_points_3d to every other namespace on the
    # robot_status server that has a target2base_matrix.
    python3 scripts/ur_rack_calibration.py --config localize_rack_at_working_pos.json --broadcast

Usage (library):
    from ur_rack_calibration import RackCalibrator

    cal = RackCalibrator()
    cal.set_config('localize_rack_at_working_pos.json')
    cal.run()                       # spawns the ros2 workflow
    result = cal.get_result()       # {'rack2base_matrix': ndarray,
                                    #  'rack_points_3d':   ndarray}
    cal.broadcast()                 # optional: propagate to every other
                                    # namespace that has a target2base_matrix

Prerequisites:
    - UR15 bringup must already be running:
        ros2 launch robot_bringup ur15_bringup.py
    - Redis-backed robot_status service must be reachable (default: localhost:6379)
    - The colcon workspace must be sourced so `ros2 run ur_workflow` resolves
"""

import argparse
import json
import subprocess
import sys
from pathlib import Path
from typing import Optional

import numpy as np

# Resolve the project root using the shared helper in the ``common`` package.
# Insert ``colcon_ws/src`` first so ``common.workspace_utils`` is importable
# even when the colcon overlay has not been sourced (development mode); if
# that still fails we fall back to a ``__file__``-based guess.
try:
    sys.path.insert(
        0,
        str(Path(__file__).resolve().parent.parent / 'colcon_ws' / 'src'),
    )
    from common.workspace_utils import get_workspace_root  # noqa: E402

    _detected_root = get_workspace_root()
    _WORKSPACE_ROOT = (
        Path(_detected_root) if _detected_root
        else Path(__file__).resolve().parent.parent
    )
except ImportError:
    _WORKSPACE_ROOT = Path(__file__).resolve().parent.parent

# Requires the colcon overlay to be sourced
# (``source colcon_ws/install/setup.bash``), which is already listed under
# Prerequisites in the module docstring above.
from robot_status_redis.client_utils import RobotStatusClient  # noqa: E402


WORKFLOW_DIR = _WORKSPACE_ROOT / 'temp' / 'workflow_files'
FALLBACK_STATUS_NAMESPACE = 'ur15'
RESULT_KEYS = ('rack2base_matrix', 'rack_points_3d')


def _detect_status_namespace(
    workflow_path: Path,
    fallback: str = FALLBACK_STATUS_NAMESPACE,
) -> Optional[str]:
    """Pull the robot_status_redis namespace out of a workflow JSON.

    Resolution order:
      1. The first ``positioning`` step whose ``local2world_matrix_name``
         is ``'rack2base_matrix'`` — that step's ``status_namespace`` is
         the one the workflow will write the rack matrix under, so it's
         also the one we must read it back from.
      2. The workflow's top-level ``context.status_namespace`` (rarely
         present but honoured for symmetry with the workflow runner).
      3. ``fallback`` (defaults to ``'ur15'``).

    Returns ``None`` only if the file cannot be read or parsed; in that
    case the caller should keep its existing namespace.
    """
    try:
        with open(workflow_path, 'r') as f:
            data = json.load(f)
    except (OSError, json.JSONDecodeError):
        return None

    if isinstance(data, dict):
        for step in data.get('workflow', []) or []:
            if not isinstance(step, dict):
                continue
            if step.get('type') != 'positioning':
                continue
            if step.get('local2world_matrix_name') != 'rack2base_matrix':
                continue
            ns = step.get('status_namespace')
            if ns:
                return ns

        ctx = data.get('context')
        if isinstance(ctx, dict):
            ns = ctx.get('status_namespace')
            if ns:
                return ns

    return fallback


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

    The workflow is executed via `ros2 run ur_workflow run_workflow.py
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
        namespace: Optional[str] = None,
        result_keys: tuple = RESULT_KEYS,
        status_client: Optional[RobotStatusClient] = None,
    ):
        # If the caller pins a namespace, ``set_config()`` will leave it
        # alone; otherwise we auto-detect from the workflow JSON.
        self._explicit_namespace = namespace
        self._namespace = namespace or FALLBACK_STATUS_NAMESPACE
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

        Side effect: unless the constructor was called with an explicit
        ``namespace=``, the ``status_namespace`` to read results back
        from is auto-detected from the workflow JSON (see
        :func:`_detect_status_namespace`).

        Raises:
            FileNotFoundError: if the resolved file does not exist.
        """
        path = resolve_workflow_path(config)
        if not path.exists():
            raise FileNotFoundError(f"Workflow file not found: {path}")
        self._workflow_path = path
        if self._explicit_namespace is None:
            detected = _detect_status_namespace(path)
            if detected:
                self._namespace = detected
        return path

    @property
    def config(self) -> Optional[Path]:
        """Currently configured workflow file (absolute path) or None."""
        return self._workflow_path

    @property
    def namespace(self) -> str:
        """Status-redis namespace this calibrator reads results from.

        Auto-detected from the workflow JSON on ``set_config()`` unless
        the constructor was called with an explicit ``namespace=``.
        """
        return self._namespace

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
            'ros2', 'run', 'ur_workflow', 'run_workflow.py',
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

    # ------------------------------------------------------------- broadcast
    def broadcast(
        self,
        target_namespaces=None,
        dry_run: bool = False,
    ) -> dict:
        """Propagate the freshly calibrated rack pose to other robots.

        Uses :class:`RackCalibrationBroadcaster` with ``self.namespace`` as
        the base. ``rack2base_matrix`` and (if present) ``rack_points_3d``
        on the base namespace are re-expressed in every other robot's
        base frame via the shared ``target2base_matrix`` anchor.

        Parameters
        ----------
        target_namespaces : iterable of str, optional
            Limit broadcast to these namespaces. Default: auto-discover.
        dry_run : bool, optional
            Compute but do not write back.

        Returns
        -------
        dict
            The per-namespace report from
            :meth:`RackCalibrationBroadcaster.broadcast`.
        """
        # Defer the import so the broadcaster is only loaded when needed,
        # and add the script directory to sys.path so library users who
        # import this module from anywhere can still reach its sibling.
        _scripts_dir = str(Path(__file__).resolve().parent)
        if _scripts_dir not in sys.path:
            sys.path.insert(0, _scripts_dir)
        from ur_broadcase_rack_calibration import RackCalibrationBroadcaster

        broadcaster = RackCalibrationBroadcaster(
            base_namespace=self._namespace,
            status_client=self._status_client,
        )
        return broadcaster.broadcast(
            target_namespaces=target_namespaces,
            dry_run=dry_run,
        )


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run a rack-calibration workflow via `ros2 run ur_workflow` "
            "and print the resulting rack2base_matrix and rack_points_3d "
            "from robot status."
        )
    )
    parser.add_argument(
        'workflow',
        nargs='?',
        default=None,
        help=(
            "Workflow file: basename under temp/workflow_files/ "
            "(e.g. localize_rack_at_working_pos.json) or an absolute path. "
            "May also be passed as --config for parity with the underlying "
            "`ros2 run ur_workflow run_workflow.py --config ...` command."
        ),
    )
    parser.add_argument(
        '--config', '-c',
        dest='config',
        default=None,
        help='Alternative way to pass the workflow file (same as the positional argument).',
    )
    # Accept both '--broadcast' (correct spelling) and '--broadcase' (matches
    # the sibling script filename) so either form works on the CLI.
    parser.add_argument(
        '--broadcast', '--broadcase',
        action='store_true',
        help=(
            "After a successful calibration, propagate rack2base_matrix "
            "(and rack_points_3d if present) from this robot's namespace "
            "to every other namespace on the robot_status server that has "
            "a target2base_matrix. See scripts/ur_broadcase_rack_calibration.py."
        ),
    )
    args = parser.parse_args()

    workflow_arg = args.workflow or args.config
    if not workflow_arg:
        parser.error(
            "a workflow file is required (pass it positionally or via --config/-c)"
        )

    cal = RackCalibrator()
    try:
        cal.set_config(workflow_arg)
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
            f"✗ 'rack2base_matrix' not found under namespace '{cal.namespace}'.\n"
            "  The workflow may have failed to compute a result.",
            file=sys.stderr,
        )
        return 4

    for key, value in result.items():
        print(f"\n{key} (namespace='{cal.namespace}'):")
        if value is None:
            print("  <not stored>")
        else:
            print(format_matrix(value))

    if args.broadcast:
        print(
            f"\n>>> Broadcasting rack calibration from '{cal.namespace}' to "
            f"every other namespace with a target2base_matrix..."
        )
        try:
            report = cal.broadcast()
        except RuntimeError as exc:
            print(f"✗ Broadcast failed: {exc}", file=sys.stderr)
            return 5
        except ConnectionError as exc:
            print(
                f"✗ Cannot reach robot status (Redis) during broadcast: {exc}",
                file=sys.stderr,
            )
            return 3

        if not report:
            print(
                "  No other namespaces with 'target2base_matrix' found; "
                "nothing to broadcast."
            )
        else:
            written = skipped = failed = 0
            for ns, info in report.items():
                status = info['status']
                marker = {'ok': '✓', 'skipped': '·', 'failed': '✗'}.get(status, '?')
                print(f"  {marker} {ns}: {status} — {info['message']}")
                if status == 'ok':
                    written += 1
                elif status == 'skipped':
                    skipped += 1
                else:
                    failed += 1
            print(
                f"  Summary: {written} written, {skipped} skipped, {failed} failed"
            )
            if failed:
                return 1

    return 0


if __name__ == '__main__':
    sys.exit(main())
