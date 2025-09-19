import math
import numpy as np


def quat_to_rot_matrix(qx, qy, qz, qw) -> np.ndarray:
    """Convert quaternion to rotation matrix"""
    # Normalize
    n = math.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    if n < 1e-12:
        return np.eye(3)
    x, y, z, w = qx/n, qy/n, qz/n, qw/n
    xx, yy, zz = x*x, y*y, z*z
    xy, xz, yz = x*y, x*z, y*z
    wx, wy, wz = w*x, w*y, w*z
    R = np.array([
        [1 - 2*(yy + zz), 2*(xy - wz),   2*(xz + wy)],
        [2*(xy + wz),     1 - 2*(xx+zz), 2*(yz - wx)],
        [2*(xz - wy),     2*(yz + wx),   1 - 2*(xx+yy)]
    ], dtype=float)
    return R