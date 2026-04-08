"""Pure-numpy SE3 / quaternion utilities."""

from __future__ import annotations

import numpy as np

from tongrobot.types.robot_state import Pose


def quaternion_to_matrix(q: np.ndarray) -> np.ndarray:
    """Convert a quaternion (xyzw) to a 3×3 rotation matrix.

    Parameters
    ----------
    q:
        Array of shape (4,) in xyzw order.

    Returns
    -------
    np.ndarray
        Shape (3, 3) rotation matrix.
    """
    x, y, z, w = q
    return np.array(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ],
        dtype=float,
    )


def matrix_to_quaternion(R: np.ndarray) -> np.ndarray:
    """Convert a 3×3 rotation matrix to a quaternion (xyzw).

    Uses Shepperd's method for numerical stability.

    Parameters
    ----------
    R:
        Shape (3, 3) rotation matrix.

    Returns
    -------
    np.ndarray
        Shape (4,) quaternion in xyzw order.
    """
    trace = R[0, 0] + R[1, 1] + R[2, 2]
    if trace > 0:
        s = 0.5 / np.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (R[2, 1] - R[1, 2]) * s
        y = (R[0, 2] - R[2, 0]) * s
        z = (R[1, 0] - R[0, 1]) * s
    elif R[0, 0] > R[1, 1] and R[0, 0] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2])
        w = (R[2, 1] - R[1, 2]) / s
        x = 0.25 * s
        y = (R[0, 1] + R[1, 0]) / s
        z = (R[0, 2] + R[2, 0]) / s
    elif R[1, 1] > R[2, 2]:
        s = 2.0 * np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2])
        w = (R[0, 2] - R[2, 0]) / s
        x = (R[0, 1] + R[1, 0]) / s
        y = 0.25 * s
        z = (R[1, 2] + R[2, 1]) / s
    else:
        s = 2.0 * np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1])
        w = (R[1, 0] - R[0, 1]) / s
        x = (R[0, 2] + R[2, 0]) / s
        y = (R[1, 2] + R[2, 1]) / s
        z = 0.25 * s
    return np.array([x, y, z, w], dtype=float)


def pose_to_matrix(pose: Pose) -> np.ndarray:
    """Convert a Pose to a 4×4 homogeneous transform matrix.

    Parameters
    ----------
    pose:
        Pose with position (3,) and quaternion (4,) in xyzw order.

    Returns
    -------
    np.ndarray
        Shape (4, 4) homogeneous matrix.
    """
    T = np.eye(4)
    T[:3, :3] = quaternion_to_matrix(pose.quaternion)
    T[:3, 3] = pose.position
    return T


def matrix_to_pose(matrix: np.ndarray) -> Pose:
    """Convert a 4×4 homogeneous matrix to a Pose.

    Parameters
    ----------
    matrix:
        Shape (4, 4) homogeneous transform matrix.

    Returns
    -------
    Pose
    """
    return Pose(
        position=matrix[:3, 3].copy(),
        quaternion=matrix_to_quaternion(matrix[:3, :3]),
    )
