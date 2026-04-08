from __future__ import annotations
from dataclasses import dataclass
from typing import Dict, Optional
import numpy as np


@dataclass
class JointState:
    positions: np.ndarray
    velocities: np.ndarray
    efforts: np.ndarray
    timestamp: float


@dataclass
class BaseState:
    # shape (3,): x, y, theta
    position: np.ndarray
    # shape (3,): vx, vy, omega
    velocity: np.ndarray
    timestamp: float


@dataclass
class GripperState:
    position: float
    is_open: bool
    timestamp: float


@dataclass
class RobotState:
    arms: Dict[str, JointState]
    base: Optional[BaseState]
    grippers: Dict[str, GripperState]
    timestamp: float


@dataclass
class Pose:
    # shape (3,)
    position: np.ndarray
    # shape (4,) in xyzw order
    quaternion: np.ndarray


@dataclass
class Transform:
    # shape (3,)
    position: np.ndarray
    # shape (4,) in xyzw order
    quaternion: np.ndarray

    @property
    def matrix(self) -> np.ndarray:
        """Return the 4x4 homogeneous transform matrix."""
        from tongrobot.utils.transforms import pose_to_matrix, Pose as _Pose

        return pose_to_matrix(_Pose(position=self.position, quaternion=self.quaternion))
