from dataclasses import dataclass
import numpy as np


@dataclass
class CameraFrame:
    # shape (H, W, C) uint8
    data: np.ndarray
    timestamp: float
    encoding: str


@dataclass
class LaserScan:
    ranges: np.ndarray
    angle_min: float
    angle_max: float
    angle_increment: float
    timestamp: float


@dataclass
class IMUData:
    # shape (4,) quaternion xyzw
    orientation: np.ndarray
    # shape (3,)
    angular_velocity: np.ndarray
    # shape (3,)
    linear_acceleration: np.ndarray
    timestamp: float
