from __future__ import annotations
from dataclasses import dataclass
from typing import Optional
import numpy as np


@dataclass
class VelocityCommand:
    linear_x: float
    linear_y: float
    angular_z: float


@dataclass
class JointCommand:
    positions: Optional[np.ndarray]
    velocities: Optional[np.ndarray]
