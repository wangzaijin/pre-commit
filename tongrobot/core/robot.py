"""TongRobot — the main entry point for the SDK."""

from __future__ import annotations

from typing import Callable, Optional

import numpy as np

from tongrobot.core.config import ConfigLoader
from tongrobot.core.motion import MotionManager
from tongrobot.core.sensor import SensorManager
from tongrobot.core.state import StateManager
from tongrobot.transport.base import TransportBase
from tongrobot.types import (
    BaseState,
    CameraFrame,
    IMUData,
    JointState,
    LaserScan,
    RobotState,
    Transform,
)
from tongrobot.utils.logging import get_logger
from tongrobot.utils.rate import Rate


class TongRobot:
    """Hardware-agnostic robot interface.

    Usage
    -----
    >>> with TongRobot("configs/turtlebot3.yaml") as robot:
    ...     state = robot.get_state()
    ...     robot.set_base_velocity(0.1, 0.0)
    ...     robot.stop()
    """

    def __init__(self, config_path: str) -> None:
        self._loader = ConfigLoader(config_path)
        self._hw = self._loader.hardware
        self._logger = get_logger(__name__, self._loader.logging_config)
        self._transport: Optional[TransportBase] = None
        self._state_mgr: Optional[StateManager] = None
        self._sensor_mgr: Optional[SensorManager] = None
        self._motion_mgr: Optional[MotionManager] = None

    # -------------------------------------------------------------------------
    # Connection lifecycle
    # -------------------------------------------------------------------------

    def connect(self) -> None:
        """Connect to the robot using the transport specified in the config."""
        mode = self._loader.connection_config.get("mode", "local")

        if mode == "local":
            from tongrobot.transport.local import LocalTransport

            self._transport = LocalTransport()
        elif mode == "remote":
            from tongrobot.transport.grpc_transport import GRPCTransport

            self._transport = GRPCTransport()
        else:
            raise ValueError(
                f"Unknown connection mode '{mode}'. " "Valid values: 'local', 'remote'."
            )

        self._transport.connect(self._loader._data)

        self._state_mgr = StateManager(self._transport, self._hw)
        self._sensor_mgr = SensorManager(self._transport, self._hw)
        self._motion_mgr = MotionManager(
            self._transport, self._hw, self._loader.safety_config
        )

        self._logger.info("Connected to '%s' (mode: %s)", self._loader.robot_name, mode)

    def disconnect(self) -> None:
        """Stop the robot and cleanly disconnect."""
        if self._motion_mgr:
            try:
                self._motion_mgr.stop()
            except Exception:
                pass
        if self._transport:
            self._transport.disconnect()
            self._transport = None
        self._logger.info("Disconnected from '%s'", self._loader.robot_name)

    def is_connected(self) -> bool:
        return self._transport is not None and self._transport.is_connected()

    # Context manager ----------------------------------------------------------

    def __enter__(self) -> "TongRobot":
        self.connect()
        return self

    def __exit__(self, *_: object) -> None:
        self.disconnect()

    # -------------------------------------------------------------------------
    # Public API
    # -------------------------------------------------------------------------

    def get_state(self) -> RobotState:
        """Return the current full robot state."""
        return self._state_mgr.get_state()  # type: ignore[union-attr]

    def get_base_state(self) -> BaseState:
        """Return only the mobile base state."""
        return self._state_mgr.get_base_state()  # type: ignore[union-attr]

    def get_arm_state(self, arm_name: str) -> JointState:
        """Return joint state for the named arm."""
        return self._state_mgr.get_arm_state(arm_name)  # type: ignore[union-attr]

    def get_camera(self, camera_name: str) -> np.ndarray:
        """Return the latest image as a numpy array (H, W, C)."""
        return self._sensor_mgr.get_camera(camera_name)  # type: ignore[union-attr]

    def get_camera_frame(self, camera_name: str) -> CameraFrame:
        """Return the latest ``CameraFrame`` (image + timestamp + encoding)."""
        return self._sensor_mgr.get_camera_frame(camera_name)  # type: ignore[union-attr]

    def get_laser_scan(self, lidar_name: str) -> LaserScan:
        """Return the latest laser scan."""
        return self._sensor_mgr.get_laser_scan(lidar_name)  # type: ignore[union-attr]

    def get_imu(self, imu_name: str) -> IMUData:
        """Return the latest IMU reading."""
        return self._sensor_mgr.get_imu(imu_name)  # type: ignore[union-attr]

    def set_base_velocity(self, linear: float, angular: float) -> None:
        """Send a velocity command to the mobile base.

        Parameters
        ----------
        linear:
            Forward velocity in m/s.  Negative = backward.
        angular:
            Rotational velocity in rad/s.  Positive = counter-clockwise.
        """
        self._motion_mgr.set_base_velocity(linear, angular)  # type: ignore[union-attr]

    def set_joint_positions(self, arm_name: str, positions: np.ndarray) -> None:
        """Send joint position targets to the named arm."""
        self._motion_mgr.set_joint_positions(arm_name, positions)  # type: ignore[union-attr]

    def stop(self) -> None:
        """Send zero velocity to all actuators."""
        self._motion_mgr.stop()  # type: ignore[union-attr]

    def emergency_stop(self) -> None:
        """Immediately stop and block further commands until reset_estop()."""
        self._motion_mgr.emergency_stop()  # type: ignore[union-attr]

    def reset_estop(self) -> None:
        """Clear the emergency stop flag."""
        self._motion_mgr.reset_estop()  # type: ignore[union-attr]

    def get_transform(self, target_frame: str, source_frame: str) -> Transform:
        """Look up the TF2 transform from source_frame to target_frame."""
        return self._state_mgr.get_transform(target_frame, source_frame)  # type: ignore[union-attr]

    def create_rate(self, hz: float) -> Rate:
        """Return a :class:`Rate` object for timing control loops at *hz* Hz."""
        return Rate(hz)

    def on_camera(
        self,
        camera_name: str,
        callback: Callable[[CameraFrame], None],
        fps: int = 30,
    ) -> None:
        """Register a streaming callback for camera frames."""
        self._sensor_mgr.on_camera(camera_name, callback, fps)  # type: ignore[union-attr]
