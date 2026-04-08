"""Abstract transport interface."""

from __future__ import annotations

import abc
from typing import Any, Callable, Dict

from tongrobot.types import (
    CameraFrame,
    IMUData,
    JointCommand,
    LaserScan,
    RobotState,
    Transform,
    VelocityCommand,
)


class TransportBase(abc.ABC):
    """Abstract base class that every transport must implement.

    Transports bridge the SDK to the robot hardware.  Two concrete
    implementations exist:

    * ``LocalTransport`` – direct ROS2 node (on-robot usage)
    * ``gRPCTransport`` – connects over the network to the bridge node

    All methods that read sensor data return Python dataclasses with numpy
    arrays; no ROS2 types ever cross the SDK boundary.
    """

    @abc.abstractmethod
    def connect(self, config: Dict[str, Any]) -> None:
        """Establish a connection to the robot.

        Parameters
        ----------
        config:
            The parsed YAML config dict (the full top-level dict, not a
            sub-section) so the implementation can read any field it needs.
        """

    @abc.abstractmethod
    def disconnect(self) -> None:
        """Cleanly tear down the connection and release all resources."""

    @abc.abstractmethod
    def is_connected(self) -> bool:
        """Return True if the transport is currently connected."""

    # --- State ----------------------------------------------------------------

    @abc.abstractmethod
    def get_robot_state(self) -> RobotState:
        """Return a snapshot of the full robot state (joints, base, grippers).

        Returns
        -------
        RobotState
            Latest cached state.  The ``timestamp`` field indicates when the
            state was last updated.
        """

    # --- Sensors --------------------------------------------------------------

    @abc.abstractmethod
    def get_camera_frame(self, camera_name: str) -> CameraFrame:
        """Return the latest image from the named camera.

        Parameters
        ----------
        camera_name:
            Must match a camera name defined in the hardware config.

        Returns
        -------
        CameraFrame
            Latest image as a numpy array (H, W, C) uint8 in the configured
            encoding.
        """

    @abc.abstractmethod
    def get_laser_scan(self, lidar_name: str) -> LaserScan:
        """Return the latest scan from the named LiDAR.

        Parameters
        ----------
        lidar_name:
            Must match a lidar name defined in the hardware config.
        """

    @abc.abstractmethod
    def get_imu(self, imu_name: str) -> IMUData:
        """Return the latest IMU reading.

        Parameters
        ----------
        imu_name:
            Must match an IMU name defined in the hardware config.
        """

    # --- Commands -------------------------------------------------------------

    @abc.abstractmethod
    def set_velocity_command(self, cmd: VelocityCommand) -> None:
        """Send a velocity command to the mobile base.

        Values should already be clamped to hardware limits before calling
        this method (the MotionManager is responsible for clamping).

        Parameters
        ----------
        cmd:
            Desired linear and angular velocity.
        """

    @abc.abstractmethod
    def set_joint_command(self, arm_name: str, cmd: JointCommand) -> None:
        """Send a joint-space command to a named arm.

        Parameters
        ----------
        arm_name:
            Must match an arm name defined in the hardware config.
        cmd:
            Desired joint positions and/or velocities.
        """

    # --- Streaming ------------------------------------------------------------

    @abc.abstractmethod
    def subscribe_robot_state(self, callback: Callable[[RobotState], None]) -> None:
        """Register a callback invoked whenever the robot state is updated.

        Parameters
        ----------
        callback:
            Called with the new ``RobotState`` each time new data arrives.
        """

    @abc.abstractmethod
    def subscribe_camera(
        self,
        camera_name: str,
        callback: Callable[[CameraFrame], None],
        fps: int = 30,
    ) -> None:
        """Register a callback invoked at up to *fps* Hz with new camera frames.

        Parameters
        ----------
        camera_name:
            Must match a camera name defined in the hardware config.
        callback:
            Called with each new ``CameraFrame``.
        fps:
            Maximum delivery rate.
        """

    # --- TF -------------------------------------------------------------------

    @abc.abstractmethod
    def get_transform(self, target_frame: str, source_frame: str) -> Transform:
        """Look up a TF2 transform from *source_frame* to *target_frame*.

        Parameters
        ----------
        target_frame:
            The frame to transform into.
        source_frame:
            The frame to transform from.

        Returns
        -------
        Transform
            Latest known transform.
        """
