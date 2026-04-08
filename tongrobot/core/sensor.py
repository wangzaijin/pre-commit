"""SensorManager — access to camera, LiDAR, and IMU data."""

from __future__ import annotations

from typing import Callable

import numpy as np

from tongrobot.core.config import HardwareDescriptor
from tongrobot.exceptions import ConfigError
from tongrobot.transport.base import TransportBase
from tongrobot.types import CameraFrame, IMUData, LaserScan


class SensorManager:
    """Thin wrapper around the transport for reading sensor data."""

    def __init__(self, transport: TransportBase, hardware: HardwareDescriptor) -> None:
        self._transport = transport
        self._hw = hardware

    def get_camera(self, camera_name: str) -> np.ndarray:
        """Return the latest image as a numpy array (H, W, C).

        Raises
        ------
        ConfigError
            If the camera name is not in the hardware descriptor.
        """
        self._hw.get_camera(camera_name)  # raises ConfigError if absent
        return self._transport.get_camera_frame(camera_name).data

    def get_camera_frame(self, camera_name: str) -> CameraFrame:
        """Return the latest ``CameraFrame`` (image + timestamp + encoding).

        Raises
        ------
        ConfigError
            If the camera name is not in the hardware descriptor.
        """
        self._hw.get_camera(camera_name)
        return self._transport.get_camera_frame(camera_name)

    def get_depth(self, camera_name: str) -> np.ndarray:
        """Return the latest depth image for a camera that has a depth topic.

        Raises
        ------
        ConfigError
            If no depth topic is configured for the camera.
        """
        cam = self._hw.get_camera(camera_name)
        if not cam.depth_topic:
            raise ConfigError(
                f"Camera '{camera_name}' has no depth_topic configured. "
                "Add a 'depth_topic' field to the camera config."
            )
        # Depth retrieval delegates to transport; would need a separate
        # get_depth_frame() method on TransportBase in a future iteration.
        raise NotImplementedError(
            "Depth image retrieval is not yet implemented in LocalTransport."
        )

    def get_laser_scan(self, lidar_name: str) -> LaserScan:
        """Return the latest laser scan.

        Raises
        ------
        ConfigError
            If the lidar name is not in the hardware descriptor.
        """
        self._hw.get_lidar(lidar_name)
        return self._transport.get_laser_scan(lidar_name)

    def get_imu(self, imu_name: str) -> IMUData:
        """Return the latest IMU reading.

        Raises
        ------
        ConfigError
            If the IMU name is not in the hardware descriptor.
        """
        self._hw.get_imu(imu_name)
        return self._transport.get_imu(imu_name)

    def on_camera(
        self,
        camera_name: str,
        callback: Callable[[CameraFrame], None],
        fps: int = 30,
    ) -> None:
        """Register a streaming callback for camera frames.

        Parameters
        ----------
        camera_name:
            Must match a camera in the hardware config.
        callback:
            Called with each new ``CameraFrame``.
        fps:
            Maximum frame delivery rate.
        """
        self._hw.get_camera(camera_name)
        self._transport.subscribe_camera(camera_name, callback, fps)
