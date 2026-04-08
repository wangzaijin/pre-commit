from __future__ import annotations
from dataclasses import dataclass
from typing import Any, Dict, List, Optional
import yaml

from tongrobot.exceptions import ConfigError


# ---------------------------------------------------------------------------
# Hardware descriptor dataclasses
# ---------------------------------------------------------------------------


@dataclass
class ArmDescriptor:
    name: str
    num_joints: int
    joint_names: List[str]
    joint_state_topic: str
    joint_cmd_topic: str
    joint_limits: Dict[str, Any]  # keys: 'position', 'velocity', 'effort'
    control_rate_hz: float


@dataclass
class CameraDescriptor:
    name: str
    topic: str
    info_topic: str
    encoding: str
    resolution: List[int]
    depth_topic: Optional[str] = None


@dataclass
class LidarDescriptor:
    name: str
    topic: str
    type: str  # '2d' or '3d'


@dataclass
class IMUDescriptor:
    name: str
    topic: str


@dataclass
class BaseDescriptor:
    type: str  # e.g. 'differential'
    cmd_vel_topic: str
    odom_topic: str
    max_linear_vel: float
    max_angular_vel: float


class HardwareDescriptor:
    """Structured representation of the hardware section of the YAML config."""

    def __init__(self, hardware_cfg: Dict[str, Any]) -> None:
        # Arms
        self._arms: Dict[str, ArmDescriptor] = {}
        for arm_cfg in hardware_cfg.get("arms", []):
            desc = ArmDescriptor(
                name=arm_cfg["name"],
                num_joints=arm_cfg["num_joints"],
                joint_names=arm_cfg.get("joint_names", []),
                joint_state_topic=arm_cfg["joint_state_topic"],
                joint_cmd_topic=arm_cfg["joint_cmd_topic"],
                joint_limits=arm_cfg.get("joint_limits", {}),
                control_rate_hz=arm_cfg.get("control_rate_hz", 100.0),
            )
            self._arms[desc.name] = desc

        # Grippers (minimal — store raw dicts for now)
        self._grippers: Dict[str, Dict[str, Any]] = {}
        for grip_cfg in hardware_cfg.get("grippers", []):
            self._grippers[grip_cfg["name"]] = grip_cfg

        # Base (optional)
        self._base: Optional[BaseDescriptor] = None
        if "base" in hardware_cfg and hardware_cfg["base"]:
            b = hardware_cfg["base"]
            self._base = BaseDescriptor(
                type=b["type"],
                cmd_vel_topic=b["cmd_vel_topic"],
                odom_topic=b["odom_topic"],
                max_linear_vel=b.get("max_linear_vel", 0.5),
                max_angular_vel=b.get("max_angular_vel", 1.0),
            )

        # Sensors
        sensors_cfg = hardware_cfg.get("sensors", {})

        self._cameras: Dict[str, CameraDescriptor] = {}
        for cam_cfg in sensors_cfg.get("cameras", []):
            desc = CameraDescriptor(
                name=cam_cfg["name"],
                topic=cam_cfg["topic"],
                info_topic=cam_cfg["info_topic"],
                encoding=cam_cfg.get("encoding", "rgb8"),
                resolution=cam_cfg.get("resolution", [640, 480]),
                depth_topic=cam_cfg.get("depth_topic"),
            )
            self._cameras[desc.name] = desc

        self._lidars: Dict[str, LidarDescriptor] = {}
        for lidar_cfg in sensors_cfg.get("lidar", []):
            desc = LidarDescriptor(
                name=lidar_cfg["name"],
                topic=lidar_cfg["topic"],
                type=lidar_cfg.get("type", "2d"),
            )
            self._lidars[desc.name] = desc

        self._imus: Dict[str, IMUDescriptor] = {}
        for imu_cfg in sensors_cfg.get("imu", []):
            desc = IMUDescriptor(
                name=imu_cfg["name"],
                topic=imu_cfg["topic"],
            )
            self._imus[desc.name] = desc

    # Accessors ----------------------------------------------------------------

    def get_arm(self, name: str) -> ArmDescriptor:
        if name not in self._arms:
            available = list(self._arms.keys())
            raise ConfigError(
                f"Arm '{name}' not found in config. Available arms: {available}"
            )
        return self._arms[name]

    def get_camera(self, name: str) -> CameraDescriptor:
        if name not in self._cameras:
            available = list(self._cameras.keys())
            raise ConfigError(
                f"Camera '{name}' not found in config. Available cameras: {available}"
            )
        return self._cameras[name]

    def get_lidar(self, name: str) -> LidarDescriptor:
        if name not in self._lidars:
            available = list(self._lidars.keys())
            raise ConfigError(
                f"LiDAR '{name}' not found in config. Available lidars: {available}"
            )
        return self._lidars[name]

    def get_imu(self, name: str) -> IMUDescriptor:
        if name not in self._imus:
            available = list(self._imus.keys())
            raise ConfigError(
                f"IMU '{name}' not found in config. Available IMUs: {available}"
            )
        return self._imus[name]

    def get_all_topics(self) -> List[str]:
        """Return every ROS2 topic referenced in the hardware config."""
        topics: List[str] = []
        for arm in self._arms.values():
            topics += [arm.joint_state_topic, arm.joint_cmd_topic]
        if self._base:
            topics += [self._base.cmd_vel_topic, self._base.odom_topic]
        for cam in self._cameras.values():
            topics.append(cam.topic)
            topics.append(cam.info_topic)
            if cam.depth_topic:
                topics.append(cam.depth_topic)
        for lidar in self._lidars.values():
            topics.append(lidar.topic)
        for imu in self._imus.values():
            topics.append(imu.topic)
        return topics

    @property
    def arms(self) -> Dict[str, ArmDescriptor]:
        return self._arms

    @property
    def cameras(self) -> Dict[str, CameraDescriptor]:
        return self._cameras

    @property
    def lidars(self) -> Dict[str, LidarDescriptor]:
        return self._lidars

    @property
    def imus(self) -> Dict[str, IMUDescriptor]:
        return self._imus

    @property
    def base(self) -> Optional[BaseDescriptor]:
        return self._base


# ---------------------------------------------------------------------------
# Config loader
# ---------------------------------------------------------------------------


class ConfigLoader:
    """Load and validate a TongRobot YAML config file."""

    _REQUIRED_KEYS = ("robot", "connection", "hardware")

    def __init__(self, config_path: str) -> None:
        with open(config_path, "r") as fh:
            self._data: Dict[str, Any] = yaml.safe_load(fh)

        missing = [k for k in self._REQUIRED_KEYS if k not in self._data]
        if missing:
            raise ConfigError(
                f"Config file '{config_path}' is missing required top-level keys: {missing}"
            )

        self._hardware = HardwareDescriptor(self._data["hardware"])

    # Properties ---------------------------------------------------------------

    @property
    def robot_name(self) -> str:
        return self._data["robot"].get("name", "unknown_robot")

    @property
    def connection_config(self) -> Dict[str, Any]:
        return self._data["connection"]

    @property
    def hardware_config(self) -> Dict[str, Any]:
        return self._data["hardware"]

    @property
    def hardware(self) -> HardwareDescriptor:
        return self._hardware

    @property
    def dashboard_config(self) -> Dict[str, Any]:
        return self._data.get("dashboard", {})

    @property
    def safety_config(self) -> Dict[str, Any]:
        return self._data.get("safety", {})

    @property
    def logging_config(self) -> Dict[str, Any]:
        return self._data.get("logging", {})
