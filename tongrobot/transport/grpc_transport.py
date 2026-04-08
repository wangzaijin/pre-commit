"""GRPCTransport — SDK-side gRPC client connecting to the bridge node."""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, Optional

import grpc

from tongrobot.exceptions import ConnectionError, TimeoutError
from tongrobot.proto import bridge_service_pb2, bridge_service_pb2_grpc
from tongrobot.proto.converters import (
    proto_to_camera_frame,
    proto_to_imu,
    proto_to_laser_scan,
    proto_to_robot_state,
    proto_to_transform,
    velocity_cmd_to_proto,
)
from tongrobot.transport.base import TransportBase
from tongrobot.types import (
    CameraFrame,
    IMUData,
    JointCommand,
    LaserScan,
    RobotState,
    Transform,
    VelocityCommand,
)
from tongrobot.utils.logging import get_logger

_Empty = bridge_service_pb2.Empty
_StreamRequest = bridge_service_pb2.StreamRequest
_TransformRequest = bridge_service_pb2.TransformRequest


class GRPCTransport(TransportBase):
    """Transport that connects to the `tongrobot_bridge` gRPC server over the network."""

    def __init__(self) -> None:
        self._channel: Optional[grpc.Channel] = None
        self._stub: Optional[bridge_service_pb2_grpc.TongRobotBridgeStub] = None
        self._connected = False
        self._logger = get_logger(__name__)

        # Streaming threads
        self._stream_stop = threading.Event()
        self._stream_threads: list[threading.Thread] = []

    # -------------------------------------------------------------------------

    def connect(self, config: Dict[str, Any]) -> None:
        """Open a gRPC channel to the bridge node and verify connectivity."""
        remote = config.get("connection", {}).get("remote", {})
        host = remote.get("host", "localhost")
        port = remote.get("grpc_port", 50051)
        address = f"{host}:{port}"

        self._logger.info("Connecting to bridge at %s …", address)
        self._channel = grpc.insecure_channel(address)
        self._stub = bridge_service_pb2_grpc.TongRobotBridgeStub(self._channel)

        # Connection probe — raises ConnectionError on failure
        try:
            self._stub.GetRobotState(_Empty(), timeout=5.0)
        except grpc.RpcError as exc:
            raise ConnectionError(
                f"Cannot reach bridge node at {address}. "
                f"Is 'ros2 launch tongrobot_bridge bridge.launch.py' running? "
                f"gRPC error: {exc.code()}: {exc.details()}"
            ) from exc

        self._connected = True
        self._logger.info("Connected to bridge at %s", address)

    def disconnect(self) -> None:
        """Stop streaming threads and close the gRPC channel."""
        self._connected = False
        self._stream_stop.set()
        for t in self._stream_threads:
            t.join(timeout=2.0)
        self._stream_threads.clear()
        if self._channel:
            self._channel.close()
            self._channel = None
        self._stub = None

    def is_connected(self) -> bool:
        if not self._connected or self._stub is None:
            return False
        try:
            self._stub.GetRobotState(_Empty(), timeout=1.0)
            return True
        except grpc.RpcError:
            return False

    # -------------------------------------------------------------------------

    def get_robot_state(self) -> RobotState:
        try:
            msg = self._stub.GetRobotState(_Empty(), timeout=5.0)
            return proto_to_robot_state(msg)
        except grpc.RpcError as exc:
            raise TimeoutError(f"GetRobotState failed: {exc.details()}") from exc

    def get_camera_frame(self, camera_name: str) -> CameraFrame:
        try:
            msg = self._stub.GetCameraFrame(
                _StreamRequest(name=camera_name), timeout=5.0
            )
            return proto_to_camera_frame(msg)
        except grpc.RpcError as exc:
            raise TimeoutError(
                f"GetCameraFrame('{camera_name}') failed: {exc.details()}"
            ) from exc

    def get_laser_scan(self, lidar_name: str) -> LaserScan:
        try:
            msg = self._stub.GetLaserScan(_StreamRequest(name=lidar_name), timeout=5.0)
            return proto_to_laser_scan(msg)
        except grpc.RpcError as exc:
            raise TimeoutError(
                f"GetLaserScan('{lidar_name}') failed: {exc.details()}"
            ) from exc

    def get_imu(self, imu_name: str) -> IMUData:
        try:
            msg = self._stub.GetIMU(_StreamRequest(name=imu_name), timeout=5.0)
            return proto_to_imu(msg)
        except grpc.RpcError as exc:
            raise TimeoutError(f"GetIMU('{imu_name}') failed: {exc.details()}") from exc

    def set_velocity_command(self, cmd: VelocityCommand) -> None:
        ack = self._stub.SetVelocityCommand(velocity_cmd_to_proto(cmd), timeout=2.0)
        if not ack.success:
            self._logger.warning("SetVelocityCommand rejected: %s", ack.message)

    def set_joint_command(self, arm_name: str, cmd: JointCommand) -> None:
        from tongrobot.proto.commands_pb2 import JointCommandMsg

        msg = JointCommandMsg(arm_name=arm_name)
        if cmd.positions is not None:
            msg.positions.extend(cmd.positions.tolist())
            msg.has_positions = True
        if cmd.velocities is not None:
            msg.velocities.extend(cmd.velocities.tolist())
            msg.has_velocities = True
        ack = self._stub.SetJointCommand(msg, timeout=2.0)
        if not ack.success:
            self._logger.warning("SetJointCommand rejected: %s", ack.message)

    def subscribe_robot_state(self, callback: Callable[[RobotState], None]) -> None:
        def _stream_loop() -> None:
            while not self._stream_stop.is_set():
                try:
                    for msg in self._stub.StreamRobotState(_Empty()):
                        if self._stream_stop.is_set():
                            break
                        callback(proto_to_robot_state(msg))
                except grpc.RpcError as exc:
                    if not self._stream_stop.is_set():
                        self._logger.warning(
                            "StreamRobotState disconnected (%s), retrying in 1 s…",
                            exc.code(),
                        )
                        time.sleep(1.0)

        t = threading.Thread(target=_stream_loop, daemon=True, name="grpc_state_stream")
        self._stream_threads.append(t)
        t.start()

    def subscribe_camera(
        self,
        camera_name: str,
        callback: Callable[[CameraFrame], None],
        fps: int = 30,
    ) -> None:
        def _stream_loop() -> None:
            while not self._stream_stop.is_set():
                try:
                    for msg in self._stub.StreamCameraFrames(
                        _StreamRequest(name=camera_name, fps=fps)
                    ):
                        if self._stream_stop.is_set():
                            break
                        callback(proto_to_camera_frame(msg))
                except grpc.RpcError as exc:
                    if not self._stream_stop.is_set():
                        self._logger.warning(
                            "StreamCameraFrames('%s') disconnected (%s), retrying…",
                            camera_name,
                            exc.code(),
                        )
                        time.sleep(1.0)

        t = threading.Thread(
            target=_stream_loop, daemon=True, name=f"grpc_cam_stream_{camera_name}"
        )
        self._stream_threads.append(t)
        t.start()

    def get_transform(self, target_frame: str, source_frame: str) -> Transform:
        try:
            msg = self._stub.GetTransform(
                _TransformRequest(target_frame=target_frame, source_frame=source_frame),
                timeout=5.0,
            )
            return proto_to_transform(msg)
        except grpc.RpcError as exc:
            raise RuntimeError(
                f"GetTransform('{target_frame}', '{source_frame}') failed: {exc.details()}"
            ) from exc
