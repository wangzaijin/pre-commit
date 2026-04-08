"""Conversion utilities between tongrobot dataclasses and protobuf messages."""

from __future__ import annotations

import numpy as np

from tongrobot.types import (
    BaseState,
    CameraFrame,
    GripperState,
    IMUData,
    JointState,
    LaserScan,
    RobotState,
    Transform,
    VelocityCommand,
)
from tongrobot.proto import (
    robot_state_pb2,
    sensor_data_pb2,
    commands_pb2,
    bridge_service_pb2,
)


# ---------------------------------------------------------------------------
# RobotState
# ---------------------------------------------------------------------------


def robot_state_to_proto(state: RobotState) -> robot_state_pb2.RobotStateMsg:
    msg = robot_state_pb2.RobotStateMsg()
    msg.timestamp = state.timestamp
    for name, js in state.arms.items():
        arm_msg = robot_state_pb2.JointStateMsg()
        arm_msg.positions.extend(js.positions.tolist())
        arm_msg.velocities.extend(js.velocities.tolist())
        arm_msg.efforts.extend(js.efforts.tolist())
        arm_msg.timestamp = js.timestamp
        msg.arms[name].CopyFrom(arm_msg)
    if state.base is not None:
        msg.has_base = True
        msg.base.position.extend(state.base.position.tolist())
        msg.base.velocity.extend(state.base.velocity.tolist())
        msg.base.timestamp = state.base.timestamp
    for name, gs in state.grippers.items():
        grip_msg = robot_state_pb2.GripperStateMsg()
        grip_msg.position = gs.position
        grip_msg.is_open = gs.is_open
        grip_msg.timestamp = gs.timestamp
        msg.grippers[name].CopyFrom(grip_msg)
    return msg


def proto_to_robot_state(msg: robot_state_pb2.RobotStateMsg) -> RobotState:
    arms = {}
    for name, arm_msg in msg.arms.items():
        arms[name] = JointState(
            positions=np.array(arm_msg.positions),
            velocities=np.array(arm_msg.velocities),
            efforts=np.array(arm_msg.efforts),
            timestamp=arm_msg.timestamp,
        )
    base = None
    if msg.has_base:
        base = BaseState(
            position=np.array(msg.base.position),
            velocity=np.array(msg.base.velocity),
            timestamp=msg.base.timestamp,
        )
    grippers = {}
    for name, grip_msg in msg.grippers.items():
        grippers[name] = GripperState(
            position=grip_msg.position,
            is_open=grip_msg.is_open,
            timestamp=grip_msg.timestamp,
        )
    return RobotState(arms=arms, base=base, grippers=grippers, timestamp=msg.timestamp)


# ---------------------------------------------------------------------------
# CameraFrame
# ---------------------------------------------------------------------------


def camera_frame_to_proto(
    frame: CameraFrame,
    compress: bool = True,
    jpeg_quality: int = 80,
) -> sensor_data_pb2.CameraFrameMsg:
    import cv2

    h, w = frame.data.shape[:2]
    channels = frame.data.shape[2] if frame.data.ndim == 3 else 1

    msg = sensor_data_pb2.CameraFrameMsg()
    msg.width = w
    msg.height = h
    msg.channels = channels
    msg.timestamp = frame.timestamp
    msg.encoding = frame.encoding

    if compress:
        # SDK stores images as RGB; cv2 encodes as BGR
        bgr = cv2.cvtColor(frame.data, cv2.COLOR_RGB2BGR)
        ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])
        if not ok:
            raise RuntimeError("cv2.imencode failed for camera frame")
        msg.data = buf.tobytes()
        msg.is_compressed = True
    else:
        msg.data = frame.data.tobytes()
        msg.is_compressed = False

    return msg


def proto_to_camera_frame(msg: sensor_data_pb2.CameraFrameMsg) -> CameraFrame:
    import cv2

    if msg.is_compressed:
        bgr = cv2.imdecode(np.frombuffer(msg.data, np.uint8), cv2.IMREAD_COLOR)
        img = cv2.cvtColor(bgr, cv2.COLOR_BGR2RGB)
    else:
        dtype = np.uint8
        img = np.frombuffer(msg.data, dtype=dtype).reshape(
            (msg.height, msg.width, msg.channels)
            if msg.channels > 1
            else (msg.height, msg.width)
        )

    return CameraFrame(
        data=img, timestamp=msg.timestamp, encoding=msg.encoding or "rgb8"
    )


# ---------------------------------------------------------------------------
# LaserScan
# ---------------------------------------------------------------------------


def laser_scan_to_proto(scan: LaserScan) -> sensor_data_pb2.LaserScanMsg:
    msg = sensor_data_pb2.LaserScanMsg()
    msg.ranges.extend(scan.ranges.tolist())
    msg.angle_min = scan.angle_min
    msg.angle_max = scan.angle_max
    msg.angle_increment = scan.angle_increment
    msg.timestamp = scan.timestamp
    return msg


def proto_to_laser_scan(msg: sensor_data_pb2.LaserScanMsg) -> LaserScan:
    return LaserScan(
        ranges=np.array(msg.ranges, dtype=np.float32),
        angle_min=msg.angle_min,
        angle_max=msg.angle_max,
        angle_increment=msg.angle_increment,
        timestamp=msg.timestamp,
    )


# ---------------------------------------------------------------------------
# IMUData
# ---------------------------------------------------------------------------


def imu_to_proto(imu: IMUData) -> sensor_data_pb2.IMUMsg:
    msg = sensor_data_pb2.IMUMsg()
    msg.orientation.extend(imu.orientation.tolist())
    msg.angular_velocity.extend(imu.angular_velocity.tolist())
    msg.linear_acceleration.extend(imu.linear_acceleration.tolist())
    msg.timestamp = imu.timestamp
    return msg


def proto_to_imu(msg: sensor_data_pb2.IMUMsg) -> IMUData:
    return IMUData(
        orientation=np.array(msg.orientation),
        angular_velocity=np.array(msg.angular_velocity),
        linear_acceleration=np.array(msg.linear_acceleration),
        timestamp=msg.timestamp,
    )


# ---------------------------------------------------------------------------
# VelocityCommand
# ---------------------------------------------------------------------------


def velocity_cmd_to_proto(cmd: VelocityCommand) -> commands_pb2.VelocityCommandMsg:
    msg = commands_pb2.VelocityCommandMsg()
    msg.linear_x = cmd.linear_x
    msg.linear_y = cmd.linear_y
    msg.angular_z = cmd.angular_z
    return msg


def proto_to_velocity_cmd(msg: commands_pb2.VelocityCommandMsg) -> VelocityCommand:
    return VelocityCommand(
        linear_x=msg.linear_x,
        linear_y=msg.linear_y,
        angular_z=msg.angular_z,
    )


# ---------------------------------------------------------------------------
# Transform
# ---------------------------------------------------------------------------


def transform_to_proto(tf: Transform) -> bridge_service_pb2.TransformMsg:
    msg = bridge_service_pb2.TransformMsg()
    msg.position.extend(tf.position.tolist())
    msg.quaternion.extend(tf.quaternion.tolist())
    return msg


def proto_to_transform(msg: bridge_service_pb2.TransformMsg) -> Transform:
    return Transform(
        position=np.array(msg.position),
        quaternion=np.array(msg.quaternion),
    )
