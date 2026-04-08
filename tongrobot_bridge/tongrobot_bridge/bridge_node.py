"""TongRobot Bridge Node — ROS2 node + gRPC server.

Runs on the robot machine.  Subscribes to configured ROS2 topics and
exposes all data via a gRPC service so that a remote SDK instance can
connect over the network.

Usage:
    ros2 launch tongrobot_bridge bridge.launch.py config:=configs/turtlebot3.yaml
    # or directly:
    python3 -m tongrobot_bridge.bridge_node configs/turtlebot3.yaml
"""

from __future__ import annotations

import asyncio
import base64
import json
import math
import sys
import threading
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
from typing import Any, Dict, Optional

import grpc
import numpy as np
import rclpy
import rclpy.node
import tf2_ros
from aiohttp import web
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Image, Imu
from sensor_msgs.msg import JointState as RosJointState
from sensor_msgs.msg import LaserScan as RosLaserScan
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from tongrobot.core.config import ConfigLoader, HardwareDescriptor
from tongrobot.proto import bridge_service_pb2, bridge_service_pb2_grpc
from tongrobot.proto.converters import (
    imu_to_proto,
    laser_scan_to_proto,
    proto_to_velocity_cmd,
    robot_state_to_proto,
    camera_frame_to_proto,
    transform_to_proto,
)
from tongrobot.types import (
    BaseState,
    CameraFrame,
    IMUData,
    JointState,
    LaserScan,
    RobotState,
)
from tongrobot.transport.local import _ros_image_to_camera_frame, _stamp_to_float


# ---------------------------------------------------------------------------
# ROS2 bridge node
# ---------------------------------------------------------------------------


class BridgeNode(rclpy.node.Node):
    """ROS2 node that caches sensor data for the gRPC servicer to read."""

    def __init__(self, config_path: str) -> None:
        super().__init__("tongrobot_bridge")

        loader = ConfigLoader(config_path)
        self._hw: HardwareDescriptor = loader.hardware

        self._lock = threading.Lock()
        self._odom_cache: Optional[Odometry] = None
        self._camera_caches: Dict[str, Optional[Image]] = {}
        self._scan_caches: Dict[str, Optional[RosLaserScan]] = {}
        self._imu_caches: Dict[str, Optional[Imu]] = {}
        self._joint_caches: Dict[str, Optional[RosJointState]] = {}
        self._estop = False
        self._arm_joint_controllers: Dict[str, Optional[Any]] = {}
        # Subscriptions --------------------------------------------------------
        if self._hw.base:
            self.create_subscription(
                Odometry, self._hw.base.odom_topic, self._odom_cb, 10
            )

        for name, cam in self._hw.cameras.items():
            self._camera_caches[name] = None
            self.create_subscription(
                Image, cam.topic, lambda msg, n=name: self._camera_cb(n, msg), 10
            )

        for name, lidar in self._hw.lidars.items():
            self._scan_caches[name] = None
            self.create_subscription(
                RosLaserScan, lidar.topic, lambda msg, n=name: self._scan_cb(n, msg), 10
            )

        for name, imu in self._hw.imus.items():
            self._imu_caches[name] = None
            self.create_subscription(
                Imu, imu.topic, lambda msg, n=name: self._imu_cb(n, msg), 10
            )

        for name, arm in self._hw.arms.items():
            self._joint_caches[name] = None
            self.create_subscription(
                RosJointState,
                arm.joint_state_topic,
                lambda msg, n=name: self._joint_cb(n, msg),
                10,
            )

            # Initialize joint names from the first arm
            if not hasattr(self, "joint_names"):
                self.joint_names = arm.joint_names

            arm_control_publisher = self.create_publisher(
                JointTrajectory, arm.joint_cmd_topic, 10
            )
            self._arm_joint_controllers[name] = arm_control_publisher

        # Publisher ------------------------------------------------------------
        self._cmd_vel_pub = None
        if self._hw.base:
            self._cmd_vel_pub = self.create_publisher(
                Twist, self._hw.base.cmd_vel_topic, 10
            )

        # TF2 ------------------------------------------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        self.get_logger().info("BridgeNode initialised — waiting for sensor data…")

    # Callbacks ----------------------------------------------------------------

    def _odom_cb(self, msg: Odometry) -> None:
        with self._lock:
            self._odom_cache = msg

    def _camera_cb(self, name: str, msg: Image) -> None:
        with self._lock:
            self._camera_caches[name] = msg

    def _scan_cb(self, name: str, msg: RosLaserScan) -> None:
        with self._lock:
            self._scan_caches[name] = msg

    def _imu_cb(self, name: str, msg: Imu) -> None:
        with self._lock:
            self._imu_caches[name] = msg

    def _joint_cb(self, name: str, msg: RosJointState) -> None:
        with self._lock:
            self._joint_caches[name] = msg
            # print(f"Joint state updated for arm '{name}': positions={msg.position}  velocities={msg.velocity}  efforts={msg.effort}")

    # Helpers ------------------------------------------------------------------

    def get_robot_state(self) -> RobotState:
        with self._lock:
            odom = self._odom_cache
            joint_caches = dict(self._joint_caches)

        base = None
        ts = time.time()
        if odom is not None:
            ori = odom.pose.pose.orientation
            pos = odom.pose.pose.position
            siny = 2.0 * (ori.w * ori.z + ori.x * ori.y)
            cosy = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
            theta = math.atan2(siny, cosy)
            twist = odom.twist.twist
            base = BaseState(
                position=np.array([pos.x, pos.y, theta]),
                velocity=np.array([twist.linear.x, twist.linear.y, twist.angular.z]),
                timestamp=_stamp_to_float(odom.header.stamp),
            )
            ts = base.timestamp

        arms = {}
        for arm_name, msg in joint_caches.items():
            if msg is not None:
                arms[arm_name] = JointState(
                    positions=np.array(msg.position),
                    velocities=np.array(msg.velocity),
                    efforts=np.array(msg.effort),
                    timestamp=_stamp_to_float(msg.header.stamp),
                )

        return RobotState(arms=arms, base=base, grippers={}, timestamp=ts)

    def get_camera_frame(self, name: str) -> Optional[CameraFrame]:
        with self._lock:
            msg = self._camera_caches.get(name)
        if msg is None:
            return None
        return _ros_image_to_camera_frame(msg)

    def get_laser_scan(self, name: str) -> Optional[LaserScan]:
        with self._lock:
            msg = self._scan_caches.get(name)
        if msg is None:
            return None
        return LaserScan(
            ranges=np.array(msg.ranges, dtype=np.float32),
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            timestamp=_stamp_to_float(msg.header.stamp),
        )

    def get_imu(self, name: str) -> Optional[IMUData]:
        with self._lock:
            msg = self._imu_caches.get(name)
        if msg is None:
            return None
        q = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration
        return IMUData(
            orientation=np.array([q.x, q.y, q.z, q.w]),
            angular_velocity=np.array([av.x, av.y, av.z]),
            linear_acceleration=np.array([la.x, la.y, la.z]),
            timestamp=_stamp_to_float(msg.header.stamp),
        )

    def publish_velocity(
        self, linear_x: float, linear_y: float, angular_z: float
    ) -> None:
        if self._cmd_vel_pub is None:
            return
        twist = Twist()
        twist.linear.x = linear_x
        twist.linear.y = linear_y
        twist.angular.z = angular_z
        self._cmd_vel_pub.publish(twist)

    def publish_arm_control_joints(
        self,
        arm_name: str,
        positions: list[float],
        velocities: list[float] = None,
        accelerations: list[float] = None,
        duration=3.0,
    ) -> bool:
        """
        Controls robot arm joint movement by publishing joint position commands to the joint_cmd_topic
        specified in the configuration file.

        This function creates a JointTrajectory message and publishes it to the corresponding joint
        controller to precisely control each joint of the robot arm to reach the specified positions.

        Args:
            arm_name (str): The name of the robot arm, used to select the corresponding controller
            positions (list[float]) or list[list[float]]: List of one joint positions or a list of joint trajectory, each element corresponds to the target
                                     angle (in radians) of a joint. For panda_arm, exactly 7 position
                                     values must be provided.
            velocities (list[float], optional): List of joint velocities for smooth motion control.
                                                Defaults to zero-filled list (stationary start/end)
            accelerations (list[float], optional): List of joint accelerations for smoother motion control.
                                                  Defaults to zero-filled list
            duration (float): Execution time for the movement (in seconds), specifies the time to
                             reach the target position

        Returns:
            bool: Returns True if successfully published the joint command, False if the specified
                  arm does not exist or if parameters are incorrect.

        Raises:
            No direct exceptions, but logs error messages (e.g., if arm controller is not found or
            panda arm position count is incorrect)

        Example:
            # Control Panda robotic arm to specific joint positions
            positions = [0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785], or [[0.1, -0.685, 0.2, -2.056, 0.0, 1.571, 0.785],[0.0, -0.785, 0.0, -2.356, 0.0, 1.571, 0.785]]
            success = node.publish_arm_control_joints('panda_arm', positions, duration=2.0)
        """
        if self._arm_joint_controllers.get(arm_name) is None:
            self.get_logger().info(
                f"No joint command publisher found for arm '{arm_name}'"
            )
            return False

        if arm_name == "panda_arm" and len(positions) != 7:
            self.get_logger().error("Need exactly 7 joint positions for Panda arm")
            return False

        # Create trajectory message
        current_arm = self._hw.arms[arm_name]
        trajectory_msg = JointTrajectory()
        trajectory_msg.joint_names = current_arm.joint_names
        trajectory_msg.header.stamp = self.get_clock().now().to_msg()
        trajectory_msg.header.frame_id = arm_name

        if not isinstance(
            positions[0], list
        ):  # check positions is one point or a rajectory
            positions = [positions]

        # Create trajectory point
        for position in positions:
            point = JointTrajectoryPoint()
            # Convert positions to floats to satisfy ROS2 type requirements
            point.positions = [float(pos) for pos in position]
            point.time_from_start.sec = int(duration)
            point.time_from_start.nanosec = int((duration - int(duration)) * 1e9)

            # Add velocity and acceleration for smoother motion
            if velocities is not None and len(velocities) == 7:
                point.velocities = [float(vel) for vel in velocities]
            else:
                point.velocities = [0.0] * 7

            if accelerations is not None and len(accelerations) == 7:
                point.accelerations = [float(acc) for acc in accelerations]
            else:
                point.accelerations = [0.0] * 7

            trajectory_msg.points.append(point)

        # Publish the trajectory
        self._arm_joint_controllers[arm_name].publish(trajectory_msg)
        self.get_logger().info(
            f"Moving to joint positions: {positions}, velocities: {velocities}"
        )

        return True

    def get_transform(self, target: str, source: str) -> Any:
        import rclpy.time as rtime

        return self._tf_buffer.lookup_transform(target, source, rtime.Time())


# ---------------------------------------------------------------------------
# gRPC servicer
# ---------------------------------------------------------------------------


class BridgeServicer(bridge_service_pb2_grpc.TongRobotBridgeServicer):
    """Implements every RPC defined in bridge_service.proto."""

    def __init__(self, node: BridgeNode) -> None:
        self._node = node

    def GetRobotState(self, request, context):
        state = self._node.get_robot_state()
        return robot_state_to_proto(state)

    def StreamRobotState(self, request, context):
        while context.is_active():
            state = self._node.get_robot_state()
            yield robot_state_to_proto(state)
            time.sleep(0.02)  # ~50 Hz

    def GetCameraFrame(self, request, context):
        frame = self._node.get_camera_frame(request.name)
        if frame is None:
            context.abort(
                grpc.StatusCode.NOT_FOUND, f"No data from camera '{request.name}' yet"
            )
        return camera_frame_to_proto(frame, compress=True)

    def StreamCameraFrames(self, request, context):
        fps = max(request.fps, 1) if request.fps > 0 else 10
        interval = 1.0 / fps
        while context.is_active():
            frame = self._node.get_camera_frame(request.name)
            if frame is not None:
                yield camera_frame_to_proto(frame, compress=True)
            time.sleep(interval)

    def GetLaserScan(self, request, context):
        scan = self._node.get_laser_scan(request.name)
        if scan is None:
            context.abort(
                grpc.StatusCode.NOT_FOUND, f"No data from lidar '{request.name}' yet"
            )
        return laser_scan_to_proto(scan)

    def GetIMU(self, request, context):
        imu = self._node.get_imu(request.name)
        if imu is None:
            context.abort(
                grpc.StatusCode.NOT_FOUND, f"No data from IMU '{request.name}' yet"
            )
        return imu_to_proto(imu)

    def SetVelocityCommand(self, request, context):
        if self._node._estop:
            return bridge_service_pb2.CommandAck(
                success=False, message="Emergency stop active"
            )
        cmd = proto_to_velocity_cmd(request)
        self._node.publish_velocity(cmd.linear_x, cmd.linear_y, cmd.angular_z)
        return bridge_service_pb2.CommandAck(success=True, message="OK")

    def SetJointCommand(self, request, context):
        # TurtleBot3 has no arms; return success as a no-op
        return bridge_service_pb2.CommandAck(
            success=True, message="OK (no arms configured)"
        )

    def EmergencyStop(self, request, context):
        self._node._estop = True
        self._node.publish_velocity(0.0, 0.0, 0.0)
        return bridge_service_pb2.CommandAck(
            success=True, message="Emergency stop engaged"
        )

    def GetTransform(self, request, context):
        try:
            tf = self._node.get_transform(request.target_frame, request.source_frame)
            from tongrobot.types import Transform
            import numpy as np

            t = tf.transform.translation
            r = tf.transform.rotation
            transform = Transform(
                position=np.array([t.x, t.y, t.z]),
                quaternion=np.array([r.x, r.y, r.z, r.w]),
            )
            return transform_to_proto(transform)
        except Exception as exc:
            context.abort(grpc.StatusCode.NOT_FOUND, str(exc))


# ---------------------------------------------------------------------------
# aiohttp WebSocket + static file server (dashboard)
# ---------------------------------------------------------------------------


class DashboardServer:
    """aiohttp server: WebSocket live data + static file serving for the frontend.

    Runs in a dedicated daemon thread with its own asyncio event loop so it
    does not interfere with ROS2 spinning or the gRPC ThreadPoolExecutor.
    """

    _NOT_BUILT_HTML = """\
<!DOCTYPE html>
<html lang="en">
<head><meta charset="UTF-8"><title>TongRobot Dashboard</title>
<style>body{font-family:monospace;background:#111;color:#ccc;padding:2rem;}
  pre{background:#222;padding:1rem;border-radius:6px;color:#7ec8e3;}</style>
</head>
<body>
  <h2>TongRobot Dashboard</h2>
  <p>Frontend not built yet. Run:</p>
  <pre>bash tongrobot_dashboard/build.sh</pre>
  <p>WebSocket endpoint: <code>ws://&lt;host&gt;:&lt;port&gt;/ws</code></p>
</body>
</html>"""

    def __init__(
        self,
        node: BridgeNode,  #
        dashboard_config: dict,
        static_dir: str,
        port: int,
    ) -> None:
        self._node = node
        self._dashboard_config = dashboard_config
        self._static_dir = Path(static_dir)
        self._port = port
        self._loop: Optional[asyncio.AbstractEventLoop] = None

    # ------------------------------------------------------------------
    # aiohttp request handlers
    # ------------------------------------------------------------------

    async def _ws_handler(self, request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse()
        await ws.prepare(request)

        # Per-client streaming tasks keyed by logical subscription name.
        tasks: Dict[str, asyncio.Task] = {}

        async def _stream_robot_state() -> None:
            while not ws.closed:
                state = self._node.get_robot_state()
                payload: Dict = {
                    "type": "robot_state",
                    "data": {"base": None, "timestamp": state.timestamp},
                }
                if state.base is not None:
                    payload["data"]["base"] = {
                        "x": float(state.base.position[0]),
                        "y": float(state.base.position[1]),
                        "theta": float(state.base.position[2]),
                        "vx": float(state.base.velocity[0]),
                        "vy": float(state.base.velocity[1]),
                        "omega": float(state.base.velocity[2]),
                        "timestamp": state.base.timestamp,
                    }
                await ws.send_str(json.dumps(payload))
                await asyncio.sleep(0.1)  # 10 Hz

        async def _stream_camera(name: str, fps: int) -> None:
            import cv2

            interval = 1.0 / max(fps, 1)
            while not ws.closed:
                frame = self._node.get_camera_frame(name)
                if frame is not None:
                    bgr = cv2.cvtColor(frame.data, cv2.COLOR_RGB2BGR)
                    ok, buf = cv2.imencode(".jpg", bgr, [cv2.IMWRITE_JPEG_QUALITY, 75])
                    if ok:
                        b64 = base64.b64encode(buf.tobytes()).decode("ascii")
                        await ws.send_str(
                            json.dumps(
                                {
                                    "type": "camera_frame",
                                    "name": name,
                                    "data": b64,
                                    "timestamp": frame.timestamp,
                                }
                            )
                        )
                await asyncio.sleep(interval)

        async def _stream_laser_scan(name: str) -> None:
            while not ws.closed:
                scan = self._node.get_laser_scan(name)
                if scan is not None:
                    try:
                        # LiDAR out-of-range readings are often +inf or nan.
                        # json.dumps raises ValueError on non-finite floats,
                        # which silently kills the asyncio task.  Replace them
                        # with 0.0 before serialising (the frontend already
                        # filters r <= 0 when drawing).
                        ranges = np.nan_to_num(
                            scan.ranges, nan=0.0, posinf=0.0, neginf=0.0
                        ).tolist()
                        await ws.send_str(
                            json.dumps(
                                {
                                    "type": "laser_scan",
                                    "name": name,
                                    "data": {
                                        "ranges": ranges,
                                        "angle_min": scan.angle_min,
                                        "angle_max": scan.angle_max,
                                        "angle_increment": scan.angle_increment,
                                        "timestamp": scan.timestamp,
                                    },
                                }
                            )
                        )
                    except Exception as exc:
                        self._node.get_logger().warning(
                            f"DashboardServer: laser_scan stream error ({name}): {exc}"
                        )
                await asyncio.sleep(0.1)  # 10 Hz

        def _cancel(key: str) -> None:
            t = tasks.pop(key, None)
            if t and not t.done():
                t.cancel()

        try:
            async for msg in ws:
                if msg.type != web.WSMsgType.TEXT:
                    continue
                try:
                    data = json.loads(msg.data)
                except json.JSONDecodeError:
                    continue

                msg_type = data.get("type")

                if msg_type == "subscribe":
                    topic = data.get("topic", "")
                    if topic == "robot_state":
                        _cancel("robot_state")
                        tasks["robot_state"] = asyncio.create_task(
                            _stream_robot_state()
                        )

                    elif topic == "camera":
                        name = data.get("name", "main_camera")
                        fps = int(data.get("fps", 10))
                        key = f"camera_{name}"
                        _cancel(key)
                        tasks[key] = asyncio.create_task(_stream_camera(name, fps))

                    elif topic == "laser_scan":
                        name = data.get("name", "base_scan")
                        key = f"laser_scan_{name}"
                        _cancel(key)
                        tasks[key] = asyncio.create_task(_stream_laser_scan(name))

                elif msg_type == "unsubscribe":
                    topic = data.get("topic", "")
                    for key in [
                        k for k in tasks if k == topic or k.startswith(topic + "_")
                    ]:
                        _cancel(key)

                elif msg_type == "cmd_vel":
                    if not self._node._estop:
                        self._node.publish_velocity(
                            float(data.get("linear", 0.0)),
                            0.0,
                            float(data.get("angular", 0.0)),
                        )

                elif msg_type == "arm_joint_cmd":
                    if self._node._hw.arms:
                        arm_name = next(iter(self._node._hw.arms.keys()))
                        positions = data.get("positions", [])
                        velocities = data.get("velocities", [])

                        # Validate that we have 7 joint positions
                        if len(positions) == 7:
                            self._node.publish_arm_control_joints(
                                arm_name, positions, velocities
                            )
                        else:
                            self._node.get_logger().warn(
                                f"Expected 7 joint positions, got {len(positions)}"
                            )

                elif msg_type == "stop":
                    self._node.publish_velocity(0.0, 0.0, 0.0)

                elif msg_type == "get_config":
                    await ws.send_str(
                        json.dumps({"type": "config", "data": self._dashboard_config})
                    )

        finally:
            for t in tasks.values():
                t.cancel()

        return ws

    async def _index_handler(self, request: web.Request) -> web.Response:
        index = self._static_dir / "index.html"
        if index.exists():
            return web.FileResponse(index)
        return web.Response(text=self._NOT_BUILT_HTML, content_type="text/html")

    # ------------------------------------------------------------------
    # Startup
    # ------------------------------------------------------------------

    def _build_app(self) -> web.Application:
        app = web.Application()
        app.router.add_get("/ws", self._ws_handler)
        app.router.add_get("/", self._index_handler)
        if self._static_dir.exists():
            # Serve built assets; must come after the explicit routes above.
            app.router.add_static(
                "/",
                path=str(self._static_dir),
                name="static",
                show_index=False,
                append_version=False,
            )
        return app

    def start_in_thread(self) -> None:
        """Start aiohttp in a daemon thread with its own event loop."""

        def _run() -> None:
            loop = asyncio.new_event_loop()
            asyncio.set_event_loop(loop)
            self._loop = loop
            app = self._build_app()
            runner = web.AppRunner(app)
            loop.run_until_complete(runner.setup())
            site = web.TCPSite(runner, "0.0.0.0", self._port)
            loop.run_until_complete(site.start())
            loop.run_forever()

        t = threading.Thread(target=_run, daemon=True, name="dashboard-aiohttp")
        t.start()


# ---------------------------------------------------------------------------
# Main entry point
# ---------------------------------------------------------------------------


def main() -> None:
    if len(sys.argv) < 2:
        print(f"Usage: {sys.argv[0]} <config_file>")
        sys.exit(1)

    config_path = sys.argv[1]
    loader = ConfigLoader(config_path)
    grpc_port = loader.connection_config.get("remote", {}).get("grpc_port", 50051)

    dashboard_cfg = loader.dashboard_config or {}
    dashboard_port = int(dashboard_cfg.get("port", 3000))
    static_dir = str(
        dashboard_cfg.get("static_dir", "tongrobot_dashboard/frontend/dist")
    )
    dashboard_enabled = bool(dashboard_cfg.get("enabled", True))

    rclpy.init()  # ROS2初始化
    node = BridgeNode(config_path)

    # gRPC server
    server = grpc.server(ThreadPoolExecutor(max_workers=10))
    bridge_service_pb2_grpc.add_TongRobotBridgeServicer_to_server(
        BridgeServicer(node), server
    )
    server.add_insecure_port(f"0.0.0.0:{grpc_port}")
    server.start()
    node.get_logger().info(f"gRPC server listening on port {grpc_port}")

    # Dashboard / WebSocket server
    if dashboard_enabled:
        dash = DashboardServer(
            node=node,
            dashboard_config=dashboard_cfg,
            static_dir=static_dir,
            port=dashboard_port,
        )
        dash.start_in_thread()
        node.get_logger().info(
            f"Dashboard server listening on http://0.0.0.0:{dashboard_port}  "
            f"(WebSocket: ws://0.0.0.0:{dashboard_port}/ws)"
        )

    try:
        # Start a separate thread to randomly control the arm joints if arms exist

        # import random
        # from std_msgs.msg import Float64MultiArray
        # import threading

        # def random_arm_control():
        #     # Wait a bit for node to fully initialize
        #     time.sleep(2)

        #     # Check if any arms are configured
        #     if hasattr(node, '_hw') and node._hw.arms:
        #         arm_name = next(iter(node._hw.arms.keys()))  # Get the first arm name
        #         node.get_logger().info(f"Starting random arm control for arm: {arm_name}")

        #         while rclpy.ok():
        #             # Generate 7 random joint angles between -π and π
        #             random_positions = [random.uniform(-math.pi, math.pi) for _ in range(7)]

        #             # Publish the random joint positions
        #             node.publish_arm_control_joints(arm_name, random_positions)

        #             # Sleep for 5 seconds before next command
        #             time.sleep(5)
        #     else:
        #         node.get_logger().info("No arms configured, skipping random arm control")

        # # Start the random arm control in a separate thread
        # arm_control_thread = threading.Thread(target=random_arm_control, daemon=True)
        # arm_control_thread.start()

        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info("Shutting down…")
    finally:
        server.stop(grace=2)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
