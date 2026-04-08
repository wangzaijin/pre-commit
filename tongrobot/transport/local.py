"""LocalTransport — direct ROS2 communication.

ROS2 packages (rclpy, sensor_msgs, …) are imported lazily so that the
SDK can be imported on machines without a sourced ROS2 environment.
"""

from __future__ import annotations

import threading
import time
from typing import Any, Callable, Dict, List, Optional

import numpy as np

from tongrobot.core.config import ConfigLoader, HardwareDescriptor
from tongrobot.exceptions import TimeoutError
from tongrobot.transport.base import TransportBase
from tongrobot.types import (
    BaseState,
    CameraFrame,
    IMUData,
    JointCommand,
    JointState,
    LaserScan,
    RobotState,
    Transform,
    VelocityCommand,
)


class LocalTransport(TransportBase):
    """Transport that creates a rclpy node and talks directly to the ROS2 graph."""

    def __init__(self) -> None:
        self._node: Any = None
        self._executor: Any = None
        self._spin_thread: Optional[threading.Thread] = None
        self._lock = threading.Lock()
        self._config: Dict[str, Any] = {}
        self._hw: Optional[HardwareDescriptor] = None

        # Caches
        self._odom_cache: Optional[Any] = None  # nav_msgs.msg.Odometry
        self._camera_caches: Dict[str, Any] = {}  # camera_name -> sensor_msgs.msg.Image
        self._scan_caches: Dict[str, Any] = (
            {}
        )  # lidar_name -> sensor_msgs.msg.LaserScan
        self._imu_caches: Dict[str, Any] = {}  # imu_name -> sensor_msgs.msg.Imu
        self._joint_caches: Dict[str, Any] = (
            {}
        )  # arm_name -> sensor_msgs.msg.JointState

        # Publishers / subscribers
        self._cmd_vel_pub: Any = None
        self._tf_buffer: Any = None
        self._tf_listener: Any = None

        # State callbacks
        self._state_callbacks: List[Callable] = []

        self._connected = False

    # -------------------------------------------------------------------------
    # connect / disconnect
    # -------------------------------------------------------------------------

    def connect(self, config: Dict[str, Any]) -> None:
        """Initialise rclpy, create the node, subscribe to all configured topics."""
        # Lazy ROS2 imports — only runs when a LocalTransport is actually used.
        import rclpy
        import rclpy.executors
        import rclpy.node
        import tf2_ros
        from geometry_msgs.msg import Twist
        from nav_msgs.msg import Odometry
        from sensor_msgs.msg import Image, Imu
        from sensor_msgs.msg import LaserScan as RosLaserScan
        from sensor_msgs.msg import JointState as RosJointState

        self._config = config

        # Build hardware descriptor from config
        loader = ConfigLoader.__new__(ConfigLoader)
        loader._data = config
        loader._hardware = HardwareDescriptor(config["hardware"])
        self._hw = loader._hardware

        # --- rclpy init -------------------------------------------------------
        if not rclpy.ok():
            rclpy.init()

        self._node = rclpy.node.Node("tongrobot_client")

        # --- Subscribers ------------------------------------------------------
        # Base odometry
        if self._hw.base:
            self._node.create_subscription(
                Odometry,
                self._hw.base.odom_topic,
                self._odom_callback,
                10,
            )

        # Cameras
        for name, cam in self._hw.cameras.items():
            self._camera_caches[name] = None
            self._node.create_subscription(
                Image,
                cam.topic,
                lambda msg, n=name: self._camera_callback(n, msg),
                10,
            )

        # LiDARs
        for name, lidar in self._hw.lidars.items():
            self._scan_caches[name] = None
            self._node.create_subscription(
                RosLaserScan,
                lidar.topic,
                lambda msg, n=name: self._scan_callback(n, msg),
                10,
            )

        # IMUs
        for name, imu in self._hw.imus.items():
            self._imu_caches[name] = None
            self._node.create_subscription(
                Imu,
                imu.topic,
                lambda msg, n=name: self._imu_callback(n, msg),
                10,
            )

        # Arm joint states
        for name, arm in self._hw.arms.items():
            self._joint_caches[name] = None
            self._node.create_subscription(
                RosJointState,
                arm.joint_state_topic,
                lambda msg, n=name: self._joint_callback(n, msg),
                10,
            )

        # --- Publisher --------------------------------------------------------
        if self._hw.base:
            self._cmd_vel_pub = self._node.create_publisher(
                Twist, self._hw.base.cmd_vel_topic, 10
            )

        # --- TF2 --------------------------------------------------------------
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node)

        # --- Spin thread ------------------------------------------------------
        self._executor = rclpy.executors.MultiThreadedExecutor()
        self._executor.add_node(self._node)
        self._spin_thread = threading.Thread(
            target=self._executor.spin, daemon=True, name="tongrobot_spin"
        )
        self._spin_thread.start()

        self._connected = True

    def disconnect(self) -> None:
        """Stop spinning, destroy the node, shut down rclpy."""
        import rclpy

        self._connected = False
        if self._executor:
            self._executor.shutdown()
        if self._spin_thread:
            self._spin_thread.join(timeout=2.0)
        if self._node:
            self._node.destroy_node()
        rclpy.shutdown()

        self._node = None
        self._executor = None
        self._spin_thread = None

    def is_connected(self) -> bool:
        return self._connected

    # -------------------------------------------------------------------------
    # ROS2 callbacks (run on executor thread)
    # -------------------------------------------------------------------------

    def _odom_callback(self, msg: Any) -> None:
        with self._lock:
            self._odom_cache = msg
        # Notify state subscribers
        if self._state_callbacks:
            state = self._build_robot_state(msg)
            for cb in self._state_callbacks:
                try:
                    cb(state)
                except Exception:
                    pass

    def _camera_callback(self, name: str, msg: Any) -> None:
        with self._lock:
            self._camera_caches[name] = msg

    def _scan_callback(self, name: str, msg: Any) -> None:
        with self._lock:
            self._scan_caches[name] = msg

    def _imu_callback(self, name: str, msg: Any) -> None:
        with self._lock:
            self._imu_caches[name] = msg

    def _joint_callback(self, name: str, msg: Any) -> None:
        with self._lock:
            self._joint_caches[name] = msg

    # -------------------------------------------------------------------------
    # TransportBase methods
    # -------------------------------------------------------------------------

    def get_robot_state(self) -> RobotState:
        with self._lock:
            odom = self._odom_cache

        ts = time.time()

        # Base state from odometry
        base_state: Optional[BaseState] = None
        if odom is not None:
            pos = odom.pose.pose.position
            ori = odom.pose.pose.orientation
            # Convert quaternion to yaw (theta)
            import math

            siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
            cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
            theta = math.atan2(siny_cosp, cosy_cosp)
            twist = odom.twist.twist
            base_state = BaseState(
                position=np.array([pos.x, pos.y, theta]),
                velocity=np.array([twist.linear.x, twist.linear.y, twist.angular.z]),
                timestamp=_stamp_to_float(odom.header.stamp),
            )
            ts = base_state.timestamp

        # Arm joint states
        arms: Dict[str, JointState] = {}
        with self._lock:
            joint_caches = dict(self._joint_caches)
        for arm_name, msg in joint_caches.items():
            if msg is not None:
                arms[arm_name] = JointState(
                    positions=np.array(msg.position),
                    velocities=np.array(msg.velocity),
                    efforts=np.array(msg.effort),
                    timestamp=_stamp_to_float(msg.header.stamp),
                )

        return RobotState(arms=arms, base=base_state, grippers={}, timestamp=ts)

    def _build_robot_state(self, odom_msg: Any) -> RobotState:
        """Fast path — build RobotState already holding the lock."""
        import math

        ori = odom_msg.pose.pose.orientation
        pos = odom_msg.pose.pose.position
        siny_cosp = 2.0 * (ori.w * ori.z + ori.x * ori.y)
        cosy_cosp = 1.0 - 2.0 * (ori.y * ori.y + ori.z * ori.z)
        theta = math.atan2(siny_cosp, cosy_cosp)
        twist = odom_msg.twist.twist
        base_state = BaseState(
            position=np.array([pos.x, pos.y, theta]),
            velocity=np.array([twist.linear.x, twist.linear.y, twist.angular.z]),
            timestamp=_stamp_to_float(odom_msg.header.stamp),
        )
        return RobotState(
            arms={},
            base=base_state,
            grippers={},
            timestamp=base_state.timestamp,
        )

    def get_camera_frame(self, camera_name: str) -> CameraFrame:
        with self._lock:
            msg = self._camera_caches.get(camera_name)
        if msg is None:
            raise TimeoutError(
                f"No data received from camera '{camera_name}' yet. "
                f"Is the topic publishing? "
                f"Check with: ros2 topic hz {self._hw.cameras[camera_name].topic}"
            )
        return _ros_image_to_camera_frame(msg)

    def get_laser_scan(self, lidar_name: str) -> LaserScan:
        with self._lock:
            msg = self._scan_caches.get(lidar_name)
        if msg is None:
            raise TimeoutError(
                f"No data received from lidar '{lidar_name}' yet. "
                f"Is the topic publishing? "
                f"Check with: ros2 topic hz {self._hw.lidars[lidar_name].topic}"
            )
        return LaserScan(
            ranges=np.array(msg.ranges, dtype=np.float32),
            angle_min=msg.angle_min,
            angle_max=msg.angle_max,
            angle_increment=msg.angle_increment,
            timestamp=_stamp_to_float(msg.header.stamp),
        )

    def get_imu(self, imu_name: str) -> IMUData:
        with self._lock:
            msg = self._imu_caches.get(imu_name)
        if msg is None:
            raise TimeoutError(
                f"No data received from IMU '{imu_name}' yet. "
                f"Is the topic publishing? "
                f"Check with: ros2 topic hz {self._hw.imus[imu_name].topic}"
            )
        q = msg.orientation
        av = msg.angular_velocity
        la = msg.linear_acceleration
        return IMUData(
            orientation=np.array([q.x, q.y, q.z, q.w]),
            angular_velocity=np.array([av.x, av.y, av.z]),
            linear_acceleration=np.array([la.x, la.y, la.z]),
            timestamp=_stamp_to_float(msg.header.stamp),
        )

    def set_velocity_command(self, cmd: VelocityCommand) -> None:
        from geometry_msgs.msg import Twist

        if self._cmd_vel_pub is None:
            return
        twist = Twist()
        twist.linear.x = cmd.linear_x
        twist.linear.y = cmd.linear_y
        twist.angular.z = cmd.angular_z
        self._cmd_vel_pub.publish(twist)

    def set_joint_command(self, arm_name: str, cmd: JointCommand) -> None:
        # Arm control not needed for TurtleBot3; subclasses override this.
        pass

    def subscribe_robot_state(self, callback: Callable[[RobotState], None]) -> None:
        self._state_callbacks.append(callback)

    def subscribe_camera(
        self,
        camera_name: str,
        callback: Callable[[CameraFrame], None],
        fps: int = 30,
    ) -> None:
        interval = 1.0 / max(fps, 1)

        def _loop() -> None:
            while self._connected:
                try:
                    frame = self.get_camera_frame(camera_name)
                    callback(frame)
                except TimeoutError:
                    pass
                time.sleep(interval)

        t = threading.Thread(
            target=_loop, daemon=True, name=f"cam_stream_{camera_name}"
        )
        t.start()

    def get_transform(self, target_frame: str, source_frame: str) -> Transform:
        import rclpy.time

        if self._tf_buffer is None:
            raise RuntimeError("TF2 buffer not initialised — call connect() first.")
        try:
            tf = self._tf_buffer.lookup_transform(
                target_frame, source_frame, rclpy.time.Time()
            )
        except Exception as exc:
            raise RuntimeError(
                f"Could not look up TF from '{source_frame}' to '{target_frame}': {exc}"
            ) from exc

        t = tf.transform.translation
        r = tf.transform.rotation
        return Transform(
            position=np.array([t.x, t.y, t.z]),
            quaternion=np.array([r.x, r.y, r.z, r.w]),
        )


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------


def _stamp_to_float(stamp: Any) -> float:
    """Convert a ROS2 Time stamp to a Python float (seconds)."""
    return stamp.sec + stamp.nanosec * 1e-9


def _ros_image_to_camera_frame(msg: Any) -> CameraFrame:
    """Convert a sensor_msgs/Image to a CameraFrame with a numpy array."""
    try:
        import cv_bridge  # type: ignore

        bridge = cv_bridge.CvBridge()
        cv_img = bridge.imgmsg_to_cv2(msg, desired_encoding="rgb8")
        return CameraFrame(
            data=cv_img, timestamp=_stamp_to_float(msg.header.stamp), encoding="rgb8"
        )
    except ImportError:
        pass

    # Manual conversion fallback
    import cv2

    encoding = msg.encoding.lower()
    dtype = np.uint8

    if encoding in ("rgb8", "bgr8", "mono8"):
        channels = 1 if encoding == "mono8" else 3
    elif encoding in ("rgba8", "bgra8"):
        channels = 4
    else:
        channels = 3  # best-effort

    raw = np.frombuffer(msg.data, dtype=dtype)
    img = raw.reshape(
        (msg.height, msg.width, channels) if channels > 1 else (msg.height, msg.width)
    )

    # Convert BGR → RGB if needed
    if encoding == "bgr8":
        img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
    elif encoding == "bgra8":
        img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGBA)

    return CameraFrame(
        data=img, timestamp=_stamp_to_float(msg.header.stamp), encoding="rgb8"
    )
