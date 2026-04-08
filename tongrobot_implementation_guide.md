# TongRobot Minimal System — Implementation Guide for Claude Code Agent

## Purpose of this document

This document provides step-by-step instructions for a Claude Code agent to implement a minimal working TongRobot system covering Phase 1 (Foundation), Phase 2 (Remote Operation), and Phase 3 (Web Dashboard). The demonstration platform is a **TurtleBot3 with a camera and differential-drive mobile base** running in Gazebo simulation.

**Critical rule:** Do NOT generate large blocks of code all at once. Work incrementally — implement one module, test it, then move to the next. Each step below should be a separate task.

---

## Reference documents

Before starting any step, read the architecture specification file `tongrobot_architecture.md` in the project root. It contains the full system design, project structure, config schema, API surface, data types, and transport layer design. Every implementation decision must align with that document.

---

## Prerequisites

Before beginning implementation, verify that the following are installed and working on the development machine:

- Python 3.10 or higher
- ROS2 Humble (or newer LTS release) with `ros2 topic`, `ros2 service`, `ros2 launch` working
- Gazebo (the version bundled with your ROS2 distro)
- TurtleBot3 simulation packages (`turtlebot3_gazebo`, `turtlebot3_description`)
- Node.js 18+ and npm (for the web dashboard frontend)
- `pip`, `grpcio-tools`, `protobuf` compiler (`protoc`)

Set the environment variable `TURTLEBOT3_MODEL=burger` (or `waffle` if you want a camera) before launching any simulation.

---

## Phase 1 — Foundation

The goal of Phase 1 is to build the core Python SDK that can control a TurtleBot3 running locally (on the same machine) via direct ROS2 communication.

### Step 1.1 — Project scaffolding

Create the directory structure as specified in the architecture document section 3 ("Project structure"). For the minimal system, you only need these directories initially:

```
tongrobot/
├── tongrobot/
│   ├── __init__.py
│   ├── core/
│   ├── transport/
│   ├── types/
│   └── utils/
├── tongrobot_bridge/
├── tongrobot_dashboard/
├── proto/
├── configs/
├── examples/
├── pyproject.toml
└── README.md
```

Create all `__init__.py` files in every Python package directory. Each `__init__.py` should be empty for now except `tongrobot/__init__.py`, which should export the `TongRobot` class (add the import after the class exists).

Create `pyproject.toml` with:
- Project name: `tongrobot`
- Python requirement: `>=3.10`
- Dependencies: `numpy`, `pyyaml`, `grpcio`, `grpcio-tools`, `protobuf`, `websockets`, `opencv-python`
- The ROS2 Python packages (`rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`) are NOT listed as pip dependencies — they come from the ROS2 installation and are available when the ROS2 environment is sourced.
- Make the package installable in editable mode with `pip install -e .`

Create a minimal `README.md` with the project name and one-line description.

### Step 1.2 — Data types

Implement the data type module under `tongrobot/types/`. Refer to architecture document section 11 ("Data types") for the exact dataclass definitions.

Create three files:

**`robot_state.py`** — Define these dataclasses:
- `JointState` with fields: `positions` (np.ndarray), `velocities` (np.ndarray), `efforts` (np.ndarray), `timestamp` (float).
- `BaseState` with fields: `position` (np.ndarray of shape (3,) for x, y, theta), `velocity` (np.ndarray of shape (3,) for vx, vy, omega), `timestamp` (float).
- `GripperState` with fields: `position` (float), `is_open` (bool), `timestamp` (float).
- `RobotState` with fields: `arms` (Dict[str, JointState]), `base` (Optional[BaseState]), `grippers` (Dict[str, GripperState]), `timestamp` (float).
- `Pose` with fields: `position` (np.ndarray shape (3,)), `quaternion` (np.ndarray shape (4,) in xyzw order).
- `Transform` with fields: `position` (np.ndarray shape (3,)), `quaternion` (np.ndarray shape (4,)), and a `matrix` property that computes the 4x4 homogeneous transform.

**`sensor_data.py`** — Define:
- `CameraFrame` with fields: `data` (np.ndarray of shape (H, W, C) uint8), `timestamp` (float), `encoding` (str).
- `LaserScan` with fields: `ranges` (np.ndarray), `angle_min` (float), `angle_max` (float), `angle_increment` (float), `timestamp` (float).
- `IMUData` with fields: `orientation` (np.ndarray (4,)), `angular_velocity` (np.ndarray (3,)), `linear_acceleration` (np.ndarray (3,)), `timestamp` (float).

**`commands.py`** — Define:
- `VelocityCommand` with fields: `linear_x` (float), `linear_y` (float), `angular_z` (float).
- `JointCommand` with fields: `positions` (Optional[np.ndarray]), `velocities` (Optional[np.ndarray]).

All dataclasses should use `@dataclass` decorator. Add proper type hints. Import numpy as np. Create an `__init__.py` in the types directory that re-exports all types.

### Step 1.3 — Configuration system

Implement `tongrobot/core/config.py`. Refer to architecture document section 4 ("Configuration schema") for the YAML structure.

**`ConfigLoader` class:**
- Constructor takes a file path string to a YAML file.
- Parses the YAML file using `pyyaml`.
- Validates that required top-level keys exist: `robot`, `connection`, `hardware`.
- Raises `ConfigError` (define this in `tongrobot/exceptions.py`) if validation fails.
- Provides accessor properties: `robot_name`, `connection_config`, `hardware_config`, `dashboard_config`, `safety_config`.

**`HardwareDescriptor` class:**
- Built by `ConfigLoader` from the `hardware` section of the YAML.
- Holds structured information about arms (list), grippers (list), base (optional), and sensors (cameras, lidar, imu, force_torque — each a list).
- Each arm descriptor should be a dataclass with fields: `name`, `num_joints`, `joint_names`, `joint_state_topic`, `joint_cmd_topic`, `joint_limits`, `control_rate_hz`.
- Each camera descriptor: `name`, `topic`, `info_topic`, `depth_topic` (optional), `encoding`, `resolution`.
- Base descriptor: `type`, `cmd_vel_topic`, `odom_topic`, `max_linear_vel`, `max_angular_vel`.
- Provides methods like `get_arm(name)`, `get_camera(name)`, `get_all_topics()` for easy lookup.

Also create `tongrobot/exceptions.py` with these exception classes: `TongRobotError` (base), `ConfigError`, `ConnectionError`, `TimeoutError`, `SafetyViolation`, `HardwareError`. Each inherits from `TongRobotError`.

### Step 1.4 — TurtleBot3 configuration file

Create `configs/turtlebot3.yaml` — the configuration file for a TurtleBot3 Waffle running in Gazebo. This is the reference config for all testing.

Key details for TurtleBot3 Waffle in Gazebo:
- Robot name: `turtlebot3_waffle`
- Connection mode: `local` (for Phase 1)
- No arms, no grippers
- Base: type `differential`, cmd_vel_topic `/cmd_vel`, odom_topic `/odom`, max_linear_vel `0.26`, max_angular_vel `1.82`
- Cameras: one camera named `main_camera`, topic `/camera/image_raw`, info_topic `/camera/camera_info`, encoding `rgb8`, resolution `[1920, 1080]` (Waffle has a camera)
- LiDAR: one named `base_scan`, topic `/scan`, type `2d`
- IMU: one named `base_imu`, topic `/imu`
- Dashboard: enabled on port 3000, default panels for camera and base velocity
- Safety: enable velocity limit check, watchdog_timeout_ms 500

Also include the `connection.remote` section with placeholder values (host `192.168.1.100`, grpc_port `50051`, ws_port `8765`) so the same config can be switched to remote mode in Phase 2.

### Step 1.5 — Transport base class

Implement `tongrobot/transport/base.py`. This is the abstract interface that all transports must implement. Refer to architecture document section 6 ("Transport layer design").

Define `TransportBase` as an abstract base class (use `abc.ABC` and `@abstractmethod`) with these methods:
- `connect(config: dict) -> None`
- `disconnect() -> None`
- `is_connected() -> bool`
- `get_robot_state() -> RobotState`
- `get_camera_frame(camera_name: str) -> CameraFrame`
- `get_laser_scan(lidar_name: str) -> LaserScan`
- `get_imu(imu_name: str) -> IMUData`
- `set_velocity_command(cmd: VelocityCommand) -> None`
- `set_joint_command(arm_name: str, cmd: JointCommand) -> None`
- `subscribe_robot_state(callback: Callable) -> None`
- `subscribe_camera(camera_name: str, callback: Callable, fps: int = 30) -> None`
- `get_transform(target_frame: str, source_frame: str) -> Transform`

Each method should have a docstring explaining its purpose, parameters, and return type.

### Step 1.6 — Local transport

Implement `tongrobot/transport/local.py` — the `LocalTransport` class that extends `TransportBase`.

This is the most important transport and the one used for all Phase 1 testing. It creates a `rclpy` node internally and communicates directly with the ROS2 graph.

**Implementation details:**

`connect()`:
- Call `rclpy.init()` if not already initialized (use a try/except or check).
- Create a `rclpy.node.Node` named `tongrobot_client`.
- Read the hardware descriptor from the config to know which topics to subscribe to.
- For each configured camera: create a subscriber to the camera topic. Use `sensor_msgs.msg.Image`. Store the latest frame in a thread-safe cache (use `threading.Lock`).
- For the base odom topic: create a subscriber to `nav_msgs.msg.Odometry`. Cache the latest odometry.
- For each LiDAR: subscribe to `sensor_msgs.msg.LaserScan`. Cache latest scan.
- For each IMU: subscribe to `sensor_msgs.msg.Imu`. Cache latest reading.
- Create a publisher for the base `cmd_vel` topic using `geometry_msgs.msg.Twist`.
- Create a TF2 buffer and listener: `tf2_ros.Buffer` and `tf2_ros.TransformListener`.
- Spin the node in a background daemon thread using `rclpy.executors.MultiThreadedExecutor` so that callbacks are processed without blocking the user's main thread.

`get_robot_state()`:
- Return the latest cached `RobotState`. For the TurtleBot3 demo, `arms` will be an empty dict, `grippers` will be empty, and `base` will contain the latest odometry data converted to a `BaseState`.

`get_camera_frame(camera_name)`:
- Look up the camera name in the cache. Convert the ROS2 `Image` message to a numpy array. Handle encoding conversion (e.g., `bgr8` to `rgb8` using OpenCV if needed). Return a `CameraFrame`.

`get_laser_scan(lidar_name)`:
- Return the latest cached scan data as a `LaserScan` dataclass.

`set_velocity_command(cmd)`:
- Create a `Twist` message from the `VelocityCommand`, clamp values to the configured max velocities from `HardwareDescriptor`, and publish to the cmd_vel topic.

`get_transform(target, source)`:
- Use the TF2 buffer to look up the transform. Convert to a `Transform` dataclass.

`disconnect()`:
- Shut down subscribers, publishers, stop the spin thread, destroy the node, and call `rclpy.shutdown()`.

**Important implementation notes:**
- Use `cv_bridge` if available, otherwise manually convert ROS2 image messages to numpy arrays by reshaping `msg.data` based on `msg.height`, `msg.width`, and `msg.step`.
- All cached data must be protected by locks since ROS2 callbacks run on the executor thread while user code runs on the main thread.
- If a sensor has not yet received any data, `get_camera_frame()` and similar methods should raise `TimeoutError` with a clear message like "No data received from camera 'main_camera' yet. Is the topic publishing?".

### Step 1.7 — Utility modules

Implement these utility modules:

**`tongrobot/utils/rate.py`** — A `Rate` class that provides loop timing:
- Constructor takes `frequency_hz` (float).
- `sleep()` method that sleeps for the remaining time to maintain the target frequency. Use `time.monotonic()` for timing. Account for the time already elapsed since the last `sleep()` call.
- If the loop is running slower than the target rate, do not sleep (do not try to "catch up") but optionally log a warning.

**`tongrobot/utils/transforms.py`** — Transform utility functions:
- `quaternion_to_matrix(q: np.ndarray) -> np.ndarray`: Convert quaternion (xyzw) to 3x3 rotation matrix.
- `matrix_to_quaternion(R: np.ndarray) -> np.ndarray`: Convert 3x3 rotation matrix to quaternion (xyzw).
- `pose_to_matrix(pose: Pose) -> np.ndarray`: Convert a Pose to a 4x4 homogeneous matrix.
- `matrix_to_pose(matrix: np.ndarray) -> Pose`: Convert a 4x4 matrix back to a Pose.
- These are pure numpy implementations — do not use external transform libraries.

**`tongrobot/utils/logging.py`** — A thin wrapper around Python's `logging` module:
- `get_logger(name: str) -> logging.Logger`: Returns a logger with a consistent format including timestamp and module name.
- The log level should be configurable from the YAML config's `logging.level` field.

### Step 1.8 — Core manager classes

Implement the three manager classes under `tongrobot/core/`. These are thin wrappers around the transport that provide the user-friendly API.

**`tongrobot/core/state.py` — `StateManager`:**
- Constructor takes a `TransportBase` instance and a `HardwareDescriptor`.
- `get_state() -> RobotState`: Delegates to `transport.get_robot_state()`.
- `get_base_state() -> BaseState`: Convenience method that extracts the base state from `get_state()`.
- `get_arm_state(arm_name: str) -> JointState`: Convenience method for arm joint state. Raise `ConfigError` if no arm with that name exists in the hardware descriptor.
- `get_transform(target_frame, source_frame) -> Transform`: Delegates to transport.

**`tongrobot/core/sensor.py` — `SensorManager`:**
- Constructor takes a `TransportBase` instance and a `HardwareDescriptor`.
- `get_camera(camera_name: str) -> np.ndarray`: Returns the raw image as a numpy array (H, W, C). Raises `ConfigError` if the camera name is not in the hardware descriptor.
- `get_camera_frame(camera_name: str) -> CameraFrame`: Returns the full `CameraFrame` dataclass with timestamp and encoding.
- `get_depth(camera_name: str) -> np.ndarray`: Returns depth image if the camera has a `depth_topic` configured. Raise `ConfigError` if no depth topic.
- `get_laser_scan(lidar_name: str) -> LaserScan`: Delegates to transport.
- `get_imu(imu_name: str) -> IMUData`: Delegates to transport.
- `on_camera(camera_name: str, callback: Callable, fps: int = 30)`: Registers a callback for streaming camera data.

**`tongrobot/core/motion.py` — `MotionManager`:**
- Constructor takes a `TransportBase` instance, a `HardwareDescriptor`, and the safety config dict.
- `set_base_velocity(linear: float, angular: float)`: Creates a `VelocityCommand` and sends it. Before sending, clamp linear and angular values to the max velocities defined in the hardware descriptor. Log a warning if clamping occurs.
- `set_joint_positions(arm_name: str, positions: np.ndarray)`: Validate the arm name, check joint limits from the hardware descriptor (raise `SafetyViolation` if out of range), then send via transport.
- `stop()`: Send a zero velocity command to the base and zero commands to all arms.
- `emergency_stop()`: Call `stop()` and set an internal e-stop flag that prevents any further motion commands until `reset_estop()` is called.

### Step 1.9 — TongRobot main class

Implement `tongrobot/core/robot.py` — the `TongRobot` class. This is the main entry point that users interact with.

**Constructor** `__init__(self, config_path: str)`:
- Load the config using `ConfigLoader`.
- Build the `HardwareDescriptor`.
- Do NOT connect yet — just store the config.

**`connect()`**:
- Based on `connection.mode` in the config:
  - If `"local"`: instantiate `LocalTransport`.
  - If `"remote"`: instantiate `gRPCTransport` (this will exist after Phase 2; for now, raise `NotImplementedError` with a clear message).
- Call `transport.connect(config)`.
- Create `StateManager`, `SensorManager`, and `MotionManager` using the transport and hardware descriptor.
- Log "Connected to {robot_name}" at INFO level.

**Public API methods** — these are convenience methods that delegate to the managers:
- `get_state() -> RobotState` → `self._state_manager.get_state()`
- `get_camera(camera_name) -> np.ndarray` → `self._sensor_manager.get_camera(camera_name)`
- `get_camera_frame(camera_name) -> CameraFrame` → delegates to sensor manager
- `get_laser_scan(lidar_name) -> LaserScan` → delegates to sensor manager
- `get_imu(imu_name) -> IMUData` → delegates to sensor manager
- `set_base_velocity(linear, angular)` → delegates to motion manager
- `set_joint_positions(arm_name, positions)` → delegates to motion manager
- `stop()` → delegates to motion manager
- `emergency_stop()` → delegates to motion manager
- `get_transform(target, source) -> Transform` → delegates to state manager
- `create_rate(hz) -> Rate` → returns a new `Rate(hz)`
- `is_connected() -> bool` → delegates to transport
- `on_camera(camera_name, callback, fps=30)` → delegates to sensor manager

**`disconnect()`**: Calls `motion_manager.stop()` then `transport.disconnect()`. Logs "Disconnected from {robot_name}".

**Context manager support**: Implement `__enter__` (calls `connect()`, returns self) and `__exit__` (calls `disconnect()`).

**Update `tongrobot/__init__.py`**: Import and re-export `TongRobot` from `tongrobot.core.robot`, and re-export all data types from `tongrobot.types`.

### Step 1.10 — Phase 1 testing

Create `examples/01_turtlebot_basic.py` — a script that demonstrates the Phase 1 functionality:

This script should:
1. Launch a TurtleBot3 Waffle in Gazebo (print instructions to the user to run `ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py` in a separate terminal).
2. Import `TongRobot` from `tongrobot`.
3. Instantiate with `configs/turtlebot3.yaml`.
4. Use the context manager: `with TongRobot("configs/turtlebot3.yaml") as robot:`.
5. Get and print the robot state (base position, velocity).
6. Get a camera frame, print its shape and dtype.
7. Get a laser scan, print the number of range readings.
8. Drive the robot forward at 0.1 m/s for 3 seconds using a rate loop at 10 Hz.
9. Stop the robot.
10. Print "Phase 1 demo complete!".

Run this script against the Gazebo simulation to verify everything works. Debug and fix any issues until the full demo runs cleanly.

---

## Phase 2 — Remote operation

The goal of Phase 2 is to add the gRPC transport layer so that the SDK can control the robot from a remote machine. The bridge node runs on the robot (or the machine running Gazebo) and the SDK connects over the network.

### Step 2.1 — Protobuf definitions

Create the protobuf definition files under `proto/`. These define the wire format for all gRPC communication.

**`proto/robot_state.proto`**:
- Package name: `tongrobot`
- Message `JointStateMsg`: repeated double `positions`, repeated double `velocities`, repeated double `efforts`, double `timestamp`.
- Message `BaseStateMsg`: repeated double `position` (3 elements: x, y, theta), repeated double `velocity` (3 elements: vx, vy, omega), double `timestamp`.
- Message `GripperStateMsg`: double `position`, bool `is_open`, double `timestamp`.
- Message `RobotStateMsg`: map<string, JointStateMsg> `arms`, BaseStateMsg `base`, map<string, GripperStateMsg> `grippers`, double `timestamp`, bool `has_base`.

**`proto/sensor_data.proto`**:
- Message `CameraFrameMsg`: bytes `data` (JPEG-compressed image bytes), int32 `width`, int32 `height`, int32 `channels`, double `timestamp`, string `encoding`, bool `is_compressed`.
- Message `LaserScanMsg`: repeated float `ranges`, float `angle_min`, float `angle_max`, float `angle_increment`, double `timestamp`.
- Message `IMUMsg`: repeated double `orientation` (4), repeated double `angular_velocity` (3), repeated double `linear_acceleration` (3), double `timestamp`.

**`proto/commands.proto`**:
- Message `VelocityCommandMsg`: double `linear_x`, double `linear_y`, double `angular_z`.
- Message `JointCommandMsg`: string `arm_name`, repeated double `positions`, repeated double `velocities`, bool `has_positions`, bool `has_velocities`.

**`proto/bridge_service.proto`**:
- Import the other three proto files.
- Message `Empty` (empty message).
- Message `StreamRequest`: string `name` (sensor/arm name), int32 `fps` (for cameras).
- Message `TransformRequest`: string `target_frame`, string `source_frame`.
- Message `TransformMsg`: repeated double `position` (3), repeated double `quaternion` (4), double `timestamp`.
- Message `CommandAck`: bool `success`, string `message`.
- Service `TongRobotBridge` with these RPCs:
  - `GetRobotState(Empty) returns (RobotStateMsg)` — unary, snapshot read
  - `StreamRobotState(Empty) returns (stream RobotStateMsg)` — server streaming
  - `GetCameraFrame(StreamRequest) returns (CameraFrameMsg)` — unary, snapshot
  - `StreamCameraFrames(StreamRequest) returns (stream CameraFrameMsg)` — server streaming
  - `GetLaserScan(StreamRequest) returns (LaserScanMsg)` — unary
  - `GetIMU(StreamRequest) returns (IMUMsg)` — unary
  - `SetVelocityCommand(VelocityCommandMsg) returns (CommandAck)` — unary
  - `SetJointCommand(JointCommandMsg) returns (CommandAck)` — unary
  - `GetTransform(TransformRequest) returns (TransformMsg)` — unary
  - `EmergencyStop(Empty) returns (CommandAck)` — unary

Create a build script `proto/build.sh` that runs `python -m grpc_tools.protoc` to compile all `.proto` files. The output should go to `tongrobot/proto/` (create this directory). The script should:
1. Create the output directory `tongrobot/proto/` if it does not exist.
2. Run protoc for each proto file with `--python_out` and `--grpc_python_out` pointing to `tongrobot/proto/`.
3. Create an `__init__.py` in `tongrobot/proto/`.
4. Fix the import paths in the generated `*_pb2_grpc.py` files — protoc generates absolute imports like `import robot_state_pb2`, but these need to be relative imports like `from . import robot_state_pb2` to work correctly as a package. Use `sed` or a small Python script for this fixup.

Run the build script and verify that the generated Python files import without errors.

### Step 2.2 — Conversion utilities

Create `tongrobot/proto/converters.py` — functions that convert between the Python dataclasses (from `tongrobot/types/`) and the protobuf messages (from the generated code).

Implement pairs of conversion functions:
- `robot_state_to_proto(state: RobotState) -> RobotStateMsg` and `proto_to_robot_state(msg: RobotStateMsg) -> RobotState`
- `camera_frame_to_proto(frame: CameraFrame, compress: bool = True, jpeg_quality: int = 80) -> CameraFrameMsg` and `proto_to_camera_frame(msg: CameraFrameMsg) -> CameraFrame`
- `laser_scan_to_proto(scan: LaserScan) -> LaserScanMsg` and `proto_to_laser_scan(msg: LaserScanMsg) -> LaserScan`
- `imu_to_proto(imu: IMUData) -> IMUMsg` and `proto_to_imu(msg: IMUMsg) -> IMUData`
- `velocity_cmd_to_proto(cmd: VelocityCommand) -> VelocityCommandMsg` and `proto_to_velocity_cmd(msg: VelocityCommandMsg) -> VelocityCommand`
- `transform_to_proto(tf: Transform) -> TransformMsg` and `proto_to_transform(msg: TransformMsg) -> Transform`

For camera frame compression: use `cv2.imencode('.jpg', frame.data, [cv2.IMWRITE_JPEG_QUALITY, jpeg_quality])` to compress, and `cv2.imdecode(np.frombuffer(data, np.uint8), cv2.IMREAD_COLOR)` to decompress.

### Step 2.3 — Bridge node (gRPC server)

Implement the bridge node under `tongrobot_bridge/`. This is a ROS2 node that runs on the robot's machine and exposes robot data via gRPC.

**`tongrobot_bridge/tongrobot_bridge/bridge_node.py`:**

This file is both a ROS2 node and a gRPC server. Here is how to structure it:

The `BridgeNode` class:
- Inherits from `rclpy.node.Node`.
- Constructor takes the config file path as a parameter.
- Uses `ConfigLoader` and `HardwareDescriptor` from the SDK to know which topics to subscribe to (just like `LocalTransport`, but now it also serves data via gRPC).
- Creates all the same ROS2 subscribers and publishers as `LocalTransport` (odom, cameras, lidar, imu, cmd_vel), storing latest data in a thread-safe cache.
- Creates a TF2 buffer and listener.

The `BridgeServicer` class:
- Inherits from the generated `TongRobotBridgeServicer`.
- Holds a reference to the `BridgeNode` so it can read cached data.
- Implements every RPC defined in `bridge_service.proto`:
  - `GetRobotState`: Read cached state, convert to proto, return.
  - `StreamRobotState`: Loop that yields `RobotStateMsg` at ~50 Hz. Use `time.sleep` between yields. Check `context.is_active()` to stop when client disconnects.
  - `GetCameraFrame`: Read cached frame, compress to JPEG, convert to proto, return.
  - `StreamCameraFrames`: Loop that yields compressed frames at the requested fps. Respect `context.is_active()`.
  - `GetLaserScan`: Read cached scan, convert to proto, return.
  - `GetIMU`: Read cached IMU, convert to proto, return.
  - `SetVelocityCommand`: Convert proto to `VelocityCommand`, publish to cmd_vel topic on the ROS2 side.
  - `SetJointCommand`: Convert proto, publish to the arm's command topic. (For TurtleBot3 demo, this is a no-op that returns success since there are no arms.)
  - `GetTransform`: Use TF2 buffer to look up, convert to proto, return.
  - `EmergencyStop`: Publish zero velocity, set an e-stop flag.

**Main entry point** (at the bottom of `bridge_node.py` or in a `main()` function):
1. Parse command line argument for config file path.
2. Call `rclpy.init()`.
3. Create `BridgeNode`.
4. Create a gRPC server with `grpc.server(futures.ThreadPoolExecutor(max_workers=10))`.
5. Add the `BridgeServicer` to the server.
6. Bind the server to `0.0.0.0:{grpc_port}` from the config.
7. Start the gRPC server.
8. Spin the ROS2 node in the current thread (use `rclpy.spin()`).
9. On shutdown (KeyboardInterrupt), stop the gRPC server and destroy the node.

**`tongrobot_bridge/launch/bridge.launch.py`:**
- Create a ROS2 launch file that launches the bridge node.
- Accept a `config` launch argument for the config file path.
- Pass it to the bridge node as a parameter or command line argument.

**`tongrobot_bridge/package.xml` and `tongrobot_bridge/setup.py`:**
- Create standard ROS2 package files.
- Package name: `tongrobot_bridge`.
- Dependencies: `rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`, `tf2_ros`.
- Register the bridge node entry point in `setup.py` under `console_scripts`.

### Step 2.4 — gRPC transport (client side)

Implement `tongrobot/transport/grpc_transport.py` — the `gRPCTransport` class that extends `TransportBase`.

This is the SDK-side client that connects to the bridge node's gRPC server.

**`connect(config)`**:
- Read host and grpc_port from `config["connection"]["remote"]`.
- Create a `grpc.insecure_channel(f"{host}:{port}")`.
- Create a `TongRobotBridgeStub` from the generated gRPC code.
- Test the connection by calling `GetRobotState` with a short timeout. If it fails, raise `ConnectionError` with the host and port in the message.
- Store the channel and stub.

**`get_robot_state()`**:
- Call `stub.GetRobotState(Empty())`.
- Convert the response using `proto_to_robot_state()`.
- Return the `RobotState`.

**`get_camera_frame(camera_name)`**:
- Call `stub.GetCameraFrame(StreamRequest(name=camera_name))`.
- Convert using `proto_to_camera_frame()`.
- Return the `CameraFrame`.

**`get_laser_scan(lidar_name)`**: Call `stub.GetLaserScan(StreamRequest(name=lidar_name))`, convert, return.

**`get_imu(imu_name)`**: Call `stub.GetIMU(StreamRequest(name=imu_name))`, convert, return.

**`set_velocity_command(cmd)`**:
- Convert to proto using `velocity_cmd_to_proto(cmd)`.
- Call `stub.SetVelocityCommand(proto_msg)`.
- Check the `CommandAck` response; if not success, log a warning.

**`set_joint_command(arm_name, cmd)`**: Convert to proto, call `stub.SetJointCommand()`.

**`subscribe_robot_state(callback)`**:
- Start a background thread that calls `stub.StreamRobotState(Empty())`.
- In the thread, iterate over the stream, convert each message, and call the callback.
- Handle `grpc.RpcError` for disconnections — log the error and attempt to reconnect.

**`subscribe_camera(camera_name, callback, fps)`**:
- Start a background thread that calls `stub.StreamCameraFrames(StreamRequest(name=camera_name, fps=fps))`.
- Iterate, convert, call callback. Handle disconnections.

**`get_transform(target, source)`**: Call `stub.GetTransform(TransformRequest(...))`, convert, return.

**`disconnect()`**: Close the gRPC channel. Stop all streaming threads (use threading events to signal them to stop).

**`is_connected()`**: Return True if the channel is open and a ping to `GetRobotState` succeeds.

### Step 2.5 — Wire up remote mode in TongRobot

Update `tongrobot/core/robot.py`:
- In the `connect()` method, when `connection.mode == "remote"`, instantiate `gRPCTransport` instead of `LocalTransport`.
- Everything else stays the same — the manager classes don't know or care which transport is being used.

### Step 2.6 — Phase 2 testing

Create `examples/02_turtlebot_remote.py` — a script that demonstrates remote operation.

This script should print instructions explaining the two-terminal setup:

Terminal 1 (robot side):
1. Launch TurtleBot3 in Gazebo.
2. Launch the bridge node: `ros2 launch tongrobot_bridge bridge.launch.py config:=configs/turtlebot3.yaml`

Terminal 2 (remote side):
1. Modify `configs/turtlebot3.yaml` to set `connection.mode: "remote"` and set `host` to `localhost` (for same-machine testing) or the robot machine's IP.
2. Run the example script.

The script itself should do the same things as `01_turtlebot_basic.py` but over gRPC:
1. Connect using the remote config.
2. Read and print robot state.
3. Get a camera frame, print its shape (verifying JPEG decompression worked).
4. Drive the robot forward for 3 seconds.
5. Stop and disconnect.

Also create a config file variant `configs/turtlebot3_remote.yaml` that is identical to `turtlebot3.yaml` but with `connection.mode: "remote"` and `host: "localhost"`.

Test this by running both terminals on the same machine first (using `localhost`), then if possible, on two different machines on the same network.

---

## Phase 3 — Web dashboard

The goal of Phase 3 is to build a web-based dashboard that connects to the bridge node via WebSocket, allowing users to monitor robot state and send simple commands from a browser.

### Step 3.1 — WebSocket server in the bridge node

Extend the bridge node to also run a WebSocket server alongside the gRPC server. This serves the dashboard frontend.

**Add to `bridge_node.py`:**

Add a WebSocket server using Python's `websockets` library (or `aiohttp` for combined HTTP + WebSocket).

Choose `aiohttp` because it can serve both:
- Static files (the built dashboard frontend)
- WebSocket connections (for live data streaming)

The WebSocket server should:
- Run on the port specified in `config.dashboard.ws_port` (use the same port as the dashboard, or a separate ws_port field — for simplicity, run the HTTP static server and WebSocket on the same port).
- Accept WebSocket connections at the `/ws` path.
- Handle incoming JSON messages with a `type` field that determines the action:
  - `{"type": "subscribe", "topic": "robot_state"}` — start streaming robot state to this client at 10 Hz.
  - `{"type": "subscribe", "topic": "camera", "name": "main_camera", "fps": 10}` — start streaming camera frames (JPEG-compressed, base64-encoded) to this client.
  - `{"type": "subscribe", "topic": "laser_scan", "name": "base_scan"}` — start streaming laser scan data.
  - `{"type": "unsubscribe", "topic": "..."}` — stop streaming a topic.
  - `{"type": "cmd_vel", "linear": 0.1, "angular": 0.0}` — send a velocity command.
  - `{"type": "stop"}` — send zero velocity.
  - `{"type": "get_config"}` — return the dashboard configuration (which panels to show by default).
- Send outgoing JSON messages:
  - `{"type": "robot_state", "data": {...}}` — serialized base state with position, velocity.
  - `{"type": "camera_frame", "name": "...", "data": "<base64 JPEG>", "timestamp": ...}` — camera frame as base64 JPEG.
  - `{"type": "laser_scan", "name": "...", "data": {...}}` — laser scan ranges.
  - `{"type": "config", "data": {...}}` — the dashboard config section from the YAML.

The WebSocket server should support multiple simultaneous clients. Each client maintains its own set of subscriptions. Use asyncio tasks to manage per-client streaming loops.

**Important**: The bridge node now runs three things concurrently:
1. ROS2 node spinning (receives/publishes ROS2 messages)
2. gRPC server (serves SDK clients)
3. aiohttp server (serves dashboard frontend + WebSocket)

Use threading to combine them: run the asyncio event loop (for aiohttp) in a separate thread, and run `rclpy.spin()` in the main thread. The gRPC server runs in its own thread pool (this is already the case from Phase 2). Shared data (the caches) is already thread-safe with locks.

Alternatively, run the aiohttp event loop as the main loop and spin rclpy in a background thread. Choose whichever is cleaner.

### Step 3.2 — Static file serving

Configure the aiohttp server to also serve static files from a directory. When the dashboard frontend is built (Step 3.5), the built files go into a `static/` directory. The aiohttp server should serve `index.html` for the root path `/` and all other static assets from that directory.

For development, before the frontend is built, the server should return a simple "Dashboard not built yet" HTML page at `/`.

The static files directory should be configurable — either a path relative to the bridge node package or an absolute path. A reasonable default is `tongrobot_dashboard/frontend/dist/`.

### Step 3.3 — Dashboard frontend scaffolding

Create the React + TypeScript frontend under `tongrobot_dashboard/frontend/`.

Use Vite as the build tool (fast, modern, easy setup).

Initialize the project:
- `package.json` with dependencies: `react`, `react-dom`, `typescript`, `@vitejs/plugin-react`, `vite`
- Additional dependencies: `react-grid-layout` (for draggable/resizable panel layout), `recharts` (for charts/graphs)
- `tsconfig.json` with strict mode
- `vite.config.ts` that builds to `dist/`
- `index.html` entry point

Create the following source file structure:
```
src/
├── App.tsx              # Main app with panel layout
├── main.tsx             # React entry point
├── index.css            # Global styles
├── services/
│   └── websocket.ts     # WebSocket client singleton
├── stores/
│   └── robotStore.ts    # State management (use React context or zustand)
├── components/
│   ├── PanelLayout.tsx  # Grid layout manager
│   ├── PanelToolbar.tsx # Add/remove panel controls
│   ├── CameraView.tsx   # Camera feed panel
│   ├── BaseStatePanel.tsx  # Base position/velocity display
│   ├── LaserScanPanel.tsx  # 2D laser scan visualization
│   ├── CommandPanel.tsx    # Velocity command controls
│   └── ConnectionStatus.tsx # Connection indicator
└── types/
    └── index.ts         # TypeScript type definitions
```

### Step 3.4 — WebSocket client service

Implement `src/services/websocket.ts`:

This is a singleton class that manages the WebSocket connection to the bridge node.

Key behavior:
- `connect(url: string)`: Connect to `ws://{host}:{port}/ws`. Implement automatic reconnection with exponential backoff (start at 1 second, max 10 seconds).
- `disconnect()`: Close the connection.
- `subscribe(topic: string, options?: object)`: Send a subscribe message.
- `unsubscribe(topic: string)`: Send an unsubscribe message.
- `sendCommand(type: string, payload: object)`: Send a command message (like cmd_vel).
- `onMessage(type: string, callback: Function)`: Register a handler for a specific message type. Return an unsubscribe function.
- Connection state: expose `isConnected` as a reactive property.
- On reconnection, re-send all active subscriptions.

### Step 3.5 — Robot state store

Implement `src/stores/robotStore.ts`:

Use React Context + useReducer (or zustand for simplicity — add it as a dependency if preferred) to manage:
- `baseState`: latest base position and velocity
- `cameraFrames`: map of camera name to latest base64 JPEG string
- `laserScan`: latest scan data
- `isConnected`: WebSocket connection status
- `dashboardConfig`: the panel configuration received from the server

The store should:
- Connect to the WebSocket service on initialization.
- Subscribe to robot_state, and update `baseState` on every message.
- Provide methods for components to subscribe/unsubscribe to cameras and laser scans.
- Persist panel layout to `localStorage` so it survives page refreshes.

### Step 3.6 — Dashboard components

Implement each panel component. Each panel should be a self-contained React component that receives its data from the store and renders a visualization.

**`ConnectionStatus.tsx`:**
- A small indicator in the top-right corner showing connection status.
- Green dot + "Connected" when WebSocket is open.
- Red dot + "Disconnected" with a reconnecting spinner when closed.
- Display the connected robot name (from config).

**`BaseStatePanel.tsx`:**
- Displays the robot's current x, y, theta position and linear/angular velocity.
- Use a simple table layout with labels and values.
- Values should update in real-time (at the rate robot_state is streamed, ~10 Hz).
- Format numbers to 3 decimal places.
- Optionally: show a simple 2D trail of the robot's past positions using an SVG or canvas element. Keep the last 200 positions in a ring buffer.

**`CameraView.tsx`:**
- Displays a live camera feed.
- Receives the camera name as a prop.
- On mount, subscribe to the camera stream via the WebSocket service.
- On unmount, unsubscribe.
- Render the base64 JPEG as an `<img>` element with `src={data:image/jpeg;base64,...}`.
- Show the camera name and current FPS in a small overlay.
- Handle the case where no frames are received (show a "No signal" placeholder).

**`LaserScanPanel.tsx`:**
- Displays a top-down 2D visualization of the laser scan.
- Use an SVG or Canvas element.
- Draw the scan as a series of points (or lines from center to each range reading) in a polar layout, converted to Cartesian coordinates.
- The robot should be at the center.
- Scale the visualization to fit the panel.
- Show the number of range readings in a small overlay.

**`CommandPanel.tsx`:**
- Provides controls for sending velocity commands to the robot.
- Two sliders: one for linear velocity (-max to +max), one for angular velocity (-max to +max). The max values should come from the config.
- A "Send" button that continuously sends the current slider values at 10 Hz while pressed.
- A "Stop" button that sends zero velocity.
- Alternatively: a virtual joystick (use mouse drag or touch drag on a 2D area where X maps to angular and Y maps to linear velocity).
- Display the currently commanded linear and angular velocity.

**`PanelLayout.tsx`:**
- Uses `react-grid-layout` to arrange panels in a resizable, draggable grid.
- Reads the default layout from `dashboardConfig` received from the server.
- Persists user-customized layout to `localStorage`.
- Each panel is wrapped in a card with a title bar showing the panel type and a close (X) button.

**`PanelToolbar.tsx`:**
- A toolbar (top of the page or a sidebar) with an "Add Panel" button.
- Clicking "Add Panel" shows a dropdown/modal with available panel types: Camera, Base State, Laser Scan, Command.
- For camera panels, also show a sub-selection for which camera (from config).
- Adding a panel appends it to the grid layout.

### Step 3.7 — App assembly

Implement `src/App.tsx`:
- Render the `ConnectionStatus` at the top.
- Render the `PanelToolbar`.
- Render the `PanelLayout` containing all active panels.
- On mount:
  1. Connect the WebSocket to the bridge node. The WebSocket URL should be derived from the current page URL (same host, same port, path `/ws`) so it works without hardcoding.
  2. Request the config from the server (`{"type": "get_config"}`).
  3. Subscribe to robot_state.
  4. Render default panels from the config.

Implement `src/main.tsx`: Standard React 18 entry point with `createRoot`.

Implement `src/index.css`: Clean, minimal CSS. Dark theme preferred (dark background, light text, subtle borders). Use CSS variables for theming. Style the panel cards with subtle borders and rounded corners. The dashboard should feel professional and functional, not flashy.

### Step 3.8 — Build and integrate

Create a build script `tongrobot_dashboard/build.sh` that:
1. `cd` into `tongrobot_dashboard/frontend/`
2. Run `npm install`
3. Run `npm run build`
4. The output goes to `tongrobot_dashboard/frontend/dist/`

Update the bridge node to serve files from this `dist/` directory. After building, the dashboard should be accessible at `http://{robot_ip}:{dashboard_port}/` in any browser.

### Step 3.9 — Phase 3 testing

Create `examples/03_dashboard_demo.py` — a script that:
1. Prints instructions to launch TurtleBot3 in Gazebo and the bridge node.
2. Prints the dashboard URL.
3. Optionally: runs a simple autonomous behavior (drive in a square pattern) so the dashboard has something interesting to display.

Test procedure:
1. Launch Gazebo with TurtleBot3.
2. Launch the bridge node.
3. Open the dashboard URL in a browser.
4. Verify that:
   - Connection status shows "Connected".
   - Base state panel shows updating position/velocity values.
   - Camera panel shows a live feed from the simulation.
   - Laser scan panel shows the 2D scan visualization.
   - Command panel can drive the robot (click "forward" or use joystick, see the robot move in Gazebo).
   - Panels can be rearranged by dragging, resized, and closed.
   - "Add Panel" adds new panels.
   - Layout persists after page refresh.

---

## Final integration test

After completing all three phases, run this end-to-end test:

1. Launch TurtleBot3 Waffle in Gazebo.
2. Launch the bridge node with the TurtleBot3 config.
3. Open the web dashboard in a browser and verify live data.
4. In a separate terminal, run a Python script that:
   - Connects to the robot using the remote config.
   - Reads robot state and camera images.
   - Drives the robot in a circle for 10 seconds.
   - Observe the movement in both Gazebo and the web dashboard simultaneously.
5. While the Python script is running, use the dashboard's Command Panel to override the velocity (verify that the dashboard can also send commands).
6. Verify that the Python script and dashboard can coexist (both connected to the bridge node simultaneously).

---

## Implementation checklist

Use this checklist to track progress:

- [ ] Step 1.1 — Project scaffolding
- [ ] Step 1.2 — Data types
- [ ] Step 1.3 — Configuration system
- [ ] Step 1.4 — TurtleBot3 config file
- [ ] Step 1.5 — Transport base class
- [ ] Step 1.6 — Local transport
- [ ] Step 1.7 — Utility modules
- [ ] Step 1.8 — Core manager classes
- [ ] Step 1.9 — TongRobot main class
- [ ] Step 1.10 — Phase 1 testing
- [ ] Step 2.1 — Protobuf definitions
- [ ] Step 2.2 — Conversion utilities
- [ ] Step 2.3 — Bridge node (gRPC server)
- [ ] Step 2.4 — gRPC transport (client side)
- [ ] Step 2.5 — Wire up remote mode
- [ ] Step 2.6 — Phase 2 testing
- [ ] Step 3.1 — WebSocket server in bridge node
- [ ] Step 3.2 — Static file serving
- [ ] Step 3.3 — Dashboard frontend scaffolding
- [ ] Step 3.4 — WebSocket client service
- [ ] Step 3.5 — Robot state store
- [ ] Step 3.6 — Dashboard components
- [ ] Step 3.7 — App assembly
- [ ] Step 3.8 — Build and integrate
- [ ] Step 3.9 — Phase 3 testing
- [ ] Final integration test

---

## Important notes for the Claude Code agent

1. **Work step by step.** Complete each step fully before moving to the next. Each step should result in working, tested code.

2. **Test early and often.** After completing each step that produces runnable code (especially Steps 1.6, 1.10, 2.3, 2.6, 3.9), run the code and fix all issues before proceeding.

3. **Do not over-engineer.** This is a minimal system. Do not add features not described in these steps. No plugin system, no async API, no multi-robot support yet — those are Phase 4.

4. **Handle imports carefully.** ROS2 packages (`rclpy`, `sensor_msgs`, etc.) are only available when the ROS2 environment is sourced. The SDK should import them lazily (only inside `LocalTransport` and the bridge node, not at the top level of the SDK package) so the SDK can be imported even without ROS2 for remote-only use.

5. **Thread safety.** Every piece of shared state (sensor caches, state caches) must be protected by `threading.Lock`. The ROS2 executor, gRPC server, and WebSocket server all run on separate threads.

6. **Error messages.** Every exception should include actionable information: what went wrong, what the user should check, and what the expected state is. For example: "Cannot connect to bridge at 192.168.1.100:50051. Is the bridge node running? Check with: ros2 node list".

7. **Logging.** Use the logging utility from Step 1.7. Log at INFO level for connections, disconnections, and mode switches. Log at DEBUG level for individual message sends/receives. Log at WARNING for degraded conditions (slow loop rate, dropped frames). Log at ERROR for failures.

8. **Config validation.** When a user calls `robot.get_camera("nonexistent")`, the error should say "Camera 'nonexistent' not found in config. Available cameras: main_camera" — not a KeyError stacktrace.

9. **Graceful shutdown.** `Ctrl+C` should cleanly shut down all threads, close all connections, and stop the robot (send zero velocity) before exiting. Use `signal` handlers and `threading.Event` for coordinated shutdown.

10. **The TurtleBot3 Waffle model is the reference platform.** The Burger model does not have a camera. Always use `TURTLEBOT3_MODEL=waffle` for testing. Mention this in all example scripts and documentation.
