# TongRobot System Architecture Specification

## 1. Vision and design principles

TongRobot is a **hardware-agnostic robotic middleware** that wraps ROS2 complexity behind a clean Python SDK. It targets researchers and engineers who want to control diverse robot platforms (mobile manipulators, humanoids, quadrupeds, etc.) without writing boilerplate ROS2 code — especially when heavy computation runs on a remote workstation rather than the robot's onboard computer.

**Core design principles:**

- **Hardware agnostic.** A single `config.yaml` defines the robot's joints, sensors, and communication topology. Switching platforms means swapping the config, not rewriting code.
- **Location transparent.** The same Python API works whether the SDK runs on the robot or on a remote desktop across the network. The transport layer handles the difference.
- **Pythonic first.** No ROS2 boilerplate leaks into user code. Users import `TongRobot`, instantiate with a config, and call methods. Numpy arrays in, numpy arrays out.
- **Observable by default.** A built-in web dashboard lets developers monitor robot state, visualize sensor data, and send test commands — configurable per-user, not hardcoded.
- **Extensible.** A plugin system allows adding custom sensors, controllers, or dashboard widgets without modifying core code.

---

## 2. System layers

The architecture is organized into five layers, each with a clear responsibility boundary.

### Layer 1 — User application layer

This is where your AI pipelines, task planners, teleoperation scripts, or learning algorithms live. It has no knowledge of ROS2, DDS, or transport protocols. It only talks to the TongRobot SDK.

### Layer 2 — TongRobot Python SDK (`tongrobot`)

The SDK is the core deliverable. It provides:

- `TongRobot` — the main entry point class
- `StateManager` — read-only access to robot state (joint positions, velocities, end-effector poses, battery, errors)
- `SensorManager` — access to camera frames, point clouds, IMU, force/torque, and custom sensor streams
- `MotionManager` — send joint commands, Cartesian targets, velocity commands, trajectories, and gripper actions
- `ConfigLoader` — parses `config.yaml` and builds a `HardwareDescriptor`
- `ConnectionManager` — selects transport (local ROS2, gRPC, WebSocket) based on config
- `PluginManager` — discovers and loads extension modules

### Layer 3 — Communication / transport layer

This layer makes local and remote operation transparent:

- `LocalTransport` — direct ROS2 node communication (when SDK runs on-robot)
- `gRPCTransport` — high-performance remote procedure calls with protobuf serialization (for command/state loops at 100+ Hz)
- `WebSocketTransport` — browser-friendly channel for the web dashboard and lightweight remote monitoring

### Layer 4 — ROS2 bridge node (`tongrobot_bridge`)

A ROS2 node that runs on the robot's onboard computer. It:

- Subscribes to all configured ROS2 topics (joint states, sensor data, TF) and relays them upstream
- Exposes ROS2 services and actions as gRPC/WebSocket endpoints
- Publishes commands received from the SDK back into the ROS2 graph
- Serves the web dashboard's static files and WebSocket API

### Layer 5 — Hardware abstraction layer

Standard ROS2 driver nodes for the specific hardware (arm controllers, base drivers, camera nodes, LiDAR nodes). TongRobot doesn't replace these — it sits above them.

---

## 3. Project structure

```
tongrobot/
├── tongrobot/                      # Python SDK package
│   ├── __init__.py                 # Exports TongRobot class
│   ├── core/
│   │   ├── robot.py                # TongRobot main class
│   │   ├── config.py               # ConfigLoader, HardwareDescriptor
│   │   ├── state.py                # StateManager
│   │   ├── sensor.py               # SensorManager
│   │   ├── motion.py               # MotionManager
│   │   └── plugin.py               # PluginManager
│   ├── transport/
│   │   ├── base.py                 # TransportBase abstract class
│   │   ├── local.py                # LocalTransport (direct ROS2)
│   │   ├── grpc_transport.py       # gRPCTransport
│   │   └── ws_transport.py         # WebSocketTransport
│   ├── types/
│   │   ├── robot_state.py          # RobotState, JointState, Pose dataclasses
│   │   ├── sensor_data.py          # CameraFrame, PointCloud, IMUData, etc.
│   │   └── commands.py             # JointCommand, CartesianCommand, etc.
│   ├── utils/
│   │   ├── transforms.py           # SE3, quaternion utilities
│   │   ├── rate.py                 # Rate limiter for control loops
│   │   └── logging.py              # Structured logging
│   └── plugins/                    # Built-in plugins
│       ├── recorder.py             # Data recording plugin
│       └── safety.py               # Joint limit / collision checking
│
├── tongrobot_bridge/               # ROS2 package (runs on robot)
│   ├── tongrobot_bridge/
│   │   ├── bridge_node.py          # Main bridge ROS2 node
│   │   ├── topic_relay.py          # Topic subscription/forwarding
│   │   ├── service_proxy.py        # ROS2 service ↔ gRPC mapping
│   │   ├── action_proxy.py         # ROS2 action ↔ gRPC mapping
│   │   └── tf_buffer.py            # TF2 tree buffering and forwarding
│   ├── launch/
│   │   └── bridge.launch.py        # ROS2 launch file
│   ├── package.xml
│   └── setup.py
│
├── tongrobot_dashboard/            # Web dashboard
│   ├── frontend/                   # React/Vue SPA
│   │   ├── src/
│   │   │   ├── components/
│   │   │   │   ├── JointStatePanel.tsx
│   │   │   │   ├── CameraView.tsx
│   │   │   │   ├── CommandPanel.tsx
│   │   │   │   ├── TFTreeViewer.tsx
│   │   │   │   └── PluginWidgetSlot.tsx
│   │   │   ├── stores/             # State management
│   │   │   ├── services/           # WebSocket client
│   │   │   └── App.tsx
│   │   └── package.json
│   └── backend/
│       └── dashboard_server.py     # Serves static + WebSocket API
│
├── proto/                          # Protobuf definitions
│   ├── robot_state.proto
│   ├── sensor_data.proto
│   ├── commands.proto
│   └── bridge_service.proto
│
├── configs/                        # Example robot configs
│   ├── mobile_manipulator.yaml
│   ├── humanoid.yaml
│   └── dual_arm.yaml
│
├── examples/
│   ├── 01_basic_control.py
│   ├── 02_camera_capture.py
│   ├── 03_remote_planning.py
│   └── 04_custom_plugin.py
│
├── pyproject.toml
└── README.md
```

---

## 4. Configuration schema (`config.yaml`)

The configuration file is the single source of truth for a robot platform. It defines what hardware exists, how to reach it, and what the dashboard should show.

```yaml
# config.yaml — Mobile manipulator example
robot:
  name: "my_mobile_manipulator"
  description: "UR5e on Clearpath Husky"

# Connection settings
connection:
  mode: "remote"                    # "local" | "remote"
  remote:
    host: "192.168.1.100"           # Robot onboard IP
    grpc_port: 50051
    ws_port: 8765
  local:
    ros_domain_id: 0

# Hardware description
hardware:
  arms:
    - name: "right_arm"
      num_joints: 6
      joint_names: ["shoulder_pan", "shoulder_lift", "elbow",
                     "wrist_1", "wrist_2", "wrist_3"]
      joint_state_topic: "/right_arm/joint_states"
      joint_cmd_topic: "/right_arm/joint_commands"
      joint_limits:
        position_min: [-6.28, -6.28, -3.14, -6.28, -6.28, -6.28]
        position_max: [6.28, 6.28, 3.14, 6.28, 6.28, 6.28]
        velocity_max: [3.14, 3.14, 3.14, 6.28, 6.28, 6.28]
      end_effector_frame: "right_ee_link"
      control_modes: ["position", "velocity", "trajectory"]
      control_rate_hz: 125

  grippers:
    - name: "right_gripper"
      type: "parallel"              # "parallel" | "vacuum" | "dexterous"
      action_server: "/right_gripper/gripper_action"
      open_value: 0.04
      close_value: 0.0

  base:
    type: "differential"            # "differential" | "omnidirectional" | "legged"
    cmd_vel_topic: "/cmd_vel"
    odom_topic: "/odom"
    max_linear_vel: 1.0
    max_angular_vel: 2.0

  sensors:
    cameras:
      - name: "wrist_camera"
        topic: "/wrist_cam/color/image_raw"
        info_topic: "/wrist_cam/color/camera_info"
        encoding: "rgb8"
        resolution: [640, 480]
      - name: "head_camera"
        topic: "/head_cam/color/image_raw"
        info_topic: "/head_cam/color/camera_info"
        depth_topic: "/head_cam/depth/image_rect_raw"
        encoding: "rgb8"
        resolution: [1280, 720]
    lidar:
      - name: "front_lidar"
        topic: "/scan"
        type: "2d"
    imu:
      - name: "base_imu"
        topic: "/imu/data"
    force_torque:
      - name: "wrist_ft"
        topic: "/right_arm/ft_sensor"

# Dashboard configuration
dashboard:
  enabled: true
  port: 3000
  default_panels:
    - type: "joint_state"
      source: "right_arm"
    - type: "camera"
      source: "wrist_camera"
    - type: "camera"
      source: "head_camera"
  # Users can add/remove panels at runtime via the dashboard UI

# Safety
safety:
  enable_joint_limit_check: true
  enable_velocity_limit_check: true
  emergency_stop_topic: "/emergency_stop"
  watchdog_timeout_ms: 200          # Stop robot if no cmd received

# Logging
logging:
  level: "INFO"
  record_bag: false
  bag_topics: ["joint_states", "camera"]

# Plugins
plugins:
  - name: "recorder"
    enabled: false
    config:
      output_dir: "./recordings"
  - name: "safety"
    enabled: true
```

---

## 5. Python SDK API design

### 5.1 Core usage pattern

```python
from tongrobot import TongRobot
import numpy as np

# Initialize — config defines everything
robot = TongRobot("./config.yaml")

# Connect to the robot (local or remote, based on config)
robot.connect()

# --- Read state ---
state = robot.get_state()
print(state.arms["right_arm"].joint_positions)   # np.ndarray (6,)
print(state.arms["right_arm"].joint_velocities)  # np.ndarray (6,)
print(state.base.position)                       # np.ndarray (3,) [x, y, theta]
print(state.base.velocity)                       # np.ndarray (3,) [vx, vy, omega]

# --- Read sensors ---
rgb_image = robot.get_camera("wrist_camera")             # np.ndarray (H, W, 3)
depth_image = robot.get_depth("head_camera")              # np.ndarray (H, W) float32
point_cloud = robot.get_point_cloud("head_camera")        # np.ndarray (N, 3)
imu_data = robot.get_imu("base_imu")                      # IMUData dataclass

# --- Send commands ---
target_joints = np.array([0.0, -1.57, 1.57, -1.57, -1.57, 0.0])
robot.set_joint_positions("right_arm", target_joints)

# Cartesian command (SE3 pose)
from tongrobot.types import Pose
target_pose = Pose(position=[0.5, 0.0, 0.3],
                   quaternion=[0.0, 0.707, 0.0, 0.707])
robot.set_ee_pose("right_arm", target_pose)

# Base velocity
robot.set_base_velocity(linear=0.5, angular=0.0)

# Gripper
robot.close_gripper("right_gripper")
robot.open_gripper("right_gripper")

# --- Trajectory execution ---
waypoints = [
    np.array([0.0, -1.0, 1.0, -1.0, -1.57, 0.0]),
    np.array([0.5, -1.2, 1.2, -0.8, -1.57, 0.0]),
    np.array([1.0, -1.57, 1.57, -1.57, -1.57, 0.0]),
]
robot.execute_trajectory("right_arm", waypoints, duration=3.0)

# --- Cleanup ---
robot.disconnect()
```

### 5.2 Streaming / callback pattern

For real-time control loops and continuous sensor streams:

```python
from tongrobot import TongRobot
import numpy as np

robot = TongRobot("./config.yaml")
robot.connect()

# Option A: Polling loop with rate control
rate = robot.create_rate(100)  # 100 Hz
while robot.is_connected():
    state = robot.get_state()
    image = robot.get_camera("wrist_camera")

    # Your control logic here
    cmd = my_controller(state, image)
    robot.set_joint_positions("right_arm", cmd)

    rate.sleep()

# Option B: Callback-based streaming
def on_joint_state(arm_name: str, joint_state: JointState):
    print(f"{arm_name}: {joint_state.positions}")

def on_camera_frame(cam_name: str, frame: np.ndarray):
    print(f"{cam_name}: {frame.shape}")

robot.on_joint_state("right_arm", on_joint_state)
robot.on_camera("wrist_camera", on_camera_frame, fps=30)

# Callbacks run in background threads; main thread can do other work
robot.spin()  # Block until disconnect
```

### 5.3 Context manager and async support

```python
# Context manager — auto connect/disconnect
with TongRobot("./config.yaml") as robot:
    state = robot.get_state()
    robot.set_joint_positions("right_arm", target)

# Async support for integration with asyncio-based AI frameworks
import asyncio
from tongrobot import AsyncTongRobot

async def main():
    robot = AsyncTongRobot("./config.yaml")
    await robot.connect()

    state = await robot.get_state()
    image = await robot.get_camera("wrist_camera")

    # Parallel sensor reads
    rgb, depth, imu = await asyncio.gather(
        robot.get_camera("wrist_camera"),
        robot.get_depth("head_camera"),
        robot.get_imu("base_imu"),
    )

    await robot.disconnect()

asyncio.run(main())
```

### 5.4 Multi-robot support

```python
from tongrobot import TongRobot

robot_a = TongRobot("./robot_a.yaml")
robot_b = TongRobot("./robot_b.yaml")

robot_a.connect()
robot_b.connect()

# Coordinate between robots
state_a = robot_a.get_state()
state_b = robot_b.get_state()

# Each instance is fully independent
robot_a.set_joint_positions("left_arm", cmd_a)
robot_b.set_base_velocity(linear=0.3, angular=0.0)
```

### 5.5 TF (transform) access

```python
# Get transform between any two frames
from tongrobot.types import Transform

tf = robot.get_transform(
    target_frame="right_ee_link",
    source_frame="base_link"
)
print(tf.position)      # np.ndarray (3,)
print(tf.quaternion)     # np.ndarray (4,) [x, y, z, w]
print(tf.matrix)         # np.ndarray (4, 4) homogeneous transform
```

---

## 6. Transport layer design

The transport layer implements a common `TransportBase` interface so the SDK doesn't know or care whether it's talking to a local ROS2 graph or a remote bridge node.

```python
# transport/base.py
from abc import ABC, abstractmethod
from typing import Callable, Optional
import numpy as np

class TransportBase(ABC):
    """Abstract transport interface. All transports implement this."""

    @abstractmethod
    def connect(self, config: dict) -> None: ...

    @abstractmethod
    def disconnect(self) -> None: ...

    @abstractmethod
    def get_joint_state(self, arm_name: str) -> JointState: ...

    @abstractmethod
    def set_joint_command(self, arm_name: str, cmd: JointCommand) -> None: ...

    @abstractmethod
    def get_camera_frame(self, cam_name: str) -> np.ndarray: ...

    @abstractmethod
    def subscribe_joint_state(self, arm_name: str,
                               callback: Callable) -> None: ...

    @abstractmethod
    def subscribe_camera(self, cam_name: str,
                          callback: Callable, fps: int = 30) -> None: ...

    @abstractmethod
    def call_service(self, service_name: str,
                      request: dict) -> dict: ...

    @abstractmethod
    def execute_action(self, action_name: str,
                        goal: dict,
                        feedback_cb: Optional[Callable] = None) -> dict: ...

    @abstractmethod
    def get_transform(self, target: str, source: str) -> Transform: ...
```

**Transport selection logic:**

| `connection.mode` | Transport used | Where SDK runs | Where bridge runs |
|---|---|---|---|
| `local` | `LocalTransport` | On robot | Not needed |
| `remote` | `gRPCTransport` | Remote desktop | On robot |

The `WebSocketTransport` is used exclusively by the dashboard and lightweight monitoring tools (not for high-frequency control).

### gRPC service definition

```protobuf
// proto/bridge_service.proto
syntax = "proto3";
package tongrobot;

service TongRobotBridge {
  // State streaming
  rpc StreamJointStates(StreamRequest) returns (stream JointStateMsg);
  rpc StreamCameraFrames(CameraStreamRequest) returns (stream CameraFrame);
  rpc StreamIMU(StreamRequest) returns (stream IMUMsg);

  // Snapshot reads
  rpc GetRobotState(Empty) returns (RobotStateMsg);
  rpc GetTransform(TransformRequest) returns (TransformMsg);

  // Commands
  rpc SetJointCommand(JointCommandMsg) returns (CommandAck);
  rpc SetBaseVelocity(BaseVelocityMsg) returns (CommandAck);
  rpc ExecuteTrajectory(TrajectoryMsg) returns (TrajectoryResult);

  // Service/action proxying
  rpc CallROSService(ROSServiceRequest) returns (ROSServiceResponse);
  rpc ExecuteROSAction(ROSActionGoal) returns (stream ROSActionFeedback);

  // Gripper
  rpc GripperAction(GripperGoal) returns (GripperResult);
}
```

---

## 7. ROS2 bridge node

The bridge node is the only component that touches ROS2 directly. It runs on the robot's onboard computer and exposes the full ROS2 graph through gRPC and WebSocket.

**Responsibilities:**

- **TopicRelay** — subscribes to all topics listed in the config, serializes the latest message, and serves it to gRPC/WS clients. Uses a ring buffer to keep the latest N messages for each topic.
- **ServiceProxy** — accepts gRPC calls, translates them to ROS2 service calls, and returns the result.
- **ActionProxy** — accepts gRPC action goals, creates ROS2 action clients, and streams feedback/result back.
- **TFBuffer** — maintains a `tf2_ros.Buffer` and serves transform lookups to remote clients.
- **CommandPublisher** — receives commands from gRPC, applies safety checks, and publishes to the appropriate ROS2 command topics.
- **DashboardServer** — serves the web dashboard's static files and handles WebSocket connections for browser clients.

**Launch:**

```bash
# On the robot
ros2 launch tongrobot_bridge bridge.launch.py config:=./config.yaml
```

---

## 8. Web dashboard

The dashboard is a single-page web application that connects to the bridge node via WebSocket. Its key design feature is **user-configurable panels** — it ships with sensible defaults but lets users add, remove, resize, and rearrange visualization panels.

### Default panel types (built-in)

- **Joint state panel** — live bar chart of joint positions/velocities/efforts for any arm
- **Camera view** — live video feed from any configured camera
- **3D viewer** — URDF visualization with live TF updates (using three.js)
- **Command panel** — sliders for each joint, base velocity joystick, gripper open/close buttons
- **TF tree viewer** — interactive tree of all TF frames
- **System health** — connection status, control frequency, latency, battery level
- **Log viewer** — streaming log output from the bridge node

### User customization

The dashboard persists a per-user panel layout in browser local storage. Users can:

- Add panels from a palette of available types
- Remove panels they don't need
- Resize and drag panels on a grid layout
- Configure each panel's data source (which arm, which camera, etc.)
- Add custom panels via the plugin system

### Dashboard config in `config.yaml`

```yaml
dashboard:
  enabled: true
  port: 3000
  default_panels:
    - type: "joint_state"
      source: "right_arm"
      position: { row: 0, col: 0, width: 6, height: 4 }
    - type: "camera"
      source: "wrist_camera"
      position: { row: 0, col: 6, width: 6, height: 4 }
```

This defines the default layout for new users. Once a user customizes their layout, their preferences take precedence.

---

## 9. Plugin system

Plugins extend TongRobot without modifying core code. They can add new sensor types, custom controllers, dashboard widgets, or data processing pipelines.

```python
# Plugin interface
from tongrobot.core.plugin import PluginBase

class MyPlugin(PluginBase):
    """Example plugin that logs joint states to a CSV file."""

    name = "csv_logger"

    def on_load(self, robot: "TongRobot", config: dict):
        self.output_dir = config.get("output_dir", "./logs")
        self.file = open(f"{self.output_dir}/joints.csv", "w")

    def on_state_update(self, state: RobotState):
        # Called every time robot state updates
        positions = state.arms["right_arm"].joint_positions
        self.file.write(",".join(map(str, positions)) + "\n")

    def on_unload(self):
        self.file.close()
```

**Plugin registration in config:**

```yaml
plugins:
  - name: "csv_logger"
    enabled: true
    config:
      output_dir: "./experiment_logs"
```

**Plugin discovery:** Plugins are discovered via Python entry points (`pyproject.toml`) or by placing them in `~/.tongrobot/plugins/`.

---

## 10. Safety layer

Safety is a non-optional subsystem, not a plugin. It runs as an interceptor on every outgoing command.

**Safety checks (configurable per-robot):**

- **Joint position limits** — reject commands outside the configured min/max range
- **Joint velocity limits** — clamp velocity commands and check trajectory velocities
- **Watchdog timeout** — if no command is received within `watchdog_timeout_ms`, the bridge automatically sends a zero-velocity command (soft stop)
- **Emergency stop** — publish to the e-stop topic; the bridge node monitors this topic and halts all motion immediately
- **Workspace limits** — optional bounding box or sphere check on end-effector position

```python
# Emergency stop from SDK
robot.emergency_stop()

# Check if robot is in a safe state
if robot.is_safe():
    robot.set_joint_positions("right_arm", target)
```

---

## 11. Data types

All data types are Python `dataclass` objects with numpy arrays for numerical data. They serialize efficiently over gRPC via protobuf.

```python
@dataclass
class JointState:
    positions: np.ndarray       # (num_joints,)
    velocities: np.ndarray      # (num_joints,)
    efforts: np.ndarray         # (num_joints,)
    timestamp: float            # seconds since epoch

@dataclass
class RobotState:
    arms: Dict[str, JointState]
    base: Optional[BaseState]
    grippers: Dict[str, GripperState]
    timestamp: float

@dataclass
class Pose:
    position: np.ndarray        # (3,) [x, y, z]
    quaternion: np.ndarray      # (4,) [x, y, z, w]

@dataclass
class Transform:
    position: np.ndarray        # (3,)
    quaternion: np.ndarray      # (4,)

    @property
    def matrix(self) -> np.ndarray:
        """4x4 homogeneous transformation matrix."""
        ...

@dataclass
class CameraFrame:
    data: np.ndarray            # (H, W, C) uint8
    timestamp: float
    encoding: str               # "rgb8", "bgr8", etc.

@dataclass
class IMUData:
    orientation: np.ndarray     # (4,) quaternion
    angular_velocity: np.ndarray  # (3,)
    linear_acceleration: np.ndarray  # (3,)
    timestamp: float
```

---

## 12. Deployment scenarios

### Scenario A — Local development

SDK and bridge run on the same machine (or the robot itself). Transport is `LocalTransport` (direct ROS2 node).

```yaml
connection:
  mode: "local"
```

### Scenario B — Remote control with heavy compute

SDK runs on a GPU workstation. Bridge runs on the robot. Communication via gRPC over LAN.

```yaml
connection:
  mode: "remote"
  remote:
    host: "192.168.1.100"
    grpc_port: 50051
```

Typical use: the workstation runs a vision-language model, receives camera images at 10-30 Hz, computes grasp poses, and sends joint commands back at 100+ Hz.

### Scenario C — Multi-robot fleet

Multiple `TongRobot` instances, each with its own config pointing to a different robot. A central orchestrator coordinates them.

### Scenario D — Sim-to-real

The same SDK API wraps a simulator (Isaac Sim, MuJoCo, Gazebo) by implementing a `SimTransport` that maps SDK calls to simulator APIs. User code doesn't change between sim and real.

```yaml
connection:
  mode: "sim"
  sim:
    backend: "mujoco"           # "mujoco" | "isaac" | "gazebo"
    scene_file: "./scene.xml"
```

---

## 13. Technology choices

| Component | Technology | Rationale |
|---|---|---|
| SDK language | Python 3.10+ | Target audience uses Python; numpy ecosystem |
| ROS version | ROS2 Humble+ | Long-term support, DDS, lifecycle nodes |
| Remote transport | gRPC + protobuf | Low latency, streaming, strongly typed, cross-platform |
| Dashboard transport | WebSocket | Browser-native, bidirectional, low overhead |
| Dashboard frontend | React + TypeScript | Component model suits panel architecture; strong ecosystem |
| 3D visualization | three.js + urdf-loader | Browser-native 3D, URDF rendering |
| Config format | YAML | Human-readable, widely used in ROS ecosystem |
| Serialization | protobuf (binary) | Efficient for large arrays (images, point clouds) |
| Async runtime | asyncio | Python-native, integrates with gRPC async |
| Package manager | pip + pyproject.toml | Standard Python packaging |

---

## 14. Performance considerations

- **Image compression:** Camera frames are JPEG-compressed before transmission over gRPC (configurable quality). Raw frames available locally.
- **Zero-copy on local transport:** When running on-robot, `LocalTransport` passes numpy array views from ROS2 messages without copying.
- **State caching:** `StateManager` maintains a latest-state cache updated by background subscriptions. `get_state()` returns immediately from cache rather than blocking on network.
- **Command rate limiting:** `MotionManager` enforces the configured `control_rate_hz` to prevent flooding the driver.
- **Selective subscription:** Only topics referenced in the config are subscribed. Unused sensors don't consume bandwidth.
- **Point cloud downsampling:** Configurable voxel downsampling before transmission for large LiDAR point clouds.

---

## 15. Error handling

```python
from tongrobot.exceptions import (
    ConnectionError,        # Cannot reach the bridge node
    TimeoutError,           # Operation timed out
    SafetyViolation,        # Command rejected by safety layer
    HardwareError,          # Driver reported an error
    ConfigError,            # Invalid configuration
)

try:
    robot.connect()
except ConnectionError as e:
    print(f"Cannot connect: {e}")

try:
    robot.set_joint_positions("right_arm", dangerous_cmd)
except SafetyViolation as e:
    print(f"Rejected: {e}")  # e.g., "Joint 2 exceeds position limit"
```

---

## 16. Roadmap priorities

**Phase 1 — Foundation** (Months 1-2): Core SDK with `LocalTransport`, config system, state/sensor/motion managers, basic safety layer. Support one reference platform (e.g., UR5e on mobile base).

**Phase 2 — Remote operation** (Months 2-3): gRPC transport, bridge node, remote state streaming and command sending. Validate with real latency/bandwidth tests.

**Phase 3 — Web dashboard** (Months 3-4): Dashboard with default panels, WebSocket transport, panel customization UI, live camera/joint visualization.

**Phase 4 — Ecosystem** (Months 4-6): Plugin system, sim transport (MuJoCo/Isaac), async API, multi-robot support, second reference platform (humanoid), documentation and examples.
