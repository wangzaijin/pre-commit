# TongRobot

Hardware-agnostic robotic middleware — a clean Python SDK that wraps ROS2 complexity behind a simple API.

Control diverse robot platforms (mobile bases, manipulators, humanoids) without writing boilerplate ROS2 code. One config file, one entry point. Works on-robot (local) and over the network (remote gRPC) with the same API.

```python
with TongRobot("configs/turtlebot3.yaml") as robot:
    state = robot.get_base_state()
    scan  = robot.get_laser_scan("base_scan")
    robot.set_base_velocity(linear=0.1, angular=0.0)
```

---

## Architecture overview

```
User script (laptop or robot)
    │
    ▼
TongRobot                   ← single entry point (core/robot.py)
    ├── StateManager         ← robot state (odom, joints, TF)
    ├── SensorManager        ← camera frames, laser scans, IMU
    └── MotionManager        ← velocity commands, e-stop, safety clamping
            │
    ┌───────┴───────┐
    ▼               ▼
LocalTransport   GRPCTransport         ← same TransportBase interface
    │               │
    ▼               ▼
ROS2 graph      BridgeNode (robot)     ← tongrobot_bridge
  (direct)        └── gRPC server
                  └── ROS2 subscribers/publishers
```

Switching between local and remote is a **one-line config change** — no code changes required.

---

## Project structure

```
tongrobot/
├── tongrobot/                      # Python SDK package
│   ├── __init__.py                 # exports TongRobot + all types
│   ├── exceptions.py               # exception hierarchy
│   ├── core/
│   │   ├── robot.py                # TongRobot class
│   │   ├── config.py               # ConfigLoader, HardwareDescriptor
│   │   ├── state.py                # StateManager
│   │   ├── sensor.py               # SensorManager
│   │   └── motion.py               # MotionManager
│   ├── transport/
│   │   ├── base.py                 # TransportBase abstract class
│   │   ├── local.py                # LocalTransport (direct ROS2)
│   │   └── grpc_transport.py       # GRPCTransport (network client)
│   ├── types/
│   │   ├── robot_state.py          # JointState, BaseState, RobotState, Pose, Transform
│   │   ├── sensor_data.py          # CameraFrame, LaserScan, IMUData
│   │   └── commands.py             # VelocityCommand, JointCommand
│   ├── utils/
│   │   ├── rate.py                 # Rate — loop timing helper
│   │   ├── transforms.py           # pure-numpy quaternion / SE3 utilities
│   │   └── logging.py              # get_logger()
│   └── proto/                      # generated gRPC stubs + converters
│       ├── *_pb2.py / *_pb2_grpc.py
│       └── converters.py           # dataclass ↔ protobuf conversion
├── tongrobot_bridge/               # ROS2 package (runs on the robot)
│   ├── tongrobot_bridge/
│   │   └── bridge_node.py          # BridgeNode (ROS2) + BridgeServicer (gRPC) + DashboardServer
│   ├── launch/
│   │   └── bridge.launch.py        # ROS2 launch file
│   ├── package.xml
│   └── setup.py
├── tongrobot_dashboard/            # Web dashboard
│   ├── build.sh                    # one-time frontend build → frontend/dist/
│   └── frontend/                   # React 18 + TypeScript + Vite source
│       ├── src/
│       │   ├── App.tsx             # top-level layout
│       │   ├── main.tsx            # React 18 entry point
│       │   ├── index.css           # dark-theme global styles
│       │   ├── types/index.ts      # WebSocket message types, PanelItem
│       │   ├── services/
│       │   │   └── websocket.ts    # singleton WS client (reconnect + subscription replay)
│       │   ├── stores/
│       │   │   └── robotStore.ts   # Zustand store (panel layout persisted to localStorage)
│       │   └── components/
│       │       ├── ConnectionStatus.tsx
│       │       ├── BaseStatePanel.tsx
│       │       ├── CameraView.tsx
│       │       ├── LaserScanPanel.tsx
│       │       ├── CommandPanel.tsx
│       │       ├── PanelLayout.tsx  # react-grid-layout wrapper
│       │       └── PanelToolbar.tsx # "Add Panel" dropdown
│       ├── package.json
│       ├── vite.config.ts
│       └── tsconfig.json
├── proto/                          # protobuf source definitions
│   ├── robot_state.proto
│   ├── sensor_data.proto
│   ├── commands.proto
│   ├── bridge_service.proto
│   ├── build.sh                    # compile protos → tongrobot/proto/
│   └── fix_imports.py              # post-process generated imports
├── configs/
│   ├── turtlebot3.yaml             # TurtleBot3 Waffle — local mode
│   └── turtlebot3_remote.yaml      # TurtleBot3 Waffle — remote mode
├── examples/
│   ├── 01_turtlebot_basic.py       # Phase 1: local ROS2 demo
│   ├── 02_turtlebot_remote.py      # Phase 2: gRPC remote demo
│   └── 03_dashboard_demo.py        # Phase 3: dashboard + square drive
└── pyproject.toml
```

---

## Prerequisites

| Dependency | Notes |
|---|---|
| Python ≥ 3.10 | |
| ROS2 Humble (or newer LTS) | `ros2 topic` / `ros2 launch` must work |
| Gazebo | Bundled with your ROS2 distro |
| `turtlebot3_gazebo`, `turtlebot3_description` | `sudo apt install ros-humble-turtlebot3*` |
| `opencv-python`, `grpcio`, `grpcio-tools` | Installed automatically by pip |

The ROS2 Python packages (`rclpy`, `sensor_msgs`, `geometry_msgs`, `nav_msgs`) come from the ROS2 installation. They are **not** pip dependencies — source your ROS2 environment before calling `connect()`.

---

## Installation

### 1. Install the Python SDK

```bash
source /opt/ros/humble/setup.bash
pip install -e .
```

This installs `numpy`, `grpcio`, `opencv-python`, and other pip dependencies. The ROS2 Python packages (`rclpy`, `sensor_msgs`, etc.) are not listed as pip dependencies — they come from the sourced ROS2 environment.

The SDK can be imported **without** a sourced ROS2 environment — all ROS2 imports are lazy and only load when `connect()` is called.

### 2. Install the bridge node (Phase 2 only)

`tongrobot_bridge` is a **ROS2 package** that must be built with `colcon` before it can be used with `ros2 launch`. It needs to live inside a colcon workspace.

```bash
# Create a workspace (skip if you already have one)
mkdir -p ~/ros2_ws/src

# Symlink the bridge package into the workspace
ln -s $(pwd)/tongrobot_bridge ~/ros2_ws/src/tongrobot_bridge

# Build it
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select tongrobot_bridge

# Source the workspace so ros2 can find the package
source ~/ros2_ws/install/setup.bash
```

After sourcing the workspace, `ros2 launch tongrobot_bridge bridge.launch.py …` will work.

> **No colcon?** You can also run the bridge directly without installing it as a ROS2 package — see the [Running the bridge](#running-the-bridge) section below.

### 3. Regenerate proto stubs (optional)

The compiled stubs are committed to the repo. Only re-run this if you edit the `.proto` files:

```bash
bash proto/build.sh
```

---

## Phase 1 — Local operation

The SDK runs on the same machine as the robot. `LocalTransport` creates a `rclpy` node and communicates directly with the ROS2 graph.

### Quick start

```bash
# Terminal 1 — Gazebo
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 — demo script
source /opt/ros/humble/setup.bash
python3 examples/01_turtlebot_basic.py
```

The demo reads state, camera, and laser scan, then drives the robot forward for 3 seconds.

---

## Phase 2 — Remote operation (gRPC)

The SDK runs on a **remote machine** (e.g. your laptop). The `tongrobot_bridge` node runs on the robot and exposes everything over gRPC.

### Running the bridge

The bridge node can be started in two ways.

**Option A — via `ros2 launch` (requires colcon build, see Installation)**

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash   # workspace where you built the bridge

ros2 launch tongrobot_bridge bridge.launch.py \
  config:=configs/turtlebot3.yaml
```

**Option B — direct Python (no colcon required, useful for development)**

```bash
source /opt/ros/humble/setup.bash

python3 tongrobot_bridge/tongrobot_bridge/bridge_node.py \
  configs/turtlebot3.yaml
```

Both methods start the same `BridgeNode` and gRPC server; the only difference is how ROS2 discovers the package.

### Quick start (full two-terminal flow)

```bash
# Robot machine — Terminal 1: Gazebo
export TURTLEBOT3_MODEL=waffle
source /opt/ros/humble/setup.bash
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Robot machine — Terminal 2: bridge node (Option B — no colcon needed)
source /opt/ros/humble/setup.bash
python3 tongrobot_bridge/tongrobot_bridge/bridge_node.py configs/turtlebot3.yaml

# Laptop (or same machine for local testing)
python3 examples/02_turtlebot_remote.py
```

For same-machine testing the remote config already points to `localhost`. For cross-machine testing, edit `configs/turtlebot3_remote.yaml`:

```yaml
connection:
  mode: remote
  remote:
    host: "192.168.1.42"   # ← robot machine IP
    grpc_port: 50051
```

### What the bridge does

- Subscribes to all configured ROS2 topics and caches the latest data
- Exposes a **gRPC service** (`TongRobotBridge`) with 10 RPCs:
  - `GetRobotState` / `StreamRobotState` — state snapshots and 50 Hz streaming
  - `GetCameraFrame` / `StreamCameraFrames` — JPEG-compressed frames
  - `GetLaserScan`, `GetIMU` — sensor snapshots
  - `SetVelocityCommand`, `SetJointCommand`, `EmergencyStop` — commands
  - `GetTransform` — TF2 lookups forwarded as proto messages

Camera frames are **JPEG-compressed** on the bridge before transmission to reduce bandwidth, then decompressed transparently on the SDK side.

### Switching between local and remote

The only change needed is the `connection.mode` field in the config:

```yaml
# Local (Phase 1)
connection:
  mode: local

# Remote (Phase 2)
connection:
  mode: remote
  remote:
    host: "192.168.1.42"
    grpc_port: 50051
```

Everything else — your user scripts, the API, the hardware topology — stays identical.

---

## API reference

### `TongRobot(config_path)`

```python
from tongrobot import TongRobot

robot = TongRobot("configs/turtlebot3.yaml")
robot.connect()
# … or use as a context manager:
with TongRobot("configs/turtlebot3.yaml") as robot:
    ...
```

### State

```python
state: RobotState   = robot.get_state()
base:  BaseState    = robot.get_base_state()
# base.position  → np.ndarray shape (3,): x, y, theta [m, m, rad]
# base.velocity  → np.ndarray shape (3,): vx, vy, omega [m/s, m/s, rad/s]
# base.timestamp → float (seconds)
```

### Sensors

```python
frame: CameraFrame  = robot.get_camera_frame("main_camera")
# frame.data      → np.ndarray (H, W, 3) uint8, RGB
# frame.timestamp → float
# frame.encoding  → "rgb8"

scan: LaserScan     = robot.get_laser_scan("base_scan")
# scan.ranges          → np.ndarray (N,) float32 [m]
# scan.angle_min/max   → float [rad]
# scan.angle_increment → float [rad]

imu: IMUData        = robot.get_imu("base_imu")
# imu.orientation         → np.ndarray (4,) quaternion xyzw
# imu.angular_velocity    → np.ndarray (3,) [rad/s]
# imu.linear_acceleration → np.ndarray (3,) [m/s²]

# Convenience: raw image array
img: np.ndarray = robot.get_camera("main_camera")   # shape (H, W, 3) uint8
```

### Motion

```python
robot.set_base_velocity(linear=0.1, angular=0.0)  # m/s, rad/s
robot.stop()           # zero velocity to all actuators
robot.emergency_stop() # stop + block further commands
robot.reset_estop()    # re-enable after e-stop
```

Velocity values exceeding hardware limits (from the config) are **automatically clamped** and a warning is logged. The e-stop works across both transports — `GRPCTransport` forwards it to the bridge which publishes zero velocity.

### Transforms

```python
tf: Transform = robot.get_transform("map", "base_link")
# tf.position   → np.ndarray (3,)
# tf.quaternion → np.ndarray (4,) xyzw
# tf.matrix     → np.ndarray (4, 4) homogeneous transform
```

### Control loops

```python
rate = robot.create_rate(10)  # 10 Hz
while True:
    robot.set_base_velocity(0.1, 0.0)
    rate.sleep()
```

### Streaming callbacks

```python
def on_frame(frame: CameraFrame) -> None:
    print(frame.data.shape)

robot.on_camera("main_camera", on_frame, fps=10)
```

```python
def on_state(state: RobotState) -> None:
    if state.base:
        print(state.base.position)

# (subscribe_robot_state is available on the transport directly)
robot._transport.subscribe_robot_state(on_state)
```

---

## Data types

All types can be imported directly from `tongrobot`:

```python
from tongrobot import (
    RobotState, BaseState, JointState, GripperState,
    CameraFrame, LaserScan, IMUData,
    VelocityCommand, JointCommand,
    Pose, Transform,
)
```

---

## Configuration

```yaml
robot:
  name: turtlebot3_waffle

connection:
  mode: local          # 'local' → LocalTransport
                       # 'remote' → GRPCTransport + bridge node
  remote:
    host: "192.168.1.100"
    grpc_port: 50051

hardware:
  base:
    type: differential
    cmd_vel_topic: /cmd_vel
    odom_topic: /odom
    max_linear_vel: 0.26    # m/s  — SDK clamps to this
    max_angular_vel: 1.82   # rad/s

  sensors:
    cameras:
      - name: main_camera
        topic: /camera/image_raw
        info_topic: /camera/camera_info
        encoding: rgb8

    lidar:
      - name: base_scan
        topic: /scan
        type: 2d

    imu:
      - name: base_imu
        topic: /imu

safety:
  velocity_limit_check: true
  watchdog_timeout_ms: 500
```

To add a new robot, copy `configs/turtlebot3.yaml`, update topic names and limits, and pass the new path to `TongRobot(...)`. No code changes required.

---

## Exceptions

```python
from tongrobot.exceptions import (
    TongRobotError,    # base class
    ConfigError,       # bad or missing config
    ConnectionError,   # transport can't connect (also: bridge unreachable)
    TimeoutError,      # sensor data not yet available
    SafetyViolation,   # command violates a limit or e-stop is active
    HardwareError,     # hardware reported a fault
)
```

---

## Utilities

### `tongrobot.utils.transforms`

Pure-numpy SE3 helpers — no ROS2 dependency:

```python
from tongrobot.utils.transforms import (
    quaternion_to_matrix,   # (4,) → (3,3)
    matrix_to_quaternion,   # (3,3) → (4,)
    pose_to_matrix,         # Pose → (4,4)
    matrix_to_pose,         # (4,4) → Pose
)
```

### `tongrobot.utils.rate`

```python
from tongrobot.utils.rate import Rate
rate = Rate(10)   # 10 Hz
while True:
    do_work()
    rate.sleep()
```

---

## Phase 3 — Web dashboard

The bridge node serves the web dashboard alongside gRPC. The same `bridge_node.py` process runs three concurrent components:

| Component | Thread | Port |
|-----------|--------|------|
| ROS2 spin | main | — |
| gRPC server | `ThreadPoolExecutor` | 50051 |
| aiohttp (HTTP + WebSocket) | asyncio daemon thread | 3000 |

### Build the frontend (one-time)

```bash
bash tongrobot_dashboard/build.sh
```

This runs `npm install && npm run build` and writes the compiled assets to `tongrobot_dashboard/frontend/dist/`. The bridge node's aiohttp server picks up the `dist/` directory automatically at startup — no extra launch step is needed.

If the frontend is not built, the bridge node still serves a placeholder page at `/` with instructions.

### Accessing the dashboard

Start the bridge node (see Phase 2 above), then open:

```
http://localhost:3000/
```

The WebSocket connects automatically to `ws://<same-host>:<same-port>/ws`. No configuration needed in the browser.

### Using the dashboard with local-mode operation

The bridge node has **two independent roles**: serving SDK clients over gRPC, and serving the dashboard over WebSocket. These are independent — you can use the dashboard even when your Python scripts use `connection.mode: local`.

In local mode, run the bridge node alongside your SDK script. Both subscribe to the same ROS2 topics independently and coexist without interference:

```bash
# Terminal 1 — Gazebo
export TURTLEBOT3_MODEL=waffle
ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

# Terminal 2 — Bridge node (dashboard only; gRPC is unused but harmless)
source /opt/ros/humble/setup.bash
python3 tongrobot_bridge/tongrobot_bridge/bridge_node.py configs/turtlebot3.yaml
# → Dashboard: http://localhost:3000/

# Terminal 3 — Your local-mode SDK script
source /opt/ros/humble/setup.bash
python3 examples/01_turtlebot_basic.py   # connection.mode: local
```

If you do not need the dashboard you can skip Terminal 2 entirely — the SDK works independently.

### Dashboard demo

```bash
# Terminals 1 + 2 as above, then:
python3 examples/03_dashboard_demo.py
```

The demo prints the dashboard URL, waits 5 seconds for you to open it in a browser, then drives the robot in a 1 m square so there is live data to watch.

### WebSocket protocol

**Server → client:**

| Message | Fields |
|---------|--------|
| `robot_state` | `data.base: {x, y, theta, vx, vy, omega, timestamp}` |
| `camera_frame` | `name`, `data` (base64 JPEG), `timestamp` |
| `laser_scan` | `name`, `data.{ranges, angle_min, angle_max, angle_increment, timestamp}` |
| `config` | `data` — the `dashboard:` section of the YAML config |

**Client → server:**

| Message | Fields |
|---------|--------|
| `subscribe` | `topic: "robot_state"\|"camera"\|"laser_scan"`, `name?`, `fps?` |
| `unsubscribe` | `topic` |
| `cmd_vel` | `linear` (m/s), `angular` (rad/s) |
| `stop` | — sends zero velocity |
| `get_config` | — server replies with a `config` message |

### Customising the layout

Panels are draggable and resizable (react-grid-layout). The layout is persisted to `localStorage`, so it survives page refreshes. Use the **+ Add Panel** button to add Camera, Base State, Laser Scan, or Command panels.

---

## Troubleshooting

**`TimeoutError: No data received from camera 'main_camera' yet`**
Confirm the topic is publishing:
```bash
ros2 topic hz /camera/image_raw
```

**`ConnectionError: Cannot reach bridge node at localhost:50051`**
Confirm the bridge is running and listening on the expected port:
```bash
ros2 launch tongrobot_bridge bridge.launch.py config:=configs/turtlebot3.yaml
```

**`rclpy` not found at import time**
Expected — `rclpy` is loaded lazily only when `connect()` is called. Source your ROS2 environment before calling `connect()`.

**Velocity not reaching the robot**
Check that `/cmd_vel` has a subscriber:
```bash
ros2 topic info /cmd_vel
```

**gRPC stream drops and re-connects**
Streaming threads (`subscribe_robot_state`, `subscribe_camera`) automatically reconnect on `UNAVAILABLE` errors. If the bridge node restarts, streams resume within 1 second.

**Dashboard shows "not built yet"**
Run `bash tongrobot_dashboard/build.sh` once. The bridge node picks up `frontend/dist/` automatically on next start.

**Dashboard shows "Disconnected"**
Confirm the bridge node is running and the browser is pointed at the correct port (default 3000). Check your browser's network tab for WebSocket errors.
# Test commit to verify fix
