#!/usr/bin/env python3
"""Phase 2 demo — TurtleBot3 Waffle remote control over gRPC.

Two-terminal setup
==================

Terminal 1 — robot side (machine running Gazebo):
    export TURTLEBOT3_MODEL=waffle
    source /opt/ros/humble/setup.bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &
    ros2 launch tongrobot_bridge bridge.launch.py config:=configs/turtlebot3.yaml

Terminal 2 — remote side (this script):
    # For same-machine testing, host is already 'localhost' in the config.
    # For cross-machine testing, edit configs/turtlebot3_remote.yaml and set
    # 'host' to the robot machine's IP address.
    source /opt/ros/humble/setup.bash   # only needed if SDK also uses local transport
    python3 examples/02_turtlebot_remote.py

The demo verifies:
- gRPC connection to the bridge node
- Robot state read over the network
- Camera frame retrieval (JPEG decompressed back to numpy)
- Laser scan retrieval
- Driving the robot forward for 3 seconds via the network
"""
import time

from tongrobot import TongRobot

CONFIG = "configs/turtlebot3_remote.yaml"


def main() -> None:
    print("=" * 60)
    print("Phase 2 demo — TurtleBot3 Waffle (gRPC remote)")
    print("=" * 60)
    print()
    print("Terminal 1 — robot side:")
    print("  export TURTLEBOT3_MODEL=waffle")
    print("  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py &")
    print(
        "  ros2 launch tongrobot_bridge bridge.launch.py config:=configs/turtlebot3.yaml"
    )
    print()
    print("Edit configs/turtlebot3_remote.yaml → connection.remote.host")
    print("if running on two different machines.")
    print()
    input("Press Enter when the bridge node is running…")
    print()

    with TongRobot(CONFIG) as robot:
        print("Connected via gRPC.")
        print()

        # Brief warm-up
        print("Waiting for sensor data to propagate…")
        time.sleep(1.0)

        # ── Robot state ──────────────────────────────────────────────────────
        state = robot.get_state()
        if state.base:
            b = state.base
            print(
                f"Base position : x={b.position[0]:.3f}  y={b.position[1]:.3f}  θ={b.position[2]:.3f} rad"
            )
            print(
                f"Base velocity : vx={b.velocity[0]:.3f}  ω={b.velocity[2]:.3f} rad/s"
            )
        else:
            print("Base state not yet available.")
        print()

        # ── Camera ───────────────────────────────────────────────────────────
        try:
            frame = robot.get_camera_frame("main_camera")
            print(
                f"Camera frame  : shape={frame.data.shape}  dtype={frame.data.dtype}  encoding={frame.encoding}"
            )
            print("  (JPEG compressed on the bridge, decompressed here)")
        except Exception as exc:
            print(f"Camera not available: {exc}")
        print()

        # ── Laser scan ───────────────────────────────────────────────────────
        try:
            scan = robot.get_laser_scan("base_scan")
            print(
                f"Laser scan    : {len(scan.ranges)} readings  "
                f"[{scan.angle_min:.2f}, {scan.angle_max:.2f}] rad"
            )
        except Exception as exc:
            print(f"Laser scan not available: {exc}")
        print()

        # ── Drive forward for 3 s ────────────────────────────────────────────
        print("Driving forward at 0.1 m/s for 3 seconds over gRPC…")
        rate = robot.create_rate(10)
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            robot.set_base_velocity(linear=0.1, angular=0.0)
            rate.sleep()

        robot.stop()
        print("Robot stopped.")
        print()
        print("Phase 2 demo complete!")


if __name__ == "__main__":
    main()
