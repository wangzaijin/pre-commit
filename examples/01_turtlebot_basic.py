#!/usr/bin/env python3
"""Phase 1 demo — TurtleBot3 Waffle local control via LocalTransport.

Before running this script, launch TurtleBot3 in Gazebo in a separate terminal:
    export TURTLEBOT3_MODEL=waffle
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

Then, with the ROS2 environment sourced, run:
    python3 examples/01_turtlebot_basic.py
"""
import time

from tongrobot import TongRobot


CONFIG = "configs/turtlebot3.yaml"


def main() -> None:
    print("=" * 60)
    print("Phase 1 demo — TurtleBot3 Waffle (Gazebo)")
    print("=" * 60)
    print()
    print("Prerequisite: launch Gazebo in another terminal:")
    print("  export TURTLEBOT3_MODEL=waffle")
    print("  ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py")
    print()
    input("Press Enter when Gazebo is running…")
    print()

    with TongRobot(CONFIG) as robot:
        # Brief warm-up so subscribers fill their caches
        print("Waiting for sensor data to arrive …")
        time.sleep(2.0)

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
            print("No base state received yet.")
        print()

        # ── Camera ───────────────────────────────────────────────────────────
        try:
            frame = robot.get_camera_frame("main_camera")
            print(
                f"Camera frame  : shape={frame.data.shape}  dtype={frame.data.dtype}  encoding={frame.encoding}"
            )
        except Exception as exc:
            print(f"Camera not available: {exc}")
        print()

        # ── Laser scan ───────────────────────────────────────────────────────
        try:
            scan = robot.get_laser_scan("base_scan")
            print(
                f"Laser scan    : {len(scan.ranges)} range readings  "
                f"[{scan.angle_min:.2f}, {scan.angle_max:.2f}] rad"
            )
        except Exception as exc:
            print(f"Laser scan not available: {exc}")
        print()

        # ── Drive forward for 3 s ────────────────────────────────────────────
        print("Driving forward at 0.1 m/s for 3 seconds …")
        rate = robot.create_rate(10)  # 10 Hz
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            robot.set_base_velocity(linear=0.1, angular=0.0)
            rate.sleep()

        robot.stop()
        print("Robot stopped.")
        print()
        print("Phase 1 demo complete!")


if __name__ == "__main__":
    main()
