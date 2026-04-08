"""Phase 3 demo — launches the bridge node with the dashboard.

Before running this script:

  Terminal 1 — Gazebo:
    export TURTLEBOT3_MODEL=waffle
    source /opt/ros/humble/setup.bash
    ros2 launch turtlebot3_gazebo turtlebot3_world.launch.py

  Terminal 2 — Bridge node (gRPC + dashboard):
    source /opt/ros/humble/setup.bash
    python3 tongrobot_bridge/tongrobot_bridge/bridge_node.py configs/turtlebot3.yaml

  (Optional) Build the dashboard frontend first:
    bash tongrobot_dashboard/build.sh

Then run this script to see the robot move while you watch the dashboard.
"""

import time
import sys
import os

# Allow importing tongrobot from the repo root
sys.path.insert(0, os.path.join(os.path.dirname(__file__), ".."))

DASHBOARD_PORT = 3000
GRPC_PORT = 50051
CONFIG = "configs/turtlebot3.yaml"


def print_instructions() -> None:
    print("\n" + "=" * 60)
    print("  TongRobot Phase 3 — Dashboard Demo")
    print("=" * 60)
    print(f"\n  Dashboard URL : http://localhost:{DASHBOARD_PORT}/")
    print(f"  WebSocket     : ws://localhost:{DASHBOARD_PORT}/ws")
    print(f"  gRPC bridge   : localhost:{GRPC_PORT}")
    print()
    print("  Open the dashboard URL in your browser.")
    print("  Click '+ Add Panel' to add Camera, Base State,")
    print("  Laser Scan, or Command panels.")
    print()
    print("  This script will drive the robot in a square so")
    print("  you have something interesting to watch.\n")


def drive_square() -> None:
    """Connect via gRPC and drive a 1 m square at 0.15 m/s."""
    from tongrobot import TongRobot
    from tongrobot.utils.rate import Rate

    config_path = os.path.join(
        os.path.dirname(__file__), "..", "configs", "turtlebot3_remote.yaml"
    )
    config_path = os.path.normpath(config_path)

    print(f"  Connecting to bridge (config: {config_path})…")
    try:
        robot = TongRobot(config_path)
        robot.connect()
    except Exception as exc:
        print(f"\n  [!] Could not connect: {exc}")
        print("      Is the bridge node running?")
        print(
            "      Run: python3 tongrobot_bridge/tongrobot_bridge/bridge_node.py configs/turtlebot3.yaml"
        )
        return

    print("  Connected. Driving a 1 m square (Ctrl-C to abort).\n")
    rate = Rate(10)

    try:
        for side in range(4):
            print(f"  Side {side + 1}/4 — forward…")
            t_start = time.monotonic()
            while time.monotonic() - t_start < 6.5:  # ≈ 1 m at 0.15 m/s
                robot.set_base_velocity(linear=0.15, angular=0.0)
                rate.sleep()

            print(f"  Side {side + 1}/4 — turning 90°…")
            t_start = time.monotonic()
            while time.monotonic() - t_start < 2.2:  # π/2 rad at 1.4 rad/s
                robot.set_base_velocity(linear=0.0, angular=1.4)
                rate.sleep()

        print("\n  Square complete. Stopping robot.")
    except KeyboardInterrupt:
        print("\n  Interrupted — stopping robot.")
    finally:
        robot.stop()
        robot.disconnect()

    print("\n  Phase 3 demo complete!")


if __name__ == "__main__":
    print_instructions()

    # Give the user time to open the browser before motion starts
    print("  Starting motion in 5 seconds (Ctrl-C to skip)…")
    try:
        time.sleep(5)
    except KeyboardInterrupt:
        print("  Skipping motion — dashboard URL is still active.")
        sys.exit(0)

    drive_square()
