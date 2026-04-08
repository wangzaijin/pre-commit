"""MotionManager — velocity and joint commands with safety clamping."""

from __future__ import annotations

from typing import Any, Dict

import numpy as np

from tongrobot.core.config import HardwareDescriptor
from tongrobot.exceptions import SafetyViolation
from tongrobot.transport.base import TransportBase
from tongrobot.types import JointCommand, VelocityCommand
from tongrobot.utils.logging import get_logger


class MotionManager:
    """Send motion commands to the robot with safety clamping and e-stop."""

    def __init__(
        self,
        transport: TransportBase,
        hardware: HardwareDescriptor,
        safety_config: Dict[str, Any],
    ) -> None:
        self._transport = transport
        self._hw = hardware
        self._safety = safety_config
        self._estop = False
        self._logger = get_logger(__name__)

    # -------------------------------------------------------------------------

    def set_base_velocity(self, linear: float, angular: float) -> None:
        """Send a velocity command to the mobile base.

        Values are clamped to the configured hardware limits.  If clamping is
        applied a warning is logged.

        Raises
        ------
        SafetyViolation
            If the e-stop flag is set.
        """
        self._check_estop()
        if self._hw.base is None:
            raise RuntimeError(
                "No mobile base configured — cannot send velocity command."
            )

        max_lin = self._hw.base.max_linear_vel
        max_ang = self._hw.base.max_angular_vel

        clamped_lin = float(np.clip(linear, -max_lin, max_lin))
        clamped_ang = float(np.clip(angular, -max_ang, max_ang))

        if clamped_lin != linear:
            self._logger.warning(
                "Linear velocity %.3f clamped to %.3f (limit ±%.3f m/s)",
                linear,
                clamped_lin,
                max_lin,
            )
        if clamped_ang != angular:
            self._logger.warning(
                "Angular velocity %.3f clamped to %.3f (limit ±%.3f rad/s)",
                angular,
                clamped_ang,
                max_ang,
            )

        self._transport.set_velocity_command(
            VelocityCommand(linear_x=clamped_lin, linear_y=0.0, angular_z=clamped_ang)
        )

    def set_joint_positions(self, arm_name: str, positions: np.ndarray) -> None:
        """Send joint position targets to the named arm.

        Raises
        ------
        SafetyViolation
            If any joint position is outside the configured limits or the
            e-stop is active.
        """
        self._check_estop()
        arm = self._hw.get_arm(arm_name)  # raises ConfigError if unknown

        # Check joint limits
        limits = arm.joint_limits.get("position", {})
        if limits:
            lower = np.array(limits.get("lower", []))
            upper = np.array(limits.get("upper", []))
            if lower.size == positions.size and upper.size == positions.size:
                violations = np.where((positions < lower) | (positions > upper))[0]
                if violations.size > 0:
                    raise SafetyViolation(
                        f"Joint positions for arm '{arm_name}' violate limits at "
                        f"indices {violations.tolist()}. "
                        f"Commanded: {positions[violations]}, "
                        f"Limits: [{lower[violations]}, {upper[violations]}]"
                    )

        self._transport.set_joint_command(
            arm_name, JointCommand(positions=positions, velocities=None)
        )

    def stop(self) -> None:
        """Send zero velocity to the base and zero commands to all arms."""
        if self._hw.base:
            self._transport.set_velocity_command(
                VelocityCommand(linear_x=0.0, linear_y=0.0, angular_z=0.0)
            )
        for arm_name, arm in self._hw.arms.items():
            zeros = np.zeros(arm.num_joints)
            self._transport.set_joint_command(
                arm_name, JointCommand(positions=zeros, velocities=None)
            )

    def emergency_stop(self) -> None:
        """Stop the robot immediately and block all further motion commands."""
        self._logger.error("EMERGENCY STOP triggered.")
        self._estop = True
        self.stop()

    def reset_estop(self) -> None:
        """Clear the e-stop flag to re-enable motion commands."""
        self._logger.info("E-stop cleared.")
        self._estop = False

    # -------------------------------------------------------------------------

    def _check_estop(self) -> None:
        if self._estop:
            raise SafetyViolation(
                "Emergency stop is active. Call reset_estop() before sending commands."
            )
