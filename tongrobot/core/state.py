"""StateManager — read-only access to robot state."""

from __future__ import annotations

from tongrobot.core.config import HardwareDescriptor
from tongrobot.exceptions import ConfigError
from tongrobot.transport.base import TransportBase
from tongrobot.types import BaseState, JointState, RobotState, Transform


class StateManager:
    """Thin wrapper around the transport for reading robot state."""

    def __init__(self, transport: TransportBase, hardware: HardwareDescriptor) -> None:
        self._transport = transport
        self._hw = hardware

    def get_state(self) -> RobotState:
        """Return the current full robot state snapshot."""
        return self._transport.get_robot_state()

    def get_base_state(self) -> BaseState:
        """Return only the mobile base state (position + velocity).

        Raises
        ------
        ConfigError
            If no base is configured.
        """
        if self._hw.base is None:
            raise ConfigError(
                "No mobile base configured for this robot. "
                "Add a 'base' section to the hardware config."
            )
        state = self._transport.get_robot_state()
        if state.base is None:
            raise ConfigError("Base state is not available yet.")
        return state.base

    def get_arm_state(self, arm_name: str) -> JointState:
        """Return the joint state of the named arm.

        Raises
        ------
        ConfigError
            If the arm name is not in the hardware descriptor.
        """
        self._hw.get_arm(arm_name)  # raises ConfigError if not found
        state = self._transport.get_robot_state()
        if arm_name not in state.arms:
            raise ConfigError(
                f"Arm '{arm_name}' is configured but no joint state data has "
                "arrived yet. Is the controller running?"
            )
        return state.arms[arm_name]

    def get_transform(self, target_frame: str, source_frame: str) -> Transform:
        """Look up a TF2 transform."""
        return self._transport.get_transform(target_frame, source_frame)
