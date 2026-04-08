"""TongRobot exception hierarchy."""


class TongRobotError(Exception):
    """Base exception for all TongRobot errors."""


class ConfigError(TongRobotError):
    """Raised when the configuration is invalid or a requested resource is missing."""


class ConnectionError(TongRobotError):
    """Raised when a transport cannot connect to the robot or bridge."""


class TimeoutError(TongRobotError):
    """Raised when an operation does not complete within the expected time."""


class SafetyViolation(TongRobotError):
    """Raised when a command would violate a configured safety limit."""


class HardwareError(TongRobotError):
    """Raised when the hardware reports a fault or an unexpected state."""
