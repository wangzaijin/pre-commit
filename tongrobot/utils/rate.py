"""Loop rate limiter for control loops."""

from __future__ import annotations

import logging
import time


_logger = logging.getLogger(__name__)


class Rate:
    """Sleep to maintain a fixed loop frequency.

    Example
    -------
    >>> rate = Rate(10)   # 10 Hz
    >>> while True:
    ...     do_work()
    ...     rate.sleep()
    """

    def __init__(self, frequency_hz: float) -> None:
        if frequency_hz <= 0:
            raise ValueError(f"frequency_hz must be positive, got {frequency_hz}")
        self._period = 1.0 / frequency_hz
        self._last = time.monotonic()

    def sleep(self) -> None:
        """Sleep for the remainder of the current period.

        If the loop is already running behind schedule, skip sleeping and
        optionally log a warning.
        """
        now = time.monotonic()
        elapsed = now - self._last
        remaining = self._period - elapsed

        if remaining > 0:
            time.sleep(remaining)
        else:
            _logger.debug(
                "Rate loop running %.1f ms behind schedule",
                -remaining * 1000,
            )

        self._last = time.monotonic()
