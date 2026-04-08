"""Thin structured logging wrapper."""

from __future__ import annotations

import logging
from typing import Dict, Any


_FORMAT = "%(asctime)s [%(levelname)s] %(name)s — %(message)s"
_DATE_FORMAT = "%Y-%m-%d %H:%M:%S"

_configured = False


def _ensure_configured(level: int = logging.INFO) -> None:
    global _configured
    if not _configured:
        logging.basicConfig(format=_FORMAT, datefmt=_DATE_FORMAT, level=level)
        _configured = True


def get_logger(
    name: str, logging_config: Dict[str, Any] | None = None
) -> logging.Logger:
    """Return a consistently-formatted logger.

    Parameters
    ----------
    name:
        Typically ``__name__`` of the calling module.
    logging_config:
        Optional dict from the YAML ``logging`` section.  Supports a
        ``level`` key (e.g., ``"DEBUG"``, ``"INFO"``, ``"WARNING"``).
    """
    level_str = "INFO"
    if logging_config:
        level_str = logging_config.get("level", "INFO").upper()
    level = getattr(logging, level_str, logging.INFO)
    _ensure_configured(level)
    return logging.getLogger(name)
