"""Typed trigger runtime state models."""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime
from typing import Any


@dataclass(frozen=True)
class SuccessRule:
    mode: str
    conditions: list[dict[str, Any]]


@dataclass(frozen=True)
class ScheduledReset:
    protocol: str
    finished_time: datetime
    reset_seconds: int
