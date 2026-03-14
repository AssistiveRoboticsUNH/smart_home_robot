"""Typed protocol configuration models for documentation and future migration."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Any


@dataclass(frozen=True)
class TriggerCondition:
    state: str
    value: Any = None
    op: str = "="


@dataclass(frozen=True)
class GenericStepDefinition:
    tree_name: str | None = None
    tree_params: dict[str, Any] = field(default_factory=dict)
    confirmation: dict[str, Any] | None = None
    next_step_after: Any = None


@dataclass(frozen=True)
class ProtocolDefinition:
    name: str
    runner: str
    action_steps: list[GenericStepDefinition] = field(default_factory=list)
    high_level: dict[str, Any] = field(default_factory=dict)
