#!/usr/bin/env python3
"""Protocol designer config service helpers.

This module provides file-backed load/validate/save helpers for the dashboard.
It reuses the smart_home_pytree protocol schema and run-tree metadata so the web
UI stays aligned with runtime validation.
"""

from __future__ import annotations

import copy
import os
import threading
from dataclasses import dataclass
from datetime import datetime
from pathlib import Path
from typing import Any

import yaml

from smart_home_pytree.protocol_schema import RUN_TREE_SPECS, validate_house_config
from smart_home_pytree.robot_interface import ROBOT_STATE_SPECS

_LOCK = threading.RLock()

DAY_OPTIONS = [
    "monday",
    "tuesday",
    "wednesday",
    "thursday",
    "friday",
    "saturday",
    "sunday",
]


class DashboardConfigError(RuntimeError):
    """Raised when dashboard config operations fail."""


@dataclass
class ConfigContext:
    yaml_path: Path


def resolve_house_yaml_path(explicit_path: str | None = None) -> ConfigContext:
    """Resolve the active house config path.

    Priority:
    1. explicit_path arg
    2. env var `house_yaml_path`
    """
    raw = explicit_path or os.getenv("house_yaml_path")
    if not raw:
        raise DashboardConfigError(
            "house_yaml_path is not set. Set the environment variable or pass an explicit path."
        )
    path = Path(raw).expanduser().resolve()
    if not path.exists():
        raise DashboardConfigError(f"house config file not found: {path}")
    if not path.is_file():
        raise DashboardConfigError(f"house config path is not a file: {path}")
    return ConfigContext(yaml_path=path)


def load_config(ctx: ConfigContext) -> dict:
    """Load and validate the active YAML config."""
    with _LOCK:
        with ctx.yaml_path.open("r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
    validate_house_config(data)
    return data


def validate_config_payload(data: dict) -> None:
    """Validate an in-memory config payload using runtime schema rules."""
    validate_house_config(data)


def dump_config_yaml(data: dict) -> str:
    """Serialize config to YAML for preview/saving."""
    return yaml.safe_dump(data, sort_keys=False, allow_unicode=True)


def backup_and_save_config(ctx: ConfigContext, data: dict) -> dict:
    """Validate, back up current file, and atomically save the new config."""
    validate_house_config(data)
    yaml_text = dump_config_yaml(data)

    now = datetime.now()
    timestamp = now.strftime("%Y%m%d_%H%M%S")
    backup_path = ctx.yaml_path.with_suffix(ctx.yaml_path.suffix + f".bak.{timestamp}")
    tmp_path = ctx.yaml_path.with_suffix(ctx.yaml_path.suffix + ".tmp")

    with _LOCK:
        original_text = ctx.yaml_path.read_text(encoding="utf-8")
        backup_path.write_text(original_text, encoding="utf-8")
        tmp_path.write_text(yaml_text, encoding="utf-8")
        os.replace(tmp_path, ctx.yaml_path)

    return {
        "yaml_path": str(ctx.yaml_path),
        "backup_path": str(backup_path),
        "bytes_written": len(yaml_text.encode("utf-8")),
        "saved_at": now.isoformat(timespec="seconds"),
    }


def _infer_param_kind(param_name: str, type_hint: str) -> str:
    hint = (type_hint or "").lower()
    name = (param_name or "").lower()
    if "int" in hint or name in {"num_attempts"}:
        return "integer"
    if "number" in hint:
        return "number_or_duration"
    if "location" in name:
        return "location"
    if "duration" in hint or "sleep" in name:
        return "duration"
    return "string"


def get_run_tree_metadata() -> dict[str, Any]:
    """Return UI-friendly metadata derived from RUN_TREE_SPECS."""
    out: dict[str, Any] = {}
    for tree_name, spec in RUN_TREE_SPECS.items():
        required = spec.get("required_params") or {}
        optional = spec.get("optional_params") or {}
        params = []
        for param_name, type_hint in required.items():
            display_type_hint = str(type_hint)
            if tree_name == "move_away" and param_name == "position_state_key":
                display_type_hint = (
                    "str (robot state key used to choose home/away; usually 'position', "
                    "expected values 'home' or 'away')"
                )
            params.append(
                {
                    "name": param_name,
                    "required": True,
                    "type_hint": display_type_hint,
                    "kind": _infer_param_kind(param_name, str(type_hint)),
                }
            )
        for param_name, type_hint in optional.items():
            display_type_hint = str(type_hint)
            params.append(
                {
                    "name": param_name,
                    "required": False,
                    "type_hint": display_type_hint,
                    "kind": _infer_param_kind(param_name, str(type_hint)),
                }
            )
        out[tree_name] = {
            "description": spec.get("description", ""),
            "params": params,
        }
    return out


def build_dashboard_payload(ctx: ConfigContext) -> dict[str, Any]:
    """Load config and return a UI payload with metadata and summaries."""
    data = load_config(ctx)
    protocols = data.get("protocols", {})
    generic_names = [
        name for name, p in protocols.items() if isinstance(p, dict) and p.get("runner") == "GenericProtocol"
    ]
    special_names = [
        name for name, p in protocols.items() if isinstance(p, dict) and p.get("runner") != "GenericProtocol"
    ]
    return {
        "yaml_path": str(ctx.yaml_path),
        "config": data,
        "yaml_preview": dump_config_yaml(data),
        "metadata": {
            "run_trees": get_run_tree_metadata(),
            "robot_states": get_robot_state_metadata(),
            "day_options": DAY_OPTIONS,
            "generic_protocol_names": generic_names,
            "special_protocol_names": special_names,
            "reset_types": ["eod", "instant", "periodic"],
            "ui_capabilities": {
                "supports_xy_proximity": True,
                "generic_protocol_only_editor": True,
            },
        },
    }


def make_new_generic_protocol(protocol_name: str) -> dict[str, Any]:
    """Return a minimal GenericProtocol config skeleton."""
    return {
        "runner": "GenericProtocol",
        "high_level": {
            "priority": 10,
            "triggers": {},
            "reset_pattern": {"type": "eod"},
        },
        "action": {
            "steps": [
                {
                    "tree_name": "play_text",
                    "tree_params": {"text": f"Protocol {protocol_name}"},
                }
            ]
        },
    }


def ensure_generic_protocol_only_edit(data: dict) -> dict:
    """Return a defensive copy used for UI mutation operations.

    The UI edits the full config payload, but v1 only exposes editors for GenericProtocol
    entries. Special protocols are preserved verbatim.
    """
    return copy.deepcopy(data)


def get_robot_state_metadata() -> list[dict[str, Any]]:
    """Return UI-friendly list of known RobotInterface state keys."""
    out = []
    for key in sorted(ROBOT_STATE_SPECS.keys()):
        spec = ROBOT_STATE_SPECS[key]
        out.append(
            {
                "name": key,
                "kind": spec.get("kind", "string"),
                "ui_kind": spec.get("ui_kind"),
                "description": spec.get("description", ""),
            }
        )
    return out
