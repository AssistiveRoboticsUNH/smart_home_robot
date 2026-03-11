"""Lightweight config normalization before schema validation and runtime use."""

from __future__ import annotations

from copy import deepcopy


def _normalize_execution_location(value):
    if not isinstance(value, str):
        return value
    clean = value.strip()
    lowered = clean.lower()
    if lowered in {"current", "person"}:
        return lowered
    return clean


def normalize_house_config(data: dict) -> dict:
    normalized = deepcopy(data or {})

    person_init = normalized.get("person_init")
    if isinstance(person_init, str):
        normalized["person_init"] = person_init.strip()

    for protocol_data in (normalized.get("protocols") or {}).values():
        action = protocol_data.get("action") or {}
        for step in action.get("steps") or []:
            tree_params = step.get("tree_params") or {}
            if "execution_location" in tree_params:
                tree_params["execution_location"] = _normalize_execution_location(
                    tree_params.get("execution_location")
                )
            confirmation = step.get("confirmation") or {}
            if "execution_location" in confirmation:
                confirmation["execution_location"] = _normalize_execution_location(
                    confirmation.get("execution_location")
                )

    return normalized
