#!/usr/bin/env python3

"""
Validation helpers for yaml protocol schema.

Phase 1 focus:
  - strict validation for GenericProtocol action.steps
  - strict validation for high_level trigger/reset fields
  - basic validation for special/custom runners (runner/low_level/high_level shape)
"""

from __future__ import annotations

from datetime import datetime
from typing import Any

import yaml


ALLOWED_TOP_LEVEL_KEYS = {
    "locations",
    "person_init",
    "protocols",
}

ALLOWED_RUNNERS = {
    "GenericProtocol",
    "ExerciseProtocol",
    "ExerciseRandomProtocol",
}

ALLOWED_PROTOCOL_KEYS_GENERIC = {"runner", "action", "high_level"}
ALLOWED_PROTOCOL_KEYS_SPECIAL = {"runner", "low_level", "high_level"}
ALLOWED_HIGH_LEVEL_KEYS = {"priority", "triggers", "reset_pattern", "success_on"}
ALLOWED_TRIGGER_KEYS = {"time", "event", "permissible_locations", "protocol_completion", "all", "any"}
ALLOWED_ACTION_BLOCK_KEYS = {"steps"}

ALLOWED_TREE_STEP_KEYS = {"tree_name", "tree_params", "next_step_after"}
ALLOWED_CONFIRMATION_STEP_KEYS = {"confirmation", "next_step_after"}
ALLOWED_CONFIRMATION_BLOCK_KEYS = {"question", "on_yes", "on_no", "execution_location"}
ALLOWED_RESET_TYPES = {"eod", "instant", "periodic", "default"}
ALLOWED_DAY_NAMES = {
    "monday",
    "tuesday",
    "wednesday",
    "thursday",
    "friday",
    "saturday",
    "sunday",
}
ALLOWED_EVENT_OPERATORS = {"=", "!=", ">", ">=", "<", "<="}
ALLOWED_EVENT_EDGES = {"rising", "falling"}
ALLOWED_PROTOCOL_COMPLETION_STATUSES = {"completed", "failed", "preempted"}

# Central metadata for executable step `tree_name`.
# Used for schema validation and as developer-facing documentation of supported
# tree names and allowed `tree_params`.
RUN_TREE_SPECS = {
    "move_to_person_location": {
        "description": "Run the existing MoveToPersonLocationTree.",
        "required_params": {},
        "optional_params": {
            "end_sleep": "number seconds (blocking sleep before step completion)",
        },
    },
    "play_text": {
        "description": "Speak text using the existing person-facing ReadScriptTree.",
        "required_params": {
            "text": "str",
        },
        "optional_params": {
            "execution_location": "str ('current', 'person', or landmark name)",
            "end_sleep": "number seconds (blocking sleep before step completion)",
        },
        "blackboard_value_param": "text",
    },
    "play_audio": {
        "description": "Play an audio file using the existing person-facing PlayAudioTree.",
        "required_params": {
            "audio_path": "str (file path)",
        },
        "optional_params": {
            "execution_location": "str ('current', 'person', or landmark name)",
            "end_sleep": "number seconds (blocking sleep before step completion)",
        },
        "blackboard_value_param": "audio_path",
    },
    "play_video": {
        "description": "Play a video file using the existing person-facing PlayVideoTree.",
        "required_params": {
            "video_path": "str (file path)",
        },
        "optional_params": {
            "execution_location": "str ('current', 'person', or landmark name)",
            "end_sleep": "number seconds (blocking sleep before step completion)",
        },
        "blackboard_value_param": "video_path",
    },
    "move_to_location": {
        "description": "Run the existing MoveToLocationTree with a fixed target location.",
        "required_params": {
            "location": "str (must exist in top-level locations)",
        },
        "optional_params": {
            "end_sleep": "number seconds (blocking sleep before step completion)",
        },
    },
    "charge_robot": {
        "description": "Run the existing ChargeRobotTree.",
        "required_params": {},
        "optional_params": {
            "num_attempts": "int >= 1 (default: 3)",
            "end_sleep": "number seconds (blocking sleep before step completion)",
        },
    },
    "move_away": {
        "description": (
            "Move to 'home' or configured away location based on position_state_key, "
            "optionally perform a blocking sleep before resetting a trigger state key to False."
        ),
        "required_params": {
            "away_location": "str (must exist in top-level locations)",
            "position_state_key": "str (robot state key; expected values 'home' or 'away')",
            "state_key": "str (robot state key to set False after execution)",
        },
        "optional_params": {
            "end_sleep": "number seconds (blocking sleep after move, before reset state reset)",
        },
    },
}


def load_house_config_yaml(yaml_path: str) -> dict:
    with open(yaml_path, "r") as file:
        data = yaml.safe_load(file) or {}
    validate_house_config(data)
    return data


def validate_house_config(data: dict) -> None:
    _ensure_type(data, dict, "root")
    _reject_unknown_keys(data, ALLOWED_TOP_LEVEL_KEYS, "root")

    locations = data.get("locations")
    _validate_locations(locations)
    _validate_person_init(data.get("person_init"), "person_init", locations)

    protocols = data.get("protocols")
    _ensure_type(protocols, dict, "protocols")
    if not protocols:
        raise ValueError("protocols must be a non-empty mapping")

    for protocol_name, protocol_data in protocols.items():
        _validate_protocol(protocol_name, protocol_data, locations, protocols)


def _validate_locations(locations: Any) -> None:
    _ensure_type(locations, dict, "locations")
    if not locations:
        raise ValueError("locations must be a non-empty mapping")
    for loc_name, loc in locations.items():
        _ensure_non_empty_string(loc_name, f"locations[{loc_name!r}] key")
        path = f"locations.{loc_name}"
        _ensure_type(loc, dict, path)
        _reject_unknown_keys(loc, {"x", "y", "yaw"}, path)
        for field in ("x", "y", "yaw"):
            if field not in loc:
                raise ValueError(f"{path}.{field} is required")
            _ensure_numeric_not_bool(loc.get(field), f"{path}.{field}")


def _validate_person_init(value: Any, path: str, locations: dict) -> None:
    if value is None:
        return
    _ensure_non_empty_string(value, path)
    if value not in locations:
        raise ValueError(
            f"{path}='{value}' is invalid. Expected null or a landmark defined in top-level locations."
        )


def _validate_protocol(protocol_name: str, protocol_data: Any, locations: dict, protocols: dict) -> None:
    _ensure_non_empty_string(protocol_name, "protocol name")
    _ensure_type(protocol_data, dict, f"protocols.{protocol_name}")

    runner = protocol_data.get("runner")
    _ensure_non_empty_string(runner, f"protocols.{protocol_name}.runner")
    if runner not in ALLOWED_RUNNERS:
        raise ValueError(
            f"protocols.{protocol_name}.runner='{runner}' is not allowed. "
            f"Allowed: {sorted(ALLOWED_RUNNERS)}"
        )

    if runner == "GenericProtocol":
        _reject_unknown_keys(
            protocol_data, ALLOWED_PROTOCOL_KEYS_GENERIC, f"protocols.{protocol_name}"
        )
        _validate_generic_action(protocol_name, protocol_data.get("action"), locations)
    else:
        _reject_unknown_keys(
            protocol_data, ALLOWED_PROTOCOL_KEYS_SPECIAL, f"protocols.{protocol_name}"
        )
        if "low_level" in protocol_data and protocol_data.get("low_level") is not None:
            _ensure_type(
                protocol_data.get("low_level"), dict, f"protocols.{protocol_name}.low_level"
            )

    _validate_high_level(protocol_name, protocol_data.get("high_level"), locations, protocols)


def _validate_generic_action(protocol_name: str, action: Any, locations: dict) -> None:
    _ensure_type(action, dict, f"protocols.{protocol_name}.action")
    _reject_unknown_keys(action, ALLOWED_ACTION_BLOCK_KEYS, f"protocols.{protocol_name}.action")

    steps = action.get("steps")
    _ensure_type(steps, list, f"protocols.{protocol_name}.action.steps")
    if not steps:
        raise ValueError(f"protocols.{protocol_name}.action.steps must be a non-empty list")

    for idx, step in enumerate(steps, start=1):
        _validate_action_list_step(protocol_name, idx, step, locations)


def _validate_action_list_step(protocol_name: str, idx: int, step: Any, locations: dict) -> None:
    path = f"protocols.{protocol_name}.action.steps[{idx}]"
    _validate_step_object(step, path, allow_confirmation_branch=True, locations=locations)


def _validate_step_object(step: Any, path: str, allow_confirmation_branch: bool, locations: dict) -> None:
    _ensure_type(step, dict, path)

    has_tree = "tree_name" in step
    has_confirmation = "confirmation" in step

    if has_tree and has_confirmation:
        raise ValueError(f"{path} cannot contain both 'tree_name' and 'confirmation'")
    if not has_tree and not has_confirmation:
        raise ValueError(f"{path} must contain either 'tree_name' or 'confirmation'")

    if has_confirmation:
        if not allow_confirmation_branch:
            raise ValueError(f"{path} confirmation branching is not allowed in this context")
        _reject_unknown_keys(step, ALLOWED_CONFIRMATION_STEP_KEYS, path)
        _validate_confirmation_step(step.get("confirmation"), f"{path}.confirmation", locations)
        if "next_step_after" in step:
            _validate_wait_after(step.get("next_step_after"), f"{path}.next_step_after")
        return

    _validate_tree_step(
        step,
        path,
        locations=locations,
    )


def _validate_tree_step(
    step: dict,
    path: str,
    locations: dict,
) -> None:
    _reject_unknown_keys(step, ALLOWED_TREE_STEP_KEYS, path)
    tree_name = step.get("tree_name")
    _ensure_non_empty_string(tree_name, f"{path}.tree_name")
    tree_params = step.get("tree_params", {}) if "tree_params" in step else {}
    _ensure_type(tree_params, dict, f"{path}.tree_params")
    _validate_tree_params(tree_name, tree_params, path, locations=locations)

    if "next_step_after" in step:
        _validate_wait_after(step.get("next_step_after"), f"{path}.next_step_after")


def _validate_confirmation_step(confirmation: Any, path: str, locations: dict) -> None:
    _ensure_type(confirmation, dict, path)
    _reject_unknown_keys(confirmation, ALLOWED_CONFIRMATION_BLOCK_KEYS, path)

    _ensure_non_empty_string(confirmation.get("question"), f"{path}.question")
    if "execution_location" in confirmation:
        _validate_execution_location(
            confirmation.get("execution_location"),
            f"{path}.execution_location",
            locations=locations,
        )

    for branch_name in ("on_yes", "on_no"):
        branch_steps = confirmation.get(branch_name)
        branch_path = f"{path}.{branch_name}"
        _ensure_type(branch_steps, list, branch_path)
        if len(branch_steps) == 0:
            raise ValueError(f"{branch_path} must be a non-empty list")
        for idx, branch_step in enumerate(branch_steps, start=1):
            _validate_step_object(
                branch_step,
                f"{branch_path}[{idx}]",
                allow_confirmation_branch=False,
                locations=locations,
            )


def _validate_high_level(protocol_name: str, high_level: Any, locations: dict, protocols: dict) -> None:
    path = f"protocols.{protocol_name}.high_level"
    _ensure_type(high_level, dict, path)
    _reject_unknown_keys(high_level, ALLOWED_HIGH_LEVEL_KEYS, path)

    if "priority" in high_level:
        priority = high_level["priority"]
        if not isinstance(priority, int):
            raise ValueError(f"{path}.priority must be an integer")
        if priority < 0:
            raise ValueError(f"{path}.priority must be >= 0")

    triggers = high_level.get("triggers", {})
    _validate_triggers(protocol_name, triggers, locations, protocols)

    if "reset_pattern" in high_level:
        _validate_reset_pattern(high_level["reset_pattern"], f"{path}.reset_pattern")

    if "success_on" in high_level:
        _validate_success_on(high_level["success_on"], f"{path}.success_on")


def _validate_triggers(protocol_name: str, triggers: Any, locations: dict, protocols: dict) -> None:
    path = f"protocols.{protocol_name}.high_level.triggers"
    _ensure_type(triggers, dict, path)
    _validate_trigger_block(triggers, path, locations, protocols)


def _validate_trigger_block(trigger_block: Any, path: str, locations: dict, protocols: dict) -> None:
    _ensure_type(trigger_block, dict, path)
    _reject_unknown_keys(trigger_block, ALLOWED_TRIGGER_KEYS, path)

    if "time" in trigger_block:
        _validate_time_requirement(trigger_block["time"], f"{path}.time")
    if "event" in trigger_block:
        _validate_event_conditions(trigger_block["event"], f"{path}.event", locations)
    if "permissible_locations" in trigger_block:
        _validate_permissible_locations(
            trigger_block["permissible_locations"],
            locations,
            f"{path}.permissible_locations",
        )
    if "protocol_completion" in trigger_block:
        _validate_protocol_completion_requirement(
            trigger_block["protocol_completion"],
            f"{path}.protocol_completion",
            protocols,
        )
    if "all" in trigger_block:
        _validate_trigger_group(trigger_block["all"], f"{path}.all", locations, protocols)
    if "any" in trigger_block:
        _validate_trigger_group(trigger_block["any"], f"{path}.any", locations, protocols)


def _validate_trigger_group(group_items: Any, path: str, locations: dict, protocols: dict) -> None:
    _ensure_type(group_items, list, path)
    if not group_items:
        raise ValueError(f"{path} must be a non-empty list when provided")
    for idx, item in enumerate(group_items, start=1):
        _validate_trigger_block(item, f"{path}[{idx}]", locations, protocols)


def _validate_time_requirement(time_req: Any, path: str) -> None:
    _ensure_type(time_req, dict, path)
    _reject_unknown_keys(time_req, {"from", "to", "at", "day"}, path)

    has_window = "from" in time_req or "to" in time_req
    has_at = "at" in time_req

    if has_window and has_at:
        raise ValueError(f"{path} cannot combine 'at' with 'from'/'to'")
    if has_window:
        if "from" not in time_req or "to" not in time_req:
            raise ValueError(f"{path} must contain both 'from' and 'to'")
        t_from = _parse_hhmm(time_req["from"], f"{path}.from")
        t_to = _parse_hhmm(time_req["to"], f"{path}.to")
        if t_from > t_to:
            raise ValueError(f"{path}.from must be <= {path}.to (same-day windows only)")
    elif has_at:
        _parse_hhmm(time_req["at"], f"{path}.at")
    else:
        raise ValueError(f"{path} must contain either 'at' or both 'from' and 'to'")

    if "day" in time_req:
        _validate_day_requirement(time_req["day"], f"{path}.day")


def _validate_event_conditions(event_reqs: Any, path: str, locations: dict) -> None:
    if isinstance(event_reqs, dict):
        event_reqs = [event_reqs]
    _ensure_type(event_reqs, list, path)
    if not event_reqs:
        raise ValueError(f"{path} must be a non-empty list when provided")
    for idx, cond in enumerate(event_reqs, start=1):
        cpath = f"{path}[{idx}]"
        _ensure_type(cond, dict, cpath)
        state_name = cond.get("state")
        _ensure_non_empty_string(state_name, f"{cpath}.state")

        if state_name == "robot_location_xy":
            _reject_unknown_keys(cond, {"state", "within_m", "point_xy"}, cpath)
            if "within_m" not in cond:
                raise ValueError(f"{cpath}.within_m is required for robot_location_xy event")
            within_m = cond.get("within_m")
            if isinstance(within_m, bool) or not isinstance(within_m, (int, float)):
                raise ValueError(f"{cpath}.within_m must be a number > 0")
            if within_m <= 0:
                raise ValueError(f"{cpath}.within_m must be > 0")
            point_xy = cond.get("point_xy")
            _ensure_type(point_xy, list, f"{cpath}.point_xy")
            if len(point_xy) != 2:
                raise ValueError(f"{cpath}.point_xy must contain exactly [x, y]")
            for pidx, pval in enumerate(point_xy, start=1):
                if isinstance(pval, bool) or not isinstance(pval, (int, float)):
                    raise ValueError(f"{cpath}.point_xy[{pidx}] must be numeric")
            continue

        _reject_unknown_keys(cond, {"state", "value", "op", "edge"}, cpath)
        if "value" not in cond:
            raise ValueError(f"{cpath}.value is required")
        if "op" in cond:
            op = cond.get("op")
            _ensure_non_empty_string(op, f"{cpath}.op")
            if op not in ALLOWED_EVENT_OPERATORS:
                raise ValueError(
                    f"{cpath}.op='{op}' is invalid. Allowed: {sorted(ALLOWED_EVENT_OPERATORS)}"
                )
            if op in {">", ">=", "<", "<="}:
                val = cond.get("value")
                if isinstance(val, bool) or not isinstance(val, (int, float)):
                    raise ValueError(
                        f"{cpath}.value must be numeric when using op '{op}'"
                    )
        if "edge" in cond:
            edge = cond.get("edge")
            _ensure_non_empty_string(edge, f"{cpath}.edge")
            if edge not in ALLOWED_EVENT_EDGES:
                raise ValueError(
                    f"{cpath}.edge='{edge}' is invalid. Allowed: {sorted(ALLOWED_EVENT_EDGES)}"
                )
            if "op" in cond and cond.get("op") != "=":
                raise ValueError(f"{cpath}.op must be '=' when using edge detection")
            value = cond.get("value")
            if not isinstance(value, bool):
                raise ValueError(f"{cpath}.value must be boolean when using edge detection")
            if edge == "rising" and value is not True:
                raise ValueError(f"{cpath}.value must be true for edge='rising'")
            if edge == "falling" and value is not False:
                raise ValueError(f"{cpath}.value must be false for edge='falling'")


def _validate_protocol_completion_requirement(requirement: Any, path: str, protocols: dict) -> None:
    _ensure_type(requirement, dict, path)
    _reject_unknown_keys(
        requirement,
        {"protocol", "statuses", "after_seconds", "within_seconds"},
        path,
    )

    source_protocol = requirement.get("protocol")
    _ensure_non_empty_string(source_protocol, f"{path}.protocol")
    if "." not in source_protocol:
        raise ValueError(f"{path}.protocol must be in the form '<Runner>.<protocol_name>'")
    runner, protocol_name = source_protocol.split(".", 1)
    if protocol_name not in protocols:
        raise ValueError(f"{path}.protocol references unknown protocol '{protocol_name}'")
    configured_runner = protocols[protocol_name].get("runner", "GenericProtocol")
    if configured_runner != runner:
        raise ValueError(
            f"{path}.protocol='{source_protocol}' runner mismatch. YAML defines '{configured_runner}.{protocol_name}'."
        )

    statuses = requirement.get("statuses")
    _ensure_type(statuses, list, f"{path}.statuses")
    if not statuses:
        raise ValueError(f"{path}.statuses must be a non-empty list")
    for idx, status in enumerate(statuses, start=1):
        _ensure_non_empty_string(status, f"{path}.statuses[{idx}]")
        if status not in ALLOWED_PROTOCOL_COMPLETION_STATUSES:
            raise ValueError(
                f"{path}.statuses[{idx}]='{status}' is invalid. Allowed: {sorted(ALLOWED_PROTOCOL_COMPLETION_STATUSES)}"
            )

    within_seconds = requirement.get("within_seconds")
    _ensure_numeric_not_bool(within_seconds, f"{path}.within_seconds")
    if float(within_seconds) <= 0:
        raise ValueError(f"{path}.within_seconds must be > 0")

    if "after_seconds" in requirement:
        after_seconds = requirement.get("after_seconds")
        _ensure_numeric_not_bool(after_seconds, f"{path}.after_seconds")
        if float(after_seconds) < 0:
            raise ValueError(f"{path}.after_seconds must be >= 0")
        if float(after_seconds) > float(within_seconds):
            raise ValueError(f"{path}.after_seconds must be <= {path}.within_seconds")



def _validate_day_requirement(days: Any, path: str) -> None:
    _ensure_type(days, list, path)
    for idx, day in enumerate(days, start=1):
        _ensure_non_empty_string(day, f"{path}[{idx}]")
        if str(day).strip().lower() not in ALLOWED_DAY_NAMES:
            raise ValueError(
                f"{path}[{idx}]='{day}' is invalid. Allowed day names: {sorted(ALLOWED_DAY_NAMES)}"
            )


def _validate_permissible_locations(permissible_locations: Any, locations: dict, path: str) -> None:
    _ensure_type(permissible_locations, list, path)
    for idx, loc in enumerate(permissible_locations, start=1):
        _ensure_non_empty_string(loc, f"{path}[{idx}]")
        if loc not in locations:
            raise ValueError(
                f"{path}[{idx}]='{loc}' is not defined in top-level locations"
            )


def _validate_tree_params(tree_name: str, tree_params: dict, path: str, locations: dict | None) -> None:
    spec = RUN_TREE_SPECS.get(tree_name)
    if spec is None:
        raise ValueError(
            f"{path}.tree_name='{tree_name}' is unsupported. "
            f"Supported tree names: {sorted(RUN_TREE_SPECS.keys())}"
        )

    required = set(spec.get("required_params", {}).keys())
    optional = set(spec.get("optional_params", {}).keys())
    allowed = required | optional
    provided = set(tree_params.keys())

    missing = sorted(required - provided)
    if missing:
        raise ValueError(
            f"{path}.tree_params missing required keys {missing} for tree '{tree_name}'"
        )

    unknown = sorted(provided - allowed)
    if unknown:
        raise ValueError(
            f"{path}.tree_params has unsupported keys {unknown} for tree '{tree_name}'. "
            f"Allowed keys: {sorted(allowed)}"
        )

    # Type/value checks for known params used today.
    for key, value in tree_params.items():
        key_path = f"{path}.tree_params.{key}"
        if key in {"away_location", "position_state_key", "state_key", "location"}:
            _ensure_non_empty_string(value, key_path)
            if key in {"away_location", "location"} and locations is not None:
                if value not in locations:
                    raise ValueError(
                        f"{key_path}='{value}' is not defined in top-level locations"
                    )
        elif key == "num_attempts":
            if isinstance(value, bool) or not isinstance(value, int):
                raise ValueError(f"{key_path} must be an integer >= 1")
            if value < 1:
                raise ValueError(f"{key_path} must be >= 1")
        elif key == "end_sleep":
            _validate_nonnegative_seconds_number(value, key_path)
        elif key == "execution_location":
            _validate_execution_location(value, key_path, locations=locations)


def _validate_execution_location(value: Any, path: str, locations: dict | None) -> None:
    _ensure_non_empty_string(value, path)
    raw = str(value).strip()
    lowered = raw.lower()
    if lowered in {"current", "person"}:
        return
    if locations is not None and raw not in locations:
        raise ValueError(
            f"{path}='{value}' is invalid. Expected 'current', 'person', or a landmark "
            f"defined in top-level locations."
        )


def _validate_reset_pattern(reset_pattern: Any, path: str) -> None:
    _ensure_type(reset_pattern, dict, path)
    _reject_unknown_keys(reset_pattern, {"type", "hours", "minutes"}, path)

    pattern_type = reset_pattern.get("type")
    if pattern_type is None:
        raise ValueError(f"{path}.type is required when reset_pattern is provided")
    if pattern_type not in ALLOWED_RESET_TYPES:
        raise ValueError(
            f"{path}.type='{pattern_type}' is invalid. Allowed: {sorted(ALLOWED_RESET_TYPES)}"
        )

    hours = reset_pattern.get("hours", 0)
    minutes = reset_pattern.get("minutes", 0)
    if not isinstance(hours, (int, float)) or not isinstance(minutes, (int, float)):
        raise ValueError(f"{path}.hours and {path}.minutes must be numbers if provided")
    if hours < 0 or minutes < 0:
        raise ValueError(f"{path}.hours and {path}.minutes must be >= 0")

    if pattern_type == "periodic" and (hours == 0 and minutes == 0):
        raise ValueError(f"{path} periodic reset requires hours>0 or minutes>0")


def _validate_success_on(success_on: Any, path: str) -> None:
    _ensure_type(success_on, dict, path)

    if "state" in success_on:
        _reject_unknown_keys(success_on, {"state", "value"}, path)
        _ensure_non_empty_string(success_on.get("state"), f"{path}.state")
        if "value" not in success_on:
            raise ValueError(f"{path}.value is required")
        return

    if "all" in success_on:
        _reject_unknown_keys(success_on, {"all"}, path)
        _validate_event_conditions(success_on["all"], f"{path}.all")
        return

    if "any" in success_on:
        _reject_unknown_keys(success_on, {"any"}, path)
        _validate_event_conditions(success_on["any"], f"{path}.any")
        return

    raise ValueError(
        f"{path} must be one of: {{state, value}} or {{all: [...]}} or {{any: [...]}}"
    )


def _validate_wait_after(value: Any, path: str) -> None:
    seconds = _parse_duration_for_validation(value, path)
    if seconds < 0:
        raise ValueError(f"{path} must resolve to >= 0 seconds")


def _validate_nonnegative_seconds_number(value: Any, path: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path} must be a number of seconds (int/float)")
    if value < 0:
        raise ValueError(f"{path} must be >= 0 seconds")


def _parse_hhmm(value: Any, path: str) -> datetime:
    _ensure_non_empty_string(value, path)
    try:
        return datetime.strptime(str(value), "%H:%M")
    except ValueError as exc:
        raise ValueError(f"{path} must match HH:MM (24h), got '{value}'") from exc


def _parse_duration_for_validation(value: Any, path: str) -> int:
    if isinstance(value, bool):
        raise ValueError(f"{path} must be a duration, not bool")
    if isinstance(value, (int, float)):
        return int(value)
    if isinstance(value, str):
        clean = value.strip()
        if not clean:
            raise ValueError(f"{path} must not be empty")
        if "*" in clean:
            parts = clean.split("*")
            try:
                result = 1.0
                for part in parts:
                    result *= float(part.strip())
                return int(result)
            except ValueError as exc:
                raise ValueError(f"{path} has invalid math duration '{value}'") from exc
        try:
            return int(float(clean))
        except ValueError as exc:
            raise ValueError(f"{path} has invalid duration '{value}'") from exc
    raise ValueError(f"{path} has unsupported type {type(value).__name__}")


def _ensure_type(value: Any, expected_type: type, path: str) -> None:
    if not isinstance(value, expected_type):
        raise ValueError(
            f"{path} must be {expected_type.__name__}, got {type(value).__name__}"
        )


def _ensure_numeric_not_bool(value: Any, path: str) -> None:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{path} must be numeric")


def _ensure_non_empty_string(value: Any, path: str) -> None:
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{path} must be a non-empty string")


def _reject_unknown_keys(data: dict, allowed_keys: set[str], path: str) -> None:
    unknown = set(data.keys()) - set(allowed_keys)
    if unknown:
        raise ValueError(
            f"{path} contains unsupported keys {sorted(unknown)}. "
            f"Allowed keys: {sorted(allowed_keys)}"
        )
