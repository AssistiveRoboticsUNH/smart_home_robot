"""Pure trigger evaluation helpers."""

from __future__ import annotations

from datetime import datetime
from typing import Any


def normalize_success_on(success_on: dict[str, Any]) -> dict[str, Any]:
    if "state" in success_on:
        return {"mode": "all", "conditions": [success_on]}
    if "all" in success_on:
        return {"mode": "all", "conditions": success_on["all"]}
    if "any" in success_on:
        return {"mode": "any", "conditions": success_on["any"]}
    raise ValueError(f"Invalid success_on format: {success_on}")


def extract_event_keys(protocols_yaml: dict) -> list[str]:
    event_keys = set()
    for protocol_data in protocols_yaml.get("protocols", {}).values():
        high_level_subdata = protocol_data.get("high_level", {})
        triggers = high_level_subdata.get("triggers", {})
        _collect_event_keys_from_trigger_block(triggers, event_keys)
    return sorted(event_keys)


def _normalize_event_rules(event_reqs):
    if isinstance(event_reqs, dict):
        return [event_reqs]
    return event_reqs or []


def _collect_event_keys_from_trigger_block(trigger_block: dict, event_keys: set[str]) -> None:
    if not isinstance(trigger_block, dict):
        return

    for event_rule in _normalize_event_rules(trigger_block.get("event")):
        if isinstance(event_rule, dict) and "state" in event_rule:
            event_keys.add(event_rule["state"])

    for key in ("all", "any"):
        for child in trigger_block.get(key, []) or []:
            _collect_event_keys_from_trigger_block(child, event_keys)


def collect_current_events(robot_interface, event_keys: list[str], bb_logger) -> dict[str, Any]:
    current_events = {}
    for key in event_keys:
        value = robot_interface.state.get(key)
        if value is None:
            bb_logger.debug(
                f"[TriggerMonitor] Topic '{key}' not publishing or None -> treating as False"
            )
            current_events[key] = False
        else:
            current_events[key] = value
    return current_events


def check_location_requirement(robot_interface, permissible_locations, bb_logger) -> bool:
    if not permissible_locations:
        return True
    current_loc = robot_interface.state.get("person_location")
    if current_loc is None:
        bb_logger.debug("[TriggerMonitor] 'person_location' not found in state.")
        return False
    return current_loc in permissible_locations


def check_day_requirement(day_req, current_day=None) -> bool:
    if not day_req:
        return True
    if current_day is None:
        current_day = datetime.now().strftime("%A")
    return current_day.lower() in [day.lower() for day in day_req]


def check_time_requirement(time_req, current_time_str: str) -> bool:
    if not time_req:
        return True
    fmt = "%H:%M"
    t_now = datetime.strptime(current_time_str, fmt)
    if "at" in time_req:
        t_at = datetime.strptime(time_req["at"], fmt)
        return t_now == t_at
    t_from = datetime.strptime(time_req["from"], fmt)
    t_to = datetime.strptime(time_req["to"], fmt)
    return t_from <= t_now <= t_to


def event_xy_within_point_match(current_value, event_rule: dict) -> bool:
    if (
        not isinstance(current_value, (tuple, list))
        or len(current_value) < 2
        or not isinstance(current_value[0], (int, float))
        or not isinstance(current_value[1], (int, float))
    ):
        return False

    within_m = event_rule.get("within_m")
    point_xy = event_rule.get("point_xy")
    if isinstance(within_m, bool) or not isinstance(within_m, (int, float)) or within_m <= 0:
        return False
    if (
        not isinstance(point_xy, (tuple, list))
        or len(point_xy) != 2
        or isinstance(point_xy[0], bool)
        or isinstance(point_xy[1], bool)
        or not isinstance(point_xy[0], (int, float))
        or not isinstance(point_xy[1], (int, float))
    ):
        return False

    cx, cy = float(current_value[0]), float(current_value[1])
    tx, ty = float(point_xy[0]), float(point_xy[1])
    threshold_sq = float(within_m) ** 2
    dx = cx - tx
    dy = cy - ty
    return (dx * dx + dy * dy) <= threshold_sq


def event_condition_match(current_value, expected_value, op: str) -> bool:
    if op == "=":
        return current_value == expected_value
    if op == "!=":
        return current_value != expected_value

    if (
        isinstance(current_value, bool)
        or isinstance(expected_value, bool)
        or not isinstance(current_value, (int, float))
        or not isinstance(expected_value, (int, float))
    ):
        return False

    if op == ">":
        return current_value > expected_value
    if op == ">=":
        return current_value >= expected_value
    if op == "<":
        return current_value < expected_value
    if op == "<=":
        return current_value <= expected_value
    return False


def check_event_requirement(event_reqs, current_events, bb_logger=None) -> bool:
    if not event_reqs:
        return True

    for event_rule in _normalize_event_rules(event_reqs):
        topic = event_rule["state"]
        if topic not in current_events:
            return False
        if topic == "robot_location_xy":
            if not event_xy_within_point_match(current_events[topic], event_rule):
                return False
            continue

        expected = event_rule["value"]
        op = event_rule.get("op", "=")
        if not event_condition_match(current_events[topic], expected, op):
            return False

    return True


def evaluate_trigger_block(
    trigger_block,
    *,
    current_events,
    current_time_str: str,
    current_day,
    robot_interface,
    bb_logger=None,
) -> bool:
    if not trigger_block:
        return True
    if not isinstance(trigger_block, dict):
        return False

    results = []

    if "event" in trigger_block:
        results.append(check_event_requirement(trigger_block["event"], current_events, bb_logger))
    if "time" in trigger_block:
        time_req = trigger_block["time"]
        day_req = time_req.get("day", []) if isinstance(time_req, dict) else []
        results.append(
            check_day_requirement(day_req, current_day)
            and check_time_requirement(time_req, current_time_str)
        )
    if "permissible_locations" in trigger_block:
        results.append(
            check_location_requirement(robot_interface, trigger_block["permissible_locations"], bb_logger)
        )
    if "all" in trigger_block:
        results.append(
            all(
                evaluate_trigger_block(
                    child,
                    current_events=current_events,
                    current_time_str=current_time_str,
                    current_day=current_day,
                    robot_interface=robot_interface,
                    bb_logger=bb_logger,
                )
                for child in (trigger_block.get("all") or [])
            )
        )
    if "any" in trigger_block:
        results.append(
            any(
                evaluate_trigger_block(
                    child,
                    current_events=current_events,
                    current_time_str=current_time_str,
                    current_day=current_day,
                    robot_interface=robot_interface,
                    bb_logger=bb_logger,
                )
                for child in (trigger_block.get("any") or [])
            )
        )

    return all(results) if results else True
