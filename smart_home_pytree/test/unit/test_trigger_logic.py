from __future__ import annotations

from types import SimpleNamespace

from smart_home_pytree.protocols.schema import validate_house_config
from datetime import datetime, timedelta

from smart_home_pytree.triggers.evaluator import (
    evaluate_trigger_block,
    evaluate_trigger_block_detailed,
    extract_event_keys,
)


class _LoggerStub:
    def debug(self, *_args, **_kwargs):
        return None


def _robot_interface(**state):
    return SimpleNamespace(state=state)


def test_validate_house_config_accepts_recursive_trigger_logic():
    data = {
        "locations": {
            "living_room": {"x": 0.0, "y": 0.0, "yaw": 0},
        },
        "person_init": None,
        "protocols": {
            "evening_rule": {
                "runner": "GenericProtocol",
                "action": {
                    "steps": [
                        {
                            "tree_name": "play_text",
                            "tree_params": {"text": "demo"},
                        }
                    ]
                },
                "high_level": {
                    "priority": 1,
                    "reset_pattern": {"type": "eod"},
                    "triggers": {
                        "any": [
                            {"event": {"state": "sensor1", "value": True}},
                            {"event": {"state": "sensor2", "value": True}},
                        ],
                        "time": {"at": "19:00"},
                    },
                },
            }
        },
    }

    validate_house_config(data)


def test_validate_house_config_accepts_protocol_completion_and_edge_trigger():
    data = {
        "locations": {
            "living_room": {"x": 0.0, "y": 0.0, "yaw": 0},
        },
        "person_init": None,
        "protocols": {
            "bathroom_reminder": {
                "runner": "GenericProtocol",
                "action": {"steps": [{"tree_name": "play_text", "tree_params": {"text": "go"}}]},
                "high_level": {"priority": 1, "reset_pattern": {"type": "eod"}, "triggers": {}},
            },
            "bathroom_thanks": {
                "runner": "GenericProtocol",
                "action": {"steps": [{"tree_name": "play_text", "tree_params": {"text": "thanks"}}]},
                "high_level": {
                    "priority": 1,
                    "reset_pattern": {"type": "eod"},
                    "triggers": {
                        "all": [
                            {
                                "protocol_completion": {
                                    "protocol": "GenericProtocol.bathroom_reminder",
                                    "statuses": ["completed"],
                                    "within_seconds": 120,
                                }
                            },
                            {"event": {"state": "bathroom_sensor", "value": True, "edge": "rising"}},
                        ]
                    },
                },
            },
        },
    }

    validate_house_config(data)


def test_validate_house_config_rejects_invalid_edge_value_combo():
    data = {
        "locations": {
            "living_room": {"x": 0.0, "y": 0.0, "yaw": 0},
        },
        "person_init": None,
        "protocols": {
            "demo": {
                "runner": "GenericProtocol",
                "action": {"steps": [{"tree_name": "play_text", "tree_params": {"text": "demo"}}]},
                "high_level": {
                    "priority": 1,
                    "reset_pattern": {"type": "eod"},
                    "triggers": {"event": {"state": "bathroom_sensor", "value": False, "edge": "rising"}},
                },
            }
        },
    }

    try:
        validate_house_config(data)
    except ValueError as exc:
        assert "edge='rising'" in str(exc) or "must be true" in str(exc)
    else:
        raise AssertionError("Expected invalid rising-edge/value combination to fail validation")


def test_extract_event_keys_reads_nested_groups():
    protocols_yaml = {
        "protocols": {
            "demo": {
                "high_level": {
                    "triggers": {
                        "all": [
                            {"event": {"state": "sensor1", "value": True}},
                            {
                                "any": [
                                    {"event": {"state": "sensor2", "value": True}},
                                    {"event": {"state": "sensor3", "value": True}},
                                ]
                            },
                        ]
                    }
                }
            }
        }
    }

    assert extract_event_keys(protocols_yaml) == ["sensor1", "sensor2", "sensor3"]


def test_evaluate_trigger_block_supports_or_and_exact_time():
    trigger_block = {
        "any": [
            {"event": {"state": "sensor1", "value": True}},
            {"event": {"state": "sensor2", "value": True}},
        ],
        "time": {"at": "19:00"},
    }

    assert evaluate_trigger_block(
        trigger_block,
        current_events={"sensor1": False, "sensor2": True},
        current_time_str="19:00",
        current_day=None,
        robot_interface=_robot_interface(),
        bb_logger=_LoggerStub(),
    )

    assert not evaluate_trigger_block(
        trigger_block,
        current_events={"sensor1": False, "sensor2": True},
        current_time_str="18:59",
        current_day=None,
        robot_interface=_robot_interface(),
        bb_logger=_LoggerStub(),
    )


def test_evaluate_trigger_block_preserves_legacy_and_shape():
    trigger_block = {
        "event": [
            {"state": "charging", "value": False},
            {"state": "coffee", "value": True},
        ],
        "time": {"from": "18:00", "to": "22:00", "day": ["friday"]},
        "permissible_locations": ["living_room"],
    }

    assert evaluate_trigger_block(
        trigger_block,
        current_events={"charging": False, "coffee": True},
        current_time_str="19:00",
        current_day="Friday",
        robot_interface=_robot_interface(person_location="living_room"),
        bb_logger=_LoggerStub(),
    )


def test_evaluate_trigger_block_supports_protocol_completion_and_rising_edge():
    now = datetime.now()
    trigger_block = {
        "all": [
            {
                "protocol_completion": {
                    "protocol": "GenericProtocol.bathroom_reminder",
                    "statuses": ["completed"],
                    "within_seconds": 120,
                }
            },
            {"event": {"state": "bathroom_sensor", "value": True, "edge": "rising"}},
        ]
    }
    completion_events = [
        {
            "id": 7,
            "source_protocol": "GenericProtocol.bathroom_reminder",
            "source_run_session_id": "run-1",
            "status": "completed",
            "completed_at": (now - timedelta(seconds=30)).isoformat(),
        }
    ]

    result = evaluate_trigger_block_detailed(
        trigger_block,
        current_events={"bathroom_sensor": True},
        previous_events={"bathroom_sensor": False},
        current_time_str="12:00",
        current_day="Friday",
        robot_interface=_robot_interface(),
        protocol_completion_events=completion_events,
        now=now,
        bb_logger=_LoggerStub(),
    )

    assert result["matched"] is True
    assert result["protocol_completion_event_ids"] == [7]

    assert not evaluate_trigger_block(
        trigger_block,
        current_events={"bathroom_sensor": True},
        previous_events={"bathroom_sensor": True},
        current_time_str="12:00",
        current_day="Friday",
        robot_interface=_robot_interface(),
        protocol_completion_events=completion_events,
        now=now,
        bb_logger=_LoggerStub(),
    )

    assert not evaluate_trigger_block(
        trigger_block,
        current_events={"bathroom_sensor": True},
        previous_events={"bathroom_sensor": False},
        current_time_str="12:00",
        current_day="Friday",
        robot_interface=_robot_interface(),
        protocol_completion_events=[
            {
                **completion_events[0],
                "completed_at": (now - timedelta(seconds=180)).isoformat(),
            }
        ],
        now=now,
        bb_logger=_LoggerStub(),
    )
