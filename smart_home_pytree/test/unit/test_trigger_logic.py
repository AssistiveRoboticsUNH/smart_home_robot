from __future__ import annotations

from types import SimpleNamespace

from smart_home_pytree.protocols.schema import validate_house_config
from smart_home_pytree.triggers.evaluator import evaluate_trigger_block, extract_event_keys


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

