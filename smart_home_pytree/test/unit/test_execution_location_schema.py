import pytest

from smart_home_pytree.protocol_schema import validate_house_config


def _base_config():
    return {
        "locations": {
            "home": {"x": 0.0, "y": 0.0, "yaw": 0.0},
            "living_room": {"x": 1.0, "y": 1.0, "yaw": 0.0},
        },
        "protocols": {
            "demo": {
                "runner": "GenericProtocol",
                "action": {
                    "steps": [
                        {
                            "tree_name": "play_text",
                            "tree_params": {"text": "hello"},
                        }
                    ]
                },
                "high_level": {"priority": 1, "triggers": {}},
            }
        },
    }


@pytest.mark.parametrize("value", ["current", "CURRENT", "person", "living_room"])
def test_execution_location_step_accepts_valid_values(value):
    data = _base_config()
    data["protocols"]["demo"]["action"]["steps"][0]["tree_params"]["execution_location"] = value
    validate_house_config(data)


def test_execution_location_step_rejects_unknown_landmark():
    data = _base_config()
    data["protocols"]["demo"]["action"]["steps"][0]["tree_params"]["execution_location"] = "garage"
    with pytest.raises(ValueError, match="execution_location"):
        validate_house_config(data)


@pytest.mark.parametrize("value", ["current", "person", "living_room"])
def test_execution_location_confirmation_accepts_valid_values(value):
    data = _base_config()
    data["protocols"]["demo"]["action"]["steps"] = [
        {
            "confirmation": {
                "question": "Continue?",
                "execution_location": value,
                "on_yes": [{"tree_name": "play_text", "tree_params": {"text": "yes"}}],
                "on_no": [{"tree_name": "play_text", "tree_params": {"text": "no"}}],
            }
        }
    ]
    validate_house_config(data)


def test_execution_location_confirmation_rejects_unknown_landmark():
    data = _base_config()
    data["protocols"]["demo"]["action"]["steps"] = [
        {
            "confirmation": {
                "question": "Continue?",
                "execution_location": "garage",
                "on_yes": [{"tree_name": "play_text", "tree_params": {"text": "yes"}}],
                "on_no": [{"tree_name": "play_text", "tree_params": {"text": "no"}}],
            }
        }
    ]
    with pytest.raises(ValueError, match="execution_location"):
        validate_house_config(data)
