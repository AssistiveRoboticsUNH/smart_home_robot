import textwrap

import pytest

from smart_home_pytree.protocols.loader import load_house_config_yaml
from smart_home_pytree.protocols.schema import validate_house_config
from smart_home_pytree.robot_interface import resolve_person_init_from_yaml_path


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


def test_person_init_accepts_none():
    data = _base_config()
    data["person_init"] = None
    validate_house_config(data)


def test_person_init_accepts_known_location():
    data = _base_config()
    data["person_init"] = "living_room"
    validate_house_config(data)


def test_person_init_rejects_unknown_location():
    data = _base_config()
    data["person_init"] = "garage"
    with pytest.raises(ValueError, match="person_init"):
        validate_house_config(data)


def test_resolve_person_init_from_yaml_path_returns_none_when_unset(tmp_path):
    yaml_path = tmp_path / "house.yaml"
    yaml_path.write_text(
        textwrap.dedent(
            """
            locations:
              home: {x: 0.0, y: 0.0, yaw: 0.0}
            protocols:
              demo:
                runner: GenericProtocol
                action:
                  steps:
                    - tree_name: play_text
                      tree_params:
                        text: hello
                high_level:
                  priority: 1
                  triggers: {}
            """
        ).strip()
    )

    assert resolve_person_init_from_yaml_path(str(yaml_path)) is None


def test_resolve_person_init_from_yaml_path_reads_landmark(tmp_path):
    yaml_path = tmp_path / "house.yaml"
    yaml_path.write_text(
        textwrap.dedent(
            """
            locations:
              home: {x: 0.0, y: 0.0, yaw: 0.0}
              living_room: {x: 1.0, y: 1.0, yaw: 0.0}
            person_init: living_room
            protocols:
              demo:
                runner: GenericProtocol
                action:
                  steps:
                    - tree_name: play_text
                      tree_params:
                        text: hello
                high_level:
                  priority: 1
                  triggers: {}
            """
        ).strip()
    )

    assert resolve_person_init_from_yaml_path(str(yaml_path)) == "living_room"


def test_location_yaw_degrees_validate_in_raw_config():
    data = _base_config()
    data["locations"]["home"]["yaw"] = 180.0
    validate_house_config(data)


def test_load_house_config_yaml_converts_location_yaw_degrees_to_radians(tmp_path):
    yaml_path = tmp_path / "house.yaml"
    yaml_path.write_text(
        textwrap.dedent(
            """
            locations:
              home: {x: 0.0, y: 0.0, yaw: 180.0}
            protocols:
              demo:
                runner: GenericProtocol
                action:
                  steps:
                    - tree_name: play_text
                      tree_params:
                        text: hello
                high_level:
                  priority: 1
                  triggers: {}
            """
        ).strip()
    )

    data = load_house_config_yaml(str(yaml_path))
    assert data["locations"]["home"]["yaw"] == pytest.approx(3.141592653589793)
