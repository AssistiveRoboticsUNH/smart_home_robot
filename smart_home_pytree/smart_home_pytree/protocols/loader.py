"""Load and validate protocol configuration from YAML."""

from __future__ import annotations

from smart_home_pytree.protocols.normalizer import normalize_house_config
from smart_home_pytree.protocols.schema import validate_house_config
from smart_home_pytree.utils import load_yaml_file


def load_house_config_yaml(yaml_path: str) -> dict:
    data = load_yaml_file(yaml_path)
    normalized = normalize_house_config(data)
    validate_house_config(normalized)
    return normalized
