"""Shared cross-cutting helpers for Smart Home PyTree."""

from __future__ import annotations

import logging
import os
import pprint
from typing import Union

import yaml
from rclpy.logging import LoggingSeverity

logger = logging.getLogger(__name__)


def str2bool(value: Union[str, int, bool]) -> bool:
    """Convert common truthy string/int values into a boolean."""
    return str(value).lower() in ("true", "1", "t", "yes")


def parse_duration(value: Union[str, int, float]) -> int:
    """Parse numeric or simple math-string durations into whole seconds."""
    if isinstance(value, (int, float)):
        return int(value)

    if isinstance(value, str):
        clean_value = value.strip()
        if "*" in clean_value:
            parts = clean_value.split("*")
            try:
                result = 1.0
                for part in parts:
                    result *= float(part.strip())
                return int(result)
            except ValueError:
                logger.error("Could not parse math string in duration: '%s'", value)
                return 0

        try:
            return int(float(clean_value))
        except ValueError:
            logger.warning(
                "Invalid duration string provided: '%s'. Defaulting to 0.", value
            )
            return 0

    logger.debug("Unexpected type passed to parse_duration: %s", type(value))
    return 0


def get_env(name: str, default: str | None = None) -> str | None:
    return os.getenv(name, default)



def get_house_yaml_path(env_key: str | None = None) -> str | None:
    if env_key:
        return os.getenv(env_key)
    return os.getenv("house_yaml_path") or os.getenv("HOUSE_YAML_PATH")



def load_yaml_file(yaml_path: str) -> dict:
    with open(yaml_path, "r", encoding="utf-8") as file:
        return yaml.safe_load(file) or {}


class BlackboardLogger:
    """Logger wrapper stored on the py_trees blackboard."""

    def __init__(self, node=None, debug_mode: bool = False):
        self.node = node
        self.debug_mode = debug_mode
        self._logger = None

        if self.node:
            self._logger = self.node.get_logger()
            self._logger.set_level(
                LoggingSeverity.DEBUG if self.debug_mode else LoggingSeverity.INFO
            )

        self.info("BlackboardLogger initialized")

    def info(self, message):
        if self._logger:
            self._logger.info(str(message))
        else:
            print(f"[INFO] {message}")

    def warn(self, message):
        if self._logger:
            self._logger.warn(str(message))
        else:
            print(f"[WARN] {message}")

    def error(self, message):
        if self._logger:
            self._logger.error(str(message))
        else:
            print(f"[ERROR] {message}")

    def debug(self, message):
        if self._logger:
            self._logger.debug(str(message))
            return
        if self.debug_mode:
            if isinstance(message, (dict, list)):
                print("[DEBUG] Complex Data:")
                pprint.pprint(message)
            else:
                print(f"[DEBUG] {message}")

    def notify_discord(self, message, severity: str = "INFO"):
        full_message = f"weblog= {message}"
        severity = severity.upper()

        if severity == "WARN":
            self.warn(full_message)
        elif severity == "ERROR":
            self.error(full_message)
        elif severity == "DEBUG":
            self.debug(full_message)
        else:
            self.info(full_message)
