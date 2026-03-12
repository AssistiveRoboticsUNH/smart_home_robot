"""Shared cross-cutting helpers for Smart Home PyTree."""

from __future__ import annotations

import logging
import os
import pprint
from pathlib import Path
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


def get_shr_user_dir() -> Path | None:
    raw = os.getenv("SHR_USER_DIR")
    if not raw:
        return None
    return Path(raw).expanduser().resolve()


def require_shr_user_dir() -> Path:
    user_dir = get_shr_user_dir()
    if user_dir is None:
        raise RuntimeError(
            "SHR_USER_DIR is not set. Run setup_user.sh and export SHR_USER_DIR in ~/.bashrc."
        )
    return user_dir


def get_user_subdir(name: str, required: bool = False) -> Path | None:
    user_dir = require_shr_user_dir() if required else get_shr_user_dir()
    if user_dir is None:
        return None
    return user_dir / name


def get_user_configs_dir(required: bool = False) -> Path | None:
    return get_user_subdir("configs", required=required)


def get_user_audio_dir(required: bool = False) -> Path | None:
    return get_user_subdir("audios", required=required)


def get_user_video_dir(required: bool = False) -> Path | None:
    return get_user_subdir("videos", required=required)


def get_user_images_dir(required: bool = False) -> Path | None:
    return get_user_subdir("images", required=required)


def get_user_logs_dir(required: bool = False) -> Path | None:
    return get_user_subdir("logs", required=required)


def get_user_database_dir(required: bool = False) -> Path | None:
    return get_user_subdir("database", required=required)


def get_user_map_dir(required: bool = False) -> Path | None:
    return get_user_subdir("map", required=required)


def resolve_media_path(value: str | None, media_kind: str) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return text
    if "://" in text:
        return text

    path = Path(text).expanduser()
    if path.is_absolute():
        return str(path)

    media_dir = (
        get_user_audio_dir(required=True)
        if media_kind == "audio"
        else get_user_video_dir(required=True)
    )
    return str((media_dir / text).resolve())


def resolve_media_dir_path(value: str | None, media_kind: str) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return text
    if "://" in text:
        return text

    path = Path(text).expanduser()
    if path.is_absolute():
        return str(path)

    media_dir = (
        get_user_audio_dir(required=True)
        if media_kind == "audio"
        else get_user_video_dir(required=True)
    )
    if text in {".", media_kind, f"{media_kind}s"}:
        return str(media_dir.resolve())
    return str((media_dir / text).resolve())


def resolve_config_file_path(value: str | None) -> str | None:
    if value is None:
        return None
    text = str(value).strip()
    if not text:
        return text

    path = Path(text).expanduser()
    if path.is_absolute() and path.exists():
        return str(path.resolve())

    user_configs = get_user_configs_dir(required=True)
    candidates = []
    candidates.append(user_configs / text)

    package_config = Path(__file__).resolve().parents[1] / "config" / text
    candidates.append(package_config)

    for candidate in candidates:
        if candidate.exists():
            return str(candidate.resolve())

    if path.is_absolute():
        return str(path.resolve())
    return str((user_configs / text).resolve())


def list_user_files(subdir: str, suffixes: tuple[str, ...] | None = None) -> list[str]:
    root = get_user_subdir(subdir, required=True)
    if not root.is_dir():
        return []
    entries = []
    for item in sorted(root.iterdir()):
        if not item.is_file():
            continue
        if suffixes and item.suffix.lower() not in suffixes:
            continue
        entries.append(item.name)
    return entries


def get_house_yaml_path() -> str:
    configs_dir = get_user_configs_dir(required=True)
    yaml_path = str((configs_dir / "house_config.yaml").resolve())
    if not os.path.isfile(yaml_path):
        raise RuntimeError(
            f"House config not found: {yaml_path}. Run setup_user.sh or place house_config.yaml in $SHR_USER_DIR/configs/."
        )
    return yaml_path



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
