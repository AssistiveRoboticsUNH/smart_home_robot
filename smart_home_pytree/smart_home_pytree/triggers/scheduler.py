"""Time-based trigger scheduling helpers."""

from __future__ import annotations

from datetime import datetime, timedelta


def get_current_time_string(time_offset=None) -> str:
    now = datetime.now()
    if time_offset:
        now = now + time_offset
    return now.strftime("%H:%M")


def parse_reset_pattern(reset_pattern, bb_logger=None):
    if not reset_pattern:
        return None

    hours = reset_pattern.get("hours", 0)
    minutes = reset_pattern.get("minutes", 0)
    if not isinstance(hours, (int, float)) or not isinstance(minutes, (int, float)):
        if bb_logger is not None:
            bb_logger.debug(f"[reset_pattern] Invalid format: {reset_pattern}")
        return None

    total_seconds = int(hours * 3600 + minutes * 60)
    return total_seconds if total_seconds > 0 else None


def resume_time_from_request(timestamp: float, seconds: int) -> datetime:
    return datetime.fromtimestamp(timestamp) + timedelta(seconds=seconds)
