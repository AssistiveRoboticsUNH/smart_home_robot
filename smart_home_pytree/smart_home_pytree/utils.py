"""
General utility functions for the Smart Home Robot project.

This module contains helper functions for parsing configuration values,
converting data types, and handling common string operations used across
multiple robot behaviors and orchestrators.
"""

from typing import Union
from enum import Enum
from datetime import datetime
import logging

class FailureType(Enum):
    """
    Categorizes why a tree failed.
    """
    SAFE = 0          # Intentional stop (YieldWait). Do nothing.
    BLOCKING = 1      # Protocol broken (Missing file). Blacklist this protocol.
    RETRYABLE = 2     # Transient error. Allow retry.
    SYSTEM_HALT = 3   # CRITICAL HAZARD. Stop the entire Orchestrator.
    UNKNOWN = 4        # Failure due to unhandled reasons.

def discord_logging(msg: str, robot_interface):
    """
    A function to be used to send messages to be discord

    Args:
        msg (str): a string with the message to be sent
        robot_interface (_type_): ros2 node
    """
    current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
    full_message = f"[{current_time}] {msg}"
    
    # prepend the magic key
    msg = f"weblog={full_message}"

    # publish to rosout with magic word (which Discord node subscribes to)
    robot_interface.get_logger().info(msg)

    return
    
def str2bool(value: Union[str, int, bool]) -> bool:
    """
    Convert a string, integer, or boolean input into a boolean value.

    This function is case-insensitive and recognizes common truthy strings.

    Args:
        value (Union[str, int, bool]): The input value to convert.
            Accepts 'true', '1', 't', 'yes' (case-insensitive) as True.

    Returns:
        bool: True if the input matches a truthy value, False otherwise.
    """
    return str(value).lower() in ("true", "1", "t", "yes")

def parse_duration(value: Union[str, int, float]) -> int:
    """
    Parse a duration input into an integer representing seconds.

    Handles integers, simple strings, and mathematical string expressions
    often found in configuration files (e.g., multiplication).

    Args:
        value (Union[str, int, float]): The duration value to parse.
            - Integers/Floats: Returned as int.
            - Strings: Can be a number ("50") or an expression ("2*60").

    Returns:
        int: The parsed duration in seconds. Returns 0 if parsing fails.
    """
    # # basic configuration to make sure you see the output in the console
    # logging.basicConfig(level=logging.DEBUG)
    # # Define the logger variable used in your function
    # logger = logging.getLogger(__name__)

    # 1. Handle direct numbers (int or float)
    if isinstance(value, (int, float)):
        return int(value)

    # 2. Handle strings
    if isinstance(value, str):
        # Remove whitespace for cleaner processing
        clean_value = value.strip()

        # Case A: Mathematical expression (currently supports multiplication)
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

        # Case B: Simple number string
        try:
            return int(float(clean_value))  # float cast handles "50.5" -> 50
        except ValueError:
            logger.warning(
                "Invalid duration string provided: '%s'. Defaulting to 0.", value
            )
            return 0

    # 3. Handle unexpected types (lists, dicts, None)
    logger.debug("Unexpected type passed to parse_duration: %s", type(value))
    return 0
