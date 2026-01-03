"""
General utility functions for the Smart Home Robot project.

This module contains helper functions for parsing configuration values,
converting data types, and handling common string operations used across
multiple robot behaviors and orchestrators.
"""

from typing import Union

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
            logger.warning("Invalid duration string provided: '%s'. Defaulting to 0.", value)
            return 0

    # 3. Handle unexpected types (lists, dicts, None)
    logger.debug("Unexpected type passed to parse_duration: %s", type(value))
    return 0