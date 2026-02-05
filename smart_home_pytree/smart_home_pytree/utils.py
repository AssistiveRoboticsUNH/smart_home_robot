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
            logger.warning(
                "Invalid duration string provided: '%s'. Defaulting to 0.", value
            )
            return 0

    # 3. Handle unexpected types (lists, dicts, None)
    logger.debug("Unexpected type passed to parse_duration: %s", type(value))
    return 0


# --- Setup logging logic ---
import pprint
import py_trees
from rclpy.node import Node
from rclpy.logging import LoggingSeverity

class BlackboardLogger:
    """
    A unified logger that sits on the Blackboard.
    It automatically detects if a ROS node is available.
    """
    def __init__(self, node=None, debug_mode=False):
        """
        Initialize the BlackboardLogger.

        Args:
            node (rclpy.node.Node, optional):
                ROS 2 node used to access the rclpy logger. If None,
                the logger operates in non-ROS mode and prints to stdout.
            debug_mode (bool):
                sets logger level to debug.
        """
        self.node = node
        self.debug_mode = debug_mode
        
        # Configure ROS logger if node exists
        if self.node:
            self._logger = self.node.get_logger()
            if self.debug_mode:
                self._logger.set_level(LoggingSeverity.DEBUG)
            else:
                self._logger.set_level(LoggingSeverity.INFO)
        else:
            self._logger = None
        
        self._logger.info(str("BlackboardLogger initialized"))

    def info(self, message):
        """
        Log an informational message.
        """
        if self._logger:
            self._logger.info(str(message))
        else:
            print(f"[INFO] {message}")

    def warn(self, message):
        """
        Log a warning message.
        """
        if self._logger:
            self._logger.warn(str(message))
        else:
            print(f"[WARN] {message}")

    def error(self, message):
        """
        Log an error message.
        """
        if self._logger:
            self._logger.error(str(message))
        else:
            print(f"[ERROR] {message}")

    def debug(self, message):
        """
        Log a debug message.
        """
        if self._logger:
            self._logger.debug(str(message))
        else:
            # Manual filtering for non-ROS mode
            if self.debug_mode:
                if isinstance(message, (dict, list)):
                    print("[DEBUG] Complex Data:")
                    pprint.pprint(message)
                else:
                    print(f"[DEBUG] {message}")

    def notify_discord(self, message, severity="INFO"):
        """
        Sends a log to a Discord channel via the logging backend.

        The keyword 'weblog=' is required so the Discord log scraper
        can detect and forward the message.
        """        
        key_word = "weblog="
        full_message = f"{key_word} {message}"
        severity = severity.upper()

        if severity == "WARN":
            self.warn(full_message)
        elif severity == "ERROR":
            self.error(full_message)
        elif severity == "DEBUG":
            self.debug(full_message)
        else:
            # Default to INFO
            self.info(full_message)

        
    # TODO: later to show on Tablet
    # def notify_client(self, message, severity="INFO"):

    #     """
    #     Sends a cleaned-up message to the client dashboard.
    #     Logs to ROS backend simultaneously.
    #     """
    #     # 1. Log to developer backend
    #     self.bb.logger.info(f"[CLIENT NOTIFY] {message}")
        
    #     # 2. Publish to /display_rx (your existing topic for the screen)
    #     msg = String()
    #     msg.data = f"{severity}: {message}"
    #     self.pub_client_display.publish(msg)
