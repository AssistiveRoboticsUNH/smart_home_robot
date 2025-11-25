
#!/usr/bin/env python3

import py_trees
import operator

from datetime import datetime
import rclpy
from rclpy.logging import LoggingSeverity

"""
A logging behavior that prints the current time and a custom message. It takes status as input, if its reporting a failure it should return Failure or else the robot woud get success becasue 
one of the children gave success. (For fallback to return success it requires one child to do that)

"""

class LoggingBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, message: str, status: py_trees.common.Status =py_trees.common.Status.SUCCESS):
        """
        A simple logging behavior that prints the current time and a custom message
        each time it's ticked.
        """
        super().__init__(name)
        self.message = message
        self.status = status
        self.ros_logger = rclpy.logging.get_logger("py_trees_logger")

    def setup(self, **kwargs):
        """
        No delayed setup needed for this simple logger.
        """
        self.logger.debug(f"{self.name} [LoggingBehavior::setup()]")

    def initialise(self):
        """
        Called the first time the behavior is ticked or when leaving a non-RUNNING state.
        """
        self.logger.debug(f"{self.name} [LoggingBehavior::initialise()]")

    def update(self):
        """
        Logs the current time and the provided message, then returns SUCCESS.
        """
        current_time = datetime.now().strftime("%Y-%m-%d %H:%M:%S")
        full_message = f"[{current_time}] {self.message}"
        self.feedback_message = full_message
        self.logger.info(full_message)

        # Alternatively, you can also print if you want console output:
        print(full_message)

        # prepend the magic key
        msg = f"weblog={full_message}"

        # publish to rosout with magic word (which Discord node subscribes to)
        self.ros_logger.log(
            LoggingSeverity.INFO,
            msg
        )
        
        return self.status

    def terminate(self, new_status):
        """
        Called whenever the behavior switches to a non-running state.
        """
        self.logger.debug(f"{self.name} [LoggingBehavior::terminate()][{self.status}→{new_status}]")
         