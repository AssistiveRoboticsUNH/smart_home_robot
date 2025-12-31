
#!/usr/bin/env python3

import py_trees
import operator

from datetime import datetime
import rclpy

"""
A logging behavior that prints the current time and a custom message. It takes status as input, if its reporting a failure it should return Failure or else the robot woud get success becasue
one of the children gave success. (For fallback to return success it requires one child to do that)

"""


class LoggingBehavior(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, message: str, robot_interface,
                 status: py_trees.common.Status = py_trees.common.Status.SUCCESS):
        """
        A simple logging behavior that prints the current time and a custom message
        each time it's ticked.
        """
        super().__init__(name)
        self.message = message
        self.status = status
        self.robot_interface = robot_interface

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
        # add robot_interface to get node
        self.robot_interface.get_logger().info(msg)

        return self.status

    def terminate(self, new_status):
        """
        Called whenever the behavior switches to a non-running state.
        """
        self.logger.debug(f"{self.name} [LoggingBehavior::terminate()][{self.status}→{new_status}]")


class DummyRobotInterface:
    """A minimal ROS2 node so the behavior can call get_logger()."""

    def __init__(self):
        rclpy.init()
        self.node = rclpy.create_node("dummy_logger_node")

    def get_logger(self):
        return self.node.get_logger()


def main():
    # Create dummy robot interface (gives you a working ROS node)
    robot_interface = DummyRobotInterface()

    # Create the logging behavior
    log_node = LoggingBehavior(
        name="TestLogger",
        message="Standalone Logging Test",
        robot_interface=robot_interface,
        status=py_trees.common.Status.SUCCESS
    )

    # Minimal tree to execute the behavior
    root = py_trees.composites.Sequence("Root", memory=True)
    root.add_child(log_node)

    tree = py_trees.trees.BehaviourTree(root)

    # Single tick to trigger logging
    print("==== TICKING LOGGING BEHAVIOR ====")
    tree.tick()

    # Clean shutdown
    rclpy.shutdown()


if __name__ == "__main__":
    main()
