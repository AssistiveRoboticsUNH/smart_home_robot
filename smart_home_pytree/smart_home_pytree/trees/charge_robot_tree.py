#!/usr/bin/env python3

"""
Charge robot behavior tree.

This tree:
1. Returns success immediately if the robot is already charging
2. Otherwise moves to `home`
3. Attempts docking
4. Verifies charging state
5. Retries the charge sequence up to `num_attempts`

When used from `GenericProtocol` (`tree_name: charge_robot`), `num_attempts` can
be provided via `tree_params.num_attempts`.
"""

import argparse
import operator

import py_trees
import py_trees_ros
import rclpy

from smart_home_pytree.behaviors.action_behaviors import wait
from smart_home_pytree.behaviors.check_robot_state_key import CheckRobotStateKey
from smart_home_pytree.behaviors.logging_behavior import LoggingBehavior
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree
from smart_home_pytree.utils import str2bool

try:
    # ROS 2 Jazzy / Rolling (Standard)
    from nav2_msgs.action import DockRobot
except ImportError:
    # ROS 2 Humble (Requires 'ros-humble-opennav-docking-msgs')
    from opennav_docking_msgs.action import DockRobot


def required_actions_():
    """Return required actions for the ChargeRobotTree. Should match action client names."""
    return {"smart_home_pytree": ["dock_robot", "undock_robot "]}


class ChargeRobotTree(BaseTreeRunner):
    """
    Behavior tree runner for the charging sequence.

    The tree checks if the robot is already charging. If not, it moves the robot
    to home and runs the docking action, retrying the sequence up to
    `num_attempts` times before failing.

    Configurable kwargs:
    - `num_attempts` (int, default 3)
    """

    def __init__(
        self, node_name: str, robot_interface=None, executor=None, debug=False, **kwargs
    ):
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
            **kwargs,
        )
        self.num_attempts = self.kwargs.get("num_attempts", 3)
        self.end_sleep = float(kwargs.get("end_sleep", 0) or 0)

    def create_tree(self) -> py_trees.behaviour.Behaviour:
        target_location = "home"
        num_attempts = self.num_attempts

        charge_robot = py_trees.composites.Selector(name="Tasks", memory=True)

        charging_status = CheckRobotStateKey(
            name="Check_Charging_charge_robot",
            robot_interface=self.robot_interface,
            key="charging",
            expected_value=True,
            comparison=operator.eq,
        )

        check_charging_charge_seq = CheckRobotStateKey(
            name="Check_Charging_ChargeSeq",
            robot_interface=self.robot_interface,
            key="charging",
            expected_value=True,
            comparison=operator.eq,
        )

        move_to_home_tree = MoveToLocationTree(
            node_name="move_to_location_tree",
            robot_interface=self.robot_interface,
            location=target_location,
            debug=self.debug,
            executor=self.executor,
        )
        move_to_home = move_to_home_tree.create_tree()

        docking_goal = DockRobot.Goal()
        dock_robot = py_trees_ros.actions.ActionClient(
            name="Dock_Robot",
            action_type=DockRobot,
            action_name="dock_robot",
            action_goal=docking_goal,
            wait_for_server_timeout_sec=120.0,
        )

        log_message_success = LoggingBehavior(
            name="Log_Success",
            message="Charging sequence completed successfully",
            status=py_trees.common.Status.SUCCESS,
            robot_interface=self.robot_interface,
        )

        log_message_fail = LoggingBehavior(
            name="Log_Fail",
            message="Failed to charge after retry attempts",
            status=py_trees.common.Status.FAILURE,
            robot_interface=self.robot_interface,
        )

        charge_sequence = py_trees.composites.Sequence(name="Charge Sequence", memory=True)
        charge_sequence.add_children(
            [move_to_home, dock_robot, charging_status, log_message_success]
        )

        charge_sequence_with_retry = py_trees.decorators.Retry(
            name="Charge Sequence with Retry",
            child=charge_sequence,
            num_failures=num_attempts,
        )

        charge_robot.add_children(
            [check_charging_charge_seq, charge_sequence_with_retry, log_message_fail]
        )

        if self.end_sleep <= 0:
            return charge_robot

        root = py_trees.composites.Sequence(name="ChargeRobotWithEndSleep", memory=True)
        root.add_children(
            [
                charge_robot,
                wait.Wait(
                    name=f"{self.node_name}_end_sleep",
                    duration_in_sec=self.end_sleep,
                ),
            ]
        )
        return root

    def required_actions(self):
        return required_actions_()

    def required_topics(self):
        return ["/charging"]


def main(args=None):
    """Main function to run the ChargeRobotTree."""
    parser = argparse.ArgumentParser(
        description="""Robot Charging Behavior Tree

        Handles automatic robot charging sequence:
        1. Checks if robot is already charging
        2. If not, moves to home position
        3. Attempts docking
        4. Retries up to num_attempts times if needed
                """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--run_continuous",
        type=str2bool,
        default=False,
        help="Run tree continuously (default: False)",
    )
    parser.add_argument(
        "--num_attempts",
        type=int,
        default=3,
        help="Docking retry attempts (default: 3)",
    )

    args, _ = parser.parse_known_args()

    tree_runner = ChargeRobotTree(
        node_name="charge_robot_tree",
        num_attempts=args.num_attempts,
    )
    tree_runner.setup()

    try:
        if args.run_continuous:
            tree_runner.run_continuous()
        else:
            tree_runner.run_until_done()
    finally:
        tree_runner.cleanup()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
