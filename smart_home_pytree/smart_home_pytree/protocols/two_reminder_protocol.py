#!/usr/bin/env python3

"""
This script is responsible for running the two reminder protocol.

"""

import argparse
import os

import py_trees
import rclpy
import yaml

from smart_home_pytree.behaviors.action_behaviors.yield_wait import YieldWait
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB
from smart_home_pytree.registry import load_protocols_to_bb
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.protocols.protocol_utils import make_reminder_tree
from smart_home_pytree.trees.wait_tree import WaitTree
from smart_home_pytree.utils import str2bool


class TwoReminderProtocolTree(BaseTreeRunner):
    def __init__(
        self,
        node_name: str,
        robot_interface=None,
        executor=None,
        debug=False,
        **kwargs,
    ):
        """
        Initialize the TwoReminderProtocol.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as location.
        """
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
            **kwargs,
        )
        
        self.bb = py_trees.blackboard.Blackboard()

        self.protocol_name = self.kwargs.get("protocol_name", "")
        if not self.protocol_name:
            raise ValueError(
                f"[{node_name}] CRITICAL: 'protocol_name' is missing in kwargs."
            )

        if not self.bb.exists(self.protocol_name):
            available_keys = list(self.bb.storage.keys())
            raise KeyError(
                f"[{node_name}] CRITICAL: Protocol '{self.protocol_name}' "
                f"not found in Blackboard. Available keys: {available_keys}"
            )

        self.protocol_info = self.bb.get(self.protocol_name)

        if self.debug:
            print(f"[{node_name}] Protocol Info: {self.protocol_info}")

        required_keys = ["type_first", "type_second", "wait_time_between_reminders"]
        for key in required_keys:
            if key not in self.protocol_info:
                raise KeyError(
                    f"[{node_name}] CRITICAL: Protocol '{self.protocol_name}' "
                    f"is missing required key '{key}'."
                )

        self.type_1 = self.protocol_info["type_first"]
        self.type_2 = self.protocol_info["type_second"]
        self.wait_time_key = "wait_time_between_reminders"
   

    def create_tree(self) -> py_trees.behaviour.Behaviour:
        """
        Creates the TwoReminderProtocol tree:
        Sequence:
            MoveToPersonLocation -> ReadScript -> ChargeRobot -> Wait -> MoveToPersonLocation -> PlayAudio -> ChargeRobot

        Returns:
            the root of the tree
        """

        reminder_1 = "first_reminder"
        first_tree = make_reminder_tree(
            reminder_type=self.type_1,
            node_name=f"{self.node_name}_first",
            robot_interface=self.robot_interface,
            protocol_name=self.protocol_name,
            data_key=reminder_1,
            debug=self.debug,
            executor=self.executor,
        )

        
        wait_tree = YieldWait(
            name=f"{self.node_name}_{self.wait_time_key}",
            class_name="XReminderProtocol",  # class name without tree
            protocol_name=self.protocol_name,
            wait_time_key=self.wait_time_key,
        )

        reminder_2 = "second_reminder"
        second_tree = make_reminder_tree(
            reminder_type=self.type_2,
            node_name=f"{self.node_name}_second",
            robot_interface=self.robot_interface,
            protocol_name=self.protocol_name,
            data_key=reminder_2,
            debug=self.debug,
            executor=self.executor,
        )

        # Root sequence
        root_sequence = py_trees.composites.Sequence(
            name="TwoReminderSequence", memory=True
        )

        # Add behaviors in order
        root_sequence.add_children(
            [
                first_tree,
                wait_tree,
                second_tree,
            ]
        )

        return root_sequence


def main(args=None):
    parser = argparse.ArgumentParser(
        description="""Two Reminder Protocol Tree

        Handles Playing the Two Reminder Protocol:
        1. Retries up to num_attempts times if needed
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
        "--num_attempts", type=int, default=3, help="retry attempts (default: 3)"
    )
    parser.add_argument(
        "--protocol_name",
        type=str,
        default="medicine_am",
        help="name of the protocol that needs to run (ex: medicine_am)",
    )

    args, unknown = parser.parse_known_args()
    protocol_name = args.protocol_name
    print("protocol_name: ", protocol_name)

    # yaml_path = "/home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree/config/house_info.yaml"
    yaml_file_path = os.getenv("house_yaml_path", None)

    blackboard = py_trees.blackboard.Blackboard()

    # test loading and removing from blackboard
    # load_protocol_info_from_bb(yaml_path, protocol_name)

    # print("\n Blackboard updated:")
    # print(f"  first_text: {blackboard.get('first_text')}")
    # print(f"  second_text: {blackboard.get('second_text')}")

    # remove_protocol_info_from_bb(yaml_path, protocol_name)
    # try:
    #     print("\n Blackboard updated:")
    #     print(f"  first_text: {blackboard.get('first_text')}")
    #     print(f"  second_text: {blackboard.get('second_text')}")
    # except:
    #     print("not in blackboard")
    # finish loading and removing from blackboard

    # done in base class
    # load_locations_to_blackboard(yaml_file_path)
    load_protocols_to_bb(yaml_file_path)

    tree_runner = TwoReminderProtocolTree(
        node_name="two_reminder_protocol_tree",
        protocol_name=protocol_name,
    )
    tree_runner.setup()

    print("run_continuous", args.run_continuous)
    try:
        if args.run_continuous:
            tree_runner.run_continuous()
        else:
            tree_runner.run_until_done()
    finally:
        for key, value in blackboard.storage.items():
            print(f"{key} : {value}")

        tree_runner.cleanup()

    rclpy.shutdown()


if __name__ == "__main__":
    main()


# python3 two_reminder_protocol.py --protocol_name medicine_am
