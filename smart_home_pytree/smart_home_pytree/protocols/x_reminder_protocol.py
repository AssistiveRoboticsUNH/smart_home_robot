#!/usr/bin/env python3

"""
This script is responsible for running the two reminder protocol.

"""

import os
import py_trees

import rclpy

from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
import yaml
import argparse

from smart_home_pytree.registry import load_protocols_to_bb
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB
from smart_home_pytree.behaviors.action_behaviors.yield_wait import YieldWait
from smart_home_pytree.trees.tree_utils import make_reminder_tree
from smart_home_pytree.trees.wait_tree import WaitTree


class XReminderProtocolTree(BaseTreeRunner):
    def __init__(self, node_name: str, robot_interface=None, test=False, **kwargs):
        """
        Initialize the XReminderProtocolTree.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as location.
        """
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            **kwargs
        )

        self.test = test

    def create_tree(self) -> py_trees.behaviour.Behaviour:

        protocol_name = self.kwargs.get("protocol_name", "")
        if protocol_name == "":
            raise ValueError("Missing protocol_name in kwargs")

        bb = py_trees.blackboard.Blackboard()
        protocol_info = bb.get(protocol_name)

        n = protocol_info.get("number_of_protocols", 0)
        if n == 0:
            raise ValueError(f"{protocol_name} has no number_of_protocols set")

        root = py_trees.composites.Sequence(
            name=f"{protocol_name}_XReminderSequence",
            memory=True
        )

        # Iterate through reminders in order
        for i in range(1, n + 1):

            reminder_key = f"reminder_{i}"
            type_key = f"type_{i}"
            wait_key = f"wait_{i}"

            # Message and type must exist
            if reminder_key not in protocol_info:
                raise KeyError(f"{protocol_name}: Missing {reminder_key}")

            if type_key not in protocol_info:
                raise KeyError(f"{protocol_name}: Missing {type_key}")

            reminder_type = protocol_info[type_key]
            wait_after = protocol_info.get(wait_key, 0)

            # ------------------------------------------
            # Reminder selector (skip if already done)
            # ------------------------------------------
            selector = py_trees.composites.Selector(
                name=f"Run {reminder_key} If Needed",
                memory=True
            )

            condition = CheckProtocolBB(
                name=f"Should Run {reminder_key}?",
                key=f"{protocol_name}_done.{reminder_key}_done",
                expected_value=True
            )

            reminder_tree = make_reminder_tree(
                reminder_type=reminder_type,
                node_name=f"{self.node_name}_{reminder_key}",
                robot_interface=self.robot_interface,
                protocol_name=protocol_name,
                data_key=reminder_key,
            )

            selector.add_children([condition, reminder_tree])
            root.add_child(selector)

            # ------------------------------------------
            # Wait sequence
            # ------------------------------------------
            if wait_after > 0:
                wait_flag = f"{wait_key}_done"

                wait_selector = py_trees.composites.Selector(
                    name=f"Wait After {reminder_key}",
                    memory=True
                )

                condition_wait = CheckProtocolBB(
                    name=f"Should Run Wait After {reminder_key}?",
                    key=f"{protocol_name}_done.{wait_flag}",
                    expected_value=True
                )

                if self.test:
                    wait_tree_init = WaitTree(
                        node_name=f"{self.node_name}_{wait_key}",
                        robot_interface=self.robot_interface,
                    )

                    wait_tree = wait_tree_init.create_tree(
                        protocol_name=protocol_name,
                        wait_time_key=wait_key,
                    )
                else:
                    wait_tree = YieldWait(
                        name=f"{self.node_name}_{wait_key}",
                        class_name="XReminderProtocol",  # class name without tree
                        protocol_name=protocol_name,
                        wait_time_key=wait_key
                    )

                wait_selector.add_children([condition_wait, wait_tree])
                root.add_child(wait_selector)

        return root


def str2bool(v):
    return str(v).lower() in ('true', '1', 't', 'yes')


def main(args=None):
    parser = argparse.ArgumentParser(
        description="""X Reminder Protocol Tree

        Handles Playing the X Reminder Protocol:
        1. Retries up to num_attempts times if needed
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False,
                        help="Run tree continuously (default: False)")
    parser.add_argument("--num_attempts", type=int, default=3, help="retry attempts (default: 3)")
    parser.add_argument("--protocol_name", type=str, default="medicine_am",
                        help="name of the protocol that needs to run (ex: medicine_am)")

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

    tree_runner = XReminderProtocolTree(
        node_name="x_reminder_protocol_tree",
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
