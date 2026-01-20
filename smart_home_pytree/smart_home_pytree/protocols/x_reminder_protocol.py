import argparse
import os

import py_trees

from smart_home_pytree.behaviors.action_behaviors.yield_wait import YieldWait
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB
from smart_home_pytree.registry import load_protocols_to_bb
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.protocols.protocol_utils import make_reminder_tree
from smart_home_pytree.trees.wait_tree import WaitTree
from smart_home_pytree.utils import parse_duration, str2bool


class XReminderProtocolTree(BaseTreeRunner):
    """
    A Behavior Tree runner that dynamically orchestrates a sequence of multi-step reminders.

    This class constructs a linear sequence of reminder tasks based on a protocol definition
    stored in the global Blackboard. It supports resuming partially completed protocols by
    checking completion flags before executing each step.

    The tree structure generated follows this pattern for each reminder *i* in the protocol:
        1. **Selector (Check if Done):**
           - Checks ``protocol_name_done.reminder_i_done``. If True, skips to the next step.
           - If False, executes the specific reminder behavior tree.
        2. **Selector (Optional Wait):**
           - If a ``wait_i`` duration is defined, checks ``protocol_name_done.wait_i_done``.
           - If not done, executes a wait behavior (either ``YieldWait`` for production
             or ``WaitTree`` for testing).

    Attributes:
        kwargs (dict): Configuration arguments, must contain ``protocol_name`` to look up
            the protocol definition in the Blackboard.

    Raises:
        ValueError: If ``protocol_name`` is missing from kwargs or if the protocol definition
            on the Blackboard is invalid (missing ``number_of_protocols``).
        KeyError: If specific reminder keys (message or type) are missing from the protocol definition.
    """

    def __init__(
        self,
        node_name: str,
        robot_interface=None,
        executor=None,
        debug=False,
        **kwargs,
    ):
        """
        Initialize the XReminderProtocolTree.

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

        self.num_protocols = self.protocol_info.get("number_of_protocols", 0)
        
        if self.num_protocols <= 0:
            raise ValueError(
                f"[{node_name}] CRITICAL: '{self.protocol_name}' has invalid "
                f"number_of_protocols: {self.num_protocols}. Must be > 0."
            )

        if self.debug:
            print(f"[{node_name}] Protocol '{self.protocol_name}' found with {self.num_protocols} steps.")
            

    def create_tree(self) -> py_trees.behaviour.Behaviour:
        protocol_name = self.protocol_name
        protocol_info = self.protocol_info

        root = py_trees.composites.Sequence(
            name=f"{protocol_name}_XReminderSequence", memory=True
        )

        # Iterate through reminders in order
        for i in range(1, self.num_protocols + 1):
            reminder_key = f"reminder_{i}"
            type_key = f"type_{i}"
            wait_key = f"wait_{i}"

            # Message and type must exist
            if reminder_key not in self.protocol_info:
                raise KeyError(
                    f"[{self.node_name}] CRITICAL: Protocol '{self.protocol_name}' "
                    f"expects step {i} but is missing key '{reminder_key}'."
                )

            if type_key not in self.protocol_info:
                raise KeyError(
                    f"[{self.node_name}] CRITICAL: Protocol '{self.protocol_name}' "
                    f"expects step {i} but is missing key '{type_key}'."
                )

            reminder_type = protocol_info[type_key]
            wait_after = protocol_info.get(wait_key, 0)

            reminder_tree = make_reminder_tree(
                reminder_type=reminder_type,
                node_name=f"{self.node_name}_{reminder_key}",
                robot_interface=self.robot_interface,
                protocol_name=protocol_name,
                data_key=reminder_key,
                debug=self.debug,
                executor=self.executor,
            )

            root.add_child(reminder_tree)

            # ------------------------------------------
            # Wait sequence
            # ------------------------------------------
            # 1. Parse the value first
            duration_seconds = parse_duration(wait_after)
            if duration_seconds > 0:

                wait_tree = YieldWait(
                    name=f"{self.node_name}_{wait_key}",
                    class_name="XReminderProtocol",  # class name without tree
                    protocol_name=protocol_name,
                    wait_time_key=wait_key,
                )
                root.add_child(wait_tree)

        return root


def main(args=None):
    """Main function to run the X Reminder Protocol Tree."""
    parser = argparse.ArgumentParser(
        description="""X Reminder Protocol Tree

        Handles Playing the X Reminder Protocol:
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

    args, _ = parser.parse_known_args()
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
    if args.run_continuous:
        tree_runner.run_continuous()
    else:
        tree_runner.run_until_done()
    # try:

    # finally:
    for key, value in blackboard.storage.items():
        print(f"{key} : {value}")

        # tree_runner.cleanup()


if __name__ == "__main__":
    main()


# python3 two_reminder_protocol.py --protocol_name medicine_am
