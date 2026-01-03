#!/usr/bin/env python3

import os
import py_trees

import rclpy


from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner

import argparse
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
from smart_home_pytree.registry import load_protocols_to_bb
from shr_msgs.action import QuestionRequest
import py_trees_ros
from smart_home_pytree.behaviors.action_behaviors.ask_question import AskQuestionBehavior
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB
from smart_home_pytree.utils import str2bool


class AskQuestionTree(BaseTreeRunner):
    """Class creates a tree get an answer from the user"""

    def __init__(self, node_name: str, robot_interface=None,
                 executor=None, debug=False,
                 protocol_name: str = None,
                 data_key: str = None,
                 **kwargs):
        """
        Initialize the AskQuestionTree.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as location.
        """
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
            **kwargs
        )

        # Store optional configuration ONLY USED FOR TESTING
        self.protocol_name = protocol_name
        self.data_key = data_key

    def create_tree(self, protocol_name: str = None,
                    data_key: str = None) -> py_trees.behaviour.Behaviour:
        """
        Creates a behavior tree for asking a confirmation question after moving to a person.

        The tree logic follows:
        1. Navigate to the person's location.
        2. Execute the AskQuestionBehavior (Custom Action Client).
        3. Conditionally update the Blackboard based on 'yes'/'no' answers.

        Args:
            protocol_name: Name of the protocol (defaults to self.protocol_name if None).
            data_key: Key for the specific data entry (defaults to self.data_key if None).

        Returns:
            py_trees.behaviour.Behaviour: The root node (Sequence) of the generated tree.
        """

        blackboard = py_trees.blackboard.Blackboard()

        # Configuration Setup
        protocol_name = protocol_name or self.protocol_name
        protocol_info = blackboard.get(protocol_name)
        data_key = data_key or self.data_key

        # Safety Check
        if not protocol_name or not data_key:
            raise ValueError("Create Tree failed: protocol_name or data_key is missing")

        question_text = protocol_info[data_key]

        # 1. Subtree: Navigation logic
        move_to_person_tree = MoveToPersonLocationTree(
            node_name=f"{protocol_name}_move_to_person",
            robot_interface=self.robot_interface,
            debug=self.debug,
            executor=self.executor
        )
        move_to_person = move_to_person_tree.create_tree()

        # 2. Goal Setup: Define the question parameters
        question_goal = QuestionRequest.Goal()
        question_goal.question = question_text

        # 3. Custom Behavior: Logic-heavy Action Client
        # This node handles asking the question and the conditional Blackboard updates
        ask_and_eval = AskQuestionBehavior(
            name="ask_question_and_eval",
            action_goal=question_goal,
            protocol_name=protocol_name,
            data_key=data_key
        )

        condition = CheckProtocolBB(
            name=f"Should Run {data_key}?",
            key=f"{protocol_name}_done.{data_key}_done",
            expected_value=True
        )

        # 4. Root Construction: Sequential execution with memory
        question_sequence = py_trees.composites.Sequence(
            name=f"{protocol_name}_ask_sequence",
            memory=True
        )

        question_sequence.add_children([
            move_to_person,
            ask_and_eval
        ])

        selector = py_trees.composites.Selector(
            name=f"Run Question If Needed",
            memory=True
        )

        selector.add_children([condition, question_sequence])
        return selector


def main(args=None):
    parser = argparse.ArgumentParser(
        description="""Play Video Tree

        Handles Playing the Audio logic where robot and person need to be in the same location before audio is played:
        1. Retries up to num_attempts times if needed
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False,
                        help="Run tree continuously (default: False)")
    parser.add_argument("--num_attempts", type=int, default=3, help="retry attempts (default: 3)")
    parser.add_argument("--protocol_name", type=str, default="exercise",
                        help="name of the protocol that needs to run (ex: medicine_am)")
    parser.add_argument("--data_key", type=str, default="get_confirmation",
                        help="name of the key in the protocol that needs to run (ex: medicine_am)")

    args, _ = parser.parse_known_args()
    protocol_name = args.protocol_name
    data_key = args.data_key
    print("protocol_name: ", protocol_name)

    yaml_file_path = os.getenv("house_yaml_path", None)

    load_protocols_to_bb(yaml_file_path)

    tree_runner = AskQuestionTree(
        node_name="ask_question_tree",
        protocol_name=protocol_name,
        data_key=data_key
    )
    tree_runner.setup()

    print("run_continuous", args.run_continuous)

    if args.run_continuous:
        tree_runner.run_continuous()
    else:
        tree_runner.run_until_done()


if __name__ == "__main__":
    main()
