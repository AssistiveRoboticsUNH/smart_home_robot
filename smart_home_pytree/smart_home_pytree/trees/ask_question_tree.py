#!/usr/bin/env python3

import argparse
import os

import py_trees
import py_trees_ros
import rclpy

from shr_msgs.action import QuestionRequest
from smart_home_pytree.behaviors.action_behaviors.ask_question import (
    AskQuestionBehavior,
)
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
from smart_home_pytree.registry import load_protocols_to_bb
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.utils import str2bool


class AskQuestionTree(BaseTreeRunner):
    """Class creates a tree to get an answer from the user"""

    def __init__(
        self,
        node_name: str,
        protocol_name: str,
        data_key: str,
        robot_interface=None,
        executor=None,
        debug=False,
        **kwargs,
    ):
        """
        Initialize the AskQuestionTree.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as location.
        """

        # Store optional configuration ONLY USED FOR TESTING
        self.protocol_name = protocol_name
        self.data_key = data_key
        
        blackboard = py_trees.blackboard.Blackboard()
    
        if not blackboard.exists(protocol_name):
            raise ValueError(
                f"Validation Error: Protocol '{protocol_name}' not found in Blackboard. "
                "Ensure `load_protocols_to_bb` is called before initializing this tree and yaml file includes the protocol name in the protocols."
            )
        
        protocol_info = blackboard.get(protocol_name)

        # Check if the specific data key exists within that protocol
        if self.data_key not in protocol_info:
            raise ValueError(
                f"Validation Error: Key '{self.data_key}' not found inside protocol '{protocol_name}'. "
                f"Available keys: {list(protocol_info.keys())}"
            )
        self.question_text = protocol_info[data_key]
        
        
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
            **kwargs,
        )
        

    def create_tree(self) -> py_trees.behaviour.Behaviour:
        """
        Creates a behavior tree for asking a confirmation question after moving to a person.

        The tree logic follows:
        1. Navigate to the person's location.
        2. Execute the AskQuestionBehavior (Custom Action Client).
        3. Conditionally update the Blackboard based on 'yes'/'no' answers.

        Returns:
            py_trees.behaviour.Behaviour: The root node (Sequence) of the generated tree.
        """

        # 1. Subtree: Navigation logic
        move_to_person_tree = MoveToPersonLocationTree(
            node_name=f"{self.protocol_name}_move_to_person",
            robot_interface=self.robot_interface,
            debug=self.debug,
            executor=self.executor,
        )
        move_to_person = move_to_person_tree.create_tree()

        # 2. Goal Setup: Define the question parameters
        question_goal = QuestionRequest.Goal()
        question_goal.question = self.question_text

        # 3. Custom Behavior: Logic-heavy Action Client
        # This node handles asking the question and the conditional Blackboard updates
        ask_and_eval = AskQuestionBehavior(
            name="ask_question_and_eval",
            action_goal=question_goal,
            protocol_name=self.protocol_name,
            data_key=self.data_key,
        )

        condition = CheckProtocolBB(
            name=f"Should Run {self.data_key}?",
            key=f"{self.protocol_name}_done.{self.data_key}_done",
            expected_value=True,
        )

        # 4. Root Construction: Sequential execution with memory
        question_sequence = py_trees.composites.Sequence(
            name=f"{self.protocol_name}_ask_sequence", memory=True
        )

        question_sequence.add_children([move_to_person, ask_and_eval])

        selector = py_trees.composites.Selector(
            name=f"Run Question If Needed", memory=True
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
        default="exercise",
        help="name of the protocol that needs to run (ex: medicine_am)",
    )
    parser.add_argument(
        "--data_key",
        type=str,
        default="get_confirmation",
        help="name of the key in the protocol that needs to run (ex: medicine_am)",
    )

    args, _ = parser.parse_known_args()
    protocol_name = args.protocol_name
    data_key = args.data_key
    print("protocol_name: ", protocol_name)

    yaml_file_path = os.getenv("house_yaml_path", None)

    load_protocols_to_bb(yaml_file_path)

    tree_runner = AskQuestionTree(
        node_name="ask_question_tree", protocol_name=protocol_name, data_key=data_key
    )
    tree_runner.setup()

    print("run_continuous", args.run_continuous)

    if args.run_continuous:
        tree_runner.run_continuous()
    else:
        tree_runner.run_until_done()


if __name__ == "__main__":
    main()
