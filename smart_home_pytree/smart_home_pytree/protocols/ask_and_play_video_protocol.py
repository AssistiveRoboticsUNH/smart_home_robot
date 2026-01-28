#!/usr/bin/env python3

import argparse
import os

import py_trees
import py_trees_ros
import rclpy

from shr_msgs.action import QuestionRequest, PlayVideoRequest
from smart_home_pytree.trees.ask_question_tree import AskQuestionTree
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
from smart_home_pytree.registry import load_protocols_to_bb, load_locations_to_blackboard
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.utils import str2bool
from smart_home_pytree.trees.play_video_tree import PlayVideoTree
from smart_home_pytree.trees.ask_question_tree import AskQuestionTree
from smart_home_pytree.behaviors.action_behaviors.read_script_aalp import ReadScript

class AskAndPlayVideoProtocolTree(BaseTreeRunner):
    """
    Tree that:
    1. Asks the user if they want to play a video.
    2. Plays the video if the answer is 'yes'.
    """

    def __init__(self, node_name: str, robot_interface=None, executor=None, debug=False, **kwargs):
        """
        Initialize the AskAndPlayVideoTree.

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
        
        required_keys = [
            "question_text", "video_path"]
        for key in required_keys:
            if key not in self.protocol_info:
                raise KeyError(f"Missing '{key}' in protocol config."
                               f"Available keys: {list(self.protocol_info.keys())}")
                
        self.question_key = "question_text"        
        self.question_text = self.protocol_info[self.question_key]
        self.video_key = "video_path"
        self.video_path = self.protocol_info[self.video_key]


    def create_tree(self) -> py_trees.behaviour.Behaviour:
        ask_tree = AskQuestionTree(
            node_name="ask_question_tree",
            robot_interface=self.robot_interface,
            protocol_name=self.protocol_name,
            data_key=self.question_key,
            debug=self.debug,
            executor=self.executor,
        ).create_tree()

        # Check if the user specifically said YES
        check_yes = py_trees.behaviours.CheckBlackboardVariableValue(
            name="UserSaidYes",
            check=py_trees.common.ComparisonExpression(
                variable="user_wants_video",
                value=True,
                operator=lambda a, b: a == b
            )
        )

        check_no = py_trees.behaviours.CheckBlackboardVariableValue(
            name="UserSaidNo",
            check=py_trees.common.ComparisonExpression(
                variable="user_wants_video",
                value=False,
                operator=lambda a, b: a == b
            )
        )

    
        play_video_tree = PlayVideoTree(
            node_name=self.node_name,
            robot_interface=self.robot_interface,
            debug=self.debug,
            executor=self.executor,
            protocol_name=self.protocol_name,
            data_key=self.video_key
        ).create_tree()
        
        yes_path = py_trees.composites.Sequence(
            name="YesPath",
            memory=True,
            children=[
                check_yes,
                play_video_tree,
            ],
        )

        no_path = py_trees.composites.Sequence(
            name="NoPath",
            memory=True,
            children=[
                check_no,
                ReadScript(
                    text="Okay, I will not play the video.",
                    name="SkipNotice",
                    node=self.robot_interface,
                ),
            ],
        )
        # Create the "Play Path"
        # This only runs if Ask is SUCCESS AND CheckYes is SUCCESS
        decision = py_trees.composites.Selector(
            name="UserDecision",
            memory=True,
            children=[
                yes_path,
                no_path,
            ],
        )
        
        root = py_trees.composites.Sequence(
            name="AskAndAct",
            memory=True,
            children=[
                ask_tree,
                decision,
            ],
        )

        return root
        

def main(args=None):
    parser = argparse.ArgumentParser(description="Ask and Play Video Tree", formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--run_continuous", type=str2bool, default=False)
    parser.add_argument("--protocol_name", type=str, default="coffee")

    args, _ = parser.parse_known_args()

    load_locations_to_blackboard(os.getenv("house_yaml_path", None))
    load_protocols_to_bb(os.getenv("house_yaml_path", None))

    tree_runner = AskAndPlayVideoProtocolTree(
        node_name="ask_and_play_video_tree",
        protocol_name=args.protocol_name
    )
    tree_runner.setup()

    if args.run_continuous:
        tree_runner.run_continuous()
    else:
        tree_runner.run_until_done()


if __name__ == "__main__":
    main()

# ros2 topic pub /coffee std_msgs/msg/Bool "data: true" --once 
# publisher: beginning loop
# publishing #1: std_msgs.msg.Bool(data=True)
