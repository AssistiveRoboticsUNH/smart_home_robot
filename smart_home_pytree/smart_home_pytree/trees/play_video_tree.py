#!/usr/bin/env python3

import argparse
import os

import py_trees
import py_trees_ros
import rclpy

from shr_msgs.action import PlayVideoRequest
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB

from smart_home_pytree.registry import load_protocols_to_bb, load_locations_to_blackboard
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.utils import str2bool


class PlayVideoTree(BaseTreeRunner):
    """
    This script is responsible for playing a video to the person at their location and then charging the robot.
    The reason for adding this tree and not using the action client directly is becasue after the action client we need to set the variable indicating
    that play audio was successful to true.
    """

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
        Initialize the PlayVideoTree.

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
                f"Validation Error: Protocol '{self.protocol_name}' not found in Blackboard. "
                "Ensure `load_protocols_to_bb` is called before initializing this tree and yaml file includes the protocol name in the protocols."
            )
        
        protocol_info = blackboard.get(protocol_name)
        

        # Check if the specific data key exists within that protocol
        if self.data_key not in protocol_info:
            raise ValueError(
                f"Validation Error: Key '{self.data_key}' not found inside protocol '{self.protocol_name}'. "
                f"Available keys: {list(protocol_info.keys())}"
            )
        self.video_path = protocol_info[data_key]

        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
            **kwargs,
        )

    def create_tree(self) -> py_trees.behaviour.Behaviour:
        """
        Creates the PlayVideoTree tree:
        Sequence:
            MoveToPersonLocation -> Play video

        Args:
            protocol_name (str): which protocol does this play audio belong to (same name as in yaml)
            data_key: which key to use (ex:first_reminder) has to be same as in yaml and part of the protocol

        Returns:
            the root of the tree
        """


        selector = py_trees.composites.Selector(
            name=f"Run {self.data_key} If Needed", memory=True
        )

        condition = CheckProtocolBB(
            name=f"Should Run {self.data_key}?",
            key=f"{self.protocol_name}_done.{self.data_key}_done",
            expected_value=True,
        )
        
        move_to_person_tree = MoveToPersonLocationTree(
            node_name=f"{self.protocol_name}_move_to_person",
            robot_interface=self.robot_interface,
            debug=self.debug,
            executor=self.executor,
        )
        move_to_person = move_to_person_tree.create_tree()

        # Custom behaviors
        video_goal = PlayVideoRequest.Goal()
        video_goal.file_name = self.video_path

        play_video_reminder = py_trees_ros.actions.ActionClient(
            name="Play_video",
            action_type=PlayVideoRequest,
            action_name="play_video",
            action_goal=video_goal,
            wait_for_server_timeout_sec=120.0,
        )

        set_play_video_success = SetProtocolBB(
            name="play_video_set_bb",
            key=f"{self.protocol_name}_done.{self.data_key}_done",
            value=True,
        )

        # Root sequence
        root_sequence = py_trees.composites.Sequence(
            name=f"{self.protocol_name}_play_video", memory=True
        )

        root_sequence.add_children(
            [
                move_to_person,
                play_video_reminder,
                set_play_video_success,
            ]
        )

        selector.add_children([condition, root_sequence])
        return selector


def main(args=None):
    """Main function to run the Play Video Tree."""
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
        default="medicine_am",
        help="name of the protocol that needs to run (ex: medicine_am)",
    )
    parser.add_argument(
        "--data_key",
        type=str,
        default="first_reminder",
        help="name of the key in the protocol that needs to run (ex: medicine_am)",
    )

    args, _ = parser.parse_known_args()
    protocol_name = args.protocol_name
    data_key = args.data_key
    print("protocol_name: ", protocol_name)

    yaml_file_path = os.getenv("house_yaml_path", None)
    load_locations_to_blackboard(yaml_file_path)
    load_protocols_to_bb(yaml_file_path)

    tree_runner = PlayVideoTree(
        node_name="play_video_tree", protocol_name=protocol_name, data_key=data_key
    )
    tree_runner.setup()

    print("run_continuous", args.run_continuous)

    if args.run_continuous:
        tree_runner.run_continuous()
    else:
        tree_runner.run_until_done()


if __name__ == "__main__":
    main()
