#!/usr/bin/env python3

import argparse
import os

import py_trees
import rclpy

from smart_home_pytree.behaviors.action_behaviors import play_audio
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB
from smart_home_pytree.registry import load_protocols_to_bb, load_locations_to_blackboard
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.utils import str2bool


class PlayAudioTree(BaseTreeRunner):
    """This Tree is responsible for playing an audio to the person at their location."""

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
        Initialize the PlayAudioTree.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as location.
        """

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
        self.audio_path = protocol_info[data_key]
        
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
            **kwargs,
        )

    def create_tree(self) -> py_trees.behaviour.Behaviour:
        """
        Creates the PlayAudioTree tree:
        Sequence:
            MoveToPersonLocation -> ReadScript -> ChargeRobot

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
        play_audio_reminder = play_audio.PlayAudio(
            name=f"{self.protocol_name}_play_audio", audio_path=self.audio_path
        )

        set_play_audio_success = SetProtocolBB(
            name="play_audio_set_bb",
            key=f"{self.protocol_name}_done.{self.data_key}_done",
            value=True,
        )

        # Root sequence
        root_sequence = py_trees.composites.Sequence(
            name=f"{self.protocol_name}_play_audio", memory=True
        )

        root_sequence.add_children(
            [
                move_to_person,
                play_audio_reminder,
                set_play_audio_success,
            ]
        )

        selector.add_children([condition, root_sequence])
        return selector


def main(args=None):
    parser = argparse.ArgumentParser(
        description="""Play Audio Tree

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
        default="reminder_1",
        help="name of the key in the protocol that needs to run (ex: medicine_am)",
    )

    args, _ = parser.parse_known_args()
    protocol_name = args.protocol_name
    data_key = args.data_key
    print("protocol_name: ", protocol_name)

    yaml_file_path = os.getenv("house_yaml_path", None)
    load_locations_to_blackboard(yaml_file_path)
    load_protocols_to_bb(yaml_file_path)

    tree_runner = PlayAudioTree(
        node_name="play_audio_tree",
        protocol_name=protocol_name,
        data_key=data_key,
        debug=False,
    )
    tree_runner.setup()

    print("run_continuous", args.run_continuous)
    if args.run_continuous:
        tree_runner.run_continuous()
    else:
        tree_runner.run_until_done()

    # try:
    #     if args.run_continuous:
    #         tree_runner.run_continuous()
    #     else:
    #         tree_runner.run_until_done()
    # finally:
    # remove_protocol_info_from_bb(yaml_file_path, protocol_name)
    # tree_runner.cleanup()


if __name__ == "__main__":
    main()
