#!/usr/bin/env python3

import os
import py_trees

import rclpy


from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner

import argparse
from smart_home_pytree.behaviors.action_behaviors import play_audio, wait
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
from smart_home_pytree.registry import load_protocols_to_bb
from smart_home_pytree.utils import str2bool


class PlayAudioTree(BaseTreeRunner):
    """This Tree is responsible for playing an audio to the person at their location."""

    def __init__(self, node_name: str, robot_interface=None,
                 executor=None, debug=False,
                 protocol_name: str = None,  # for tests
                 data_key: str = None,
                 **kwargs):
        """
        Initialize the PlayAudioTree.

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
        Creates the PlayAudioTree tree:
        Sequence:
            MoveToPersonLocation -> ReadScript -> ChargeRobot

        Args:
            protocol_name (str): which protocol does this play audio belong to (same name as in yaml)
            data_key: which key to use (ex:first_reminder) has to be same as in yaml and part of the protocol
            wait_time: how long should the robot wait after charging (default: 0.0s)

        Returns:
            the root of the tree
        """

        blackboard = py_trees.blackboard.Blackboard()

        # If __init__ already defines values, they take priority.
        protocol_name = protocol_name or self.protocol_name
        protocol_info = blackboard.get(protocol_name)

        data_key = data_key or self.data_key
        audio_path = protocol_info[data_key]

        move_to_person_tree = MoveToPersonLocationTree(
            node_name=f"{protocol_name}_move_to_person",
            robot_interface=self.robot_interface,
            debug=self.debug,
            executor=self.executor)
        move_to_person = move_to_person_tree.create_tree()

        # charge_robot_tree = ChargeRobotTree(node_name=f"{protocol_name}_charge_robot", robot_interface=self.robot_interface)
        # charge_robot = charge_robot_tree.create_tree()

        # Custom behaviors
        play_audio_reminder = play_audio.PlayAudio(
            name=f"{protocol_name}_play_audio", audio_path=audio_path)

        set_play_audio_success = SetProtocolBB(
            name="play_audio_set_bb",
            key=f"{protocol_name}_done.{data_key}_done",
            value=True)

        # Root sequence
        root_sequence = py_trees.composites.Sequence(
            name=f"{protocol_name}_play_audio", memory=True)

        root_sequence.add_children([
            move_to_person,
            play_audio_reminder,
            set_play_audio_success,
            # charge_robot,
        ])

        return root_sequence


def main(args=None):
    parser = argparse.ArgumentParser(
        description="""Play Audio Tree

        Handles Playing the Audio logic where robot and person need to be in the same location before audio is played:
        1. Retries up to num_attempts times if needed
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False,
                        help="Run tree continuously (default: False)")
    parser.add_argument("--num_attempts", type=int, default=3, help="retry attempts (default: 3)")
    parser.add_argument("--protocol_name", type=str, default="medicine_am",
                        help="name of the protocol that needs to run (ex: medicine_am)")
    parser.add_argument("--data_key", type=str, default="reminder_1",
                        help="name of the key in the protocol that needs to run (ex: medicine_am)")

    args, _ = parser.parse_known_args()
    protocol_name = args.protocol_name
    data_key = args.data_key
    print("protocol_name: ", protocol_name)

    yaml_file_path = os.getenv("house_yaml_path", None)

    load_protocols_to_bb(yaml_file_path)

    tree_runner = PlayAudioTree(
        node_name="play_audio_tree",
        protocol_name=protocol_name,
        data_key=data_key,
        debug=False
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
