#!/usr/bin/env python3

"""
This script is responsible for running the two reminder protocol.

"""

import argparse
import os

import py_trees
import py_trees_ros
import rclpy
import yaml

from smart_home_pytree.behaviors.action_behaviors import wait
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB
from smart_home_pytree.registry import load_protocols_to_bb
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree
from smart_home_pytree.utils import str2bool


class UpdateRobotStateKey_(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, robot_interface, key: str):
        super().__init__(name)
        self.robot_interface = robot_interface
        self.key = key

    def update(self):
        value = self.robot_interface.state.get(self.key, None)
        print("[UpdateRobotStateKey_] value: ", value)
        # set value ot false
        self.robot_interface.state.update(self.key, False)
        return py_trees.common.Status.SUCCESS


class MoveAwayProtocolTree(BaseTreeRunner):
    def __init__(
        self, node_name: str, robot_interface=None, executor=None, debug=False, **kwargs
    ):
        """
        Initialize the MoveAwayProtocolTree.

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

        required_keys = ["position_state_key", "state_key", "away_location"]
        
        for key in required_keys:
            if key not in self.protocol_info:
                raise KeyError(
                    f"[{node_name}] CRITICAL: Missing required key '{key}' "
                    f"in protocol '{self.protocol_name}' configuration."
                )
                
        self.position_state_key = self.protocol_info["position_state_key"] ##  indicates the key in the robot_interface state that indicates whether to move to home or away
        self.update_key = self.protocol_info["state_key"] ## key in the robot_interface state that is responsible to trigger the protocol
        self.away_location = self.protocol_info["away_location"] ## indicastes what the away location set to 
        
    def create_tree(self) -> py_trees.behaviour.Behaviour:
        """
        Creates the MoveAwayProtocolTree tree:
        Sequence:
            Get_which_position -> move -> wait

        Returns:
            the root of the tree
        """

        # Use the key from YAML (e.g., "position") to look up the robot's current value
        #    If robot.state["position"] is "home", we go Home.
        #    If robot.state["position"] is "away", we go to the dining room (away_location).
        move_command = self.robot_interface.state.get(self.position_state_key, None)

        if move_command is None:
            raise ValueError(
                f"[{self.node_name}] CRITICAL: Robot state is missing the required key '{self.position_state_key}'. "
                f"Cannot determine target. Current state keys: {list(self.robot_interface.state.keys())}"
            )

   
        if move_command == "home":
            target_location = "home"
            
        elif move_command == "away":
            target_location = self.away_location 
            
        else:
            raise ValueError(
                f"[{self.node_name}] CRITICAL: Invalid value '{move_command}' found for state key '{self.position_state_key}'. "
                f"Expected 'home' or 'away'."
            )

        if self.debug:
             print(f"[{self.node_name}] Command: '{move_command}' -> Target: '{target_location}'")

        # Root sequence
        root_sequence = py_trees.composites.Sequence(
            name="MoveAwaySequence", memory=True
        )

        move_to_position_tree = MoveToLocationTree(
            node_name=f"{self.protocol_name}_move_to_position",
            robot_interface=self.robot_interface,
            location=target_location,
            debug=self.debug,
            executor=self.executor,
        )
        move_to_person = move_to_position_tree.create_tree()
        wait_time = 5
        wait_behavior = wait.Wait(name="wait", duration_in_sec=wait_time)

        # set it to false
        update_key_beh = UpdateRobotStateKey_(
            name="update_beh", robot_interface=self.robot_interface, key=self.update_key
        )

        # Add behaviors in order
        root_sequence.add_children([move_to_person, wait_behavior, update_key_beh])

        return root_sequence


def main(args=None):
    parser = argparse.ArgumentParser(
        description="""Move Away Protocol Tree

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

    load_protocols_to_bb(yaml_file_path)

    tree_runner = MoveAwayProtocolTree(
        node_name="move_away_protocol_tree",
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
