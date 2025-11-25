#!/usr/bin/env python3

"""
This script is responsible for running the two reminder protocol.

"""

import py_trees

import rclpy

from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
import yaml
import argparse

from smart_home_pytree.trees.play_audio_tree import PlayAudioTree 
from smart_home_pytree.trees.read_script_tree import ReadScriptTree
from smart_home_pytree.registry import load_protocols_to_bb
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB
from shr_msgs.action import PlayVideoRequest
import py_trees_ros

from smart_home_pytree.trees.tree_utils import make_reminder_tree
    
class TwoReminderProtocolTree(BaseTreeRunner):      
    def __init__(self, node_name: str, robot_interface=None, **kwargs):
        """
        Initialize the TwoReminderProtocol.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as location.
        """
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            **kwargs
        )
    
    def create_tree(self) -> py_trees.behaviour.Behaviour:
        """
        Creates the TwoReminderProtocol tree:
        Sequence:
            MoveToPersonLocation -> ReadScript -> ChargeRobot -> Wait -> MoveToPersonLocation -> PlayAudio -> ChargeRobot

        Returns:
            the root of the tree
        """
        
        protocol_name = self.kwargs.get("protocol_name", "")
        bb = py_trees.blackboard.Blackboard()
        
        # Extract the types
        
        print("protocol_name: ", protocol_name)
        protocol_info = bb.get(protocol_name)

        
        type_1 = protocol_info["type_first"]
        type_2 = protocol_info["type_second"]
        wait_time_key = "wait_time_between_reminders"
    
        if protocol_name  == "":
            raise ValueError("protocol_name is empty. Please specify one (e.g., 'medicine_am').")
        
        # Conditional wrappers
        reminder_1 = "first_reminder"
        first_selector = py_trees.composites.Selector("Run First Script if needed", memory=True)
        condition_1 = CheckProtocolBB(
            name="Should Run First Script?",
            key=f"{protocol_name}_done.{reminder_1}_done",
            expected_value=True,
        )
        
        
        first_tree = make_reminder_tree(
            reminder_type=type_1,
            node_name=f"{self.node_name}_first",
            robot_interface=self.robot_interface,
            protocol_name=protocol_name,
            data_key=reminder_1,
            wait_time_key=wait_time_key  # only first needs wait
        )
        
        first_selector.add_children([condition_1, first_tree])
        
        reminder_2 = "second_reminder"
        second_selector = py_trees.composites.Selector("Run Second Script if needed", memory=True)
        condition_2 = CheckProtocolBB(
            name="Should Run Second Script?",
            key=f"{protocol_name}_done.{reminder_2}_done",
            expected_value=True,
        )
        
        second_tree =  make_reminder_tree(
            reminder_type=type_2,
            node_name=f"{self.node_name}_second",
            robot_interface=self.robot_interface,
            protocol_name=protocol_name,
            data_key=reminder_2,
        )
        
        second_selector.add_children([condition_2, second_tree])
        
        # Root sequence
        root_sequence = py_trees.composites.Sequence(name="TwoReminderSequence", memory=True)

        # Add behaviors in order
        root_sequence.add_children([
            first_selector,
            second_selector,
        ])

        return root_sequence
    

def str2bool(v):
    return str(v).lower() in ('true', '1', 't', 'yes')

import os
def main(args=None):    
    parser = argparse.ArgumentParser(
        description="""Two Reminder Protocol Tree 
        
        Handles Playing the Two Reminder Protocol:
        1. Retries up to num_attempts times if needed
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False, help="Run tree continuously (default: False)")
    parser.add_argument("--num_attempts", type=int, default=3, help="retry attempts (default: 3)")
    parser.add_argument("--protocol_name", type=str, default="medicine_am", help="name of the protocol that needs to run (ex: medicine_am)")


    args, unknown = parser.parse_known_args()
    protocol_name = args.protocol_name
    print("protocol_name: ", protocol_name)
    
    # yaml_path = "/home/olagh48652/smart_home_pytree_ws/src/smart_home_pytree/config/house_info.yaml"
    yaml_file_path = os.getenv("house_yaml_path", None) 
    
    blackboard = py_trees.blackboard.Blackboard()
    
    ## test loading and removing from blackboard
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
    ## finish loading and removing from blackboard
    
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
