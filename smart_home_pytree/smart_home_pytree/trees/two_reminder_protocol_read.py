#!/usr/bin/env python3

"""
This script is responsible for running the two reminder protocol.

"""


import py_trees

import rclpy

from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
import yaml
import argparse

from smart_home_pytree.trees.read_script_tree import ReadScriptTree
from smart_home_pytree.registry import load_protocols_to_bb
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB

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
   
        if protocol_name  == "":
            raise ValueError("protocol_name is empty. Please specify one (e.g., 'medicine_am').")
        
        # Conditional wrappers
        text_1 = "first_reminder"
        read_script_1_with_check = py_trees.composites.Selector("Run First Script if needed", memory=True)
        condition_1 = CheckProtocolBB(
            name="Should Run First Script?",
            key=f"{protocol_name}_done.{text_1}_done",
            expected_value=True,
        )
        
        wait_time_key = "wait_time_between_reminders"
        read_script_tree_1 = ReadScriptTree(node_name=f"{self.node_name}_read_first_script", robot_interface=self.robot_interface)
        read_script_reminder_1 = read_script_tree_1.create_tree(protocol_name=protocol_name,data_key=text_1, wait_time_key=wait_time_key)
        
        read_script_1_with_check.add_children([condition_1, read_script_reminder_1])
        
        text_2 = "second_reminder"
        read_script_2_with_check = py_trees.composites.Selector("Run Second Script if needed", memory=True)
        condition_2 = CheckProtocolBB(
            name="Should Run Second Script?",
            key=f"{protocol_name}_done.{text_2}_done",
            expected_value=True,
        )
        
        read_script_tree_2 = ReadScriptTree(node_name=f"{self.node_name}_read_second_script", robot_interface=self.robot_interface)
        read_script_reminder_2 = read_script_tree_2.create_tree(protocol_name=protocol_name,data_key=text_2) ## dont want to wait after second script
        read_script_2_with_check.add_children([condition_2, read_script_reminder_2])
        
        # # play_audio = play_audio.PlayAudio(name="play_audio", file="food_reminder.mp3")

        # Root sequence
        root_sequence = py_trees.composites.Sequence(name="TwoReminderSequence", memory=True)

        # Add behaviors in order
        root_sequence.add_children([
            read_script_1_with_check,
            read_script_2_with_check,
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
    parser.add_argument("--protocol_name", type=str, default="", help="name of the protocol that needs to run (ex: medicine_am)")


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
