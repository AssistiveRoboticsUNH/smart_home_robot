#!/usr/bin/env python3

"""
This script is responsible for running the two reminder protocol.

"""

import py_trees

import rclpy

from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
import yaml
import argparse

from smart_home_pytree.trees.move_to_tree import MoveToLocationTree
from smart_home_pytree.registry import load_protocols_to_bb
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB
from smart_home_pytree.behaviors.action_behaviors import wait
import py_trees_ros

from smart_home_pytree.trees.tree_utils import make_reminder_tree

class UpdateRobotStateKey_(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, robot_interface, key: str):
        super().__init__(name)
        self.robot_interface = robot_interface
        self.key = key

    def update(self):  
        value = self.robot_interface.state.get(self.key, None)
        print("[UpdateRobotStateKey_] value: ", value)
        ## set value ot false
        self.robot_interface.state.update(self.key, False)
        return py_trees.common.Status.SUCCESS
        
# move_away_protocol
class MoveAwayProtocolTree(BaseTreeRunner):      
    def __init__(self, node_name: str, robot_interface=None, **kwargs):
        """
        Initialize the MoveAwayProtocolTree.

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
        Creates the MoveAwayProtocolTree tree:
        Sequence:
            Get_which_position -> move -> wait

        Returns:
            the root of the tree
        """
        
        protocol_name = self.kwargs.get("protocol_name", "")
        blackboard = py_trees.blackboard.Blackboard()
        protocol_info = blackboard.get(protocol_name)
       
        if protocol_info is None:
            raise RuntimeError(f"Protocol '{protocol_name}' not found in blackboard")

        # Extract the types
        position_key = "position_state_key"
        
        state_key = protocol_info[position_key]
        update_key = protocol_info["state_key"]
        
        state = self.robot_interface.state
        position_loc = state.get(state_key, None)
        
        print(f"&&& postion loc using key {state_key} is {position_loc}")
        print(f"&&& protocol_info using key {protocol_info['away_location']}")


        if position_loc == "home":
            target_location = "home"
        else:
            target_location = protocol_info["away_location"]
        
        print(f"&&& target_location loc using key {target_location} ")
        
        # Root sequence
        root_sequence = py_trees.composites.Sequence(name="MoveAwaySequence", memory=True)
                
        move_to_position_tree = MoveToLocationTree(
            node_name=f"{protocol_name}_move_to_position",
            robot_interface=self.robot_interface,
            location=target_location  # pass any location here
        )
        move_to_person = move_to_position_tree.create_tree()
        wait_time = 5
        wait_behavior = wait.Wait(name="wait", duration_in_sec=wait_time)

        ## set it to false  
        update_key_beh = UpdateRobotStateKey_(name="update_beh", robot_interface=self.robot_interface, key=update_key)


        # Add behaviors in order
        root_sequence.add_children([
            move_to_person,
            wait_behavior,
            update_key_beh
        ])

        return root_sequence
    

def str2bool(v):
    return str(v).lower() in ('true', '1', 't', 'yes')

import os
def main(args=None):    
    parser = argparse.ArgumentParser(
        description="""Move Away Protocol Tree 
        
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
