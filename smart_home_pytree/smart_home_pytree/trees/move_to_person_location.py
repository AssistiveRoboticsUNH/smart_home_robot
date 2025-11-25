#!/usr/bin/env python3

"""
This script is repsponsible for getting the person location and moving the robot there.

The tree should move to the person if it reaches the location and the person has moved
"""


import py_trees
import rclpy

from smart_home_pytree.behaviors.robot_person_same_location import RobotPersonSameLocation
from smart_home_pytree.behaviors.get_person_location import GetPersonLocation

from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree

import argparse

def required_actions_():
        return {
            "smart_home_pytree": ["docking", "undocking"]
        }
        
        
class MoveToPersonLocationTree(BaseTreeRunner):      
    def __init__(self, node_name: str, robot_interface=None, **kwargs):
        """
        Initialize the MoveToPersonLocation.

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
        Create the MoveToPersonLocationTree tree that cancels navigation if the person moves rooms.
        """
        
        self.blackboard = py_trees.blackboard.Blackboard()
        root = py_trees.composites.Sequence("MoveToPersonSequence", memory=True)

        num_attempts = self.kwargs.get("num_attempts", 3)

        # Build the behavior tree
        
        ## Get person location
        get_person_room_pre = GetPersonLocation(self.robot_interface)
        
        # Move only if not already in the same room
        move_if_not_same_loc = py_trees.composites.Selector(name="MoveIfNotSameLoc", memory=True)
        robot_same_room_pre = RobotPersonSameLocation(self.robot_interface)
        
        get_person_room = GetPersonLocation(self.robot_interface)
        
        ## get_person_room would give failure if person_location is not set or not valid
        move_to_home_tree = MoveToLocationTree(
            node_name="move_to_location_tree",
            robot_interface=self.robot_interface,
            location_key = "person_location"
        )
        move_to_room = move_to_home_tree.create_tree()
        robot_same_room_post = RobotPersonSameLocation(self.robot_interface)
    
        go_to_person_sequence = py_trees.composites.Sequence("GotoPersonRoutine", memory=True)
        go_to_person_sequence.add_children([get_person_room, move_to_room, robot_same_room_post])
        # get_person_room is used within the retry in case the person moves location
        
        go_to_person_sequence_with_retry = py_trees.decorators.Retry(
            name="Go to Person Sequence with Retry",
            child=go_to_person_sequence,
            num_failures=num_attempts
        )
        
        move_if_not_same_loc.add_children([robot_same_room_pre ,go_to_person_sequence_with_retry])
        root.add_children([get_person_room_pre, move_if_not_same_loc])
        
        return root
    
    def required_actions(self):
        return required_actions_()

    def required_topics(self):
        return [
            "/charging",
            "/person_location",
            "/robot_location"
        ]
        
def str2bool(v):
    return str(v).lower() in ('true', '1', 't', 'yes')


def main(args=None):        
    parser = argparse.ArgumentParser(
        description="""Move to Person Location Behavior Tree 
        
        Handles automatic robot charging sequence:
        1. Gets person location
        2. moves to postion
        3. checks if robot and person are in same location
        4. Retries up to num_attempts times if needed
                """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False, help="Run tree continuously (default: False)")
    parser.add_argument("--num_attempts", type=int, default=5, help="Docking retry attempts (default: 5)")


    args, unknown = parser.parse_known_args()

    tree_runner = MoveToPersonLocationTree(
        node_name="move_to_person_location_tree",
        num_attempts=args.num_attempts
    )
    ## now run in init
    tree_runner.setup()

    print("run_continuous", args.run_continuous)
    try:
        if args.run_continuous:
            tree_runner.run_continuous()
        else:
            tree_runner.run_until_done()
    finally:
        tree_runner.cleanup()

    rclpy.shutdown()

def main_with_stopping(args=None):  
    import threading
      
    parser = argparse.ArgumentParser(
        description="""Move to Person Location Behavior Tree 
        
        Handles automatic robot charging sequence:
        1. Gets person location
        2. moves to postion
        3. checks if robot and person are in same location
        4. Retries up to num_attempts times if needed
                """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument("--num_attempts", type=int, default=5, help="Docking retry attempts (default: 5)")


    args, unknown = parser.parse_known_args()

    tree_runner = MoveToPersonLocationTree(
        node_name="move_to_person_location_tree",
        num_attempts=args.num_attempts
    )
    ## now run in init
    tree_runner.setup()
    
    runner_thread = threading.Thread(target=tree_runner.run_until_done, daemon=True)
    runner_thread.start()

    print("Press 's' + Enter to stop the tree.\n")
    try:
        while not tree_runner._stop_tree:
            user_input = input()
            if user_input.strip().lower() == 's':
                print("Stopping tree...")
                tree_runner.stop_tree()
                break
    finally:
        runner_thread.join(timeout=5)
        print("####### Tree finished with:", tree_runner.final_status)
        tree_runner.cleanup()
        rclpy.shutdown()

    

if __name__ == "__main__":
    main_with_stopping()