#!/usr/bin/env python3

"""
This script is responsible for creating the robot tree to move the robot to a location.

The tree receives an string location representing the target pose for the robot. Before initiating movement, it checks whether the robot is currently charging and, if so, commands it to undock first.
"""

import py_trees
import py_trees_ros
import rclpy
import operator
from shr_msgs.action import DockingRequest

from smart_home_pytree.behaviors.check_robot_state_key import CheckRobotStateKey

from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.behaviors.move_to_behavior import MoveToLandmark
import argparse
from smart_home_pytree.robot_interface import get_robot_interface
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
from smart_home_pytree.behaviors.action_behaviors import  wait
from smart_home_pytree.trees.charge_robot_tree import ChargeRobotTree

## launch file is using
def required_actions_():
        return {
            "smart_home_pytree": ["docking"]
        }
        
## reason for this tree is that a robot should never wait unless it is charging
class WaitTree(BaseTreeRunner):      
    def __init__(self, node_name: str,robot_interface=None, protocol_name: str = None, wait_time_key: str = None, **kwargs):
        """
        Initialize the WaitTree.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as x, y, quat.
        """
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            **kwargs
        )
        # self.robot_interface = robot_interface
        self.protocol_name = protocol_name
        self.wait_time_key = wait_time_key
    
    def create_tree(self, protocol_name: str, wait_time_key: str = None) -> py_trees.behaviour.Behaviour:
        """
        Create a tree to handle moving the robot 

        Returns:
            the root of the tree
        """
        
        blackboard = py_trees.blackboard.Blackboard()
        protocol_name = protocol_name or self.protocol_name
        protocol_info = blackboard.get(protocol_name)

        wait_time_key = wait_time_key or self.wait_time_key
        wait_time = protocol_info[wait_time_key]
        
        root = py_trees.composites.Sequence(name="WaitTree", memory=True)
        
        # state = robot_interface.state
        
        charge_robot_tree = ChargeRobotTree(node_name=f"{protocol_name}_charge_robot", robot_interface=self.robot_interface)
        charge_robot = charge_robot_tree.create_tree()
        
        wait_behavior = wait.Wait(name="wait", duration_in_sec=wait_time)
        set_wait_success = SetProtocolBB(name = "wait_set_bb", key=f"{protocol_name}_done.{wait_time_key}_done", value = True)

        root.add_children([charge_robot, wait_behavior, set_wait_success])
            
        return root
    
    ## know what action server need to be there for debugging
    def required_actions(self):
        # Start with base actions
        actions = required_actions_()

        # Add extra actions not to be run by launch file
        extra_actions = {
            "nav2": ["NavigateToPose"]
        }

        # Merge both dictionaries
        for pkg, acts in extra_actions.items():
            if pkg in actions:
                actions[pkg].extend(acts)
            else:
                actions[pkg] = acts

        return actions

    def required_topics(self):
        return [
            "/charging"
        ]
            
def str2bool(v):
    return str(v).lower() in ('true', '1', 't', 'yes')

def main(args=None):    
    parser = argparse.ArgumentParser(
        description="Run move_to_location tree for robot navigation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False, help="Run tree continuously (default: False)")
    parser.add_argument("--protocol_name", type=str, default="medicine_am", help="name of the protocol that needs to run (ex: medicine_am)")
    parser.add_argument("--wait_time_key", type=str, default="wait_time_between_reminders", help="name of the key in the protocol that needs to run (ex: medicine_am)")

    args, unknown = parser.parse_known_args()
    
    tree_runner = WaitTree(
        node_name="wait_tree",
        protocol_name=args.protocol_name,
        wait_time_key=args.wait_time_key,
    )
    tree_runner.setup()

    print("run_continuous", args.run_continuous)
    try:
        if args.run_continuous:
            tree_runner.run_continuous()
        else:
            final_status = tree_runner.run_until_done()
            print("final_status", final_status)
    finally:
        print("clean up")
        tree_runner.cleanup()
    
    rclpy.shutdown() 

if __name__ == "__main__":
    main()
  
# python3 move_to_tree.py --location kitchen --run_actions True --run_simulator False --run_continuous False
