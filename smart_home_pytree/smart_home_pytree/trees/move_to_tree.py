#!/usr/bin/env python3

"""
This script is responsible for creating the robot tree to move the robot to a location.

The tree receives an string location representing the target pose for the robot. Before initiating movement, it checks whether the robot is currently charging and, if so, commands it to undock first.
"""

import argparse
import operator

import py_trees
import py_trees_ros
import rclpy

from smart_home_pytree.behaviors.check_robot_state_key import CheckRobotStateKey
from smart_home_pytree.behaviors.move_to_behavior import MoveToLandmark
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.utils import str2bool

# from nav2_msgs.action import UndockRobot
try:
    # ROS 2 Jazzy / Rolling (Standard)
    from nav2_msgs.action import UndockRobot
except ImportError:
    # ROS 2 Humble (Requires 'ros-humble-opennav-docking-msgs')
    from opennav_docking_msgs.action import UndockRobot

# launch file is using


def required_actions_():
    return {"smart_home_pytree": ["undock_robot"]}


# Root (Sequence)
#  ├─ Charging Selector
#  │   ├─ Check not charging
#  │   └─ Undock action
#  └─ MoveToHome action

# todo register positions in blackboard


class MoveToLocationTree(BaseTreeRunner):
    def __init__(
        self, node_name: str, robot_interface=None, executor=None, debug=False, **kwargs
    ):
        """
        Initialize the MoveToLocationTree.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as x, y, quat.
        """
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
            **kwargs,
        )
        
        self.location = self.kwargs.get("location")
        if not self.location:
            raise ValueError(f"[{node_name}] CRITICAL ERROR: 'location' parameter is missing!")
        
        blackboard = py_trees.blackboard.Blackboard()

        if blackboard.exists("location_key"):
            # If it exists on blackboard, use it
            self.location_key = blackboard.get("location_key")
        else:
            # Otherwise use kwargs, defaulting to "person_location"
            self.location_key = self.kwargs.get("location_key", "person_location")


    def create_tree(self) -> py_trees.behaviour.Behaviour:
        """
        Create a tree to handle moving the robot

        Returns:
            the root of the tree
        """
        if self.debug:
            print("MoveTOtree robot_interface ", self.robot_interface)
            print("MoveTOtree self id:", id(self))

   
        root = py_trees.composites.Sequence(name="MoveTo", memory=True)

        # state = robot_interface.state

        undocking_selector = py_trees.composites.Selector(
            name="undocking_selector", memory=True
        )

        not_charging_status = CheckRobotStateKey(
            name="Check_Charging_Moveto",
            robot_interface=self.robot_interface,
            key="charging",
            expected_value=False,
            comparison=operator.eq,
        )

        undocking_goal = UndockRobot.Goal()
        undocking_goal.dock_type = ""
        undocking_goal.max_undocking_time = 30.0
        undock_robot = py_trees_ros.actions.ActionClient(
            name="Undock_Robot",
            action_type=UndockRobot,
            action_name="undock_robot",
            action_goal=undocking_goal,
            wait_for_server_timeout_sec=120.0,
        )

        root.add_child(undocking_selector)
        undocking_selector.add_children([not_charging_status, undock_robot])

        move_to_position = MoveToLandmark(
            self.robot_interface, location=self.location, location_key=self.location_key
        )

        # move_to_position =
        # py_trees.behaviours.Success(name="Move_to_Pose_Success")  # Placeholder
        # for actual move action

        root.add_child(move_to_position)
        return root

    # know what action server need to be there for debugging
    def required_actions(self):
        # Start with base actions
        actions = required_actions_()

        # Add extra actions not to be run by launch file
        extra_actions = {"nav2": ["NavigateToPose"]}

        # Merge both dictionaries
        for pkg, acts in extra_actions.items():
            if pkg in actions:
                actions[pkg].extend(acts)
            else:
                actions[pkg] = acts

        return actions

    def required_topics(self):
        return ["/charging"]


def main(args=None):
    parser = argparse.ArgumentParser(
        description="Run move_to_location tree for robot navigation",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--run_continuous",
        type=str2bool,
        default=False,
        help="Run tree continuously (default: False)",
    )
    parser.add_argument(
        "--location", type=str, default="", help="Target location name (from YAML)"
    )
    parser.add_argument(
        "--location_key",
        type=str,
        default="person_location",
        help="key to use with blackboard",
    )

    args, _ = parser.parse_known_args()

    # Print all active threads
    # import threading
    # print("Active threads before running:")
    # for t in threading.enumerate():
    #     print(f" - {t.name} (alive={t.is_alive()})")
    # print(f"Total threads: {len(threading.enumerate())}")

    tree_runner = MoveToLocationTree(
        node_name="move_to_location_tree",
        location=args.location,
        location_key=args.location_key,
    )
    tree_runner.setup()

    print("run_continuous", args.run_continuous)

    if args.run_continuous:
        tree_runner.run_continuous()
    else:
        final_status = tree_runner.run_until_done()
        print("final_status", final_status)

    # print("Active threads after cleanup:")
    # for t in threading.enumerate():
    #     print(f" - {t.name} (alive={t.is_alive()})")
    # print(f"Total threads: {len(threading.enumerate())}")


if __name__ == "__main__":
    main()

# python3 move_to_tree.py --location kitchen --run_actions True
# --run_simulator False --run_continuous False
