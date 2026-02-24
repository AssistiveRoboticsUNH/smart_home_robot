#!/usr/bin/env python3

"""
Reusable move-away tree for GenericProtocol (`tree_name: move_away`).

Behavior:
1. Read the robot state command from `position_state_key`
2. Resolve target location (`home` or `away_location`)
3. Move to the target location
4. Optionally sleep (`end_sleep`) using a normal in-tree wait
5. Reset `state_key` to False in robot_interface.state
"""

import py_trees

from smart_home_pytree.behaviors.action_behaviors import wait
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree
from smart_home_pytree.utils import parse_duration


class _UpdateRobotStateKey(py_trees.behaviour.Behaviour):
    """Set `robot_interface.state[key]` to a given value."""

    def __init__(self, name: str, robot_interface, key: str, value):
        super().__init__(name)
        self.robot_interface = robot_interface
        self.key = key
        self.value = value

    def update(self):
        self.robot_interface.state.update(self.key, self.value)
        return py_trees.common.Status.SUCCESS


class MoveAwayTree(BaseTreeRunner):
    """
    Reusable tree for the old MoveAway protocol behavior.

    Supported kwargs / tree_params:
    - `away_location` (required)
    - `position_state_key` (required)
    - `state_key` (required)
    - `end_sleep` (optional duration, default `0`)
    """

    def __init__(
        self,
        node_name: str,
        robot_interface=None,
        executor=None,
        debug=False,
        **kwargs,
    ):
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
            **kwargs,
        )
        self.away_location = self.kwargs["away_location"]
        self.position_state_key = self.kwargs["position_state_key"]
        self.state_key = self.kwargs["state_key"]
        self.home_location = "home"
        self.end_sleep = parse_duration(self.kwargs.get("end_sleep", 0))

    def create_tree(self) -> py_trees.behaviour.Behaviour:
        move_command = self.robot_interface.state.get(self.position_state_key, None)
        if move_command is None:
            raise ValueError(
                f"[{self.node_name}] Robot state missing required key "
                f"'{self.position_state_key}' for move_away"
            )

        if move_command == "home":
            target_location = self.home_location
        elif move_command == "away":
            target_location = self.away_location
        else:
            raise ValueError(
                f"[{self.node_name}] Invalid move_away command '{move_command}' for state key "
                f"'{self.position_state_key}'. Expected 'home' or 'away'."
            )

        if self.debug:
            print(f"[{self.node_name}] move_away command='{move_command}' target='{target_location}'")

        move_to_tree = MoveToLocationTree(
            node_name=f"{self.node_name}_move_to_location",
            robot_interface=self.robot_interface,
            location=target_location,
            debug=self.debug,
            executor=self.executor,
        ).create_tree()

        reset_trigger = _UpdateRobotStateKey(
            name=f"{self.node_name}_reset_state_key",
            robot_interface=self.robot_interface,
            key=self.state_key,
            value=False,
        )

        root = py_trees.composites.Sequence(name=f"{self.node_name}_move_away", memory=True)
        children = [move_to_tree]
        if self.end_sleep > 0:
            children.append(
                wait.Wait(
                    name=f"{self.node_name}_end_sleep",
                    duration_in_sec=self.end_sleep,
                )
            )
        children.append(reset_trigger)
        root.add_children(children)
        return root
