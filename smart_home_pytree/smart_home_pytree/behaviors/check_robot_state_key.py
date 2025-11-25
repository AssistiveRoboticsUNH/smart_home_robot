#!/usr/bin/env python3

'''
Behavior that takes the state of the robot, a key in the state and the expected_value and returns success if the key has the expected value
'''
import py_trees
import operator

from datetime import datetime

class CheckRobotStateKey(py_trees.behaviour.Behaviour):
    """
    Checks a key in RobotState and returns SUCCESS/FAILURE
    based on a comparison operator and expected value.
    """
    def __init__(self, name: str, robot_interface, key: str, expected_value, comparison=operator.eq):
        super().__init__(name)
        self.robot_interface = robot_interface
        self.key = key
        self.expected_value = expected_value
        self.comparison = comparison

    def update(self):
        # Retrieve safely
        print("CheckRobotStateKey robot_interface ",self.robot_interface)
        print("CheckRobotStateKey self id:", id(self))
        
        value = self.robot_interface.state.get(self.key, None)
        if value is None:
            self.logger.warning(f"{self.name}: '{self.key}' not found in RobotState")
            return py_trees.common.Status.FAILURE
        
        print("cahrging value: ", value)
        # Compare and return
        if self.comparison(value, self.expected_value):
            self.logger.debug(f"{self.name}: {self.key} == {self.expected_value} → SUCCESS")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug(f"{self.name}: {self.key} != {self.expected_value} → FAILURE")
            return py_trees.common.Status.FAILURE

    def __help__(self):
        info = self.describe_requirements()
        print(f"\n[Help: {info['behavior']}]")
        print(f"  • Requires key: '{info['required_key']}'")
        print(f"  • Expected value: {info['expected_value']}")
        print(f"  • Comparison: {info['comparison']}")
        print(f"  • From state: {info['state_source']}\n")