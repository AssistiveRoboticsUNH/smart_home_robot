#!/usr/bin/env python3

'''
Behavior that takes the state of the robot, a key in the state and the expected_value and returns success if the key has the expected value
'''
import py_trees
import operator

from datetime import datetime

class CheckProtocolBB(py_trees.behaviour.Behaviour):
    """
    takes  information sepearted with "." and checks what their value is from blackboard
    e.g. (Note: expects 2 for now) takes "medicine_am.first_text" and compares wuith expected value
    """
    def __init__(self, name: str, key: str, expected_value, comparison=operator.eq):
        super().__init__(name)
        self.key = key
        self.expected_value = expected_value
        self.comparison = comparison

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        keys = self.key.split(".")
        
        if len(keys) != 2:
                self.logger.warning(f"Expected key with one '.', got '{self.key}'")
                return py_trees.common.Status.FAILURE
            
        print("CheckProtocolBB keys: ", keys)
        
        protocol_info = blackboard.get(keys[0])
        print("CheckProtocolBB protocol_info: ", protocol_info)
        
        ## missing key results in with a None
        value = protocol_info.get(keys[1], None)
        if value is None:
            self.logger.warning(f"{keys[0]}: '{keys[1]}' not found in BlackBoard")
            return py_trees.common.Status.FAILURE
        
        print("CheckProtocolBB value: ", value)
        # Compare and return
        if self.comparison(value, self.expected_value):
            self.logger.debug(f"{keys[0]}: {keys[1]} == {self.expected_value} → SUCCESS")
            return py_trees.common.Status.SUCCESS
        else:
            self.logger.debug(f"{keys[0]}: {keys[1]} != {self.expected_value} → FAILURE")
            return py_trees.common.Status.FAILURE