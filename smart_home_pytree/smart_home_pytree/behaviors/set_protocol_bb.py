#!/usr/bin/env python3

'''
Behavior that takes the state of the robot, a key in the state and the expected_value and returns success if the key has the expected value
'''
import py_trees
import operator


class SetProtocolBB(py_trees.behaviour.Behaviour):
    """
    Behavior that sets a nested variable on the blackboard.

    Example:
        key = "medicine_am.first_text_done"
        value = True
        -> blackboard entry: [medicine_am] {first_text_done = True}
    """
    def __init__(self, name: str, key: str, value):
        super().__init__(name)
        self.key = key
        self.value = value
    

    def update(self):
        blackboard = py_trees.blackboard.Blackboard()
        
        try:
            keys = self.key.split(".")
            if len(keys) != 2:
                self.logger.warning(f"Expected key with one '.', got '{self.key}'")
                return py_trees.common.Status.FAILURE

            protocol_name, field_name = keys[0], keys[1]
          
            # Get current protocol dict if it exists, else create one
            protocol_data = blackboard.get(protocol_name)
            if protocol_data is None:
                protocol_data = {}

            # Update only the specific field
            protocol_data[field_name] = self.value

            # Write it back to the blackboard
            blackboard.set(protocol_name, protocol_data)

            self.logger.info(f"Set Blackboard variable: {protocol_name}.{field_name} = {self.value}")
            return py_trees.common.Status.SUCCESS

        except Exception as e:
            self.logger.warning(f"Failed to set {self.key} on Blackboard: {e}")
            return py_trees.common.Status.FAILURE

    
