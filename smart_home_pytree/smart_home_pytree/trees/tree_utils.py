#!/usr/bin/env python3

"""
Helper script to avoid repeating code for differnt protocols

"""
import py_trees

from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
from smart_home_pytree.utils import FailureType


def validate_protocol_keys(protocol_name, required_keys):
    """
    Checks if the protocol and its required keys exist on the Blackboard.
    
    Returns:
        (bool, py_trees.behaviour.Behaviour): 
        - If valid: Returns (True, None)
        - If invalid: Returns (False, Failure_Sequence)
    """
    bb = py_trees.blackboard.Blackboard()
    
    # 1. Check Protocol Existence
    if not bb.exists(protocol_name):
        return False, _create_failure_sequence(
            f"Protocol '{protocol_name}' missing from Blackboard."
        )

    data = bb.get(protocol_name)
    
    # 2. Check Required Keys
    missing = [key for key in required_keys if key not in data]
    
    if missing:
        msg = f"Protocol '{protocol_name}' missing keys: {missing}. Found: {list(data.keys())}"
        return False, _create_failure_sequence(msg)

    return True, None

def _create_failure_sequence(error_msg):
    """Generates a tree that writes the error and fails immediately."""
    
    # # Write reason to Blackboard (at runtime)
    # set_reason = SetProtocolBB(
    #     name="Set_Fail_Reason",
    #     key="error_reason",
    #     value=error_msg
    # )
    
    # # Write type to Blackboard (BLOCKING = Stop retrying)
    # set_type = SetProtocolBB(
    #     name="Set_Fail_Type",
    #     key="error_type",
    #     value=FailureType.BLOCKING
    # )
    
    # Fail the tree
    fail_node = py_trees.behaviours.Failure(name="Config_Validation_Failure")
    
    # Sequence: Write Reason -> Write Type -> Fail
    seq = py_trees.composites.Sequence(name="Validation_Fail_Sequence", memory=True)
    # seq.add_children([set_reason, set_type, fail_node])
    seq.add_children([fail_node])
    return seq