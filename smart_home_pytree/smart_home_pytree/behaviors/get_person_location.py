#!/usr/bin/env python3

'''
Gets person location from robot_interface and checks if its valid (Not None and part of the locations in the yaml). If its valid it registers it in the blackboard

'''
import py_trees
import operator

from datetime import datetime


class GetPersonLocation(py_trees.behaviour.Behaviour):
    """
    Node that retrieves the current person's location
    (e.g., from a topic or blackboard variable).
    """
    def __init__(self, robot_interface, name="GetPersonLocation"):
        super().__init__(name)
        self.robot_interface = robot_interface
        self.blackboard = py_trees.blackboard.Blackboard()
        self.locations = self.blackboard.get("locations")
        print("locations", self.locations)
        
    def update(self):
        value = self.robot_interface.state.get("person_location", None)
        print("value: ", value)
        print("type value: ", type(value))
        
        ## check if location is valid
        if value is None or value not in self.locations:
            print("location not in available locations")
            return py_trees.common.Status.FAILURE
        
        ## update blackboard
        print(" set in blackboard")
        self.blackboard.set("person_location", value)
        return py_trees.common.Status.SUCCESS