#!/usr/bin/env python3

'''
Check if robot and person are in the same location. gets the location from robot_interface which reads it from the topics
'''
import py_trees
import operator
import math
from datetime import datetime

class RobotPersonSameLocation(py_trees.behaviour.Behaviour):
    """
    Node that checks if robot and person are in the same location
    """
    def __init__(self, robot_interface, name="RobotPersonSameLocation", distance_threshold=1.0, debug=False):
        super().__init__(name)
        self.robot_interface = robot_interface
        self.debug_enabled = debug
        self.distance_threshold = distance_threshold
    
    def debug(self, msg):
        if self.debug_enabled:
            print(f"[DEBUG - {self.name}] {msg}")

    def update(self):
        state = self.robot_interface.state

        person_location = state.get("person_location", None)
        self.debug(f"Person location = {person_location}")

        if person_location is None:
            self.debug("Person location missing → FAILURE")
            return py_trees.common.Status.FAILURE

        # coordinates from AMCL if available
        robot_xy = state.get("robot_location_xy", None)
        robot_location = state.get("robot_location", None)

        self.debug(f"robot_location_xy = {robot_xy}")
        

        # Blackboard locations table
        bb = py_trees.blackboard.Blackboard()
        locations = bb.get("locations", {})

        if person_location not in locations:
            self.debug(f"Person location '{person_location}' not in locations table → FAILURE")
            return py_trees.common.Status.FAILURE

        target = locations[person_location]
        target_x = target["x"]
        target_y = target["y"]

        self.debug(f"Target coordinates = ({target_x:.3f}, {target_y:.3f})")

        # --- Choose coordinates based on AMCL availability ---
        if robot_xy is not None:
            robot_x, robot_y = robot_xy
            self.debug(f"Using AMCL coordinates = ({robot_x:.3f}, {robot_y:.3f})")
        else:
            self.debug(f"robot_location (name) = {robot_location}")
            if robot_location not in locations:
                self.debug("Fallback robot location name missing from table → FAILURE")
                return py_trees.common.Status.FAILURE

            if person_location == robot_location:
                print(f"[INFO] [{self.name}] Person and robot are in the SAME location: {person_location}")
                return py_trees.common.Status.SUCCESS
            else:
                print(f"[INFO] [{self.name}] Person and robot are in DIFFERENT locations -> "
                  f"Person: {person_location}, Robot: {robot_location}")
                return py_trees.common.Status.FAILURE
            
            
        # Compute distance
        dx = robot_x - target_x
        dy = robot_y - target_y
        distance = math.sqrt(dx*dx + dy*dy)

        self.debug(f"Distance = {distance:.3f} meters")

        # --- Within threshold ---
        if distance <= self.distance_threshold:
            state.update("robot_location", person_location)
            self.debug(f"Robot is within 1 meter → setting robot_location = {person_location}")
            return py_trees.common.Status.SUCCESS

        self.debug("Robot not close enough → FAILURE")
        return py_trees.common.Status.FAILURE
