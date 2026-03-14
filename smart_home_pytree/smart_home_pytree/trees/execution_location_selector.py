#!/usr/bin/env python3

"""Helpers for selecting where a tree action should execute."""

import py_trees

from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree


def normalize_execution_location(value) -> str:
    """Normalize execution location.

    Supported keywords are case-insensitive for `current` and `person`.
    Landmark names are returned as provided (trimmed) for exact matching.
    """
    if value is None:
        return "current"

    text = str(value).strip()
    if not text:
        return "current"

    lowered = text.lower()
    if lowered in {"current", "person"}:
        return lowered

    return text


def build_execution_location_subtree(
    *,
    execution_location,
    node_name: str,
    robot_interface,
    executor=None,
    debug=False,
):
    """Build movement subtree based on execution location selection.

    `current` -> no movement
    `person`  -> move to person tree
    landmark  -> move to location tree for that landmark
    """
    target = normalize_execution_location(execution_location)

    if target == "current":
        return py_trees.behaviours.Success(name=f"{node_name}_at_current_location")

    if target == "person":
        return MoveToPersonLocationTree(
            node_name=f"{node_name}_move_to_person",
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
        ).create_tree()

    return MoveToLocationTree(
        node_name=f"{node_name}_move_to_{target}",
        robot_interface=robot_interface,
        location=target,
        debug=debug,
        executor=executor,
    ).create_tree()
