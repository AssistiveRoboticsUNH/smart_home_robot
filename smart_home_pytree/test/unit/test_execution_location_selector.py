import py_trees

import smart_home_pytree.trees.execution_location_selector as selector_mod


def test_execution_location_current_returns_success_node():
    subtree = selector_mod.build_execution_location_subtree(
        execution_location="current",
        node_name="demo",
        robot_interface=object(),
        executor=object(),
        debug=False,
    )
    assert isinstance(subtree, py_trees.behaviour.Behaviour)
    assert subtree.name == "demo_at_current_location"


def test_execution_location_person_uses_move_to_person_tree(monkeypatch):
    called = {}

    class _DummyMoveToPerson:
        def __init__(self, **kwargs):
            called["kwargs"] = kwargs

        def create_tree(self):
            return py_trees.behaviours.Success(name="person_move")

    monkeypatch.setattr(selector_mod, "MoveToPersonLocationTree", _DummyMoveToPerson)

    subtree = selector_mod.build_execution_location_subtree(
        execution_location="PERSON",
        node_name="demo",
        robot_interface=object(),
        executor=object(),
        debug=True,
    )
    assert subtree.name == "person_move"
    assert called["kwargs"]["node_name"] == "demo_move_to_person"


def test_execution_location_landmark_uses_move_to_location_tree(monkeypatch):
    called = {}

    class _DummyMoveToLocation:
        def __init__(self, **kwargs):
            called["kwargs"] = kwargs

        def create_tree(self):
            return py_trees.behaviours.Success(name="landmark_move")

    monkeypatch.setattr(selector_mod, "MoveToLocationTree", _DummyMoveToLocation)

    subtree = selector_mod.build_execution_location_subtree(
        execution_location="living_room",
        node_name="demo",
        robot_interface=object(),
        executor=object(),
        debug=False,
    )
    assert subtree.name == "landmark_move"
    assert called["kwargs"]["location"] == "living_room"
