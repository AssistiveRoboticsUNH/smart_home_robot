## functions that start with test will be run

import py_trees
import pytest

import smart_home_pytree.protocols.generic_protocol as generic_protocol_mod
from smart_home_pytree.protocols.generic_protocol import GenericProtocolTree


class _DummyAskQuestionTree:
    """Test double that avoids ROS action setup while building confirmation branches."""

    def __init__(self, *args, **kwargs):
        self.args = args
        self.kwargs = kwargs

    def create_tree(self):
        return py_trees.behaviours.Success(name="DummyAskQuestion")


def setup_function(function):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.storage.clear()


def _make_tree(protocol_name: str):
    """Create GenericProtocolTree in embedded mode to avoid standalone ROS init."""
    return GenericProtocolTree(
        node_name=f"test_{protocol_name}",
        protocol_name=protocol_name,
        robot_interface=object(),
        executor=object(),
        debug=False,
    )


def test_generic_protocol_builds_action_and_wait_steps(monkeypatch):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(
        "demo_protocol",
        {
            "steps": [
                {"tree_name": "play_text", "tree_params": {"text": "hello"}, "next_step_after": 5},
                {"tree_name": "play_text", "tree_params": {"text": "bye"}},
            ],
            "step_1": "hello",
            "step_2": "bye",
            "wait_1": 5,
        },
    )
    blackboard.set(
        "demo_protocol_done",
        {"step_1_done": False, "step_2_done": False, "wait_1_done": False},
    )

    calls = []

    def _fake_make_run_tree_action(**kwargs):
        calls.append(kwargs)
        return py_trees.behaviours.Success(name=f"Stub_{kwargs['step_data_key']}")

    monkeypatch.setattr(generic_protocol_mod, "make_run_tree_action", _fake_make_run_tree_action)

    tree_runner = _make_tree("demo_protocol")
    root = tree_runner.create_tree()

    assert isinstance(root, py_trees.composites.Sequence)
    assert len(root.children) == 3  # step_1, wait_1, step_2
    assert root.children[0].name == "Stub_step_1"
    assert root.children[1].name.endswith("_wait_1")
    assert root.children[2].name == "Stub_step_2"

    assert [c["tree_name"] for c in calls] == ["play_text", "play_text"]
    assert [c["step_data_key"] for c in calls] == ["step_1", "step_2"]
    assert [c["step_done_key"] for c in calls] == ["step_1_done", "step_2_done"]


def test_generic_protocol_builds_confirmation_branch_steps(monkeypatch):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(
        "coffee",
        {
            "steps": [
                {
                    "confirmation": {
                        "question": "Would you like coffee help?",
                        "on_yes": [
                            {"tree_name": "play_video", "tree_params": {"path": "vid.mp4"}}
                        ],
                        "on_no": [
                            {"tree_name": "play_text", "tree_params": {"text": "Okay"}}
                        ],
                    }
                }
            ],
            "confirm_1": "Would you like coffee help?",
            "yes_1_step_1": "vid.mp4",
            "no_1_step_1": "Okay",
        },
    )
    blackboard.set(
        "coffee_done",
        {
            "confirm_1_done": False,
            "yes_1_step_1_done": False,
            "no_1_step_1_done": False,
        },
    )

    calls = []

    def _fake_make_run_tree_action(**kwargs):
        calls.append(kwargs)
        return py_trees.behaviours.Success(name=f"Stub_{kwargs['step_data_key']}")

    monkeypatch.setattr(generic_protocol_mod, "make_run_tree_action", _fake_make_run_tree_action)
    monkeypatch.setattr(generic_protocol_mod, "AskQuestionTree", _DummyAskQuestionTree)

    tree_runner = _make_tree("coffee")
    root = tree_runner.create_tree()

    assert isinstance(root, py_trees.composites.Sequence)
    assert len(root.children) == 1  # single confirmation step

    confirm_root = root.children[0]
    assert isinstance(confirm_root, py_trees.composites.Sequence)
    assert len(confirm_root.children) == 2  # ask + selector
    assert confirm_root.children[0].name == "DummyAskQuestion"

    assert [c["tree_name"] for c in calls] == ["play_video", "play_text"]
    assert [c["step_data_key"] for c in calls] == ["yes_1_step_1", "no_1_step_1"]
    assert [c["step_done_key"] for c in calls] == ["yes_1_step_1_done", "no_1_step_1_done"]


def test_generic_protocol_rejects_nested_confirmation(monkeypatch):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(
        "bad_protocol",
        {
            "steps": [
                {
                    "confirmation": {
                        "question": "Top level question?",
                        "on_yes": [
                            {
                                "confirmation": {
                                    "question": "Nested question?",
                                    "on_yes": [
                                        {
                                            "tree_name": "play_text",
                                            "tree_params": {"text": "yes"},
                                        }
                                    ],
                                    "on_no": [
                                        {
                                            "tree_name": "play_text",
                                            "tree_params": {"text": "no"},
                                        }
                                    ],
                                }
                            }
                        ],
                        "on_no": [
                            {"tree_name": "play_text", "tree_params": {"text": "ok"}}
                        ],
                    }
                }
            ],
            "confirm_1": "Top level question?",
        },
    )
    blackboard.set("bad_protocol_done", {"confirm_1_done": False})

    monkeypatch.setattr(generic_protocol_mod, "AskQuestionTree", _DummyAskQuestionTree)
    monkeypatch.setattr(
        generic_protocol_mod,
        "make_run_tree_action",
        lambda **kwargs: py_trees.behaviours.Success(name="Stub"),
    )

    tree_runner = _make_tree("bad_protocol")
    with pytest.raises(ValueError, match="Nested confirmation branches are not supported"):
        tree_runner.create_tree()
