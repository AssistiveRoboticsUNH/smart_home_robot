import py_trees

import smart_home_pytree.trees.ask_question_tree as ask_question_mod
import smart_home_pytree.trees.play_audio_tree as play_audio_mod
import smart_home_pytree.trees.play_video_tree as play_video_mod
import smart_home_pytree.trees.read_script_tree as read_script_mod


def setup_function(function):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.storage.clear()


def _seed_protocol(protocol_name: str, payload: dict):
    blackboard = py_trees.blackboard.Blackboard()
    blackboard.set(protocol_name, payload)


def test_read_script_tree_uses_execution_location(monkeypatch):
    _seed_protocol("p1", {"step_1": "hello"})
    called = {}

    def _fake_move_builder(**kwargs):
        called.update(kwargs)
        return py_trees.behaviours.Success(name="move_stub")

    monkeypatch.setattr(read_script_mod, "build_execution_location_subtree", _fake_move_builder)
    monkeypatch.setattr(
        read_script_mod,
        "ReadScript",
        lambda **kwargs: py_trees.behaviours.Success(name="read_stub"),
    )

    tree = read_script_mod.ReadScriptTree(
        node_name="read_script",
        protocol_name="p1",
        data_key="step_1",
        execution_location="person",
        robot_interface=object(),
        executor=object(),
    ).create_tree()

    assert isinstance(tree, py_trees.behaviour.Behaviour)
    assert called["execution_location"] == "person"


def test_play_audio_tree_uses_execution_location(monkeypatch):
    _seed_protocol("p2", {"step_1": "/tmp/a.mp3"})
    called = {}

    def _fake_move_builder(**kwargs):
        called.update(kwargs)
        return py_trees.behaviours.Success(name="move_stub")

    monkeypatch.setattr(play_audio_mod, "build_execution_location_subtree", _fake_move_builder)
    monkeypatch.setattr(
        play_audio_mod.play_audio,
        "PlayAudio",
        lambda **kwargs: py_trees.behaviours.Success(name="audio_stub"),
    )

    tree = play_audio_mod.PlayAudioTree(
        node_name="play_audio",
        protocol_name="p2",
        data_key="step_1",
        execution_location="living_room",
        robot_interface=object(),
        executor=object(),
    ).create_tree()

    assert isinstance(tree, py_trees.behaviour.Behaviour)
    assert called["execution_location"] == "living_room"


def test_play_video_tree_uses_execution_location(monkeypatch):
    _seed_protocol("p3", {"step_1": "/tmp/v.mp4"})
    called = {}

    def _fake_move_builder(**kwargs):
        called.update(kwargs)
        return py_trees.behaviours.Success(name="move_stub")

    monkeypatch.setattr(play_video_mod, "build_execution_location_subtree", _fake_move_builder)
    monkeypatch.setattr(
        play_video_mod.py_trees_ros.actions,
        "ActionClient",
        lambda **kwargs: py_trees.behaviours.Success(name="video_stub"),
    )

    tree = play_video_mod.PlayVideoTree(
        node_name="play_video",
        protocol_name="p3",
        data_key="step_1",
        execution_location="current",
        robot_interface=object(),
        executor=object(),
    ).create_tree()

    assert isinstance(tree, py_trees.behaviour.Behaviour)
    assert called["execution_location"] == "current"


def test_ask_question_tree_uses_execution_location(monkeypatch):
    _seed_protocol("p4", {"confirm_1": "Continue?"})
    called = {}

    def _fake_move_builder(**kwargs):
        called.update(kwargs)
        return py_trees.behaviours.Success(name="move_stub")

    monkeypatch.setattr(ask_question_mod, "build_execution_location_subtree", _fake_move_builder)
    monkeypatch.setattr(
        ask_question_mod,
        "AskQuestionBehavior",
        lambda **kwargs: py_trees.behaviours.Success(name="ask_stub"),
    )

    tree = ask_question_mod.AskQuestionTree(
        node_name="ask_question",
        protocol_name="p4",
        data_key="confirm_1",
        execution_location="person",
        robot_interface=object(),
        executor=object(),
    ).create_tree()

    assert isinstance(tree, py_trees.behaviour.Behaviour)
    assert called["execution_location"] == "person"
