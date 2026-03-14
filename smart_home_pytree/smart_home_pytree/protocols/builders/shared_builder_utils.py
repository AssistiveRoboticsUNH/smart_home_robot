#!/usr/bin/env python3

"""
Helper script to avoid repeating code for differnt protocols

"""

import py_trees
from smart_home_pytree.behaviors.check_protocol_bb import CheckProtocolBB
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
from smart_home_pytree.protocols.schema import RUN_TREE_SPECS
from smart_home_pytree.trees.charge_robot_tree import ChargeRobotTree
from smart_home_pytree.trees.move_away_tree import MoveAwayTree
from smart_home_pytree.trees.play_audio_tree import PlayAudioTree
from smart_home_pytree.trees.play_video_tree import PlayVideoTree
from smart_home_pytree.trees.read_script_tree import ReadScriptTree
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.trees.move_to_tree import MoveToLocationTree


def build_registered_tree_from_class(
    tree_cls,
    default_params: dict,
    *,
    tree_params: dict | None,
    node_name: str,
    robot_interface,
    protocol_name: str | None = None,
    step_data_key: str | None = None,
    executor=None,
    debug=False,
):
    """Instantiate a tree class and return its created subtree."""
    _ = protocol_name, step_data_key  # not used by most builders
    ctor_kwargs = {}
    ctor_kwargs.update(default_params or {})
    ctor_kwargs.update(tree_params or {})

    tree_obj = tree_cls(
        node_name=node_name,
        robot_interface=robot_interface,
        debug=debug,
        executor=executor,
        **ctor_kwargs,
    )
    return tree_obj.create_tree()


def build_blackboard_payload_tree(
    tree_cls,
    *,
    node_name: str,
    robot_interface,
    protocol_name: str | None,
    step_data_key: str | None,
    executor=None,
    debug=False,
    tree_params: dict | None = None,
):
    """
    Build trees that read their payload (text/audio/video path) from blackboard.

    The payload itself is synthesized into the blackboard by `registry.py` using the
    current step key (e.g. `step_1`, `yes_2_step_1`).
    """
    extra_tree_kwargs = {}
    if tree_params and "end_sleep" in tree_params:
        # Payload values (text/audio_path/video_path) are still read from blackboard;
        # only execution modifiers like end_sleep are passed through kwargs.
        extra_tree_kwargs["end_sleep"] = tree_params["end_sleep"]
    if tree_params and "execution_location" in tree_params:
        extra_tree_kwargs["execution_location"] = tree_params["execution_location"]
    if not protocol_name or not step_data_key:
        raise ValueError(
            f"[{node_name}] protocol_name and step_data_key are required for media trees"
        )

    tree_obj = tree_cls(
        node_name=node_name,
        robot_interface=robot_interface,
        debug=debug,
        executor=executor,
        protocol_name=protocol_name,
        data_key=step_data_key,
        **extra_tree_kwargs,
    )
    return tree_obj.create_tree()


RUN_TREE_BUILDERS = {
    "play_text": lambda **kwargs: build_blackboard_payload_tree(ReadScriptTree, **kwargs),
    "play_audio": lambda **kwargs: build_blackboard_payload_tree(PlayAudioTree, **kwargs),
    "play_video": lambda **kwargs: build_blackboard_payload_tree(PlayVideoTree, **kwargs),
    "move_to_person_location": lambda **kwargs: build_registered_tree_from_class(
        MoveToPersonLocationTree, {}, **kwargs
    ),
    "move_to_location": lambda **kwargs: build_registered_tree_from_class(
        MoveToLocationTree, {}, **kwargs
    ),
    "charge_robot": lambda **kwargs: build_registered_tree_from_class(
        ChargeRobotTree, {}, **kwargs
    ),
    "move_away": lambda **kwargs: build_registered_tree_from_class(
        MoveAwayTree, {}, **kwargs
    ),
}

def make_run_tree_action(
    tree_name: str,
    tree_params: dict | None,
    node_name: str,
    robot_interface,
    protocol_name: str,
    step_done_key: str,
    step_data_key: str | None = None,
    executor=None,
    debug=False,
):
    """
    Build a generic `run_tree` step from the centralized run_tree registry and
    mark completion in the protocol's `_done` blackboard dictionary when it succeeds.
    """
    params = dict(tree_params or {})
    builder = RUN_TREE_BUILDERS.get(tree_name)
    if builder is None:
        raise ValueError(
            f"Unsupported run_tree tree_name '{tree_name}'. "
            f"Supported: {sorted(RUN_TREE_SPECS.keys())}"
        )
    subtree = builder(
        tree_params=params,
        node_name=node_name,
        robot_interface=robot_interface,
        protocol_name=protocol_name,
        step_data_key=step_data_key,
        executor=executor,
        debug=debug,
    )

    mark_done = SetProtocolBB(
        name=f"{node_name}_mark_done",
        key=f"{protocol_name}_done.{step_done_key}",
        value=True,
    )

    run_once = py_trees.composites.Sequence(name=f"{node_name}_run_tree", memory=True)
    run_once.add_children([subtree, mark_done])

    selector = py_trees.composites.Selector(name=f"{node_name}_if_needed", memory=True)
    selector.add_children(
        [
            CheckProtocolBB(
                name=f"{node_name}_already_done",
                key=f"{protocol_name}_done.{step_done_key}",
                expected_value=True,
            ),
            run_once,
        ]
    )
    return selector
