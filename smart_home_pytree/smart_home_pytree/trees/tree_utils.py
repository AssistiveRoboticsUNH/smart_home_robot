
#!/usr/bin/env python3

"""
Helper script to avoid repeating code for differnt protocols

"""


from smart_home_pytree.trees.play_audio_tree import PlayAudioTree
from smart_home_pytree.trees.read_script_tree import ReadScriptTree
from smart_home_pytree.trees.play_video_tree import PlayVideoTree
import py_trees_ros


def make_reminder_tree(reminder_type: str,
                       node_name: str,
                       robot_interface,
                       protocol_name: str,
                       data_key: str):
    """
    Returns a behavior tree subtree for the given reminder type.
    """

    if reminder_type == "text":
        tree = ReadScriptTree(node_name=node_name,
                              robot_interface=robot_interface)
        return tree.create_tree(protocol_name=protocol_name,
                                data_key=data_key)

    elif reminder_type == "audio":
        tree = PlayAudioTree(node_name=node_name,
                             robot_interface=robot_interface)
        return tree.create_tree(protocol_name=protocol_name,
                                data_key=data_key)

    elif reminder_type == "video":
        tree = PlayVideoTree(node_name=node_name,
                             robot_interface=robot_interface)
        return tree.create_tree(protocol_name=protocol_name,
                                data_key=data_key)
    else:
        raise ValueError(
            f"Unknown reminder type: {reminder_type} available types are text, audio, video, question_answer ")
