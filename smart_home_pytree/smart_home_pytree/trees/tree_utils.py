
#!/usr/bin/env python3

"""
Helper script to avoid repeating code for differnt protocols

"""



from smart_home_pytree.trees.play_audio_tree import PlayAudioTree 
from smart_home_pytree.trees.read_script_tree import ReadScriptTree
from shr_msgs.action import PlayVideoRequest
import py_trees_ros

def make_reminder_tree(reminder_type: str,
                node_name: str,
                robot_interface,
                protocol_name: str,
                data_key: str,
                wait_time_key: str | None = None):
    """
    Returns a behavior tree subtree for the given reminder type.
    """

    if reminder_type == "text":
        tree = ReadScriptTree(node_name=node_name,
                                robot_interface=robot_interface)
        return tree.create_tree(protocol_name=protocol_name,
                                data_key=data_key,
                                wait_time_key=wait_time_key)

    elif reminder_type == "audio":
        tree = PlayAudioTree(node_name=node_name,
                                robot_interface=robot_interface)
        return tree.create_tree(protocol_name=protocol_name,
                                data_key=data_key,
                                wait_time_key=wait_time_key)
        
    elif  reminder_type == "video":
        video_goal = PlayVideoRequest.Goal()
        
        return py_trees_ros.actions.ActionClient(
            name="Dock_Robot",
            action_type=PlayVideoRequest,
            action_name="play_video",
            action_goal=video_goal,
            wait_for_server_timeout_sec=120.0
        )
    else:
        raise ValueError(f"Unknown reminder type: {reminder_type} available types are text, audio, video, question_answer ")
    