import json
import py_trees
from smart_home_pytree.behaviors.action_behaviors.read_script import ReadScript
from smart_home_pytree.behaviors.action_behaviors.wait import Wait
import yaml
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
import py_trees_ros
from shr_msgs.action import PlayVideoRequest
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.trees.charge_robot_tree import ChargeRobotTree

# Blackboard helper behaviors
class CheckDoneBB(py_trees.behaviour.Behaviour):
    ## return True if value is true
    def __init__(self, name, key):
        super().__init__(name)
        if not key.startswith("/"):
            key = "/" + key
        self.key = key

    def update(self):
        bb = py_trees.blackboard.Blackboard()
        value = bb.storage.get(self.key, False)
        return (py_trees.common.Status.SUCCESS
                if value else py_trees.common.Status.FAILURE)

class SetDoneBB(py_trees.behaviour.Behaviour):
    def __init__(self, name, key):
        super().__init__(name)
        self.key = key

    def update(self):
        bb = py_trees.blackboard.Blackboard()
        bb.set(self.key, True)
        return py_trees.common.Status.SUCCESS


class ClearExerciseProgressBB(py_trees.behaviour.Behaviour):
    def __init__(self, name="ClearExerciseProgressBB", key_prefix="exercise_key"):
        super().__init__(name)
        self.key_prefix = key_prefix

    def update(self):
        bb = py_trees.blackboard.Blackboard()
        for key in list(bb.storage.keys()):
            if key.lstrip("/").startswith(self.key_prefix):
                bb.unset(key)
        return py_trees.common.Status.SUCCESS

### TODO: need to add to stop handler stopping the video

# MAIN PROTOCOL TREE
## TODO: make sure that if ExerciseProtocolTree follows the set yaml, set as env variable
## video_full = f"file:///storage/emulated/0/Download/Exercise_Videos/{series['type']}/{ex['video_name']}" a param

class ExerciseProtocolTree(BaseTreeRunner):
    def __init__(self, node_name: str, robot_interface=None, **kwargs):
        """
        Initialize the TwoReminderProtocol.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as location.
        """
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            **kwargs
        )
        
        self.node_name = node_name
        self.key_word = "exercise_key"
        self.bb = py_trees.blackboard.Blackboard() 
        
        self.yaml_ex = "/home/olagh48652/smart_home_pytree_ws/src/smart_home_robot/smart_home_pytree/config/exercise_routine_test.yaml"
        
        
    def load_exercise_yaml(self, path):
        """
        Load and parse an exercise YAML file.

        Returns:
            data (dict): Parsed YAML content.
        """
        with open(path, "r") as f:
            data = yaml.safe_load(f)

        if not isinstance(data, dict):
            raise ValueError("YAML root must be a dictionary.")

        return data

    def create_tree(self):
        data = self.load_exercise_yaml(self.yaml_ex)
        key_prefix = self.key_word  # "exercise_key"

        # Full protocol wrapped by the guard
        protocol = py_trees.composites.Sequence(
            name="ExerciseProtocolCore",
            memory=True
        )

        # INTRO
        intro_flag = f"{key_prefix}_intro_done"
        protocol.add_child(
            py_trees.composites.Selector(
                name="IntroSelector",
                memory=True,
                children=[
                    CheckDoneBB("IntroAlreadyDone?", intro_flag),
                    py_trees.composites.Sequence(name=f"{intro_flag}_Intro",
                        memory=True,
                        children=[
                        ReadScript(data["introduction"], "IntroScript"),
                        SetDoneBB("MarkIntroDone", intro_flag),
                    ])
                ]
            )
        )

        # SERIES
        for series_key, series in data.items():
            if not series_key.startswith("series_"):
                continue

            series_prefix = f"{key_prefix}_{series_key}"

            series_seq = py_trees.composites.Sequence(
                name=f"{series_key}_Sequence",
                memory=True
            )

            # SERIES INTRO
            s_intro_flag = f"{series_prefix}_intro_done"
            series_seq.add_child(
                py_trees.composites.Selector(
                    name=f"{series_key}_IntroSelector",
                    memory=True,
                    children=[
                        CheckDoneBB("SeriesIntroDone?", s_intro_flag),
                        py_trees.composites.Sequence(name="DoIntroSequence",
                        memory=True,
                        children=[
                            ReadScript(series["introduction"], f"{series_key}_Intro"),
                            SetDoneBB(f"{series_key}_MarkIntroDone", s_intro_flag),
                        ])
                    ]
                )
            )

            # EXERCISES
            for ex_key, ex in series.items():
                if ex_key.startswith("ex"):
                    series_seq.add_child(self.build_exercise_block(series_key, ex_key, series, ex))

            protocol.add_child(series_seq)

        # ENDING
        if "ending" in data:
            protocol.add_child(ReadScript(data["ending"], "Ending"))

        # ALWAYS clear BB keys at the end
        protocol.add_child(ClearExerciseProgressBB(key_prefix))

        # Guard condition — stop when /exercise_stop == True
        ## WHEN GAURD IS FALSE, TREE IS EXITED
        def guard_condition():
            stop_exercise = self.robot_interface.state.get("stop_exercise", False)
            return not stop_exercise

        guarded_protocol = py_trees.decorators.EternalGuard(
            name="StopExerciseGuard",
            condition=guard_condition,
            child=protocol
        )

        # STOP HANDLER — runs when guard blocks protocol
        stop_handler = py_trees.composites.Sequence(
            name="StopHandler",
            memory=True,
            children=[
                ReadScript("Stopping exercise now.", "StopNotice"),
                ClearExerciseProgressBB(key_prefix),
                py_trees.behaviours.Success(name="StopSuccess")
            ]
        )

        # Selector: If guard passes → run protocol ELSE If guard blocks → run stop handler
        exercise_block = py_trees.composites.Selector(
            name="ExerciseOrStop",
            memory=True,
            children=[guarded_protocol, stop_handler]
        )

        move_to_person_tree = MoveToPersonLocationTree(node_name=f"{node_name}_move_to_person", robot_interface=self.robot_interface)
        move_to_person = move_to_person_tree.create_tree()
        
        charge_robot_tree = ChargeRobotTree(node_name=f"{node_name}_charge_robot", robot_interface=self.robot_interface)
        charge_robot = charge_robot_tree.create_tree()


        # Full pipeline
        root = py_trees.composites.Sequence(
            name="FullExercisePipeline",
            children=[
                move_to_person,
                exercise_block,
                charge_robot
            ]
        )

        return root

        # return exercise_block

    # BUILD EXERCISE BLOCK
    def build_exercise_block(self, series_key, ex_key, series, ex):
        prefix = f"{self.key_word}_{series_key}_{ex_key}"
        seq = py_trees.composites.Sequence(
            name=f"{series_key}_{ex_key}",
            memory=True
        )

        # Exercise description (resumable)
        desc_flag = f"{prefix}_desc_done"
        seq.add_child(
            py_trees.composites.Selector(
                name=f"{ex_key}_DescSelector",
                memory=True,
                children=[
                    CheckDoneBB("DescDone?", desc_flag),
                    py_trees.composites.Sequence(name="Do_{ex_key}_DescSelector",
                        memory=True,
                        children=[
                        ReadScript(ex["text"], f"{ex_key}_Description"),
                        SetDoneBB(f"{ex_key}_MarkDescDone", desc_flag)
                    ])
                ]
            )
        )

        # REPS LOOP
        rep = ex["rep"]
        wait_time = ex["time_between_rep_in_sec"]

        video_path = f"file:///storage/emulated/0/Download/Exercise_Videos/{series['type']}/{ex['video_name']}"
            
        for i in range(rep):
            rep_flag = f"{prefix}_rep_{i+1}_done"
            
            video_goal = PlayVideoRequest.Goal()
            video_goal.file_name = video_path
            
            play_video_reminder = py_trees_ros.actions.ActionClient(
                name=f"{ex_key}_Video_{i+1}",
                action_type=PlayVideoRequest,
                action_name="play_video",
                action_goal=video_goal,
                wait_for_server_timeout_sec=120.0
            )
            
            rep_selector = py_trees.composites.Selector(
                name=f"{ex_key}_Rep_{i+1}",
                memory=True,
                children=[
                    CheckDoneBB(f"Rep{i+1}Done?", rep_flag),
                    py_trees.composites.Sequence(name="Do_Rep_{ex_key}_Rep_{i+1}",
                        memory=True,
                        children=[
                        play_video_reminder,
                        SetDoneBB(f"{ex_key}_MarkRep{i+1}Done", rep_flag),
                        Wait(wait_time, f"{ex_key}_Wait_{i+1}") if i < rep - 1 else py_trees.behaviours.Success()
                    ])
                ]
            )

            seq.add_child(rep_selector)

        return seq

  
def str2bool(v):
    return str(v).lower() in ('true', '1', 't', 'yes')

import argparse
import rclpy
def main(args=None):    
    parser = argparse.ArgumentParser(
        description="""Two Reminder Protocol Tree 
        
        Handles Playing the Two Reminder Protocol:
        1. Retries up to num_attempts times if needed
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False, help="Run tree continuously (default: False)")

    args, unknown = parser.parse_known_args()
    
    blackboard = py_trees.blackboard.Blackboard()
    
    tree_runner = ExerciseProtocolTree(
        node_name="exercise_protocol_tree",
    )
    tree_runner.setup()
    
    print("run_continuous", args.run_continuous)
    try:
        if args.run_continuous:
            tree_runner.run_continuous()
        else:
            tree_runner.run_until_done()
    finally:
        for key, value in blackboard.storage.items():
            print(f"{key} : {value}")
    
        tree_runner.cleanup()

    rclpy.shutdown()


if __name__ == "__main__":
    main()
    
    
# python3 exercise_protocol.py 
