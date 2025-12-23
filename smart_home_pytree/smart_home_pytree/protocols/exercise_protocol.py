import json
import py_trees
from smart_home_pytree.behaviors.action_behaviors.read_script import ReadScript
from smart_home_pytree.behaviors.action_behaviors.wait import Wait
import yaml
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
import py_trees_ros
from shr_msgs.action import PlayVideoRequest
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
# from smart_home_pytree.trees.charge_robot_tree import ChargeRobotTree
from smart_home_pytree.trees.ask_question_tree import AskQuestionTree
import argparse
import rclpy

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
    def __init__(self, name="ClearExerciseProgressBB", robot_interface=None, start_key ="start_exercise" , key_prefix="exercise_key"):
        super().__init__(name)
        self.key_prefix = key_prefix
        self.robot_interface = robot_interface
        self.start_key = start_key
    def update(self):
        bb = py_trees.blackboard.Blackboard()
        for key in list(bb.storage.keys()):
            if key.lstrip("/").startswith(self.key_prefix):
                bb.unset(key)
        ###
        ## set start_exercise to false
        self.robot_interface.state.update(self.start_key, False)
        
        return py_trees.common.Status.SUCCESS

class CheckRobotStateKey_(py_trees.behaviour.Behaviour):
    def __init__(self, name: str, robot_interface, key: str):
        super().__init__(name)
        self.robot_interface = robot_interface
        self.key = key

    def update(self):  
        value = self.robot_interface.state.get(self.key, None)
        if value is None:
            self.logger.warning(f"{self.name}: '{self.key}' not found in RobotState")
            value = False
        
        print("charging value: ", value)
        if value == True:
            ## fail when true 
            self.logger.debug(f"STOP EXERCISE, FAILURE")
            return py_trees.common.Status.FAILURE 
        else:
            self.logger.debug(f"Not STOP EXERCISE, SUCCESS ")
            return py_trees.common.Status.SUCCESS

# MAIN PROTOCOL TREE
class ExerciseProtocolTree(BaseTreeRunner):
    def __init__(self, node_name: str, robot_interface=None, **kwargs):
        """
        Initialize the ExerciseProtocolTree.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as location.
        """
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            **kwargs
        )
        
        # self.node_name = node_name
        self.key_word = "exercise_key"
        self.bb = py_trees.blackboard.Blackboard()         
        
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
        
        protocol_name = self.kwargs.get("protocol_name", "")
        if protocol_name  == "":
            raise ValueError("protocol_name is empty. Please specify one (e.g., 'medicine_am').")
        
        protocol_info = self.bb.get(protocol_name)
        
        exercise_yaml_file = protocol_info["exercise_yaml_file"]
        self.start_key = protocol_info["start_state_key"]
        self.stop_key = protocol_info["stop_state_key"]
        self.video_dir_path = protocol_info["video_dir_path"]
        
        confirmation_key = "get_confirmation"
        try:
            self.get_confirmation = protocol_info[confirmation_key]
        except:
            self.get_confirmation = ""
            
        print("self.start_key", self.start_key)
        print("self.stop_key", self.stop_key)
        
        print("exercise_yaml_file: ", exercise_yaml_file)
        data = self.load_exercise_yaml(exercise_yaml_file)
        
        key_prefix = self.key_word + protocol_name # "exercise_key"
        print("key_prefix: ", key_prefix)
        
    
        protocol = py_trees.composites.Sequence(
            name="ExerciseProtocolCore",
            memory=True # False
        )

        # INTRO
        intro_flag = f"{key_prefix}_intro_done"
        protocol.add_child(
            py_trees.composites.Selector(
                name="IntroSelector",
                memory=True, # False
                children=[
                    CheckDoneBB("IntroAlreadyDone?", intro_flag),
                    py_trees.composites.Sequence(name=f"{intro_flag}_Intro",
                        memory=True, # False
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
                memory=True, # False
            )

            # SERIES INTRO
            s_intro_flag = f"{series_prefix}_intro_done"
            series_seq.add_child(
                py_trees.composites.Selector(
                    name=f"{series_key}_IntroSelector",
                    memory=True, # False
                    children=[
                        CheckDoneBB("SeriesIntroDone?", s_intro_flag),
                        py_trees.composites.Sequence(name="DoIntroSequence",
                        memory=True, # False
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

        # Clear BB keys at the end
        protocol.add_child(ClearExerciseProgressBB(key_prefix=key_prefix, robot_interface=self.robot_interface, start_key=self.start_key))

        guarded_protocol = py_trees.composites.Parallel(
            name="StopExerciseGuard",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=False     ## skips child with success
            ),
            children=[
                CheckRobotStateKey_("StopCheck", self.robot_interface, self.stop_key),
                protocol
            ]
        )
        
        # STOP HANDLER — runs when guard blocks protocol
        stop_handler = py_trees.composites.Sequence(
            name="StopHandler",
            memory=False,
            children=[
                ReadScript("Stopping exercise now.", "StopNotice"),
                ClearExerciseProgressBB(name="StopHandlerClearExerciseProgressBB", key_prefix=key_prefix, robot_interface=self.robot_interface, start_key=self.start_key),
            ]
        )

        # Selector: If guard passes: run protocol ELSE If guard blocks: run stop handler
        exercise_block = py_trees.composites.Selector(
            name="ExerciseOrStop",
            memory=True, ## False,
            children=[guarded_protocol, stop_handler]
        )

        # return exercise_block

        move_to_person_tree = MoveToPersonLocationTree(node_name=f"{protocol_name}_move_to_person", robot_interface=self.robot_interface)
        move_to_person = move_to_person_tree.create_tree()
        
        # charge_robot_tree = ChargeRobotTree(node_name=f"{protocol_name}_charge_robot", robot_interface=self.robot_interface)
        # charge_robot = charge_robot_tree.create_tree()

        if self.get_confirmation: 
            ask_question_tree = AskQuestionTree(
                node_name="ask_question_tree",
                robot_interface=self.robot_interface,
                protocol_name=protocol_name,
                data_key="get_confirmation"
                )
            ask_question = ask_question_tree.create_tree()
            inverted_ask_question = py_trees.decorators.Inverter(
                name="InvertConfirmation",
                child=ask_question
            ) # if success then should go to fallbakc and play the protocol
           
            
            
        # Full pipeline
        if self.get_confirmation:
            selector = py_trees.composites.Selector(
                name=f"Run Question If Needed",
                memory=True
            )
            
            ## ask first
            # root = py_trees.composites.Sequence(
            #     name="FullExercisePipeline",
            #     memory=True,
            #     children=[
            #         exercise_block,
            #         # charge_robot
            #     ]
            # )
            selector.add_children([inverted_ask_question, exercise_block])
            return selector

        else:
            # Standard execution without permission check
            root = py_trees.composites.Sequence(
                name="FullExercisePipeline",
                memory=True,
                children=[
                    move_to_person,
                    exercise_block,
                    # charge_robot ## not really needed
                ]
            )

            return root

        
    # BUILD EXERCISE BLOCK
    def build_exercise_block(self, series_key, ex_key, series, ex):
        prefix = f"{self.key_word}_{series_key}_{ex_key}"
        seq = py_trees.composites.Sequence(
            name=f"{series_key}_{ex_key}",
            memory=True ## False
        )

        # Exercise description (resumable)
        desc_flag = f"{prefix}_desc_done"
        seq.add_child(
            py_trees.composites.Selector(
                name=f"{ex_key}_DescSelector",
                memory=True, ## False
                children=[
                    CheckDoneBB("DescDone?", desc_flag),
                    py_trees.composites.Sequence(name="Do_{ex_key}_DescSelector",
                        memory=True, ## False
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

        # video_path = f"file:///storage/emulated/0/Download/Exercise_Videos/{series['type']}/{ex['video_name']}"
        video_path = f"{self.video_dir_path}/{series['type']}/{ex['video_name']}"
        
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
                memory=True, ## False
                children=[
                    CheckDoneBB(f"Rep{i+1}Done?", rep_flag),
                    py_trees.composites.Sequence(name="Do_Rep_{ex_key}_Rep_{i+1}",
                        memory=True, ## False
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

import os
from smart_home_pytree.registry import load_protocols_to_bb 
def main(args=None):    
    parser = argparse.ArgumentParser(
        description="""Exercise Protocol Tree 
        
        Handles Playing the Exercise Protocol
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument('--run_continuous', type=str2bool, default=False, help="Run tree continuously (default: False)")
    parser.add_argument("--protocol_name", type=str, default="exercise", help="name of the protocol that needs to run (ex: medicine_am)")
    
    args, unknown = parser.parse_known_args()
    protocol_name = args.protocol_name
    print("protocol_name: ", protocol_name)
    
    yaml_file_path = os.getenv("house_yaml_path", None) 
    blackboard = py_trees.blackboard.Blackboard()
    load_protocols_to_bb(yaml_file_path, debug=False)
    tree_runner = ExerciseProtocolTree(
        node_name="exercise_protocol_tree",
        protocol_name=protocol_name
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
