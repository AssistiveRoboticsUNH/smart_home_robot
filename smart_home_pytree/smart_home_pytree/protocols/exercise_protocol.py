import argparse
import os

import py_trees
import py_trees_ros
import rclpy
import yaml

from shr_msgs.action import PlayVideoRequest
# from smart_home_pytree.behaviors.action_behaviors.read_script_aalp import ReadScript ## getting stick with aalp
from smart_home_pytree.behaviors.action_behaviors.read_script import ReadScript
from smart_home_pytree.behaviors.action_behaviors.wait import Wait
from smart_home_pytree.registry import load_protocols_to_bb, load_locations_to_blackboard
from smart_home_pytree.trees.ask_question_tree import AskQuestionTree
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.utils import str2bool

# Blackboard helper behaviors
class CheckDoneBB(py_trees.behaviour.Behaviour):
    # return True if value is true
    def __init__(self, name, key):
        super().__init__(name)
        if not key.startswith("/"):
            key = "/" + key
        self.key = key

    def update(self):
        bb = py_trees.blackboard.Blackboard()
        value = bb.storage.get(self.key, False)
        return (
            py_trees.common.Status.SUCCESS if value else py_trees.common.Status.FAILURE
        )

class SetDoneBB(py_trees.behaviour.Behaviour):
    def __init__(self, name, key):
        super().__init__(name)
        self.key = key

    def update(self):
        bb = py_trees.blackboard.Blackboard()
        bb.set(self.key, True)
        return py_trees.common.Status.SUCCESS


class ClearExerciseProgressBB(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        name="ClearExerciseProgressBB",
        robot_interface=None,
        start_key="start_exercise",
        key_prefix="exercise_key",
    ):
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
        # set start_exercise to false
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
            # self.logger.warning(f"{self.name}: '{self.key}' not found in RobotState")
            value = False

        if value is True:
            # fail when true
            self.logger.debug(f"STOP EXERCISE, FAILURE")
            return py_trees.common.Status.FAILURE
        else:
            self.logger.debug(f"Not STOP EXERCISE, SUCCESS ")
            return py_trees.common.Status.SUCCESS

# MAIN PROTOCOL TREE
class ExerciseProtocolTree(BaseTreeRunner):
    def __init__(
        self, node_name: str, robot_interface=None, executor=None, debug=False, **kwargs
    ):
        """
        Initialize the ExerciseProtocolTree.

        Args:
            node_name (str): name of the ROS node.
            **kwargs: extra arguments such as location.
        """
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
            **kwargs,
        )

        # self.node_name = node_name
        self.key_word = "exercise_key"
        self.bb = py_trees.blackboard.Blackboard()
        
        self.protocol_name = self.kwargs.get("protocol_name", "")
        if not self.protocol_name:
            raise ValueError(
                f"[{node_name}] CRITICAL: 'protocol_name' is missing in kwargs."
            )
            
        if not self.bb.exists(self.protocol_name):
            raise KeyError(
                    f"[{node_name}] CRITICAL: Protocol '{self.protocol_name}' "
                    f"was not found in the Blackboard. keys available: {list(self.bb.storage.keys())}"
                )
        
        self.protocol_info = self.bb.get(self.protocol_name)
     
        # List of keys that MUST exist in the protocol info
        required_keys = [
            "exercise_yaml_file",
            "start_state_key",
            "stop_state_key",
            "video_dir_path",
        ]

        for key in required_keys:
            if key not in self.protocol_info:
                raise KeyError(
                    f"[{node_name}] CRITICAL: Missing required key '{key}' in protocol '{self.protocol_name}' configuration."
                )

        exercise_yaml_path = self.protocol_info["exercise_yaml_file"]
        self.start_key = self.protocol_info["start_state_key"]
        self.stop_key = self.protocol_info["stop_state_key"]
        self.video_dir_path = self.protocol_info["video_dir_path"]
        
        if not os.path.isfile(exercise_yaml_path):
            raise FileNotFoundError(
                f"[{node_name}] CRITICAL: Exercise YAML file not found at: {exercise_yaml_path}"
            )
        
        # Pre-load data to ensure YAML is valid right now
        self.exercise_data = self.load_exercise_yaml(exercise_yaml_path)

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
        protocol_name = self.protocol_name
        protocol_info = self.bb.get(protocol_name)

        confirmation_key = "get_confirmation"
        try:
            self.get_confirmation = protocol_info[confirmation_key]
        except BaseException:
            self.get_confirmation = ""

        print("self.start_key", self.start_key)
        print("self.stop_key", self.stop_key)

        data = self.exercise_data 

        key_prefix = self.key_word + protocol_name  # "exercise_key"
        print("key_prefix: ", key_prefix)

        protocol = py_trees.composites.Sequence(
            name="ExerciseProtocolCore",
            memory=True,  # False
        )

        # INTRO
        intro_flag = f"{key_prefix}_intro_done"
        protocol.add_child(
            py_trees.composites.Selector(
                name="IntroSelector",
                memory=True,  # False
                children=[
                    CheckDoneBB("IntroAlreadyDone?", intro_flag),
                    py_trees.composites.Sequence(
                        name=f"{intro_flag}_Intro",
                        memory=True,  # False
                        children=[
                            ReadScript(text=data["introduction"], name="IntroScript", node=self.robot_interface),
                            SetDoneBB("MarkIntroDone", intro_flag),
                        ],
                    ),
                ],
            )
        )

        # SERIES
        for series_key, series in data.items():
            if not series_key.startswith("series_"):
                continue

            series_prefix = f"{key_prefix}_{series_key}"

            series_seq = py_trees.composites.Sequence(
                name=f"{series_key}_Sequence",
                memory=True,  # False
            )

            # SERIES INTRO
            s_intro_flag = f"{series_prefix}_intro_done"
            series_seq.add_child(
                py_trees.composites.Selector(
                    name=f"{series_key}_IntroSelector",
                    memory=True,  # False
                    children=[
                        CheckDoneBB("SeriesIntroDone?", s_intro_flag),
                        py_trees.composites.Sequence(
                            name="DoIntroSequence",
                            memory=True,  # False
                            children=[
                                ReadScript(
                                    text=series["introduction"], name=f"{series_key}_Intro", node=self.robot_interface
                                ),
                                SetDoneBB(f"{series_key}_MarkIntroDone", s_intro_flag),
                            ],
                        ),
                    ],
                )
            )

            # EXERCISES
            for ex_key, ex in series.items():
                if ex_key.startswith("ex"):
                    series_seq.add_child(
                        self.build_exercise_block(series_key, ex_key, series, ex)
                    )

            protocol.add_child(series_seq)

        # ENDING
        if "ending" in data:
            protocol.add_child(ReadScript(text=data["ending"], name="Ending", node=self.robot_interface))

        # Clear BB keys at the end
        protocol.add_child(
            ClearExerciseProgressBB(
                key_prefix=key_prefix,
                robot_interface=self.robot_interface,
                start_key=self.start_key,
            )
        )

        guarded_protocol = py_trees.composites.Parallel(
            name="StopExerciseGuard",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(
                synchronise=False  # skips child with success
            ),
            children=[
                CheckRobotStateKey_("StopCheck", self.robot_interface, self.stop_key),
                protocol,
            ],
        )

        # STOP HANDLER — runs when guard blocks protocol
        stop_handler = py_trees.composites.Sequence(
            name="StopHandler",
            memory=False,
            children=[
                ReadScript(text="Stopping exercise now.", name="StopNotice", node=self.robot_interface),
                ClearExerciseProgressBB(
                    name="StopHandlerClearExerciseProgressBB",
                    key_prefix=key_prefix,
                    robot_interface=self.robot_interface,
                    start_key=self.start_key,
                ),
            ],
        )

        # Selector: If guard passes: run protocol ELSE If guard blocks: run stop handler
        exercise_block = py_trees.composites.Selector(
            name="ExerciseOrStop",
            memory=True,  # False,
            children=[guarded_protocol, stop_handler],
        )

        # return exercise_block
        move_to_person_tree = MoveToPersonLocationTree(
            node_name=f"{protocol_name}_move_to_person",
            robot_interface=self.robot_interface,
            debug=self.debug,
            executor=self.executor,
        )
        move_to_person = move_to_person_tree.create_tree()

        ## todo: gaurd for confirmation
        if self.get_confirmation:
            ask_question_tree = AskQuestionTree(
                node_name="ask_question_tree",
                robot_interface=self.robot_interface,
                protocol_name=protocol_name,
                data_key="get_confirmation",
                debug=self.debug,
                executor=self.executor,
            ).create_tree()
            
            # Check if the user specifically said YES
            check_yes = py_trees.behaviours.CheckBlackboardVariableValue(
                name="Did User Say Yes?",
                check=py_trees.common.ComparisonExpression(
                    variable="user_wants_video",
                    value=True,
                    operator=lambda a, b: a == b
                )
            )
            
            # Create the "Play Path"
            # This only runs if Ask is SUCCESS AND CheckYes is SUCCESS
            play_protocol_sequence = py_trees.composites.Sequence(name="PlayProtocolPath", memory=True)
            play_protocol_sequence.add_children([ask_question_tree, check_yes, exercise_block])

            # Create the "Skip Path"
            # If the sequence above fails (User said NO)
            root_selector = py_trees.composites.Selector(name="CheckPermission", memory=True)
            root_selector.add_children([
                play_protocol_sequence,
                ReadScript(text="User said No. Skipping exercise.", name="SkipNotice", node=self.robot_interface)
            ])
            
            return root_selector

        else:
            # Standard execution without permission check
            root = py_trees.composites.Sequence(
                name="FullExercisePipeline",
                memory=True,
                children=[
                    move_to_person,
                    exercise_block,
                ],
            )

            return root

    # BUILD EXERCISE BLOCK

    def build_exercise_block(self, series_key, ex_key, series, ex):
        prefix = f"{self.key_word}_{series_key}_{ex_key}"
        seq = py_trees.composites.Sequence(
            name=f"{series_key}_{ex_key}",
            memory=True,  # False
        )

        # Exercise description (resumable)
        desc_flag = f"{prefix}_desc_done"
        seq.add_child(
            py_trees.composites.Selector(
                name=f"{ex_key}_DescSelector",
                memory=True,  # False
                children=[
                    CheckDoneBB("DescDone?", desc_flag),
                    py_trees.composites.Sequence(
                        name="Do_{ex_key}_DescSelector",
                        memory=True,  # False
                        children=[
                            ReadScript(text=ex["text"], name=f"{ex_key}_Description", node=self.robot_interface),
                            SetDoneBB(f"{ex_key}_MarkDescDone", desc_flag),
                        ],
                    ),
                ],
            )
        )

        # REPS LOOP
        rep = ex["rep"]
        wait_time = ex["time_between_rep_in_sec"]

        # video_path = f"file:///storage/emulated/0/Download/Exercise_Videos/{series['type']}/{ex['video_name']}"
        video_path = f"{self.video_dir_path}/{series['type']}/{ex['video_name']}"

        for i in range(rep):
            rep_flag = f"{prefix}_rep_{i + 1}_done"

            video_goal = PlayVideoRequest.Goal()
            video_goal.file_name = video_path

            play_video_reminder = py_trees_ros.actions.ActionClient(
                name=f"{ex_key}_Video_{i + 1}",
                action_type=PlayVideoRequest,
                action_name="play_video",
                action_goal=video_goal,
                wait_for_server_timeout_sec=120.0,
            )

            rep_selector = py_trees.composites.Selector(
                name=f"{ex_key}_Rep_{i + 1}",
                memory=True,  # False
                children=[
                    CheckDoneBB(f"Rep{i + 1}Done?", rep_flag),
                    py_trees.composites.Sequence(
                        name="Do_Rep_{ex_key}_Rep_{i+1}",
                        memory=True,  # False
                        children=[
                            play_video_reminder,
                            SetDoneBB(f"{ex_key}_MarkRep{i + 1}Done", rep_flag),
                            Wait(wait_time, f"{ex_key}_Wait_{i + 1}")
                            if i < rep - 1
                            else py_trees.behaviours.Success(),
                        ],
                    ),
                ],
            )

            seq.add_child(rep_selector)

        return seq


def main(args=None):
    parser = argparse.ArgumentParser(
        description="""Exercise Protocol Tree

        Handles Playing the Exercise Protocol
        """,
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )

    parser.add_argument(
        "--run_continuous",
        type=str2bool,
        default=False,
        help="Run tree continuously (default: False)",
    )
    parser.add_argument(
        "--protocol_name",
        type=str,
        default="exercise",
        help="name of the protocol that needs to run (ex: medicine_am)",
    )

    args, unknown = parser.parse_known_args()
    protocol_name = args.protocol_name
    print("protocol_name: ", protocol_name)

    yaml_file_path = os.getenv("house_yaml_path", None)
    
    load_locations_to_blackboard(yaml_file_path, debug=False)
    load_protocols_to_bb(yaml_file_path, debug=False)
    
    tree_runner = ExerciseProtocolTree(
        node_name="exercise_protocol_tree", protocol_name=protocol_name
    )
    
    tree_runner.setup()    
    tree_runner.run_until_done()
  



if __name__ == "__main__":
    main()


# python3 exercise_protocol.py
# ros2 topic pub /display_rx std_msgs/msg/String "data: 'exercise_requested'" 