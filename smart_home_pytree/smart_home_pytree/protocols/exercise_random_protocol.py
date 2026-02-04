import argparse
import os
import random
import yaml
from typing import Dict, List, Optional
from pathlib import Path

import py_trees
import py_trees_ros
import rclpy

from shr_msgs.action import PlayVideoRequest
from smart_home_pytree.behaviors.action_behaviors.read_script_aalp import ReadScript
from smart_home_pytree.behaviors.action_behaviors.wait import Wait
from smart_home_pytree.registry import load_protocols_to_bb, load_locations_to_blackboard
from smart_home_pytree.trees.ask_question_tree import AskQuestionTree
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.trees.move_to_person_location import MoveToPersonLocationTree
from smart_home_pytree.utils import str2bool

# --- Helper Behaviors ---
class CheckRobotStateKey_(py_trees.behaviour.Behaviour):
    """
    Checks a robot state key. 
    Returns FAILURE if the stop signal is True (triggering the stop sequence).
    Returns SUCCESS if it is safe to proceed.
    """
    def __init__(self, name: str, robot_interface, key: str):
        super().__init__(name)
        self.robot_interface = robot_interface
        self.key = key

    def update(self):
        value = self.robot_interface.state.get(self.key, None)
        if value is None:
            self.logger.warning(f"{self.name}: '{self.key}' not found in RobotState")
            value = False

        if value == True:
            # fail when true
            self.logger.debug(f"STOP EXERCISE, FAILURE")
            return py_trees.common.Status.FAILURE
        else:
            self.logger.debug(f"Not STOP EXERCISE, SUCCESS ")
            return py_trees.common.Status.SUCCESS

class SetBlackboardKey(py_trees.behaviour.Behaviour):
    def __init__(self, name, key, value):
        super().__init__(name)
        self.key = key
        self.value = value

    def update(self):
        py_trees.blackboard.Blackboard().set(self.key, self.value)
        return py_trees.common.Status.SUCCESS

class DecrementRemainingExercises(py_trees.behaviour.Behaviour):
    def __init__(self, name, key="exercise_remaining"):
        super().__init__(name)
        self.key = key
        self.bb = py_trees.blackboard.Blackboard()        

    def update(self):
        remaining = self.bb.get(self.key)
        if remaining is None:
            return py_trees.common.Status.FAILURE
        
        new_val = max(remaining - 1, 0)
        self.bb.set(self.key, new_val)
        self.logger.info(f"Exercises remaining: {new_val}")
        return py_trees.common.Status.SUCCESS

class ClearExerciseRunning(py_trees.behaviour.Behaviour):
    def __init__(self, name, start_key="start_exercise", robot_interface=None):
        super().__init__(name)
        self.start_key = start_key
        self.robot_interface = robot_interface
        self.bb = py_trees.blackboard.Blackboard()

    def update(self):
        self.bb.set("exercise_running", False)
        # Also clear the selected videos so next run is fresh
        self.bb.set("exercise_selected_videos", None)
        self.bb.set("exercise_remaining", None)
        # set start_exercise to false
        self.robot_interface.state.update(self.start_key, False)

        return py_trees.common.Status.SUCCESS

# --- Main Tree ---

class ExerciseRandomProtocolTree(BaseTreeRunner):
    def __init__(
        self, node_name: str, robot_interface=None, executor=None, debug=False, **kwargs
    ):
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
            **kwargs,
        )

        self.bb = py_trees.blackboard.Blackboard()
        self.bb_logger = self.bb.get("logger") or rclpy.logging.get_logger(node_name)
        
        self.protocol_name = self.kwargs.get("protocol_name", "")
        if not self.protocol_name:
            raise ValueError(f"[{node_name}] CRITICAL: 'protocol_name' missing.")
            
        if not self.bb.exists(self.protocol_name):
            raise KeyError(f"[{node_name}] CRITICAL: Protocol '{self.protocol_name}' not found.")
        
        self.protocol_info = self.bb.get(self.protocol_name)
        
        # Validate Keys
        required_keys = [
            "total_num_exercises", "difficulty_level", "time_limit_minutes",
            "reps_per_exercise", "time_between_exercises", "time_between_reps",
            "start_state_key", "stop_state_key", "video_dir_path",
        ]
        for key in required_keys:
            if key not in self.protocol_info:
                raise KeyError(f"Missing '{key}' in protocol config.")
                
        # Load Config
        self.difficulty_level = self.protocol_info["difficulty_level"]
        self.total_num_exercises = self.protocol_info["total_num_exercises"]
        self.time_limit_minutes = self.protocol_info["time_limit_minutes"]
        self.reps_per_exercise = self.protocol_info["reps_per_exercise"]
        self.time_between_exercises = self.protocol_info["time_between_exercises"]
        self.time_between_reps = self.protocol_info["time_between_reps"]
        self.start_key = self.protocol_info["start_state_key"]
        self.stop_key = self.protocol_info["stop_state_key"]
        self.video_dir_path = self.protocol_info["video_dir_path"]

    def extract_difficulty(self, video_path: Path) -> str:
        """
        Extract difficulty from a video filename.

        Expected format:
            FINAL_exercise_name_<difficulty>.mp4

        :param video_path: Path to the video file
        :return: difficulty string
        :raises ValueError: if difficulty cannot be determined
        """
        VALID_DIFFICULTIES = {"low", "medium", "high"}
        stem_parts = video_path.stem.split("_")
        difficulty = stem_parts[-1].lower()
        if difficulty not in VALID_DIFFICULTIES:
            raise ValueError(f"Invalid difficulty in: {video_path.name}")
        return difficulty

    def build_video_pool(self, root_dir: str) -> List[Dict]:
        """
        Recursively collect all exercise videos under a root directory.

        :param root_dir: Root directory containing exercise subfolders
        :return: List of dicts with video metadata
        """
        root = Path(root_dir).expanduser().resolve()
        if not root.exists():
            raise FileNotFoundError(f"Directory not found: {root}")

        video_pool = []
        for video_path in root.rglob("*.mp4"):
            try:
                diff = self.extract_difficulty(video_path)
                video_pool.append({
                    "name": video_path.stem,
                    "path": str(video_path),
                    "difficulty": diff,
                    "category": video_path.parent.name,
                })
            except ValueError as e:
                pass # logging skipped for brevity
        return video_pool

    def sample_exercises(self, video_pool, total_num, difficulty_filter=None):
        """
        Sample exercises from a structured video pool.

        :param video_pool: list of video dicts
        :param total_num: number of exercises to sample
        :param difficulty_filter: list like ["low", "medium"] or empty / None
        """
        if difficulty_filter:
            # FIX: Ensure filter is treated as a list/set for exact matching
            if isinstance(difficulty_filter, str):
                difficulty_filter = [difficulty_filter]
                
            candidates = [v for v in video_pool if v["difficulty"] in difficulty_filter]
        else:
            candidates = video_pool

        if not candidates:
            # Fallback: if no candidates match, return random or raise error
            self.bb_logger.warning(f"No videos found for difficulty {difficulty_filter}")
            return []

        return random.sample(candidates, min(total_num, len(candidates)))

    def build_exercise_sequence(self, exercise_path: str, reps: int, wait_between_reps: int):
        """
        Build a subtree that plays one exercise for N reps.
        """
        exercise_seq = py_trees.composites.Sequence(
            name=f"Ex_{Path(exercise_path).stem}",
            memory=True,
        )
        for i in range(reps):
            goal = PlayVideoRequest.Goal()
            goal.file_name = str(exercise_path)
            
            play_video = py_trees_ros.actions.ActionClient(
                name=f"Play Rep {i+1}",
                action_type=PlayVideoRequest,
                action_name="play_video",
                action_goal=goal,
                wait_for_server_timeout_sec=30.0,
            )
            exercise_seq.add_child(play_video)
            
            if i < reps - 1:
                exercise_seq.add_child(Wait(wait_between_reps, f"Rest {i+1}"))
                
        return exercise_seq

    def create_tree(self):
        """
        Build a full tree.
        """
        protocol_name = self.protocol_name
        protocol_info = self.bb.get(protocol_name)
        
        confirmation_key = "get_confirmation"
        self.get_confirmation = protocol_info.get(confirmation_key, "")

        print("self.start_key", self.start_key)
        print("self.stop_key", self.stop_key)
        
        # --- Exercise Selection & Resumption Logic ---
        if self.bb.exists("exercise_running"):
            is_running = self.bb.get("exercise_running")
        else:
            is_running = False
            
        selected_exercises = []

        if is_running and self.bb.exists("exercise_selected_videos"):
            self.bb_logger.info("Resuming interrupted exercise protocol...")
            
            all_exercises = self.bb.get("exercise_selected_videos")
            if self.bb.exists("exercise_remaining"):
                remaining_count = self.bb.get("exercise_remaining")
            else:
                remaining_count = None
                self.bb_logger.warn("Missing 'exercise_remaining' on blackboard during resumption.")

            if remaining_count is not None and all_exercises:
                start_index = len(all_exercises) - remaining_count
                if start_index < len(all_exercises):
                    selected_exercises = all_exercises[start_index:]
                else:
                    # Logic edge case
                    selected_exercises = []
            else:
                # Fallback if state is corrupted
                is_running = False 
        
        if not is_running or not selected_exercises:
            # Start Fresh
            video_pool = self.build_video_pool(self.video_dir_path)
            selected_exercises = self.sample_exercises(
                video_pool,
                self.total_num_exercises, 
                self.difficulty_level
            )
            self.bb.set("exercise_selected_videos", selected_exercises)
            self.bb.set("exercise_remaining", len(selected_exercises))

        self.bb_logger.info(f"Queueing {len(selected_exercises)} exercises.")

        # --- Build Protocol Sequence ---
        mark_running = SetBlackboardKey("MarkRunning", key="exercise_running", value=True)
        
        protocol_seq = py_trees.composites.Sequence(name="ExerciseLoop", memory=True)

        for i, exercise in enumerate(selected_exercises):
            # Exercise block
            ex_node = self.build_exercise_sequence(
                exercise["path"],
                self.reps_per_exercise,
                self.time_between_reps
            )
            protocol_seq.add_child(ex_node)

            # Decrement counter AFTER success of exercise
            protocol_seq.add_child(DecrementRemainingExercises("DecrCount"))

            # Rest only if this is NOT the last exercise
            if i < len(selected_exercises) - 1:
                protocol_seq.add_child(
                    Wait(self.time_between_exercises, "RestBetweenEx")
                )

        # --- Guards ---
        stop_guard = CheckRobotStateKey_(
            "StopExerciseCheck",
            self.robot_interface,
            self.stop_key,
        )
        
        guarded_execution = py_trees.composites.Parallel(
            name="GuardedExecution",
            policy=py_trees.common.ParallelPolicy.SuccessOnAll(synchronise=False),
            children=[stop_guard, protocol_seq],
        )

        # --- Main Lifecycle (Mark -> Run -> Stop -> Cleanup) ---
        lifecycle = py_trees.composites.Sequence(
            name="ExerciseLifecycle",
            memory=True,
            children=[
                mark_running,
                py_trees.composites.Selector(
                    name="RunOrStop",
                    memory=True,
                    children=[
                        guarded_execution,
                        ReadScript(text="Stopping exercise now.", name="StopNotice", node=self.robot_interface)
                    ],
                ),
                ClearExerciseRunning("CleanupState",  start_key=self.start_key, robot_interface=self.robot_interface),
            ],
        )

        # --- Movement Tree ---
        move_to_person_tree = MoveToPersonLocationTree(
            node_name=f"{protocol_name}_move_to_person",
            robot_interface=self.robot_interface,
            debug=self.debug,
            executor=self.executor,
        ).create_tree()
        
        # --- Confirmation Logic ---
        if self.get_confirmation and not is_running:

            ask_tree = AskQuestionTree(
                node_name="ask_question_tree",
                robot_interface=self.robot_interface,
                protocol_name=protocol_name,
                data_key="get_confirmation",
                debug=self.debug,
                executor=self.executor,
            ).create_tree()

            # YES check
            check_yes = py_trees.behaviours.CheckBlackboardVariableValue(
                name="UserSaidYes",
                check=py_trees.common.ComparisonExpression(
                    variable="user_wants_video",
                    value=True,
                    operator=lambda a, b: a == b,
                ),
            )

            # NO check (explicit fallback)
            check_no = py_trees.behaviours.CheckBlackboardVariableValue(
                name="UserSaidNo",
                check=py_trees.common.ComparisonExpression(
                    variable="user_wants_video",
                    value=False,
                    operator=lambda a, b: a == b,
                ),
            )

            # YES path → run lifecycle
            yes_path = py_trees.composites.Sequence(
                name="YesPath",
                memory=True,
                children=[
                    check_yes,
                    lifecycle,
                ],
            )

            # NO path → explain and succeed
            no_path = py_trees.composites.Sequence(
                name="NoPath",
                memory=True,
                children=[
                    check_no,
                    ReadScript(
                        text="Okay, I will skip the exercise.",
                        name="SkipNotice",
                        node=self.robot_interface,
                    ),
                    ClearExerciseRunning("CleanupStateInNoPath",  start_key=self.start_key, robot_interface=self.robot_interface),
                ],
            )

            # Decision gate: YES → NO → FAIL
            decision = py_trees.composites.Selector(
                name="UserDecision",
                memory=True,
                children=[
                    yes_path,
                    no_path,
                ],
            )

            # Ask must succeed, decision must be explicit
            root = py_trees.composites.Sequence(
                name="AskAndRunExercise",
                memory=True,
                children=[
                    ask_tree,
                    decision,
                ],
            )
            return root
        else:
            # Standard execution without permission check
            final_root = py_trees.composites.Sequence(
                name="MoveThenRun",
                memory=True,
                children=[
                    move_to_person_tree,
                    lifecycle,
                ],
            )
            return final_root


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
        default="exercise_rand",
        help="name of the protocol that needs to run (ex: medicine_am)",
    )

    args, unknown = parser.parse_known_args()
    protocol_name = args.protocol_name
    print("protocol_name: ", protocol_name)

    yaml_file_path = os.getenv("house_yaml_path", None)
    load_locations_to_blackboard(yaml_file_path, debug=False)
    load_protocols_to_bb(yaml_file_path, debug=True)

    tree_runner = ExerciseRandomProtocolTree(
        node_name="exercise_protocol_tree", protocol_name=protocol_name
    )
    # tree_runner.create_tree()
    tree_runner.setup()

    tree_runner.run_until_done()


if __name__ == "__main__":
    main()


# python3 exercise_protocol.py
