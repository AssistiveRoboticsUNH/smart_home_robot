#!/usr/bin/env python3

"""
This is an automated protocol behavior tree builder that constructs a `py_trees`
from a YAML-defined protocol action block.

This module provides a YAML-driven protocol runner that translates a validated
`action.steps` list (loaded onto the blackboard by `registry.py`) into a
`py_trees` subtree at runtime.

This is intentionally a constrained builder, not a general-purpose BT compiler.


Supported step features:
- executable tree steps (`tree_name`, `tree_params`) via the centralized tree registry
- explicit confirmation branching (`confirmation.question`, `on_yes`, `on_no`)
- `next_step_after` pauses between steps (via `YieldWait`)


Standalone usage example (run `medicine_am` once):
    export SHR_USER_DIR=$HOME/shr_user/<user_name>
    python3 smart_home_pytree/smart_home_pytree/protocols/generic_protocol.py --protocol_name medicine_am

What happens in standalone mode:
- Loads `locations` and `protocols` from YAML into the py_trees blackboard
- Builds the behavior tree for the selected protocol from `action.steps`
- Runs it once (`run_until_done`) unless `--run_continuous true` is passed
- Uses the same underlying action/media/question trees as the orchestrated system
"""

import argparse
import os

import py_trees

from smart_home_pytree.behaviors.action_behaviors.increment_step_progress import IncrementStepProgress
from smart_home_pytree.behaviors.action_behaviors.yield_wait import YieldWait
from smart_home_pytree.protocols.registry import load_locations_to_blackboard, load_protocols_to_bb
from smart_home_pytree.trees.ask_question_tree import AskQuestionTree
from smart_home_pytree.trees.base_tree_runner import BaseTreeRunner
from smart_home_pytree.protocols.builders.shared_builder_utils import (
    make_run_tree_action,
)
from smart_home_pytree.utils import get_house_yaml_path, parse_duration, str2bool


class GenericProtocolTree(BaseTreeRunner):
    """
    Build and run a sequential behavior tree from blackboard action-step data.

    Expected blackboard data under `protocol_name` (loaded by registry):
    - `steps`: list of action step dictionaries
    - synthesized `step_i` / `confirm_i` / `wait_i` keys for top-level steps
    - synthesized `yes_i_step_j` / `no_i_step_j` keys for confirmation branches

    Notes:
    - The blackboard synthesis is done in `load_protocols_to_bb()`.
    - This class only composes behavior subtrees; it does not validate YAML
      schema (that is handled earlier by `protocol_schema.py`).
    """

    # Generic confirmation result boolean written by AskQuestionBehavior.
    USER_CONFIRMATION_RESULT_KEY = "user_confirmation_result"

    def __init__(
        self,
        node_name: str,
        robot_interface=None,
        executor=None,
        debug=False,
        **kwargs,
    ):
        super().__init__(
            node_name=node_name,
            robot_interface=robot_interface,
            debug=debug,
            executor=executor,
            **kwargs,
        )
        self.bb = py_trees.blackboard.Blackboard()
        self.protocol_name = self.kwargs.get("protocol_name", "")
        if not self.protocol_name:
            raise ValueError(f"[{node_name}] CRITICAL: 'protocol_name' is missing in kwargs.")

        if not self.bb.exists(self.protocol_name):
            raise KeyError(
                f"[{node_name}] CRITICAL: Protocol '{self.protocol_name}' "
                f"not found in Blackboard. Available keys: {list(self.bb.storage.keys())}"
            )

        self.protocol_info = self.bb.get(self.protocol_name) or {}
        self.steps = self.protocol_info.get("steps", [])
        if not isinstance(self.steps, list) or len(self.steps) == 0:
            raise ValueError(
                f"[{node_name}] CRITICAL: '{self.protocol_name}' must define a non-empty action.steps list."
            )

    def _step_key(self, idx: int, prefix: str = "") -> str:
        """Return blackboard key name for a top-level or prefixed step payload."""
        if prefix:
            return f"{prefix}_step_{idx}"
        return f"step_{idx}"

    def _confirm_key(self, idx: int, prefix: str = "") -> str:
        """Return blackboard key name for a top-level or prefixed confirmation question."""
        if prefix:
            return f"{prefix}_confirm_{idx}"
        return f"confirm_{idx}"

    def _wait_key(self, idx: int, prefix: str = "") -> str:
        """Return blackboard key name for a top-level or prefixed wait duration."""
        if prefix:
            return f"{prefix}_wait_{idx}"
        return f"wait_{idx}"

    def _branch_step_key(self, step_idx: int, branch_name: str, branch_step_idx: int) -> str:
        """Return synthesized blackboard key for a confirmation-branch action payload."""
        return f"{branch_name}_{step_idx}_step_{branch_step_idx}"

    def _branch_wait_key(self, step_idx: int, branch_name: str, branch_step_idx: int) -> str:
        """Return synthesized blackboard key for a confirmation-branch wait duration."""
        return f"{branch_name}_{step_idx}_wait_{branch_step_idx}"

    def _split_branch_prefix(self, branch_prefix: str | None):
        """
        Parse branch prefix like `yes_2` / `no_3`.

        Returns:
            tuple[str, int] | None: (`branch_name`, `parent_step_index`) or None.
        """
        if not branch_prefix:
            return None
        branch_name, step_idx_str = branch_prefix.split("_", 1)
        return branch_name, int(step_idx_str)

    def _build_ask_confirmation_tree(
        self, confirm_data_key: str, execution_location: str = "current"
    ):
        """Create the AskQuestionTree subtree for a synthesized confirmation key."""
        return AskQuestionTree(
            node_name=f"{self.node_name}_{confirm_data_key}",
            robot_interface=self.robot_interface,
            protocol_name=self.protocol_name,
            data_key=confirm_data_key,
            execution_location=execution_location,
            debug=self.debug,
            executor=self.executor,
        ).create_tree()

    def _build_bool_check(self, name: str, expected_value: bool):
        """Create a blackboard boolean check used for confirmation branching."""
        return py_trees.behaviours.CheckBlackboardVariableValue(
            name=name,
            check=py_trees.common.ComparisonExpression(
                variable=self.USER_CONFIRMATION_RESULT_KEY,
                value=expected_value,
                operator=lambda a, b: a == b,
            ),
        )

    def _maybe_add_wait(
        self,
        parent_sequence: py_trees.composites.Sequence,
        wait_key: str,
        next_step_after,
    ) -> None:
        """
        Append a YieldWait child if `next_step_after` resolves to a positive duration.

        This helper keeps wait insertion logic consistent across top-level and
        branch sequences.
        """
        wait_seconds = parse_duration(next_step_after)
        if wait_seconds <= 0:
            return

        parent_sequence.add_child(
            YieldWait(
                name=f"{self.node_name}_{wait_key}",
                class_name="GenericProtocol",
                protocol_name=self.protocol_name,
                wait_time_key=wait_key,
            )
        )

    def _build_action_step_subtree(self, idx: int, step: dict, data_key: str | None = None):
        """
        Build a subtree for a single non-branch executable step.

        Args:
            idx: Step index within the current sequence (top-level or branch-local).
            step: Executable step dictionary (tree_name + tree_params).
            data_key: Optional synthesized blackboard key override (used in branches).
        """
        tree_name = step.get("tree_name", "")
        step_key = data_key or self._step_key(idx)
        if not tree_name:
            raise ValueError(f"[{self.node_name}] Step {idx} requires tree_name.")

        return make_run_tree_action(
            tree_name=tree_name,
            tree_params=step.get("tree_params") or {},
            node_name=f"{self.node_name}_{step_key}_{tree_name}",
            robot_interface=self.robot_interface,
            protocol_name=self.protocol_name,
            step_done_key=f"{step_key}_done",
            step_data_key=step_key,
            executor=self.executor,
            debug=self.debug,
        )

    def _build_step_sequence(
        self,
        steps: list,
        sequence_name: str,
        branch_prefix: str | None = None,
        allow_confirmation_branches: bool = True,
        protocol_full_name: str | None = None,
    ) -> py_trees.behaviour.Behaviour:
        """
        Build a sequential subtree from a list of action steps.

        This method is used both for the top-level protocol action sequence and for the `on_yes`
        / `on_no` branches of explicit confirmation steps.
        """
        if not steps:
            return py_trees.behaviours.Success(name=f"{sequence_name}_Noop")

        root = py_trees.composites.Sequence(name=sequence_name, memory=True)
        total_steps = len(steps)
        branch_info = self._split_branch_prefix(branch_prefix)

        for idx, step in enumerate(steps, start=1):
            if not isinstance(step, dict):
                raise ValueError(
                    f"[{self.node_name}] Step {idx} in {sequence_name} must be a dictionary. "
                    f"Got: {type(step)}"
                )

            if "confirmation" in step:
                if not allow_confirmation_branches:
                    raise ValueError(
                        f"[{self.node_name}] Nested confirmation branches are not supported "
                        f"in {sequence_name} step {idx}."
                    )
                step_subtree = self._build_confirmation_branch_step(idx, step)
            else:
                data_key = None
                if branch_info:
                    branch_name, parent_step_idx = branch_info
                    data_key = self._branch_step_key(
                        step_idx=parent_step_idx,
                        branch_name=branch_name,
                        branch_step_idx=idx,
                    )
                step_subtree = self._build_action_step_subtree(idx, step, data_key=data_key)

            root.add_child(step_subtree)

            # Track top-level step progress in the persistent tracker
            if protocol_full_name and branch_prefix is None:
                root.add_child(
                    IncrementStepProgress(
                        protocol_full_name=protocol_full_name,
                        step_index=idx,
                        name=f"{self.node_name}_progress_{idx}",
                    )
                )

            if idx >= total_steps:
                continue

            wait_key = (
                self._branch_wait_key(
                    step_idx=branch_info[1],
                    branch_name=branch_info[0],
                    branch_step_idx=idx,
                )
                if branch_info
                else self._wait_key(idx)
            )
            self._maybe_add_wait(root, wait_key=wait_key, next_step_after=step.get("next_step_after", 0))

        return root

    def _build_confirmation_branch_step(self, idx: int, step: dict):
        """
        Build an explicit confirmation branch step.

        YAML shape:
            - confirmation:
                question: "..."
                on_yes: [ ...steps... ]
                on_no:  [ ...steps... ]
        """
        confirmation = step.get("confirmation") or {}
        confirm_key = self._confirm_key(idx)
        execution_location = confirmation.get("execution_location", "current")
        ask_tree = self._build_ask_confirmation_tree(
            confirm_key, execution_location=execution_location
        )

        yes_steps = confirmation.get("on_yes") or []
        no_steps = confirmation.get("on_no") or []

        yes_subtree = self._build_step_sequence(
            yes_steps,
            sequence_name=f"{self.protocol_name}_yes_branch_{idx}",
            branch_prefix=f"yes_{idx}",
            allow_confirmation_branches=False,
        )
        no_subtree = self._build_step_sequence(
            no_steps,
            sequence_name=f"{self.protocol_name}_no_branch_{idx}",
            branch_prefix=f"no_{idx}",
            allow_confirmation_branches=False,
        )

        yes_path = py_trees.composites.Sequence(name=f"ConfirmYesPath_{idx}", memory=True)
        yes_path.add_children(
            [
                self._build_bool_check(name=f"ConfirmYes_{idx}", expected_value=True),
                yes_subtree,
            ]
        )

        no_path = py_trees.composites.Sequence(name=f"ConfirmNoPath_{idx}", memory=True)
        no_path.add_children(
            [
                self._build_bool_check(name=f"ConfirmNo_{idx}", expected_value=False),
                no_subtree,
            ]
        )

        decision = py_trees.composites.Selector(
            name=f"ConfirmBranchDecision_{idx}",
            memory=True,
            children=[yes_path, no_path],
        )

        root = py_trees.composites.Sequence(
            name=f"ConfirmBranch_{idx}",
            memory=True,
            children=[ask_tree, decision],
        )
        return root

    def create_tree(self) -> py_trees.behaviour.Behaviour:
        """
        Create the root behavior tree for this protocol instance.

        Returns:
            A memory `Sequence` containing the synthesized action execution tree.
        """
        # Resolve the full protocol name (e.g. "GenericProtocol.medicine_am")
        # used by IncrementStepProgress for the tracker DB key.
        protocol_full_name = f"GenericProtocol.{self.protocol_name}"
        return self._build_step_sequence(
            self.steps,
            sequence_name=f"{self.protocol_name}_GenericActionSequence",
            allow_confirmation_branches=True,
            protocol_full_name=protocol_full_name,
        )


def main(args=None):
    """CLI entry point for manually running a generic protocol tree."""
    parser = argparse.ArgumentParser(
        description="Generic Protocol Tree",
        formatter_class=argparse.RawDescriptionHelpFormatter,
    )
    parser.add_argument("--run_continuous", type=str2bool, default=False)
    parser.add_argument("--protocol_name", type=str, default="medicine_am")

    args, _ = parser.parse_known_args()
    yaml_file_path = get_house_yaml_path()
    load_locations_to_blackboard(yaml_file_path)
    load_protocols_to_bb(yaml_file_path)

    tree_runner = GenericProtocolTree(
        node_name="generic_protocol_tree",
        protocol_name=args.protocol_name,
    )
    tree_runner.setup()

    if args.run_continuous:
        tree_runner.run_continuous()
    else:
        tree_runner.run_until_done()


if __name__ == "__main__":
    main()
