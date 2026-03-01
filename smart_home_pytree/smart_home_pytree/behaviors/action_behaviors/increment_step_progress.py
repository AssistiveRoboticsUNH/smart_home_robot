#!/usr/bin/env python3

"""
Behavior that increments the completed_step counter in the protocol tracker
after a top-level step finishes successfully.

This is inserted into the top-level GenericProtocol action sequence after each
step subtree so the dashboard can display live step-by-step progress.
"""

import py_trees


class IncrementStepProgress(py_trees.behaviour.Behaviour):
    """Increment ``completed_step`` for a protocol in the persistent tracker.

    The tracker instance is read from the py_trees blackboard under the
    ``protocol_tracker`` key (placed there by the orchestrator at startup).

    Always returns SUCCESS so it never blocks the sequence.
    """

    def __init__(self, protocol_full_name: str, step_index: int, name: str = "StepProgress"):
        super().__init__(name)
        self.protocol_full_name = protocol_full_name
        self.step_index = step_index

    def update(self):
        try:
            bb = py_trees.blackboard.Blackboard()
            tracker = bb.get("protocol_tracker") if bb.exists("protocol_tracker") else None
            if tracker is not None:
                tracker.update_step_progress(self.protocol_full_name, self.step_index)
        except Exception as exc:
            # Never fail the sequence because of a tracking issue
            self.logger.warning(f"[StepProgress] Failed to update progress: {exc}")
        return py_trees.common.Status.SUCCESS
