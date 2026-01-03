#!/usr/bin/env python3

'''
wait behavior records a timed resume request on the blackboard and immediately succeeds, yielding control
so the orchestrator can run other protocols until the scheduled time is reached.
'''

import py_trees
import time
from smart_home_pytree.registry import load_protocols_to_bb
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
from smart_home_pytree.utils import parse_duration


class YieldWait(py_trees.behaviour.Behaviour):
    """
    A behavior that yields execution by registering a wait request with the TriggerMonitor.

    This behavior posts a wait request to the blackboard and blocks (returns RUNNING)
    until the TriggerMonitor's collect_wait_requests() method processes it. This ensures
    the protocol is properly marked as waiting before the behavior tree completes,
    preventing race conditions where the orchestrator might immediately restart the protocol.

    Workflow:
        1. On initialise(): Posts wait request to blackboard with timestamp and duration
        2. On update(): Returns RUNNING until TriggerMonitor removes the request from blackboard
        3. Once processed (or timeout): Returns FAILURE to signal tree completion

    The FAILURE status is intentional - it tells the behavior tree that this branch
    is "done for now" and the protocol should yield control to allow other protocols
    (like charging) to run while waiting.

    Blackboard Usage:
        Reads:
            - {protocol_name}: Dict containing protocol state including wait duration
        Writes:
            - wait_requests: Dict of pending wait requests for TriggerMonitor
            - {protocol_name}_done.{wait_time_key}_done: Marks this wait step as complete
    """

    def __init__(self, protocol_name, class_name, wait_time_key,
                 debug=False, name="YieldWait", max_wait=2.0):
        """
        Initialize the YieldWait behavior.

        Args:
            protocol_name (str): The protocol identifier used as blackboard key
            class_name (str): Protocol class name for constructing request key
            wait_time_key (str): Key to lookup wait duration in protocol_info
            name (str, optional): Display name for this behavior. Defaults to "YieldWait".
            max_wait (float, optional): Timeout in seconds for TriggerMonitor pickup.
                                       Defaults to 2.0.
        """
        super().__init__(name)
        self.class_name = class_name
        self.protocol_name = protocol_name
        self.wait_time_key = wait_time_key
        self.max_wait = max_wait
        self.start_time = None
        self.debug = True  # debug

    def initialise(self):
        """
        Post the wait request to the blackboard when this behavior first executes.

        This method:
        1. Records the start time for timeout tracking
        2. Reads wait duration from protocol_info on blackboard
        3. Creates a wait request with the protocol's full name as key
        4. Posts request to wait_requests dict on blackboard
        5. Marks the wait step as "done" so the selector won't retry it

        The wait request contains:
            - wait_key: Which wait step this is (e.g., "wait_1")
            - seconds: How long to wait before resuming
            - timestamp: When this request was created (for resume time calculation)

        Called automatically by py_trees when behavior transitions to RUNNING.
        """
        self.start_time = time.time()

        bb = py_trees.blackboard.Blackboard()
        protocol_info = bb.get(self.protocol_name)
        wait_time_unparsed = protocol_info[self.wait_time_key]
        wait_seconds = parse_duration(wait_time_unparsed)
        # Construct full protocol identifier (e.g., "XReminderProtocol.medicine_am")
        request_key = f"{self.class_name}.{self.protocol_name}"

        # Ensure wait_requests dict exists on blackboard
        if not bb.exists("wait_requests"):
            bb.set("wait_requests", {})

        wait_requests = bb.get("wait_requests")
        wait_requests[request_key] = {
            "wait_key": self.wait_time_key,
            "seconds": wait_seconds,
            "timestamp": time.time()
        }
        bb.set("wait_requests", wait_requests)

        # Mark this wait step as complete so selector skips it on next run
        set_bb_node = SetProtocolBB(
            name="MarkWaitDone",
            key=f"{self.protocol_name}_done.{self.wait_time_key}_done",
            value=True
        )
        set_bb_node.update()
        if self.debug:
            print(f"[YieldWait] Posted wait request for {request_key}")

    def update(self):
        """
        Check if TriggerMonitor has processed the wait request.

        Returns:
            py_trees.common.Status.RUNNING: If request still in blackboard (not yet processed)
            py_trees.common.Status.FAILURE: If request was processed or timeout occurred

        The FAILURE status indicates this branch is complete and the tree should finish,
        allowing the protocol to yield control. This is the desired behavior - not an error.

        Timeout Behavior:
            If max_wait seconds elapse without TriggerMonitor processing the request,
            returns FAILURE anyway to prevent indefinite blocking. This is a safety
            mechanism in case TriggerMonitor is not running or has issues.
        """
        bb = py_trees.blackboard.Blackboard()
        wait_requests = bb.get("wait_requests")
        request_key = "{self.class_name}.{self.protocol_name}"

        # Check if TriggerMonitor picked up the request (removed from dict)
        if request_key not in wait_requests:
            if self.debug:
                print("[YieldWait] Request processed by TriggerMonitor")
            return py_trees.common.Status.FAILURE

        # Safety timeout to prevent infinite blocking
        if time.time() - self.start_time > self.max_wait:
            if self.debug:
                print("[YieldWait] Timeout - proceeding anyway")
            return py_trees.common.Status.FAILURE

        # Still waiting for TriggerMonitor to process
        return py_trees.common.Status.RUNNING


def main():
    # ## todo  adjust to have things wiped from blackboard
    import pprint
    import os

    yaml_file_path = os.getenv("house_yaml_path", None)
    load_protocols_to_bb(yaml_file_path)

    # ---- setup blackboard ----
    bb = py_trees.blackboard.Blackboard()

    class_name = "TwoReminderProtocol"
    protocol_name = "medicine_am"
    wait_time_key = "wait_time_between_reminders"

    # from smart_home_pytree run python3 -m smart_home_pytree.behaviors.action_behaviors.yield_wait
    # since load_protocols_to_bb is used
    # ---- create and run YieldWait ----
    wait_node = YieldWait(
        class_name=class_name,
        protocol_name=protocol_name,
        wait_time_key=wait_time_key,
    )

    status = wait_node.update()

    # ---- output ----
    print("\n=== YieldWait returned ===")
    print(status)

    print("\n=== Blackboard contents ===")
    pprint.pprint(bb.storage, width=120)


if __name__ == "__main__":
    main()
