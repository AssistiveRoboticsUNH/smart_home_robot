#!/usr/bin/env python3

'''
wait behavior records a timed resume request on the blackboard and immediately succeeds, yielding control
so the orchestrator can run other protocols until the scheduled time is reached.
'''

import py_trees
import time
from smart_home_pytree.registry import load_protocols_to_bb
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB


class YieldWait(py_trees.behaviour.Behaviour):
    def __init__(self, protocol_name, class_name, wait_time_key, name="YieldWait"):
        super().__init__(name)
        # class name without tree
        self.class_name = class_name
        self.protocol_name = protocol_name
        self.wait_time_key = wait_time_key

    def update(self):
        bb = py_trees.blackboard.Blackboard()
        protocol_info = bb.get(self.protocol_name)
        wait_seconds = protocol_info[self.wait_time_key]

        # Flattened key
        request_key = f"{self.class_name}.{self.protocol_name}"

        # Get existing dict or create one
        if not bb.exists("wait_requests"):
            bb.set("wait_requests", {})

        wait_requests = bb.get("wait_requests")
        # Set the wait request
        wait_requests[request_key] = {
            "wait_key": self.wait_time_key,
            "seconds": wait_seconds,
            "timestamp": time.time()
        }

        # Write back to blackboard
        bb.set("wait_requests", wait_requests)

        # Mark wait as done so selector skips it
        set_bb_node = SetProtocolBB(
            name="MarkWaitDone",
            key=f"{self.protocol_name}_done.{self.wait_time_key}_done",
            value=True
        )
        set_bb_node.update()

        return py_trees.common.Status.FAILURE


def main():
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
