#!/usr/bin/env python3

"""
Behavior that waits.
This should be extended to allow the robot to perform other protocol while its waiting
"""

import time

import py_trees


class Wait(py_trees.behaviour.Behaviour):
    """
    Non-blocking wait behavior.
    Returns RUNNING until the duration has passed, then SUCCESS.
    """

    def __init__(self, duration_in_sec: float, name="Wait"):
        super().__init__(name)
        self.duration = duration_in_sec
        self.start_time = None

    def initialise(self):
        self.start_time = time.time()
        print(f"[WAIT] Started wait: {self.duration}s")

    def update(self):
        elapsed = time.time() - self.start_time

        if elapsed < self.duration:
            return py_trees.common.Status.RUNNING

        print("[WAIT] Done waiting")
        return py_trees.common.Status.SUCCESS

    def terminate(self, new_status):
        print(f"[WAIT] Terminated with status: {new_status}")


def main():
    wait = Wait(10)
    wait.initialise()

    while True:
        status = wait.update()
        print("STATUS:", status)

        if status != py_trees.common.Status.RUNNING:
            break

        time.sleep(0.1)

    wait.terminate(status)
    wait.update()


if __name__ == "__main__":
    main()
