#!/usr/bin/env python3

'''
Behavior that waits.
This should be extended to allow the robot to perform other protocol while its waiting
'''

import py_trees
import time

class Wait(py_trees.behaviour.Behaviour):
    """
    Behavior that waits.
    This should be extended to allow the robot to perform other protocol while its waiting
    """
    def __init__(self, duration_in_sec: int, name="ReadScript"):
        super().__init__(name)
        self.duration_in_sec = duration_in_sec

    def update(self):
        print("Waiting ...")

        time.sleep(self.duration_in_sec)
        print("Done Waiting ")
        return py_trees.common.Status.SUCCESS
  
def main():
    wait = Wait(10)
    wait.update()
        
if __name__ == "__main__":
    main()