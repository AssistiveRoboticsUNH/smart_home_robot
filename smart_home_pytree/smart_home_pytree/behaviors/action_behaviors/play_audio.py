import os
import subprocess
import time

import py_trees
from std_msgs.msg import Bool
from smart_home_pytree.utils import FailureType

class PlayAudio(py_trees.behaviour.Behaviour):
    """
    Non-blocking audio playback using mpg321.
    Returns:
        RUNNING while the audio is playing
        SUCCESS when playback finishes normally
        FAILURE if file missing or playback errors
    """

    def __init__(self, audio_path: str, debug: Bool = False,  name="PlayAudio"):
        super().__init__(name)
        self.audio_path = audio_path
        self.proc = None
        self.blackboard = py_trees.blackboard.Blackboard()
        self.debug = debug
        
    def initialise(self):
        """Called once when the behavior starts."""
        print("[PlayAudio] initialise")

        # Check file exists
        if not os.path.isfile(self.audio_path):
            if self.debug:
                print("[PlayAudio] File does not exist:", self.audio_path)
            self.blackboard.set("error_reason", f"Audio file not found: {self.audio_path}")
            self.blackboard.set("error_type", FailureType.BLOCKING)
            self.proc = None
            return

        try:
            # Launch mpg321 asynchronously
            self.proc = subprocess.Popen(
                ["mpg321", "-q", self.audio_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL,
            )
            print(f"[PlayAudio] Started: {self.audio_path}")

        except Exception as e:
            print("[PlayAudio] Failed to start audio:", e)
            self.proc = None

    def update(self):
        if self.proc is None:
            return py_trees.common.Status.FAILURE

        ret = self.proc.poll()  # None = still running

        if ret is None:
            # Audio still playing
            return py_trees.common.Status.RUNNING

        # Finished playing
        if ret == 0:
            print("[PlayAudio] Finished successfully")
            return py_trees.common.Status.SUCCESS
        else:
            print("[PlayAudio] Playback error")
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        if self.proc and self.proc.poll() is None:
            print("[PlayAudio] Interrupted, stopping audio")
            self.proc.terminate()

        self.proc = None


def main():
    play_audio = PlayAudio(
        "/home/olagh48652/smart-home/src/smart-home-robot/shr_resources/resources/food_reminder.mp3"
    )

    play_audio.initialise()

    while True:
        status = play_audio.update()
        print("STATUS:", status)

        if status != py_trees.common.Status.RUNNING:
            break

        time.sleep(0.1)

    play_audio.terminate(status)


if __name__ == "__main__":
    main()
