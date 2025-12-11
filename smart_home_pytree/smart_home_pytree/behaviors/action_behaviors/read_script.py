import py_trees
import subprocess
import tempfile
import os
from gtts import gTTS
import time

class ReadScript(py_trees.behaviour.Behaviour):
    def __init__(self, text: str, name="ReadScript"):
        super().__init__(name)
        self.text = text
        self.proc = None
        self.tmp_path = None

    def initialise(self):
        """
        Creates the audio file and launches the audio player subprocess.
        """
        try:
            # Create temp audio file
            tts = gTTS(self.text)
            tmp = tempfile.NamedTemporaryFile(delete=False, suffix=".mp3")
            self.tmp_path = tmp.name
            tmp.close()
            tts.save(self.tmp_path)

            # Start audio playback asynchronously
            self.proc = subprocess.Popen(
                ["mpg321", "-q", self.tmp_path],
                stdout=subprocess.DEVNULL,
                stderr=subprocess.DEVNULL
            )

        except Exception as e:
            print("[ERROR] Failed to initialise speech:", e)
            self.proc = None

    def update(self):
        """
        If audio is still playing, return RUNNING.
        When finished, return SUCCESS or FAILURE.
        """
        if self.proc is None:
            return py_trees.common.Status.FAILURE

        ret = self.proc.poll() 

        if ret is None:
            # Still running
            return py_trees.common.Status.RUNNING

        # Process finished
        if ret == 0:
            return py_trees.common.Status.SUCCESS
        else:
            return py_trees.common.Status.FAILURE

    def terminate(self, new_status):
        """
        Stop audio and clean up.
        """
        if self.proc is not None and self.proc.poll() is None:
            # Still playing, kill it
            self.proc.terminate()

        # Clean up the temp file
        if self.tmp_path and os.path.exists(self.tmp_path):
            os.remove(self.tmp_path)

        self.proc = None
        self.tmp_path = None
        
        
def main():
    read_script = ReadScript(
        "Kimleri sevdik, kimleri sildik Kimlerin peşine düştük genç ömrümüzde"
    )

    read_script.initialise()

    while True:
        status = read_script.update()
        print("STATUS:", status)

        if status != py_trees.common.Status.RUNNING:
            break

        time.sleep(0.1)

    read_script.terminate(status)


if __name__ == "__main__":
    main()