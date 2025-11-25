#!/usr/bin/env python3

'''
Check if robot and person are in the same location. gets the location from robot_interface which reads it from the topics
'''
import py_trees

from gtts import gTTS
import tempfile

import os
import subprocess
import shlex
import tempfile


import os
import shlex
import tempfile
import subprocess
from gtts import gTTS
import py_trees

class ReadScript(py_trees.behaviour.Behaviour):
    """
    Behavior that reads a text aloud using gTTS and mpg321.
    Returns SUCCESS if playback completes, FAILURE otherwise.
    """
    def __init__(self, text: str, name="ReadScript"):
        super().__init__(name)
        self.text = text

    def update(self):
        print("Reading script...")

        try:
            print(f"[Reading aloud] {self.text}")
            # Convert text to speech
            tts = gTTS(self.text)

            # Create a temporary file for the audio
            with tempfile.NamedTemporaryFile(delete=False, suffix=".mp3") as tmpfile:
                tmp_path = tmpfile.name
                tts.save(tmp_path)

            # Play the audio file
            cmd = f"mpg321 -q {tmp_path}"  # -q for quiet mode
            result = subprocess.run(shlex.split(cmd), capture_output=True)

            # Clean up
            os.remove(tmp_path)

            # Check if playback succeeded
            if result.returncode != 0:
                print(f"[ERROR] Audio playback failed: {result.stderr.decode().strip()}")
                return py_trees.common.Status.FAILURE

            print("[SUCCESS] Script read successfully.")
            return py_trees.common.Status.SUCCESS

        except Exception as e:
            print(f"[ERROR] ReadScript failed: {e}")
            return py_trees.common.Status.FAILURE
  
def main():
    read_script = ReadScript("Kimleri sevdik, kimleri sildik Kimlerin peşine düştük genç ömrümüzde")
    read_script.update()
        
if __name__ == "__main__":
    main()
    