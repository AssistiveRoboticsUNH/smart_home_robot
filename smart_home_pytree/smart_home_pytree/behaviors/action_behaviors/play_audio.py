
from rclpy.action import ActionServer
from rclpy.node import Node
import rclpy
import os
import py_trees

class PlayAudio(py_trees.behaviour.Behaviour):
    """
    Behavior that reads a text aloud using gTTS and mpg321.
    Returns SUCCESS if playback completes, FAILURE otherwise.
    """
    def __init__(self, audio_path: str, name="PlayAudio"):
        super().__init__(name)
        self.audio_path = audio_path

    def update(self):
        print("Play audio...")
        if not os.path.isfile(self.audio_path):
            print("weblog="+'audio file does not exist')
            return py_trees.common.Status.FAILURE
        
        try:
            # Play the audio
            command = 'mpg321 ' + self.audio_path
            os.system(command)
            
            print("[SUCCESS] Play audio successfully.")
            return py_trees.common.Status.SUCCESS

        except Exception as e:
            print(f"[ERROR] PlayAudio failed: {e}")
            return py_trees.common.Status.FAILURE
        



def main():
    play_audio = PlayAudio("/home/olagh48652/smart-home/src/smart-home-robot/shr_resources/resources/food_reminder.mp3")
    play_audio.update()
        
if __name__ == "__main__":
    main()