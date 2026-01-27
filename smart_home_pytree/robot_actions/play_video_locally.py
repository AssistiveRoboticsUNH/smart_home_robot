"""
ROS 2 Action Server for Local Video Playback.

This node implements a polling-based ROS 2 action server that plays a video
locally on the host machine using an external video player. The video path
is provided directly through the action goal. No topics are used for control
or signaling.

The action:
- Starts video playback upon receiving a goal
- Publishes continuous feedback indicating the action is running
- Polls the video process until completion
- Supports clean cancellation by terminating playback

Intended for long-running UI or media tasks where synchronous execution
is not appropriate.
"""

import subprocess
import time

import rclpy
from rclpy.action import CancelResponse

from shr_msgs.action import PlayVideoRequest
from .generic_action_server import GenericActionServer, run_action_server


class PlayVideoLocalActionServer(GenericActionServer):
    """
    Action server that plays a video locally on the host machine.

    This server launches an external video player process (ffplay) using
    the file path provided in the action goal. Execution is managed using
    a polling loop rather than callbacks to ensure deterministic behavior.

    The server continuously publishes feedback indicating that playback
    is running until the video finishes or the goal is canceled.
    """

    def __init__(self):
        """
        Initialize the local video playback action server.

        Sets up internal state used to track the external video process
        and cancellation requests.
        """
        super().__init__(PlayVideoRequest, "play_video_local")
        self.process = None
        self.cancel_requested = False

    def cancel_callback(self, goal_handle):
        """
        Handle cancel requests for the video playback action.

        If a cancel request is received while a video is playing, the
        external video process is terminated immediately.

        :param goal_handle: The handle for the active action goal
        :return: CancelResponse.ACCEPT to allow cancellation
        """
        self.get_logger().info("Cancel requested: stopping video")
        self.cancel_requested = True

        if self.process and self.process.poll() is None:
            self.process.terminate()

        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        """
        Execute the video playback action.

        Launches the external video player and enters a polling loop that:
        - Continuously publishes running feedback
        - Monitors the video process for completion
        - Responds to cancellation requests

        The action succeeds when the video finishes playing and is
        canceled if a cancel request is received.

        :param goal_handle: The handle for the accepted action goal
        :return: Result message describing the outcome
        """
        video_path = goal_handle.request.file_name
        self.get_logger().info(f"Playing video locally: {video_path}")

        self.cancel_requested = False

        feedback = self._action_type.Feedback()
        result = self._action_type.Result()

        # Launch the external video player
        self.process = subprocess.Popen(
            [
                "ffplay",
                "-autoexit",
                "-loglevel", "quiet",
                video_path,
            ],
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )

        # Polling loop
        while rclpy.ok():
            # Always publish running feedback
            feedback.running = True
            goal_handle.publish_feedback(feedback)

            if self.cancel_requested:
                goal_handle.canceled()
                result.status = "video canceled"
                return result

            if self.process.poll() is not None:
                goal_handle.succeed()
                result.status = "video finished"
                return result

            time.sleep(0.5)


def main():
    """
    Entry point for running the local video playback action server.
    """
    run_action_server(PlayVideoLocalActionServer)


if __name__ == "__main__":
    main()
