import time

import rclpy
from rclpy.action import CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from std_msgs.msg import String

from shr_msgs.action import PlayVideoRequest
from smart_home_pytree.utils import validate_media_asset_exists

from .generic_action_server import GenericActionServer, run_action_server


class PlayVideoActionServer(GenericActionServer):
    def __init__(self):
        super().__init__(PlayVideoRequest, "play_video")
        self.display_cb_group = MutuallyExclusiveCallbackGroup()

        self.display_pub = self.create_publisher(String, "display_tx", 10)
        self.display_sub = self.create_subscription(
            String,
            "display_rx",
            self.display_callback,
            10,
            callback_group=self.display_cb_group,
        )

    def display_callback(self, msg):
        if "RES:video_finished" in msg.data:
            self.video_finished = True
            self.get_logger().info("Video finished received!")

    # turn video off
    def cancel_callback(self, goal_handle):
        """
        Called when a cancel request is received.
        """
        self.get_logger().info(f"[{self._action_name}] Cancel requested: {goal_handle}")

        # stop video
        self.display_pub.publish(String(data="stop_video"))
        # By default, allow cancel
        return CancelResponse.ACCEPT

    def execute_callback(self, goal_handle):
        requested_video = goal_handle.request.file_name
        self.get_logger().info(f"Received video goal: {requested_video}")

        # Optional feedback
        feedback = self._action_type.Feedback()
        feedback.running = True
        goal_handle.publish_feedback(feedback)

        try:
            video_relative_path, video_abs_path = validate_media_asset_exists(
                requested_video, "video"
            )
        except (ValueError, FileNotFoundError) as exc:
            self.get_logger().error(f"Video request rejected: {exc}")
            goal_handle.abort()
            result = self._action_type.Result()
            result.status = f"video request rejected: {exc}"
            return result

        self.get_logger().info(
            f"Sending tablet play command for '{video_relative_path}' (source: {video_abs_path})"
        )
        self.display_pub.publish(String(data=f"play_video:{video_relative_path}"))

        # set to false whenever a video is recieved
        self.video_finished = False

        # Wait for up to 3 minutes for video to finish, check every second
        start_time = self.get_clock().now()
        timeout = rclpy.time.Duration(seconds=3 * 60)  # 3 minutes
        while not self.video_finished:
            time.sleep(1)
            if self.get_clock().now() - start_time > timeout:
                self.get_logger().warn("Video did not finish in 3 minutes, aborting")
                goal_handle.abort()
                result = self._action_type.Result()
                result.status = "video failed or timeout"
                return result

        self.get_logger().info("Video finish, success")

        goal_handle.succeed()
        result = self._action_type.Result()
        result.status = "video sent"
        return result


def main():
    run_action_server(PlayVideoActionServer)


# ----------------------------
# Use generic main to run the server
# ----------------------------
if __name__ == "__main__":
    main()
