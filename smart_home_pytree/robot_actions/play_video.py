import rclpy
from rclpy.action import ActionServer
from shr_msgs.action import PlayVideoRequest
from std_msgs.msg import String
import time
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from .generic_action_server import run_action_server, GenericActionServer


class PlayVideoActionServer(GenericActionServer):
    def __init__(self):
        super().__init__(PlayVideoRequest, 'play_video')
        self.display_cb_group = MutuallyExclusiveCallbackGroup()

        self.display_pub = self.create_publisher(String, 'display_tx', 10)
        self.display_sub = self.create_subscription(String, 'display_rx', self.display_callback, 10, callback_group=self.display_cb_group)

    def display_callback(self, msg):
        # print(f"[Received] {msg.data}")  # print the message content
        if "RES:video_finished" in msg.data:
            self.video_finished = True
            print("callback self.video_finished", self.video_finished)
            self.get_logger().info("Video finished received!")


    def execute_callback(self, goal_handle):
        video_path = goal_handle.request.file_name
        self.get_logger().info(f"Received video goal: {video_path}")
        
        # Optional feedback
        feedback = self._action_type.Feedback()
        feedback.running = True
        goal_handle.publish_feedback(feedback)

        # Send video path 
        self.display_pub.publish(String(data=video_path))

        ## set to false whenever a video is recieved
        self.video_finished = False

        # Wait for up to 3 minutes for video to finish, check every second
        start_time = self.get_clock().now()
        timeout = rclpy.time.Duration(seconds=3*60)  # 3 minutes
        while not self.video_finished:
            time.sleep(1)
            if self.get_clock().now() - start_time > timeout:
                self.get_logger().warn(" Video did not finish in 5 minutes, aborting")
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