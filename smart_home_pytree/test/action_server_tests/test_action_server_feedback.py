import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node
from shr_msgs.action import DockingRequest

class DockingClient(Node):
    def __init__(self):
        super().__init__("feedback_listener")
        self._action_client = ActionClient(self, DockingRequest, "/undocking")

    def send_goal(self):
        self.get_logger().info("Waiting for action server...")
        self._action_client.wait_for_server()

        goal_msg = DockingRequest.Goal()
        self.get_logger().info("Sending goal...")
        send_goal_future = self._action_client.send_goal_async(
            goal_msg, feedback_callback=self.feedback_callback
        )
        send_goal_future.add_done_callback(self.goal_response_callback)

    def feedback_callback(self, feedback_msg):
        self.get_logger().info(f"Received feedback: {feedback_msg.feedback.percentage_completed}")

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().error("Goal was rejected by the server")
            rclpy.shutdown()
            return

        self.get_logger().info("Goal accepted, waiting for result...")
        result_future = goal_handle.get_result_async()
        result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        status = future.result().status
        
        print("result", result)
        print("status", status)
        
        # action_msgs/msg/GoalStatus.
        # int8 STATUS_UNKNOWN=0
        # int8 STATUS_ACCEPTED=1
        # int8 STATUS_EXECUTING=2
        # int8 STATUS_CANCELING=3
        # int8 STATUS_SUCCEEDED=4
        # int8 STATUS_CANCELED=5
        # int8 STATUS_ABORTED=6
        # action_msgs/msg/GoalInfo goal_info
        # int8 status
        
        if status == 6:  # STATUS_ABORTED
            self.get_logger().error(f"Action failed with status ABORTED")
        elif status == 4:  # STATUS_SUCCEEDED
            self.get_logger().info(f"Action succeeded: {result}")
        else:
            self.get_logger().warn(f"Action finished with status code: {status}")

        self.get_logger().info("Shutting down client node.")
        rclpy.shutdown()

def main():
    rclpy.init()
    client = DockingClient()
    client.send_goal()
    rclpy.spin(client)

if __name__ == "__main__":
    main()
