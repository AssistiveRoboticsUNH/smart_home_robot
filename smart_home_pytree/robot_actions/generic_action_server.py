#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.executors import MultiThreadedExecutor
from datetime import datetime

"""
Make sure in the execute callback override to publish feedback

example of how to do that:
    feedback_msg = self._action_type.Feedback()
    feedback_msg.percentage_completed = percentage
    goal_handle.publish_feedback(feedback_msg)
    self.get_logger().info(f"[{self._action_name}] Feedback: {percentage:.2f}%")

Raises:
    NotImplementedError: if execute_callabck is not implemented in your class

"""
    
class GenericActionServer(Node):
    """
    Generic Action Server template to be inherited for all actions.
    """

    def __init__(self, action_type, action_name):
        super().__init__(f"{action_name}_server")
        self._action_type = action_type
        self._action_name = action_name

        self._action_server = ActionServer(
            self,
            action_type,
            action_name,
            execute_callback=self._execute_with_logging,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback
        )

        self.get_logger().info(f"[{self._action_name}] Action Server initialized.")

    def goal_callback(self, goal_request):
        """
        Called when a goal is received.
        """
        self.get_logger().info(f"[{self._action_name}] Received goal: {goal_request}")
        # By default, accept all goals
        return GoalResponse.ACCEPT

    def cancel_callback(self, goal_handle):
        """
        Called when a cancel request is received.
        """
        self.get_logger().info(f"[{self._action_name}] Cancel requested: {goal_handle}")
        # By default, allow cancel
        return CancelResponse.ACCEPT

    # -----------------------------
    # Logging wrapper
    # To make sure all action servers log the same way
    # -----------------------------
    def _execute_with_logging(self, goal_handle):
        """
        Wraps the child execute_callback with logging before, during, and after execution.
        """
        start_time = datetime.now()
        self.get_logger().info(f"[{self._action_name}] Starting execution at {start_time}")

        try:
            # Call the overridden execute_callback in subclass
            result = self.execute_callback(goal_handle)
            status = "SUCCESS"
        except Exception as e:
            self.get_logger().error(f"[{self._action_name}] Execution failed: {e}")
            result = self._action_type.Result()  # default empty result
            status = "FAILURE"

        end_time = datetime.now()
        duration = (end_time - start_time).total_seconds()
        self.get_logger().info(f"[{self._action_name}] Execution finished at {end_time} ({duration:.2f}s) Status: {status}")

        return result
    

    def execute_callback(self, goal_handle):
        """
        Actual execution logic goes here in subclass.
        Must return an instance of action_type.Result
        """
        raise NotImplementedError("execute_callback must be implemented in the subclass")

    def shutdown(self):
        """
        Clean shutdown of the server.
        """
        self._action_server.destroy()
        self.get_logger().info(f"[{self._action_name}] Action server destroyed.")


"""
USE MultiThreadedExecutor to allow for cancelling the goal
"""

def run_action_server(action_server_class):
    """
    Generic main function to run an action server.

    Args:
        server_class: The class of the action server (must inherit GenericActionServer)
        *args, **kwargs: Arguments to pass to the server_class constructor
    """
    
    rclpy.init(args=None)

    server_instance = action_server_class()
    executor = MultiThreadedExecutor()
    executor.add_node(server_instance)

    try:
        executor.spin()
    except KeyboardInterrupt:
        server_instance.get_logger().info(f"[{server_instance._action_name}] Shutdown requested by user")
    finally:
        server_instance.shutdown()
        rclpy.try_shutdown()