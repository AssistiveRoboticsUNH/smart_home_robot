import os
import pprint
import time

import py_trees
import py_trees_ros
import rclpy

from shr_msgs.action import QuestionRequest  # Ensure this import is correct
from smart_home_pytree.behaviors.set_protocol_bb import SetProtocolBB
from smart_home_pytree.registry import load_protocols_to_bb


class AskQuestionBehavior(py_trees_ros.actions.ActionClient):
    """
    A custom ROS 2 action client behavior that evaluates user responses.

    This behavior sends a goal to the 'ask_question' action server. It interprets
    the resulting string:
    - If "yes": Sets Blackboard completion key to True and returns SUCCESS.
    - If "no": Sets Blackboard completion key to False and returns FAILURE.
    - If invalid/fail: Returns FAILURE without updating the Blackboard.

    Attributes:
        protocol_name (str): The name of the current protocol (e.g., 'medicine_reminder').
        data_key (str): The specific task key (e.g., 'first_dose').
        blackboard (py_trees.blackboard.Blackboard): Local handle to the global blackboard.
    """

    def __init__(self, name, action_goal, protocol_name, data_key):
        """
        Initializes the AskQuestionBehavior.

        Args:
            name: Name of the behavior node.
            action_goal: The goal message containing the file_name/question.
            protocol_name: Used to construct the blackboard storage path.
            data_key: Used to construct the blackboard storage path.
        """
        super().__init__(
            name=name,
            action_type=QuestionRequest,
            action_name="ask_question",
            action_goal=action_goal,
            wait_for_server_timeout_sec=120.0,
        )
        self.protocol_name = protocol_name
        self.data_key = data_key
        self.blackboard = py_trees.blackboard.Blackboard()
        self.blackboard.goal = action_goal

    def update(self) -> py_trees.common.Status:
        """
        Check the status of the action and evaluate the result on success.

        Returns:
            py_trees.common.Status: SUCCESS if 'yes', FAILURE if 'no' or error,
                                   otherwise the current action status (RUNNING).
        """
        status = super().update()

        # Still running or failed upstream
        if status != py_trees.common.Status.SUCCESS:
            return status

        # Result is stored HERE by py_trees_ros
        result_msg = self.result_message
        if result_msg is None:
            self.logger.error("Action finished but result_message is None")
            return py_trees.common.Status.FAILURE

        print("result_msg: ", result_msg)
        # ROS 2 action result structure
        answer = (result_msg.result.response or "").strip().lower()
        self.logger.info(f"User answered: {answer}")

        # Mark wait as done so selector skips it
        set_bb_node = SetProtocolBB(
            name="MarkAskQuestionDone",
            key=f"{self.protocol_name}_done.{self.data_key}_done",
            value=True,
        )

        if answer == "yes":
            set_bb_node.update()
            return py_trees.common.Status.SUCCESS
        elif answer == "no":
            # set_bb_node.update() ## no need to set it will just fail the tree
            return py_trees.common.Status.FAILURE
        else:
            # done not updated
            self.logger.error("Unexpected answer")
            return py_trees.common.Status.FAILURE


def main():
    rclpy.init()
    # 1. Create the node manually
    node = rclpy.create_node("test_node")
    yaml_file_path = os.getenv("house_yaml_path", None)

    blackboard = py_trees.blackboard.Blackboard()
    load_protocols_to_bb(yaml_file_path, debug=False)

    protocol_name = "exercise"
    protocol_info = blackboard.get(protocol_name)
    data_key = "get_confirmation"
    question_text = protocol_info[data_key]

    question_goal = QuestionRequest.Goal()
    question_goal.question = question_text
    print(question_goal)
    # 2. Instantiate your behavior
    # Note: Using your custom behavior from earlier
    behavior = AskQuestionBehavior(
        name="test_ask",
        action_goal=question_goal,
        protocol_name=protocol_name,
        data_key=data_key,
    )

    # 3. CRITICAL: Setup requires the ROS node
    behavior.setup(node=node)

    behavior.initialise()

    while rclpy.ok():
        # 4. Process ROS messages (listen for the action answer)
        rclpy.spin_once(node, timeout_sec=0.01)

        status = behavior.update()
        print(f"STATUS: {status}")

        if status != py_trees.common.Status.RUNNING:
            break

        time.sleep(0.1)

    print("\n=== Blackboard contents ===")
    pprint.pprint(blackboard.storage, width=120)

    behavior.terminate(status)
    rclpy.shutdown()


if __name__ == "__main__":
    main()
