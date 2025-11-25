#!/usr/bin/env python3

import py_trees
import py_trees_ros
from geometry_msgs.msg import PoseStamped, Quaternion
from nav2_msgs.action import NavigateToPose
import rclpy
from rclpy.action import ActionClient
import tf_transformations

def yaw_to_quaternion(yaw):
    # Convert yaw to quaternion (roll=0, pitch=0)
    q = tf_transformations.quaternion_from_euler(0, 0, yaw)
    return Quaternion(x=q[0], y=q[1], z=q[2], w=q[3])

class MoveToLandmark(py_trees.behaviour.Behaviour):
    """
    Dynamic ActionClient behavior that reads target location from blackboard/state and sends a navigation goal to Nav2's /navigate_to_pose action server.
    Uses the ROS2 native ActionClient (not py_trees_ros).
    """
    def __init__(self, robot_interface, location="", location_key="person_location", name="MoveToLandmark"):
        super().__init__(name)
        self.robot_interface = robot_interface
        
        self.location = location
        self.location_key = location_key
        
        self.blackboard = py_trees.blackboard.Blackboard()
        
        self.goal_future = None
        self.result_future = None
        self.goal_handle = None
        self.sent_goal = False
        
        self.debug = False   
        
        
    def setup(self, **kwargs):
        
        print("MoveTO behavior robot_interface ",self.robot_interface)
        print("MoveTO behavior self id:", id(self))
        
        if not self.robot_interface:
            raise RuntimeError("MoveToLandmark requires a ROS2 node during setup.")
        
        # Initialize ROS2 node and action client shell
        self.action_client = ActionClient(self.robot_interface, NavigateToPose, '/navigate_to_pose')

        print(f"[INFO] [{self.name}] Waiting for /navigate_to_pose server...")
        if not self.action_client.wait_for_server(timeout_sec=10.0):
            print(f"[ERROR] [{self.name}] Action server not available after waiting")
            return py_trees.common.Status.FAILURE
        
        print(f"[INFO] [{self.name}] Setup complete.")
        

    def initialise(self):
        # Get target location from state/blackboard and Send the goal request.
        locations = self.blackboard.get("locations")
        
        if self.debug:
            print(f"[DEBUG] [{self.name}] Preparing to send goal")

        if self.location:
            target_name = self.location
        else:
            target_name = self.blackboard.get(self.location_key)
            
        self.blackboard.set("going_to_location",target_name)
        
        # target_name = self.location or self.blackboard.get(self.location_key)

        
        if target_name not in locations:
            self.logger.error(f"Target {target_name} not in locations.")
            self.feedback_message = "Invalid target location"
            self.action_client = None
            return py_trees.common.Status.FAILURE

        target = locations[target_name]
        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.pose.position.x = target["x"]
        pose.pose.position.y = target["y"]
        yaw = target["yaw"]
        quat = yaw_to_quaternion(yaw)
        pose.pose.orientation = quat

        goal_msg = NavigateToPose.Goal()
        goal_msg.pose = pose

        if self.debug:
            print(f"[DEBUG] [{self.name}] Sending goal -> x={pose.pose.position.x}, y={pose.pose.position.y}")

            
        self.goal_future = self.action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self.goal_future.add_done_callback(self.goal_response_callback)
        self.sent_goal = True

    def feedback_callback(self, feedback_msg):
        """Handle feedback from the action server."""
        feedback = feedback_msg.feedback
        if self.debug:
            print(f"[DEBUG] [{self.name}] Feedback: distance_remaining={feedback.distance_remaining:.2f}")

    def goal_response_callback(self, future):
        """Handle goal acceptance or rejection."""
        self.goal_handle = future.result()
        if not self.goal_handle.accepted:
            print(f"[WARN] [{self.name}] Goal rejected by server.")
            self.sent_goal = False
            self.feedback_message = "Goal rejected"
            try:
                self.blackboard.unset("going_to_location")
                # print("goal_response_callback unset self.blackboard ")
            except:
                print("goal_response_callback: going_to_location not set so couldnt unset")
            return py_trees.common.Status.FAILURE

        if self.debug:
            print(f"[DEBUG] [{self.name}] Goal accepted. Waiting for result...")
        self.result_future = self.goal_handle.get_result_async()
            
    def update(self):
        """Periodic tick. Check goal progress and result."""
        if not self.sent_goal:
            if self.debug:
                print(f"[DEBUG] [{self.name}] No active goal.")
            self.blackboard.unset("going_to_location")
            return py_trees.common.Status.FAILURE

        # its already 
        # rclpy.spin_once(self.robot_interface, timeout_sec=0.1)

        if self.result_future and self.result_future.done():
            result = self.result_future.result()
            status = result.status
            if status == 4:  # SUCCEEDED
                print(f"[INFO] [{self.name}] Goal succeeded.")
                self.sent_goal = False
                try:
                    self.blackboard.unset("going_to_location")
                    # print(" update unset self.blackboard ")
                except:
                    print("update going_to_location not set so couldnt unset")
                return py_trees.common.Status.SUCCESS
            else:
                print(f"[WARN] [{self.name}] Goal failed with status {status}.")
                self.sent_goal = False
                
                try:
                    self.blackboard.unset("going_to_location")
                    print("update unset self.blackboard ")
                except:
                    print("update going_to_location not set so couldnt unset")            

                return py_trees.common.Status.FAILURE

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        """Cancel goal if behavior is stopped or preempted."""
        if self.goal_handle and self.sent_goal:
            print(f"[DEBUG] [{self.name}] Cancelling current goal...")
            cancel_future = self.goal_handle.cancel_goal_async()
            cancel_future.add_done_callback(self._cancel_done_callback)
            try:
                self.blackboard.unset("going_to_location")
                # print("in cancel unset self.blackboard ")
            except:
                print("in cancel going_to_location not set so couldnt unset")
        else:
            print(f"[DEBUG] [{self.name}] No active goal to cancel on terminate.")
     

    def _cancel_done_callback(self, future):
        """Confirm goal cancellation."""
        try:
            cancel_response = future.result()
            if len(cancel_response.goals_canceling) > 0:
                print(f"[INFO] [{self.name}] Goal successfully cancelled.")
            else:
                print(f"[WARN] [{self.name}] No active goals were cancelled.")
        except Exception as e:
            print(f"[ERROR] [{self.name}] Exception during cancel: {e}")

