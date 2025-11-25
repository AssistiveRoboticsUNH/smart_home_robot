#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse

from shr_msgs.action import DockingRequest  # using as a placeholder action
from .generic_action_server import run_action_server, GenericActionServer
from rclpy.executors import MultiThreadedExecutor
import time
from geometry_msgs.msg import Twist
import os
import std_msgs.msg import Bool

class DockingActionServer(GenericActionServer):
    def __init__(self):
        super().__init__(DockingRequest, "undocking")
        
        self.vel_pub = self.create_publisher(Twist, os.getenv("cmd_vel"), 10)

        self.time_out = 2
        # self.min_range = None
        
        self.charging_sub = self.create_subscription(
            Bool,
            "charging",
            self.charging_callback,
            10
        )
        self.is_robot_charging = True

    def charging_callback(self, msg):
        if msg is not None:
            self.is_robot_charging = msg.data

    def execute_callback(self, goal_handle):
        # Do the docking logic here
        print(" Starting undocking! ")
        
        feedback = self._action_type.Feedback()
        result = self._action_type.Result()
        
        start_time = time.time()
        speed = 3.14 / 15.0
        msg = Twist()
        
        while time.time() - start_time < self.time_out:
            if goal_handle.is_cancel_requested:
                goal_handle.canceled()
                return self._action_type.Result()
            
            msg.linear.x = speed # Go forward to undock
            self.vel_pub.publish(msg)
            
            feedback.running = True
            goal_handle.publish_feedback(feedback)
        
        # stop the robot after undocking
        msg.linear.x = 0.0
        self.vel_pub.publish(msg)  
        
        if not self.is_robot_charging:
            goal_handle.succeed()
            result.result = True 
            print('undocking is successful')
        else:
            goal_handle.abort()
            result.result = False 
            print('undocking failed, robot is still charging')
    
        return result

  
def main():
    run_action_server(DockingActionServer)  
    
# ----------------------------
# Use generic main to run the server
# ----------------------------
if __name__ == "__main__":
    run_action_server(DockingActionServer)  
    
    
## this works
# if __name__ == "__main__":
#      # Replace with your actual action
#     rclpy.init()

#     server = DockingActionServer()
#     executor = MultiThreadedExecutor()
#     try:
#         executor.add_node(server)
#         executor.spin()
#     except KeyboardInterrupt:
#         pass
#     finally:
#         server.shutdown()
#         rclpy.try_shutdown()