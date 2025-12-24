#!/usr/bin/env python3


"""
This script is responsible to get input from user when wake word is triggered and convert it to topics for robot_interface

"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy


class HumanInterface(Node):
    def __init__(self, node_name:str = 'voice_trigger', robot_interface = None):
        super().__init__(node_name)

        # QoS: reliable state
        self.state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        self.robot_interface = robot_interface
        ### Every topic that needs a person to trigger it needs to be here
        ## in the voice_user_callback, implmeemnt the logic in how you want to set it based on the input of the user
        
        # Publishers 
        # self.move_away_pub = self.create_publisher(
        #     Bool,
        #     'move_away',
        #     self.state_qos
        # )

        # self.position_pub = self.create_publisher(
        #     String,
        #     'position',
        #     self.state_qos
        # )

        # Voice Subscriptions 
        self.create_subscription(
            String,
            '/voice/status',
            self.voice_status_callback,
            10
        )

        self.create_subscription(
            String,
            '/voice/user',
            self.voice_user_callback,
            10
        )

    def voice_status_callback(self, msg: String):
        wakeup_triggered = "WAKEWORD_TRIGGER"
        done_waking = "IDLE"
        # set event here
        # event
        if msg.data == wakeup_triggered:
            print("trigger thread ot stop protocol and listen")
        if msg.data == done_waking
            print("stop event")
        self.get_logger().info(f"[VOICE STATUS] {msg.data}")

    def voice_user_callback(self, msg: String):
        text = msg.data.lower()
        self.get_logger().info(f"[VOICE USER] '{text}'")
        print(f"[VOICE USER] '{text}'")
        
        if "move home" in text:
            self.get_logger().info(
                "[DECISION] User said 'move home', set move_away=True, position='home'"
            )
            print("move home found")
            
            ## instead of publishing to topic directly update robot state
            self.state.update('move_away', msg.data)
            self.state.update('position', "home")
            # self.publish_state(True, "home")

        elif "move away" in text:
            self.get_logger().info(
                "[DECISION] User said 'move away' → set move_away=True, position='away'"
            )
            print("move_away found")
            
            ## instead of publishing to topic directly update robot state
            self.state.update('move_away', msg.data)
            self.state.update('position', "away")
            # self.publish_state(True, "away")

    
    # def publish_state(self, move_away: bool, position: str):
    #     move_msg = Bool()
    #     move_msg.data = move_away

    #     pos_msg = String()
    #     pos_msg.data = position

    #     self.move_away_pub.publish(move_msg)
    #     self.position_pub.publish(pos_msg)

    #     self.get_logger().info(
    #         f"[PUBLISHED] move_away={move_away}, position='{position}'"
    #     )


def main(args=None):
    rclpy.init(args=args)
    node = HumanInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
