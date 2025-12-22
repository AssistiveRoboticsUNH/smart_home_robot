#!/usr/bin/env python3


"""
This script is responsible to get input from user when wake word is triggered and convert it to topics for robot_interface

"""

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)

### TODO: check if making human interace as robot interface, a singleton, instead of publishign topics, 
#### run the humn interface in a thread in robot_interface and update the states directly
### or run callback in a thread on its own in robot_interface
class HumanInterface(Node):
    def __init__(self):
        super().__init__('voice_trigger')

        # QoS: reliable state
        self.state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.VOLATILE,
        )

        ### Every topic that needs a person to trigger it needs to be here
        ## in the voice_user_callback, implmeemnt the logic in how you want to set it based on the input of the user
        # Publishers 
        self.move_away_pub = self.create_publisher(
            Bool,
            'move_away',
            self.state_qos
        )

        self.position_pub = self.create_publisher(
            String,
            'position',
            self.state_qos
        )

        # Voice Subscriptions 
        ### TODO: get when wakeword is triggere form status to trigger stop protocol
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

    # VOICE CALLBACKS

    def voice_status_callback(self, msg: String):
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
            self.publish_state(True, "home")

        elif "move away" in text:
            self.get_logger().info(
                "[DECISION] User said 'move away' → set move_away=True, position='away'"
            )
            print("move_away found")
            self.publish_state(True, "away")

    # PUBLISH HELPERS 
    def publish_state(self, move_away: bool, position: str):
        move_msg = Bool()
        move_msg.data = move_away

        pos_msg = String()
        pos_msg.data = position

        self.move_away_pub.publish(move_msg)
        self.position_pub.publish(pos_msg)

        self.get_logger().info(
            f"[PUBLISHED] move_away={move_away}, position='{position}'"
        )


def main(args=None):
    rclpy.init(args=args)
    node = HumanInterface()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
