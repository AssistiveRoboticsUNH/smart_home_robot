#!/usr/bin/env python3

import rclpy
from rclpy.node import Node

from std_msgs.msg import String, Bool
from rclpy.qos import (
    QoSProfile,
    ReliabilityPolicy,
    DurabilityPolicy,
    HistoryPolicy
)


class VoiceTriggerLatched(Node):

    def __init__(self):
        super().__init__('voice_trigger_latched')

        # -------- QoS: latched / reliable state --------
        self.state_qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        # -------- Publishers (LATCED) --------
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

        # -------- Voice Subscriptions --------
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

        self.create_subscription(
            String,
            '/voice/robot',
            self.voice_robot_callback,
            10
        )

        # -------- State Subscriptions (LATCED) --------
        self.create_subscription(
            Bool,
            'move_away',
            self.move_away_callback,
            self.state_qos
        )

        self.create_subscription(
            String,
            'position',
            self.position_location_callback,
            self.state_qos
        )

        self.get_logger().info("VoiceTriggerLatched node started (TRANSIENT_LOCAL enabled).")

    # ================= VOICE CALLBACKS =================

    def voice_status_callback(self, msg: String):
        self.get_logger().info(f"[VOICE STATUS] {msg.data}")

    def voice_user_callback(self, msg: String):
        text = msg.data.lower()
        self.get_logger().info(f"[VOICE USER] '{text}'")

        if "move home" in text:
            self.get_logger().info(
                "[DECISION] User said 'move home' → set move_away=True, position='home'"
            )
            self.publish_state(True, "home")

    def voice_robot_callback(self, msg: String):
        text = msg.data.lower()
        self.get_logger().info(f"[VOICE ROBOT] '{text}'")

        if "move away" in text:
            self.get_logger().info(
                "[DECISION] Robot said 'move away' → set move_away=True, position='away'"
            )
            self.publish_state(True, "away")

    # ================= STATE CALLBACKS =================

    def move_away_callback(self, msg: Bool):
        self.get_logger().info(f"[STATE SUB] move_away = {msg.data}")

    def position_location_callback(self, msg: String):
        self.get_logger().info(f"[STATE SUB] position = '{msg.data}'")

    # ================= PUBLISH HELPERS =================

    def publish_state(self, move_away: bool, position: str):
        move_msg = Bool()
        move_msg.data = move_away

        pos_msg = String()
        pos_msg.data = position

        self.move_away_pub.publish(move_msg)
        self.position_pub.publish(pos_msg)

        self.get_logger().info(
            f"[PUBLISHED (LATCHED)] move_away={move_away}, position='{position}'"
        )


def main(args=None):
    rclpy.init(args=args)
    node = VoiceTriggerLatched()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
