import time
import py_trees
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

VOICE_QOS = QoSProfile(
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
    depth=10,
)

class ReadScript(py_trees.behaviour.Behaviour):
    def __init__(
        self,
        text: str,
        node: Node,
        name="ReadScript",
        timeout=50.0,
    ):
        super().__init__(name)
        self.text = text
        self.node = node
        self.timeout = timeout

        self.pub = None
        self.sub_robot = None
        self.sub_status = None

        self._acknowledged = False
        self._speaking = False
        self._finished = False
        self._start_time = None

    def initialise(self):
        self._acknowledged = False
        self._speaking = False
        self._finished = False
        self._start_time = time.time()

        self.pub = self.node.create_publisher(
            String, "/voice/speak", VOICE_QOS
        )

        self.sub_robot = self.node.create_subscription(
            String, "/voice/robot", self._robot_cb, VOICE_QOS
        )

        self.sub_status = self.node.create_subscription(
            String, "/voice/status", self._status_cb, VOICE_QOS
        )

        msg = String()
        msg.data = self.text
        self.pub.publish(msg)

        self.node.get_logger().info(
            f"[ReadScript] Sent script to /voice/speak"
        )

    def _robot_cb(self, msg: String):
        """
        Robot confirms it accepted the script.
        """
        if msg.data == self.text:
            self._acknowledged = True
            self.node.get_logger().info(
                "[ReadScript] Script acknowledged by robot"
            )

    def _status_cb(self, msg: String):
        """
        Tracks speech lifecycle.
        """
        status = msg.data.upper()

        if not self._acknowledged:
            return

        if status == "SPEAKING":
            self._speaking = True
            self.node.get_logger().info("[ReadScript] Speaking started")

        elif status == "IDLE" and self._speaking:
            self._finished = True
            self.node.get_logger().info("[ReadScript] Speaking finished")

    def update(self):
        if time.time() - self._start_time > self.timeout:
            self.node.get_logger().error(
                "[ReadScript] Timeout waiting for speech completion"
            )
            return py_trees.common.Status.FAILURE

        if self._finished:
            return py_trees.common.Status.SUCCESS

        return py_trees.common.Status.RUNNING

    def terminate(self, new_status):
        if self.sub_robot:
            self.node.destroy_subscription(self.sub_robot)
            self.sub_robot = None

        if self.sub_status:
            self.node.destroy_subscription(self.sub_status)
            self.sub_status = None

        self.pub = None


def main():
    rclpy.init()
    node = rclpy.create_node("read_script_bt")

    behaviour = ReadScript(
        "Kimleri sevdik, kimleri sildik Kimlerin peşine düştük genç ömrümüzde",
        node=node,
    )

    behaviour.initialise()

    try:
        while rclpy.ok():
            rclpy.spin_once(node, timeout_sec=0.1)
            status = behaviour.update()
            print("STATUS:", status)

            if status != py_trees.common.Status.RUNNING:
                break
    finally:
        behaviour.terminate(status)
        node.destroy_node()
        rclpy.shutdown()



if __name__ == "__main__":
    main()