import time
import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy, DurabilityPolicy
from std_msgs.msg import Bool

try:
    from nav2_msgs.action import DockRobot, UndockRobot
except ImportError:
    from opennav_docking_msgs.action import DockRobot, UndockRobot


class ChargingDockNode(Node):

    def __init__(self):
        super().__init__('charging_dock_manager')

        # 1. Define QoS Profile (Non-Volatile / Latched)
        # TRANSIENT_LOCAL means new subscribers get the last message immediately
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL
        )

        # 2. Create Publisher
        self.publisher_ = self.create_publisher(Bool, '/charging', self.qos_profile)

        # Initialize charging state (Assume not charging at start, or set based on logic)
        self._publish_charging_status(False)

        # 3. Create Action Servers
        self._dock_action_server = ActionServer(
            self,
            DockRobot,
            'dock_robot',
            self.execute_dock_callback
        )

        self._undock_action_server = ActionServer(
            self,
            UndockRobot,
            'undock_robot',
            self.execute_undock_callback
        )

        self.get_logger().info('Charging Dock Manager Started. Action Servers ready.')

    def _publish_charging_status(self, is_charging: bool):
        """Helper to publish the boolean status."""
        msg = Bool()
        msg.data = is_charging
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published /charging: {msg.data}')

    def execute_dock_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Dock Robot...')

        # Simulate the time it takes to dock (optional)
        time.sleep(1.0)

        # --- UPDATE CHARGING STATUS ---
        self._publish_charging_status(True)

        # Mark goal as successful
        goal_handle.succeed()

        result = DockRobot.Result()
        result.success = True
        return result

    def execute_undock_callback(self, goal_handle):
        self.get_logger().info('Executing goal: Undock Robot...')

        # Simulate the time it takes to undock (optional)
        time.sleep(1.0)

        # --- UPDATE CHARGING STATUS ---
        self._publish_charging_status(False)

        # Mark goal as successful
        goal_handle.succeed()

        result = UndockRobot.Result()
        result.success = True
        return result


def main(args=None):
    rclpy.init(args=args)

    node = ChargingDockNode()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
