import threading

import rclpy
from geometry_msgs.msg import PoseWithCovarianceStamped
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool, Int32, String


class RobotState:
    """Thread-safe singleton state container."""

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super(RobotState, cls).__new__(cls)
                    cls._instance._data = {}
                    cls._instance._owner = None
        return cls._instance

    # Data Management
    def update(self, key, value):
        """Update or add a key-value pair."""
        self._data[key] = value

    def get(self, key, default=None):
        """Get a value safely."""
        return self._data.get(key, default)

    def keys(self):
        """Return a list of all available keys in the state."""
        return list(self._data.keys())

    # Debug / Introspection
    def __str__(self):
        keys = list(self._data.keys())
        owner = self._owner or "None"
        return (
            f"=== RobotState ===\n"
            f"Keys: {keys if keys else 'No data yet'}\n"
            f"Owner: {owner}\n"
            f"=================="
        )

    __repr__ = __str__


class RobotInterface(Node):
    """Singleton ROS2 interface running in a background thread."""

    _instance = None
    _lock = threading.Lock()

    def __new__(cls):
        if cls._instance is None:
            with cls._lock:
                if cls._instance is None:
                    cls._instance = super().__new__(cls)
        return cls._instance

    def __init__(self):
        if getattr(self, "_initialized", False):
            return

        super().__init__("robot_interface")
        self.state = RobotState()
        self.qos_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )

        self.move_away_profile = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE,
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            durability=DurabilityPolicy.VOLATILE,
        )

        # Subscriptions
        self.create_subscription(
            PoseWithCovarianceStamped,
            "/amcl_pose",
            self.amcl_callback,
            self.qos_profile,
        )
        self.create_subscription(
            String, "robot_location", self.robot_location_callback, 10
        )
        self.create_subscription(
            String, "person_location", self.person_location_callback, 10
        )

        # can be postion1 or position 2
        self.create_subscription(
            String, "position", self.position_location_callback, self.move_away_profile
        )
        self.create_subscription(
            Bool, "move_away", self.move_away_callback, self.move_away_profile
        )

        # subcription for protocol events
        self.create_subscription(Bool, "coffee", self.coffee_callback, 10)
        self.create_subscription(Bool, "coffee_pot", self.coffee_pot_callback, 10)
        self.create_subscription(
            Bool, "charging", self.charging_callback, self.qos_profile
        )

        self.create_subscription(String, "display_rx", self.display_callback, 10)

        self.create_subscription(String, "sim_time", self.sim_time_callback, 10)

        self._initialized = True
        self.get_logger().info(
            "RobotInterface initialized and spinning in background thread."
        )
        # self.state.update('robot_location_xy', None)

        self.get_logger().warn(
            "person_location initialset to living_room. SHOULD BE ONLY USED FOR TESTING"
        )
        self.state.update("person_location", "living_room")

    def amcl_callback(self, msg):
        print("updating robot amcl")
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.state.update("robot_location_xy", (x, y))

    def shutdown(self):
        """Gracefully stop the background spinner."""
        self.get_logger().info("Shutting down RobotInterface...")
        self.destroy_node()
        self.get_logger().info("RobotInterface shutdown complete.")

    # --- Callbacks ---
    def display_callback(self, msg):
        if "exercise_requested" in msg.data:
            self.state.update("start_exercise", True)
            self.state.update("stop_exercise", False)
        elif "exercise_stop" in msg.data:
            self.state.update("stop_exercise", True)
            self.state.update("start_exercise", False)

    def robot_location_callback(self, msg):
        self.get_logger().debug(f"Robot location: {msg.data}")
        self.state.update("robot_location", msg.data)

    def position_location_callback(self, msg):
        self.get_logger().debug(f"Position location: {msg.data}")
        self.state.update("position", msg.data)

    def person_location_callback(self, msg):
        self.get_logger().debug(f"Person location: {msg.data}")
        self.state.update("person_location", msg.data)

    def charging_callback(self, msg):
        self.get_logger().debug(f"Charging: {msg.data}")
        self.state.update("charging", msg.data)

    def move_away_callback(self, msg):
        self.get_logger().debug(f"Move away: {msg.data}")
        self.state.update("move_away", msg.data)

    def coffee_callback(self, msg):
        self.get_logger().debug(f"coffee : {msg.data}")
        self.state.update("coffee", msg.data)

    def coffee_pot_callback(self, msg):
        self.get_logger().debug(f"coffee_pot: {msg.data}")
        self.state.update("coffee_pot", msg.data)

    def sim_time_callback(self, msg: String):
        self.get_logger().debug(f"Simulated time: {msg.data}")
        # Example: msg.data = "10:30"
        self.state["sim_time"] = msg.data


# ros2 topic pub /sim_time std_msgs/msg/String "{data: '10:30'}"


def main():
    """for standalone testing ONLY"""
    rclpy.init()

    robot_interface = RobotInterface()
    executor = MultiThreadedExecutor()
    executor.add_node(robot_interface)

    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        robot_interface.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
