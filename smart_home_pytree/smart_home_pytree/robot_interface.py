import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from std_msgs.msg import String, Bool
import threading
from std_msgs.msg import String
from geometry_msgs.msg import PoseWithCovarianceStamped

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

        if not rclpy.ok():
            rclpy.init()

        super().__init__('robot_interface')
        self.state = RobotState()
        
        # Subscriptions
        self.create_subscription(PoseWithCovarianceStamped, "/amcl_pose", self.amcl_callback, 10)
        self.create_subscription(String, 'robot_location', self.robot_location_callback, 10)
        self.create_subscription(String, 'person_location', self.person_location_callback, 10)
        
        ## subcription for protocol events
        self.create_subscription(Bool, 'coffee', self.coffee_callback, 10)
        self.create_subscription(Bool, 'coffee_pot', self.coffee_pot_callback, 10)
        self.create_subscription(Bool, 'charging', self.charging_callback, 10)

        self.create_subscription(String, 'sim_time', self.sim_time_callback, 10)
        # Background spinning thread
        self._stop_event = threading.Event()
        self.spin_thread = threading.Thread(target=self._spin_background, daemon=True)
        self.spin_thread.start()

        self._initialized = True
        self.get_logger().info("RobotInterface initialized and spinning in background thread.")
        self.state.update('robot_location_xy', None)
    
    def amcl_callback(self, msg):
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.state.update('robot_location_xy', (x,y))

    # --- Spinning ---
    def _spin_background(self):
        while rclpy.ok() and not self._stop_event.is_set():
            rclpy.spin_once(self, timeout_sec=0.1)

    def shutdown(self):
        """Gracefully stop the background spinner."""
        self.get_logger().info("Shutting down RobotInterface...")
        self._stop_event.set()
        self.spin_thread.join(timeout=1.0)
        self.destroy_node()
        self.get_logger().info("RobotInterface shutdown complete.")
        
    # --- Callbacks ---
    def robot_location_callback(self, msg):
        self.get_logger().debug(f"Robot location: {msg.data}")
        self.state.update('robot_location', msg.data)

    def person_location_callback(self, msg):
        self.get_logger().debug(f"Person location: {msg.data}")
        self.state.update('person_location', msg.data)

    def charging_callback(self, msg):
        self.get_logger().debug(f"Charging: {msg.data}")
        self.state.update('charging', msg.data)
        
    def coffee_callback(self, msg):
        self.get_logger().debug(f"coffee : {msg.data}")
        self.state.update('coffee', msg.data)
    
    def coffee_pot_callback(self, msg):
        self.get_logger().debug(f"coffee_pot: {msg.data}")
        self.state.update('coffee_pot', msg.data)

    def sim_time_callback(self, msg: String):
        self.get_logger().debug(f"Simulated time: {msg.data}")
        # Example: msg.data = "10:30"
        self.state["sim_time"] = msg.data

## ros2 topic pub /sim_time std_msgs/msg/String "{data: '10:30'}"
def get_robot_interface():
    if not rclpy.ok():
        rclpy.init()
    return RobotInterface()


def main():
    rclpy.init()
    robot_interface = get_robot_interface()
    try:
        while rclpy.ok():
            pass
    except KeyboardInterrupt:
        pass
    finally:
        robot_interface.shutdown()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
